#include "mdasp_main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "hal/i2s_hal.h"
#include "esp_dsp.h"
#include "compressor/compressor.h"
#include "bt_service.h"
#include "trapezoidal_state_filter.c"

static inline float db2lin(float db){ // dB to linear
	return exp10f(0.05 * db);//powf(10.0f, 0.05f * db);
}

// Audio Buffer Variables
int16_t txbuf[BUF_LEN_STEREO];                      // output stereo samples
int16_t delaybuf[BUF_LEN_STEREO*(1+msDelayMax)];    // (delayed) input stereo samples

float l_dsp_buff[BUF_LEN], r_dsp_buff[BUF_LEN];
sf_sample_st comp_buff[BUF_LEN];                    // compressor buffer

// EQ coefficients
float hs_coeffs[2][6], ls_coeffs[2][6];

// Q values for cascading biquadratic filters to obtain 4th order butterworth response
static const float CASC_BUTTER_Q[2] = {0.54119610, 1.3065630};

uint32_t msDelayModel = 0;
bool delayUpdate = false;

float volumeModel = -30.0f;
bool volumeUpdate = false;

sf_compressor_state_st compState;

// Audio effect global variable models
ParametricEQ paramEQModel = {
    .passthrough = true,
    .hp = false,
    .hs = false,
    .br = false,
    .lp = false,
    .ls = false,
    .gain = 1.0f,
    .hp_freq = 0.25f,
    .hs_freq = 0.25f,
    .br_freq = 0.25f,
    .lp_freq = 0.25f,
    .ls_freq = 0.25f,
    .hs_amount = -20.0f,
    .br_amount = -20.0f,
    .ls_amount = -20.0f
    };

bool paramEqUpdate = false;

AdvancedCompressor compressorModel = {
    .passthrough = true,
    .makeupgain = true,
    .pregain = 0.0f,
    .threshold = -24.0f,
    .knee = 30.0f,
    .ratio = 12.0f,
    .attack = 0.003f,           // FL Maximus audio preset for attack = 0.002f;
    .release = 0.250f,          // FL Maximus audio preset for release = 0.01f;
    .predelay = 0.006f,
	.releasezone1 = 0.090f,
	.releasezone2 = 0.160f,
    .releasezone3 = 0.420f,
	.releasezone4 = 0.980f,
	.postgain = 0.000f,
	.wet = 1.000f
    };

bool drcUpdate = false;

/******************************************************************************************/
void i2s_audio_processor(void);

/******************************************************************************************/
 
void i2s_audio_processor(void) {
    size_t read_size;
    size_t write_size;
    esp_err_t read_ret = ESP_OK;
    esp_err_t write_ret = ESP_OK;

    uint16_t delayreadpos = 0;
    uint16_t delaywritepos = 0;
    uint32_t delaybufsize = 1;

    // EQ Variables
    float w[20][2]; // used to store previous inputs for filters
    memset(w, 0, sizeof(w));

    float dbVolume = volumeModel;
    AdvancedCompressor compressor = compressorModel;
    ParametricEQ paramEQ = paramEQModel;

    float linearVolume = db2lin(dbVolume);

    dsp_trap_state_gen_f32(&hs_coeffs[0][0], paramEQ.hs_freq, CASC_BUTTER_Q[0], 8);
    dsp_trap_state_gen_f32(&hs_coeffs[1][0], paramEQ.hs_freq, CASC_BUTTER_Q[1], 8);
    dsp_trap_state_gen_f32(&ls_coeffs[0][0], paramEQ.ls_freq, CASC_BUTTER_Q[0], 7);
    dsp_trap_state_gen_f32(&ls_coeffs[1][0], paramEQ.ls_freq, CASC_BUTTER_Q[1], 7);

    sf_defaultcomp(&compState, AUDIO_SAMPLE_RATE);

    while (1) {

        if (delayUpdate) {
            if (delaybufsize != (1 + msDelayModel)) {
                delaybufsize = (1 + msDelayModel);
                delaywritepos = 0;
                delayreadpos = delaybufsize > 1 ? BUF_LEN_STEREO : 0;
            }
        }
        
        // read stereo samples from DMA_in
        read_ret = i2s_read(I2S_NUM, &delaybuf[delaywritepos], BUF_BYTES, &read_size, portMAX_DELAY);
        //increment delay positions
		delayreadpos = (delayreadpos + BUF_LEN_STEREO) % (delaybufsize*BUF_LEN_STEREO);
		delaywritepos = (delaywritepos + BUF_LEN_STEREO) % (delaybufsize*BUF_LEN_STEREO);
        //read_ret = i2s_read(I2S_NUM, &rxbuf[0], BUF_BYTES, &read_size, portMAX_DELAY);
        if (read_ret == ESP_OK && read_size == BUF_BYTES) {

            // extract stereo samples to mono buffers
            int y = 0;
            for (int i = 0; i < BUF_LEN_STEREO; i = i+2) {
                l_dsp_buff[y] = (float) delaybuf[delayreadpos+i];//rxbuf[i];
                r_dsp_buff[y] = (float) delaybuf[delayreadpos+(i+1)];//rxbuf[i+1];
                y++;
            }

            if (volumeUpdate) {
                volumeUpdate = false;
                dbVolume = volumeModel;
                linearVolume = db2lin(dbVolume);
            }

            // subtract 90dB to leave us with a signal that can operate in <0dB range (signed int is 20log(2^15)=90dB)
            // we do this since max value of sf_compressor_process is 0dB, or 1. Volume control is also here (multiply linear volume)
            dsps_mulc_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, MINUS_90_DB * linearVolume, 1, 1);
            dsps_mulc_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, MINUS_90_DB * linearVolume, 1, 1);

            // Generate IIR Coefficients for 4th order Butterworth Filters
            // NO LONGER ARE WE TETHERED TO (most) GROSS COEFFICIENTS
            if (paramEqUpdate) {
                paramEqUpdate = false;
                if (paramEQ.hs_freq != paramEQModel.hs_freq || paramEQ.hs_amount != paramEQModel.hs_amount) {
                    paramEQ = paramEQModel;
                    dsp_trap_state_gen_f32(&hs_coeffs[0][0], paramEQ.hs_freq, CASC_BUTTER_Q[0], 8);
                    dsp_trap_state_gen_f32(&hs_coeffs[1][0], paramEQ.hs_freq, CASC_BUTTER_Q[1], 8);
                }
                if (paramEQ.ls_freq != paramEQModel.ls_freq || paramEQ.ls_amount != paramEQModel.ls_amount) {
                    paramEQ = paramEQModel;
                    dsp_trap_state_gen_f32(&ls_coeffs[0][0], paramEQ.ls_freq, CASC_BUTTER_Q[0], 7);
                    dsp_trap_state_gen_f32(&ls_coeffs[1][0], paramEQ.ls_freq, CASC_BUTTER_Q[1], 7);
                }
                paramEQ = paramEQModel;
            }
            // Generate new DRC parameters
            if (drcUpdate) {
                drcUpdate = false;
                compressor = compressorModel;
                sf_advancecomp(
                    &compState, 
                    AUDIO_SAMPLE_RATE,
                    compressor.pregain,
                    compressor.threshold,
                    compressor.knee,
                    compressor.ratio,
                    compressor.attack,
                    compressor.release,
                    compressor.predelay,
                    compressor.releasezone1,
                    compressor.releasezone2,
                    compressor.releasezone3,
                    compressor.releasezone4,
                    compressor.postgain,
                    compressor.wet,
                    compressor.makeupgain
                );
            }

            // Parametric EQ Filtering Section
            if (!paramEQ.passthrough) {
                if (paramEQ.hp) {
                    dsp_otf_state_filt_f32_sine(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, 0, paramEQ.hp_freq, CASC_BUTTER_Q[0], &w[0][0]);
                    dsp_otf_state_filt_f32_sine(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, 0, paramEQ.hp_freq, CASC_BUTTER_Q[0], &w[1][0]);
                    dsp_otf_state_filt_f32_sine(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, 0, paramEQ.hp_freq, CASC_BUTTER_Q[1], &w[2][0]);
                    dsp_otf_state_filt_f32_sine(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, 0, paramEQ.hp_freq, CASC_BUTTER_Q[1], &w[3][0]);
                }  else if (paramEQ.ls) {
                    dsp_trap_state_filt_f32_ansi(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, &ls_coeffs[0][0], &w[16][0]);
                    dsp_trap_state_filt_f32_ansi(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, &ls_coeffs[0][0], &w[17][0]);
                    dsp_trap_state_filt_f32_ansi(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, &ls_coeffs[1][0], &w[18][0]);
                    dsp_trap_state_filt_f32_ansi(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, &ls_coeffs[1][0], &w[19][0]);
                }
                if (paramEQ.br) {
                    dsp_otf_state_filt_f32_sine(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, 3, paramEQ.br_freq, CASC_BUTTER_Q[0], &w[8][0]);
                    dsp_otf_state_filt_f32_sine(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, 3, paramEQ.br_freq, CASC_BUTTER_Q[0], &w[9][0]);
                    dsp_otf_state_filt_f32_sine(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, 3, paramEQ.br_freq, CASC_BUTTER_Q[1], &w[10][0]);
                    dsp_otf_state_filt_f32_sine(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, 3, paramEQ.br_freq, CASC_BUTTER_Q[1], &w[11][0]);
                }
                if (paramEQ.lp) {
                    dsp_otf_state_filt_f32_sine(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, 2, paramEQ.lp_freq, CASC_BUTTER_Q[0], &w[12][0]);
                    dsp_otf_state_filt_f32_sine(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, 2, paramEQ.lp_freq, CASC_BUTTER_Q[0], &w[13][0]);
                    dsp_otf_state_filt_f32_sine(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, 2, paramEQ.lp_freq, CASC_BUTTER_Q[1], &w[14][0]);
                    dsp_otf_state_filt_f32_sine(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, 2, paramEQ.lp_freq, CASC_BUTTER_Q[1], &w[15][0]);
                } else if (paramEQ.hs) {
                    dsp_trap_state_filt_f32_ansi(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, &hs_coeffs[0][0], &w[4][0]);
                    dsp_trap_state_filt_f32_ansi(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, &hs_coeffs[0][0], &w[5][0]);
                    dsp_trap_state_filt_f32_ansi(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, &hs_coeffs[1][0], &w[6][0]);
                    dsp_trap_state_filt_f32_ansi(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, &hs_coeffs[1][0], &w[7][0]);
                }
            }

            // Dynamic Range Compression Section
            if (!compressor.passthrough) {
                for (int i = 0; i < BUF_LEN; i++) {
                    comp_buff[i] = (sf_sample_st) { .L = l_dsp_buff[i], .R = r_dsp_buff[i] };
                }
                sf_compressor_process(&compState, BUF_LEN, &comp_buff[0], &comp_buff[0]);
                for (int i = 0; i < BUF_LEN; i++) {
                    l_dsp_buff[i] = comp_buff[i].L;
                    r_dsp_buff[i] = comp_buff[i].R;
                }
            }

            // scale the signal back up by adding 90dB to bring us back to the correct output scale
            dsps_mulc_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, PLUS_90_DB, 1, 1);
            dsps_mulc_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, PLUS_90_DB, 1, 1);

            //merge l and r buffers into a mixed buffer for i2s
            y = 0;
            for (int i = 0; i < BUF_LEN; i++) {
                txbuf[y] = (int16_t) l_dsp_buff[i];
                txbuf[y+1] = (int16_t) r_dsp_buff[i];
                y = y+2;
            }
        
            //write BUF_BYTES number of samples to DMA_out
            write_ret = i2s_write(I2S_NUM, &txbuf[0], BUF_BYTES, &write_size, portMAX_DELAY);
            if (write_ret != ESP_OK || write_size != BUF_BYTES) {
                printf("i2s write failed. write status %d, written bytes %d\n", write_ret, write_size);
            }
        } else {
            printf("i2s read failed. read status %d, read bytes %d\n", read_ret, read_size);
        }
    }
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_ERROR);      // set all components to ERROR level
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX,
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB, //equivalent to I2S_COMM_FORMAT_I2S_LSB which handles 16-bit right aligned data
        .tx_desc_auto_clear = true,
        .use_apll = true,
        .dma_buf_count = 8,
        .dma_buf_len = 64,//128,    // # frames per DMA buffer. frame size = # channels * bytes per sample
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_MCK_IO,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO
    };
    //QueueHandle_t evt_que; //use in place of NULL in driver to see if you ever loose data
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);// &evt_que);
    i2s_set_pin(I2S_NUM, &pin_config);
    // You can reset parameters by calling 'i2s_set_clk'

    bt_gatt_run();

    xTaskCreate(i2s_audio_processor, "i2s_audio_processor", 4096, NULL, 3, NULL);
/*     while (1) {
        i2s_event_t evt;
        xQueueReceive(evt_que, &evt, portMAX_DELAY);
        if (evt.type == I2S_EVENT_RX_Q_OVF) {
            printf("RX data dropped\n");
        }
    } */
}
