#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include <math.h>
#include "hal/i2s_hal.h"
#include "esp_dsp.h"
#include "compressor/compressor.h"
 

#define AUDIO_SAMPLE_RATE   48000         //48kHz
#define BUF_LEN             256           //buffer length
#define BUF_LEN_STEREO     BUF_LEN*2    //buffer length * 2 channels (stereo)
#define BUF_BYTES    BUF_LEN_STEREO*2    //stereo buffer size * 16-bit (2 bytes)
#define I2S_NUM             I2S_NUM_0     //i2s port number
#define I2S_MCK_IO          GPIO_NUM_3    //i2s master clock
#define I2S_BCK_IO          GPIO_NUM_15   //i2s bit clock
#define I2S_WS_IO           GPIO_NUM_13   //i2s word select
#define I2S_DO_IO           GPIO_NUM_2    //i2s data out
#define I2S_DI_IO           GPIO_NUM_0    //i2s data in


typedef struct ParametricEQ {
    bool passthrough;
    bool hp;    // high pass
    bool hs;    // high shelf
    bool br;    // band reject
    bool lp;    // low pass
    bool ls;    // low shelf

    float gain;
    float hp_freq;
    float hs_freq;
    float br_freq;
    float lp_freq;
    float ls_freq;

    float hs_amount;
    float br_amount;
    float ls_amount;
} ParametricEQ;

typedef struct SimpleCompressor {
    bool passthrough;

    float pregain;
    float threshold;
    float knee;
    float ratio;
    float attack;
    float release;
} SimpleCompressor;

// Audio Buffer Variables
int16_t rxbuf[BUF_LEN_STEREO], txbuf[BUF_LEN_STEREO];    // stereo samples

float l_dsp_buff[BUF_LEN], r_dsp_buff[BUF_LEN];
sf_sample_st inCompBuff[BUF_LEN];
sf_sample_st outCompBuff[BUF_LEN];
size_t BUF_SIZE = sizeof(l_dsp_buff);

ParametricEQ paramEQModel = {
    .passthrough = false,
    .hp = false,
    .hs = false,
    .br = false,
    .lp = false,
    .ls = false,
    .gain = 1.0f,
    .hp_freq = 0.0f,
    .hs_freq = 0.0f,
    .br_freq = 0.25f,
    .lp_freq = 0.5f,
    .ls_freq = 0.5f,
    .hs_amount = 1.0f,
    .br_amount = 1.0f,
    .ls_amount = 1.0f
    };

bool paramEqUpdate = true;

float hp_coeffs[2][5];
float hs_coeffs[2][5];
float br_coeffs[2][5];
float lp_coeffs[2][5];
float ls_coeffs[2][5];

static const float CASC_BUTTER_Q[2] = {0.54119610, 1.3065630}; // biquad Q values to obtain 4th order butterworth response

sf_compressor_state_st compState;

SimpleCompressor compressorModel = {
    .passthrough = false,
    .pregain = 5.0f,
    .threshold = -24.0f,
    .knee = 30.0f,
    .ratio = 12.0f,
    .attack = 0.003f,
    .release = 0.250f
    };

bool drcUpdate = true;

/******************************************************************************************/
 
static void i2s_audio_processor(void *args) {
    size_t read_size;
    size_t write_size;
    esp_err_t dsp_err;
    esp_err_t read_ret;
    esp_err_t write_ret;

    // EQ Variables
    float w1[2][2], w2[2][2];
    memset(w1, 0, sizeof(w1));
    memset(w2, 0, sizeof(w2));
    SimpleCompressor compressor;
    ParametricEQ paramEQ;
    paramEQModel.lp = true;
    paramEQModel.lp_freq = 0.05f;
    compressorModel.passthrough = false;
    paramEQModel.passthrough = true;

    while (1) {

        // read stereo samples from DMA_in
        read_ret = i2s_read(I2S_NUM, &rxbuf[0], BUF_BYTES, &read_size, portMAX_DELAY);
        if (read_ret == ESP_OK && read_size == BUF_BYTES) {

            // extract stereo samples to mono buffers
            int y = 0;
            for (int i = 0; i < BUF_LEN_STEREO; i = i+2) {
                l_dsp_buff[y] = (float) rxbuf[i];
                r_dsp_buff[y] = (float) rxbuf[i+1];
                y++;
            }

            // Generate IIR Coefficients for 4th order Butterworth Filters
            if (paramEqUpdate) {
                paramEQ = paramEQModel;
                paramEqUpdate = false; // mutex write
                for (int i = 0; i < 2; i++) {
                    dsps_biquad_gen_hpf_f32(&hp_coeffs[i][0], paramEQ.hp_freq, CASC_BUTTER_Q[i]);
                    dsps_biquad_gen_highShelf_f32(&hs_coeffs[i][0], paramEQ.hs_freq, paramEQ.hs_amount, CASC_BUTTER_Q[i]);
                    dsps_biquad_gen_notch_f32(&br_coeffs[i][0], paramEQ.br_freq, paramEQ.br_amount, CASC_BUTTER_Q[i]);
                    dsps_biquad_gen_lowShelf_f32(&ls_coeffs[i][0], paramEQ.ls_freq, paramEQ.ls_amount, CASC_BUTTER_Q[i]);
                    dsps_biquad_gen_lpf_f32(&lp_coeffs[i][0], paramEQ.lp_freq, CASC_BUTTER_Q[i]);
                }
            }
            // Generate DRC values
            if (drcUpdate) {
                compressor = compressorModel;
                drcUpdate = false;
                sf_simplecomp(
                    &compState, 
                    AUDIO_SAMPLE_RATE,
                    compressor.pregain,
                    compressor.threshold,
                    compressor.knee,
                    compressor.ratio,
                    compressor.attack,
                    compressor.release
                );
            }

            // Parametric EQ Filtering Section
            if (!paramEQ.passthrough) {
                for (int i = 0; i < 2; i++) {
                    dsps_mulc_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, 0.75f, 1, 1);
                    dsps_mulc_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, 0.75f, 1, 1);
                    if (paramEQ.hp) {
                        dsp_err = dsps_biquad_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, &hp_coeffs[i][0], &w1[i][0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, &hp_coeffs[i][0], &w2[i][0]);
                    }
                    if (paramEQ.hs) {
                        dsp_err = dsps_biquad_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, &hs_coeffs[i][0], &w1[i][0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, &hs_coeffs[i][0], &w2[i][0]);
                    }
                    if (paramEQ.br) {
                        dsp_err = dsps_biquad_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, &br_coeffs[i][0], &w1[i][0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, &br_coeffs[i][0], &w2[i][0]);
                    }
                    if (paramEQ.lp) {
                        dsp_err = dsps_biquad_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, &lp_coeffs[i][0], &w1[i][0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, &lp_coeffs[i][0], &w2[i][0]);
                    }
                    if (paramEQ.ls) {
                        dsp_err = dsps_biquad_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, &ls_coeffs[i][0], &w1[i][0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, &ls_coeffs[i][0], &w2[i][0]);
                    }
                }
            }

            // DRC Section
            if (!compressor.passthrough) {
                memcpy(&inCompBuff[0].L, l_dsp_buff, BUF_SIZE);
                memcpy(&inCompBuff[0].R, r_dsp_buff, BUF_SIZE);
                sf_compressor_process(&compState, BUF_SIZE, &inCompBuff[0], &outCompBuff[0]);
                memcpy(l_dsp_buff, &outCompBuff[0].L, BUF_SIZE);
                memcpy(r_dsp_buff, &outCompBuff[0].R, BUF_SIZE);
            }

            //merge two l and r buffers into a mixed buffer and write back to HW
            y = 0;
            for (int i = 0; i < BUF_LEN; i++) {
                txbuf[y] = (int16_t) l_dsp_buff[i];
                txbuf[y+1] = (int16_t) r_dsp_buff[i];
                y = y+2;
            }
        
            //write BUF_SIZE samples to DMA_out
            write_ret = i2s_write(I2S_NUM, &txbuf[0], BUF_BYTES, &write_size, portMAX_DELAY);
            if (write_ret != ESP_OK || write_size != BUF_BYTES) {
                printf("i2s write failed. write status %d, written bytes %d\n", write_ret, write_size);
            }
        } else {
            printf("i2s read failed. read status %d, read bytes %d\n", read_ret, read_size);
        }
    }
}

/* static void bt_gatt_server(void *args) {
    while (1) {

    }

} */

void app_main(void) {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX,
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB, //equivalent to I2S_COMM_FORMAT_I2S_LSB which handles 16-bit right aligned data
        .tx_desc_auto_clear = true,
        .use_apll = true,
        .dma_buf_count = 8,
        .dma_buf_len = 128,    // # frames per DMA buffer. frame size = # channels * bytes per sample
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

    xTaskCreate(i2s_audio_processor, "i2s_audio_processor", 4096, NULL, 2, NULL);
    //xTaskCreate(bt_gatt_server, "bt_gatt_server", 4096, NULL, 1, NULL);
/*     while (1) {
        i2s_event_t evt;
        xQueueReceive(evt_que, &evt, portMAX_DELAY);
        if (evt.type == I2S_EVENT_RX_Q_OVF) {
            printf("RX data dropped\n");
        }
    } */
}
