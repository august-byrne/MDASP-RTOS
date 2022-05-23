#include "mdasp_main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "hal/i2s_hal.h"
#include "esp_dsp.h"
#include "compressor/compressor.h"
#include "bt_service.c"

// Audio Buffer Variables
int16_t rxbuf[BUF_LEN_STEREO], txbuf[BUF_LEN_STEREO];    // stereo samples

float l_dsp_buff[BUF_LEN], r_dsp_buff[BUF_LEN];
sf_sample_st inCompBuff[BUF_LEN];
sf_sample_st outCompBuff[BUF_LEN];
float l_dsp_buff_out[BUF_LEN], r_dsp_buff_out[BUF_LEN];

float hp_coeffs[2][5];
float hs_coeffs[2][5];
float br_coeffs[2][5];
float lp_coeffs[2][5];
float ls_coeffs[2][5];

static const float CASC_BUTTER_Q[2] = {0.54119610, 1.3065630}; // biquad Q values to obtain 4th order butterworth response

sf_compressor_state_st compState;

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
    AdvancedCompressor compressor;
    ParametricEQ paramEQ;
    paramEQModel.lp = true;
    paramEQModel.lp_freq = 0.05f;
    paramEQModel.hp = true;
    paramEQModel.hp_freq = 0.25f;
    paramEQModel.passthrough = true;
    compressorModel.passthrough = true;
    compressorModel.pregain = 12.0f;
    compressorModel.postgain = 90.0f;
    compressorModel.threshold = -64.0f;
    compressorModel.knee = 40.0f;
    compressorModel.ratio = 4.0f;
    //compressorModel.attack = 0.002f;
    //compressorModel.release = 0.01f;

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
                    compressor.wet
                );
                //sf_defaultcomp(&compState, AUDIO_SAMPLE_RATE);
            }

            //dsps_mulc_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, MINUS_90_DB, 1, 1);
            //dsps_mulc_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, MINUS_90_DB, 1, 1);

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

            // Dynamic Range Compression Section
            if (!compressor.passthrough) {
                // subtract 90dB to leave us with a signal that can operate in <0dB range (signed int is 20log(2^15)=90dB)
                // we do this since max value of sf_compressor_process is 0dB, or 1
                dsps_mulc_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, MINUS_90_DB, 1, 1);
                dsps_mulc_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, MINUS_90_DB, 1, 1);
                for (int i = 0; i < BUF_LEN; i++) {
                    inCompBuff[i] = (sf_sample_st) { .L = l_dsp_buff[i], .R = r_dsp_buff[i] };
                }
                sf_compressor_process(&compState, BUF_LEN, &inCompBuff[0], &outCompBuff[0]);
                // sf_compressor_process also adds 90dB (post gain) to bring us back to the correct output scale
                for (int i = 0; i < BUF_LEN; i++) {
                    l_dsp_buff_out[i] = outCompBuff[i].L;
                    r_dsp_buff_out[i] = outCompBuff[i].R;
                }
            }

            //dsps_mulc_f32_ae32(&l_dsp_buff[0], &l_dsp_buff[0], BUF_LEN, 31622.77, 1, 1);
            //dsps_mulc_f32_ae32(&r_dsp_buff[0], &r_dsp_buff[0], BUF_LEN, 31622.77, 1, 1);

            //merge two l and r buffers into a mixed buffer and write back to HW
            y = 0;
            for (int i = 0; i < BUF_LEN; i++) {
                txbuf[y] = (int16_t) l_dsp_buff_out[i];
                txbuf[y+1] = (int16_t) r_dsp_buff_out[i];
                y = y+2;
            }
        
            //write BUF_BYTES samples to DMA_out
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
        
        vTaskDelay(1);
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

    bt_gatt_initialize();

    xTaskCreate(i2s_audio_processor, "i2s_audio_processor", 4096, NULL, 3, NULL);
    //xTaskCreate(bt_gatt_server, "bt_gatt_server", 4096, NULL, 2, NULL);
/*     while (1) {
        i2s_event_t evt;
        xQueueReceive(evt_que, &evt, portMAX_DELAY);
        if (evt.type == I2S_EVENT_RX_Q_OVF) {
            printf("RX data dropped\n");
        }
    } */
}
