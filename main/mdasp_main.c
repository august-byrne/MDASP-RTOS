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
//#include "compressor/compressor.c"
#include "compressor/compressor.h"
//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#include "esp_log.h"
 

#define AUDIO_SAMPLE_RATE   48000         //48kHz
#define BUFF_LEN            256           //buffer length
#define I2S_NUM             I2S_NUM_0     //i2s port number
#define I2S_BCK_IO          GPIO_NUM_15   //i2s bit clock
#define I2S_WS_IO           GPIO_NUM_13   //i2s word select
#define I2S_DO_IO           GPIO_NUM_2    //i2s data out
#define I2S_DI_IO           GPIO_NUM_0    //i2s data in


typedef struct ParametricEQ {
    bool consumable_update;
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
    bool consumable_update;
    bool passthrough;

    float pregain;
    float threshold;
    float knee;
    float ratio;
    float attack;
    float release;
} SimpleCompressor;

// Audio Buffer Variables
int16_t rxbuf[BUFF_LEN*4], txbuf[BUFF_LEN*4];    // # of bytes = sizebuf_len * (bits_per_sample / 8) * buf_count * num_of_channels; 1024 works best so far
float l_in[BUFF_LEN*2], r_in[BUFF_LEN*2];

// EQ Variables
float w1[2], w2[2];
//filter buffers
float l_buff_eq[BUFF_LEN*2], r_buff_eq[BUFF_LEN*2];
//drc buffers
float l_buff_drc[BUFF_LEN*2], r_buff_drc[BUFF_LEN*2];
//volume buffers
float l_out[BUFF_LEN*2], r_out[BUFF_LEN*2];

ParametricEQ paramEQ = {
    .consumable_update = true,
    .passthrough = false,
    .hp = false,
    .hs = false,
    .br = false,
    .lp = false,
    .ls = false,
    .gain = 1.0f,
    .hp_freq = 0f,
    .hs_freq = 0f,
    .br_freq = 0.25f,
    .lp_freq = 0.5f,
    .ls_freq = 0.5f,
    .hs_amount = 1.0f,
    .br_amount = 1.0f,
    .ls_amount = 1.0f
    };

float hp_coeffs[2][5];
float hs_coeffs[2][5];
float br_coeffs[2][5];
float lp_coeffs[2][5];
float ls_coeffs[2][5];

/* static const float BUTTER_Q[10] = {  // Biquad Q values for cascading to obtain a butterworth response
    0.50154610,
    0.51420760,
    0.54119610,
    0.58641385,
    0.65754350,
    0.76988452,
    0.95694043,
    1.3065630,
    2.1418288,
    6.3727474};  // Q values required for a 20th order butterworth equivalent */

/* static const float SHORT_BUTTER_Q[6] = {  // Biquad Q values for cascading to obtain a 12th order butterworth response
    0.50431448,
    0.54119610,
    0.63023621,
    0.82133982,
    1.3065630,
    3.8306488}; */

static const float CASC_BUTTER_Q[2] = {0.54119610, 1.3065630}; // biquad Q values to obtain 4th order butterworth response

sf_compressor_state_st compState;

SimpleCompressor compressor = {
    .consumable_update = true,
    .passthrough = false,
    .pregain = 5.0f,
    .threshold = -24.0f,
    .knee = 30.0f,
    .ratio = 12.0f
    .attack = 0.003f,
    .release = 0.250f
    };

/******************************************************************************************/
 
static void i2s_passthrough(void *args) {
    size_t read_size;
    size_t write_size;
    esp_err_t dsp_err;
    esp_err_t read_ret;
    esp_err_t write_ret;
    int BUF_SIZE = sizeof(l_in);
    int BUF_BYTES = sizeof(rxbuf);

    paramEQ.lp = true;
    paramEQ.lp_freq = 0.1f;

    while (1) {

        // read stereo samples from DMA_in
        read_ret = i2s_read(I2S_NUM, &rxbuf[0], BUF_BYTES, &read_size, portMAX_DELAY);
        if (read_ret == ESP_OK && read_size == BUF_BYTES) {

            // extract stereo samples to mono buffers
            int y = 0;
            for (int i = 0; i < BUF_BYTES; i = i+2) {
                l_in[y] = (float) rxbuf[i];
                r_in[y] = (float) rxbuf[i+1];
                y++;
            }

            // Generate IIR Coefficients for 4th order Butterworth Filters
            if (paramEQ.consumable_update) {
                paramEQ.consumable_update = false; // mutex write
                for (int i = 0; i < 2; i++) {
                    dsps_biquad_gen_hpf_f32(&hp_coeffs[i][0], paramEQ.hp_freq, BUTTER_Q[i]);
                    dsps_biquad_gen_highShelf_f32(&hs_coeffs[i][0], paramEQ.hs_freq, paramEQ.hs_amount, BUTTER_Q[i]);
                    dsps_biquad_gen_notch_f32(&br_coeffs[i][0], paramEQ.br_freq, paramEQ.br_amount, BUTTER_Q[i]);
                    dsps_biquad_gen_lowShelf_f32(&ls_coeffs[i][0], paramEQ.ls_freq, paramEQ.ls_amount, BUTTER_Q[i]);
                    dsps_biquad_gen_lpf_f32(&lp_coeffs[i][0], paramEQ.lp_freq, BUTTER_Q[i]);
                }
            }
            // Generate 
            if (compressor.consumable_update) {
                compressor.consumable_update = false;
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
            memcpy(l_buff_eq, l_in, BUF_SIZE);
            memcpy(r_buff_eq, r_in, BUF_SIZE);
            if (!paramEQ.passthrough) {
                for (int i = 0; i < 2; i++) {
                    if (paramEQ.hp) {
                        memset(w1, 0, sizeof(w1));
                        memset(w2, 0, sizeof(w2));
                        dsp_err = dsps_biquad_f32_ae32(&l_buff_eq[0], &l_buff_eq[0], BUF_SIZE, &hp_coeffs[i][0], &w1[0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_buff_eq[0], &r_buff_eq[0], BUF_SIZE, &hp_coeffs[i][0], &w2[0]);
                    }
                    if (paramEQ.hs) {
                        memset(w1, 0, sizeof(w1));
                        memset(w2, 0, sizeof(w2));
                        dsp_err = dsps_biquad_f32_ae32(&l_buff_eq[0], &l_buff_eq[0], BUF_SIZE, &hs_coeffs[i][0], &w1[0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_buff_eq[0], &r_buff_eq[0], BUF_SIZE, &hs_coeffs[i][0], &w2[0]);
                    }
                    if (paramEQ.br) {
                        memset(w1, 0, sizeof(w1));
                        memset(w2, 0, sizeof(w2));
                        dsp_err = dsps_biquad_f32_ae32(&l_buff_eq[0], &l_buff_eq[0], BUF_SIZE, &br_coeffs[i][0], &w1[0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_buff_eq[0], &r_buff_eq[0], BUF_SIZE, &br_coeffs[i][0], &w2[0]);
                    }
                    if (paramEQ.lp) {
                        memset(w1, 0, sizeof(w1));
                        memset(w2, 0, sizeof(w2));
                        dsp_err = dsps_biquad_f32_ae32(&l_buff_eq[0], &l_buff_eq[0], BUF_SIZE, &lp_coeffs[i][0], &w1[0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_buff_eq[0], &r_buff_eq[0], BUF_SIZE, &lp_coeffs[i][0], &w2[0]);
                    }
                    if (paramEQ.ls) {
                        memset(w1, 0, sizeof(w1));
                        memset(w2, 0, sizeof(w2));
                        dsp_err = dsps_biquad_f32_ae32(&l_buff_eq[0], &l_buff_eq[0], BUF_SIZE, &ls_coeffs[i][0], &w1[0]);
                        dsp_err = dsps_biquad_f32_ae32(&r_buff_eq[0], &r_buff_eq[0], BUF_SIZE, &ls_coeffs[i][0], &w2[0]);
                    }
                }

                // Post EQ Gain Section
                for(int i = 0; i < BUF_SIZE; i++) {
                    l_buff_eq[i] = paramEQ.gain * l_buff_eq[i];
                    r_buff_eq[i] = paramEQ.gain * r_buff_eq[i];
                }

            }


            // CREATE DRC HERE
            memcpy(l_buff_drc, l_buff_eq, BUF_SIZE);
            memcpy(r_buff_drc, r_buff_eq, BUF_SIZE);
            //sf_compressor_process(sf_compressor_state_st *state, BUF_SIZE, &l_buff_eq[0], &l_buff_drc[0]);
            //sf_compressor_process(sf_compressor_state_st *state, BUF_SIZE, &l_buff_eq[0], &l_buff_drc[0]);


            // Post DRC Gain Section
            memcpy(l_out, l_buff_drc, BUF_SIZE);
            memcpy(r_out, r_buff_drc, BUF_SIZE);

            //merge two l and r buffers into a mixed buffer and write back to HW
            y = 0;
            for (int i = 0; i < BUF_SIZE; i++) {
                txbuf[y] = (int16_t) l_out[i];
                txbuf[y+1] = (int16_t) r_out[i];
                y = y+2;
            }
        
            //write 4096 samples to DMA_out
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
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t) I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX,
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB, //equivalent to I2S_COMM_FORMAT_I2S_LSB which handles 16-bit right aligned data
        .tx_desc_auto_clear = true,
        .use_apll = true,
        .dma_buf_count = 8,
        .dma_buf_len = BUFF_LEN,    // # frames per DMA buffer. frame size = # channels * bytes per sample
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num = 3,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO
    };
    //QueueHandle_t evt_que; //use in place of NULL in driver to see if you ever loose data
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);// &evt_que);
    i2s_set_pin(I2S_NUM, &pin_config);
    
    // You can reset parameters by calling 'i2s_set_clk'

    xTaskCreate(i2s_passthrough, "i2s_passthrough", 4096, NULL, 2, NULL);
/*     while (1) {
        i2s_event_t evt;
        xQueueReceive(evt_que, &evt, portMAX_DELAY);
        if (evt.type == I2S_EVENT_RX_Q_OVF) {
            printf("RX data dropped\n");
        }
    } */
}
