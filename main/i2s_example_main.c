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
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
 

#define AUDIO_SAMPLE_RATE   48000         //48kHz
//#define BUFF_LEN            256           //buffer length
#define I2S_NUM             I2S_NUM_0     //i2s port number
#define I2S_BCK_IO          GPIO_NUM_15   //i2s bit clock
#define I2S_WS_IO           GPIO_NUM_13   //i2s word select
#define I2S_DO_IO           GPIO_NUM_2    //i2s data out
#define I2S_DI_IO           GPIO_NUM_0    //i2s data in

// Audio Buffer Variables
uint16_t rxbuf[4096], txbuf[4096];    // # of bytes = sizebuf_len * (bits_per_sample / 8) * buf_count * num_of_channels; 1024 works best so far
float l_in[2048], r_in[2048];
/* float l_buff_hp_eq[256], r_buff_hp_eq[256], l_buff_bp_eq[256], r_buff_bp_eq[256], l_buff_lp_eq[256], r_buff_lp_eq[256];
float l_temp_buff[256], r_temp_buff[256];
float l_buff_eq_out[256], r_buff_eq_out[256];
float l_buff_drc[256], r_buff_drc[256]; */
float l_out[2048], r_out[2048];

// EQ Variables
float lowpass_coeffs[6][5];
float w1[2], w2[2];
float l_lp_buf[2048], r_lp_buf[2048];
float l_lp_out[2048], r_lp_out[2048];

void normalize_float_array(const float *input, float *output, float scale, int len);

/* //float hp_freq;
float current_hp_freq;
//float bp_freq;
float current_bp_freq;
//float lp_freq;
float current_lp_freq;
float hp_coeffs[20][5];
float bp_coeffs[20][5];
float lp_coeffs[20][5]; */

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

static const float SHORT_BUTTER_Q[6] = {  // Biquad Q values for cascading to obtain a 12th order butterworth response
    0.50431448,
    0.54119610,
    0.63023621,
    0.82133982,
    1.3065630,
    3.8306488};

/******************************************************************************************/
 /* 
static void i2s_passthrough(void *args) {
    size_t read_size = 0;   //measured in bytes (i think)
    size_t write_size = 0;

    float hp_freq = 0.25f;
    float bp_freq = 0.25f;
    float lp_freq = 0.25f;

        float hp_amount = 0;
        float bp_amount = 0;
        float lp_amount = 1;

    while (1) {
        //read 256 samples (256 stereo samples) from DMA_in
        esp_err_t read_ret = i2s_read(I2S_NUM, &rxbuf[0], 512*2, &read_size, portMAX_DELAY); //portMAX_DELAY for no timeout
        if (read_ret == ESP_OK && read_size == 512*2) {
            int y = 0;
            //extract stereo samples to mono buffers
            for (int i=0; i<512; i=i+2) {
                l_in[y] = (float) rxbuf[i];
                r_in[y] = (float) rxbuf[i+1];
                y++;
            }
            printf("\n\ni2s read success, %d bytes read.\n", read_size);
            printf("\ndata read: %d %d %d %d\n---------------------------------------------------\n", (int) &rxbuf[0], (int) &rxbuf[1], (int) &rxbuf[2], (int) &rxbuf[3]);

            // Generate IIR Coefficients for 20th order Butterworth
            if (hp_freq != current_hp_freq) {
                current_hp_freq = hp_freq;
                for (int i = 0; i<20; i++) {
                    dsps_biquad_gen_hpf_f32(&hp_coeffs[i][0], current_hp_freq, BUTTER_Q[i]);
                }
            }
            if (bp_freq != current_bp_freq) {
                current_bp_freq = bp_freq;
                for (int i = 0; i<20; i++) {
                    dsps_biquad_gen_bpf_f32(&bp_coeffs[i][0], current_bp_freq, BUTTER_Q[i]);
                }
            }
            if (lp_freq != current_lp_freq) {
                current_lp_freq = lp_freq;
                for (int i = 0; i<20; i++) {
                    dsps_biquad_gen_lpf_f32(&lp_coeffs[i][0], current_lp_freq, BUTTER_Q[i]);
                }
            }

            // HPF
            printf("HP Section -> ");
            esp_err_t dsp_err;
            for(int i=0;i<256;i++) {
                l_temp_buff[i] = l_in[i];
                r_temp_buff[i] = r_in[i];
            }
            for (int i=0;i<20;i++) {
                float w[2] = {0};
                dsp_err = dsps_biquad_f32_ae32(&r_temp_buff[0], &l_buff_hp_eq[0], BUFF_LEN, &hp_coeffs[i][0], &w[0]);
                dsp_err = dsps_biquad_f32_ae32(&r_temp_buff[0], &r_buff_hp_eq[0], BUFF_LEN, &hp_coeffs[i][0], &w[0]);
                if (i != 19) {
                    for(int i=0;i<256;i++) {
                        l_temp_buff[i] = l_buff_hp_eq[i];
                        r_temp_buff[i] = r_buff_hp_eq[i];
                    }
                }
            }

            // BPF
            printf("BP Section -> ");
            for(int i=0;i<256;i++) {
                l_temp_buff[i] = l_in[i];
                r_temp_buff[i] = r_in[i];
            }
            for (int i=0;i<20;i++) {
                float w[2] = {0};
                dsp_err = dsps_biquad_f32_ae32(&l_temp_buff[0], &l_buff_bp_eq[0], BUFF_LEN, &bp_coeffs[i][0], &w[0]);
                dsp_err = dsps_biquad_f32_ae32(&r_temp_buff[0], &r_buff_bp_eq[0], BUFF_LEN, &bp_coeffs[i][0], &w[0]);
                if (i != 19) {
                    for(int i=0;i<256;i++) {
                        l_temp_buff[i] = l_buff_bp_eq[i];
                        r_temp_buff[i] = r_buff_bp_eq[i];
                    }
                }
            }

            // LPF
            printf("LP Section -> ");
            for(int i=0;i<256;i++) {
                l_temp_buff[i] = l_in[i];
                r_temp_buff[i] = r_in[i];
            }
            for (int i=0;i<20;i++) {
                float w[2] = {0};
                dsp_err = dsps_biquad_f32_ae32(&l_temp_buff[0], &l_buff_lp_eq[0], BUFF_LEN, &lp_coeffs[i][0], &w[0]);
                dsp_err = dsps_biquad_f32_ae32(&r_temp_buff[0], &r_buff_lp_eq[0], BUFF_LEN, &lp_coeffs[i][0], &w[0]);
                if (i != 19) {
                    for(int i=0;i<256;i++) {
                        l_temp_buff[i] = l_buff_lp_eq[i];
                        r_temp_buff[i] = r_buff_lp_eq[i];
                    }
                }
            }


            // Mix filter amount with input for buff_eq_out & subtractive volume control
            printf("Mixing & Volume Section ==> ");
            for(int i=0;i<256;i++) {
                l_buff_eq_out[i] =
                (((1/(1+hp_amount+bp_amount+lp_amount))*l_in[i]) + 
                ((hp_amount/(1+hp_amount+bp_amount+lp_amount))*l_buff_hp_eq[i]) + 
                ((bp_amount/(1+hp_amount+bp_amount+lp_amount))*l_buff_bp_eq[i]) +
                ((lp_amount/(1+hp_amount+bp_amount+lp_amount))*l_buff_lp_eq[i]));

                r_buff_eq_out[i] =
                (((1/(1+hp_amount+bp_amount+lp_amount))*r_in[i]) + 
                ((hp_amount/(1+hp_amount+bp_amount+lp_amount))*r_buff_hp_eq[i]) + 
                ((bp_amount/(1+hp_amount+bp_amount+lp_amount))*r_buff_bp_eq[i]) +
                ((lp_amount/(1+hp_amount+bp_amount+lp_amount))*r_buff_lp_eq[i]));
            }


            // CREATE DRC HERE




            // Create Volume Control Here Instead


            //merge the left and right buffers into a mixed buffer and write back to HW
            y = 0;
            printf("Output Reconstruction\n");
            for(int i=0;i<256;i++) {
                txbuf[y] = (int) l_buff_eq_out[i];//l_out[i];
                txbuf[y+1] = (int) r_buff_eq_out[i];//r_out[i];
                y = y + 2;
            }
        
            //write 512 samples to DMA_out
            //temp writing rxbuf out for a fully transparent passthrough
            esp_err_t write_ret = i2s_write(I2S_NUM, &rxbuf[0], 512*2, &write_size, 400);
            if (write_ret == ESP_OK && write_size == 512*2) { //portMAX_DELAY
                printf("i2s write success, %d bytes written.\n", write_size);
            } else {
                printf("i2s write failed. write status %d, written bytes %d\n", write_ret, write_size);
            }
        } else {
            printf("i2s read failed. read status %d, read bytes %d\n", read_ret, read_size);
        }
    }
}
 */
static void app_test(void *args) {
    esp_err_t rxfb;
    esp_err_t dsp_err;
    size_t write_size;
    size_t read_size;
    int BUF_BYTES = sizeof(rxbuf);
    printf("Number is: %d", BUF_BYTES);
    int LP_BUF_SIZE = sizeof(l_lp_buf);
    for (int i = 0; i<6; i++) {
        dsps_biquad_gen_lpf_f32(&lowpass_coeffs[i][0], 0.25f, SHORT_BUTTER_Q[i]);
    }
    while (1) {
        //read 4096 samples (2048 stereo samples)
        rxfb = i2s_read(I2S_NUM, &rxbuf[0], BUF_BYTES, &read_size, portMAX_DELAY);
        if (rxfb == ESP_OK && read_size == BUF_BYTES) {
            //printf("i2s read success, %d bytes read.\n", read_size);
            
            //extract stereo samples to mono buffers
            int y=0;
            for (int i=0; i<4096; i=i+2) {
                l_in[y] = (float) rxbuf[i];
                r_in[y] = (float) rxbuf[i+1];
                printf("val{%u uint}{%u float_ish}\n", rxbuf[i], (uint16_t) l_in[y]);
                y++;
            }
            // LPF
            normalize_float_array(&l_in[0], &l_lp_buf[0], 65536.0f, sizeof(l_lp_buf));
            normalize_float_array(&r_in[0], &r_lp_buf[0], 65536.0f, sizeof(r_lp_buf));
            for (int i=0;i<6;i++) {
                memset(w1, 0, sizeof(w1));
                memset(w2, 0, sizeof(w2));
                dsp_err = dsps_biquad_f32_ae32(&l_lp_buf[0], &l_lp_out[0], 2048, &lowpass_coeffs[i][0], &w1[0]);
                dsp_err = dsps_biquad_f32_ae32(&r_lp_buf[0], &r_lp_out[0], 2048, &lowpass_coeffs[i][0], &w2[0]);
                memcpy(l_lp_buf, l_lp_out, LP_BUF_SIZE);
                memcpy(r_lp_buf, r_lp_out, LP_BUF_SIZE);
                normalize_float_array(&l_lp_buf[0], &l_lp_buf[0], 65536.0f, sizeof(l_lp_buf));
                normalize_float_array(&r_lp_buf[0], &r_lp_buf[0], 65536.0f, sizeof(r_lp_buf));
            }
            memset(w1, 0, sizeof(w1));
            memset(w2, 0, sizeof(w2));
            dsp_err = dsps_biquad_f32_ae32(&l_lp_buf[0], &l_lp_out[0], 2048, &lowpass_coeffs[5][0], &w1[0]);
            dsp_err = dsps_biquad_f32_ae32(&r_lp_buf[0], &r_lp_out[0], 2048, &lowpass_coeffs[5][0], &w2[0]);
            normalize_float_array(&l_lp_out[0], &l_lp_out[0], 65536.0f, sizeof(l_lp_out));
            normalize_float_array(&r_lp_out[0], &r_lp_out[0], 65536.0f, sizeof(r_lp_out));

            //merge two l and r buffers into a mixed buffer and write back to HW
            y=0;
            for (int i=0;i<2048;i++) {
                txbuf[y] = (uint16_t) l_lp_out[i];
                txbuf[y+1] = (uint16_t) r_lp_out[i];
                y=y+2;
            }
            rxfb = i2s_write(I2S_NUM, &txbuf[0], BUF_BYTES, &write_size, portMAX_DELAY);
            if (rxfb == ESP_OK && write_size == BUF_BYTES) {
                //printf("i2s write success, %d bytes written.\n", write_size);
            } else {
                printf("failed to write: error %x. bytes written: %d\n", rxfb, write_size);
            }
        } else {
            printf("failed to read: error %x. bytes read: %d\n", rxfb, read_size);
        }
    }
}


void normalize_float_array(const float *input, float *output, float scale, int len) {
    /* enforce the contract */
    assert(input && output && scale && len);
    size_t i;
    float maxValue = input[0];

    // find max value of array
    for (i = 1; i < len; ++i) {
        if (input[i] > maxValue) {
            maxValue = input[i];
            printf("max val candidate: %u ", (uint16_t) maxValue);
        }
    }
    printf("max value was: %u\n", (uint16_t) maxValue);
    for (i = 0; i < len; i++) {
        output[i] = input[i] * (scale / maxValue);
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
        .dma_buf_len = 1024,    // # frames per DMA buffer. frame size = # channels * bytes per sample
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

    xTaskCreate(app_test, "i2s_passthrough", 4096, NULL, 4, NULL);
/*     while (1) {
        i2s_event_t evt;
        xQueueReceive(evt_que, &evt, portMAX_DELAY);
        if (evt.type == I2S_EVENT_RX_Q_OVF) {
            printf("RX data dropped\n");
        }
    } */
}
