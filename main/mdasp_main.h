#ifndef MDASP_MAIN__H
#define MDASP_MAIN__H

#include <stdbool.h>
#include <stdint.h>

#define AUDIO_SAMPLE_RATE   48000               // 48kHz
#define MINUS_90_DB         0.00003162277f      // -90dB definition
#define PLUS_90_DB          31622.77f           // +90dB definition
#define msDelayMax          400                 // max delay on top the the intrinsic delay of BUF_LEN (1ms)
#define BUF_LEN             48                  // buffer length
#define BUF_LEN_STEREO      BUF_LEN*2           // buffer length * 2 channels (stereo)
#define BUF_BYTES           BUF_LEN_STEREO*2    // stereo buffer size * 16-bit (2 bytes)
#define I2S_NUM             I2S_NUM_0           // i2s port number
#define I2S_MCK_IO          GPIO_NUM_0          // i2s master clock
#define I2S_BCK_IO          GPIO_NUM_17         // i2s bit clock
#define I2S_WS_IO           GPIO_NUM_16         // i2s word select
#define I2S_DO_IO           GPIO_NUM_18         // i2s data out
#define I2S_DI_IO           GPIO_NUM_19         // i2s data in


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

typedef struct AdvancedCompressor {
    bool passthrough;
    bool makeupgain;
    float pregain;
    float threshold;
    float knee;
    float ratio;
    float attack;
    float release;
    float predelay;
	float releasezone1;
	float releasezone2;
    float releasezone3;
	float releasezone4;
	float postgain;
	float wet;
} AdvancedCompressor;

typedef struct AudioModel {
    struct ParametricEQ eq;
    struct AdvancedCompressor compressor;
    float volume;
    uint32_t delay;
} AudioModel;

/* Global Variables */
extern uint32_t msDelayModel;   // 0 to 400 millisecond delay
extern bool delayUpdate;
extern float volumeModel;       // 0dB to -90dB
extern bool volumeUpdate;
extern ParametricEQ paramEQModel;
extern bool paramEqUpdate;
extern AdvancedCompressor compressorModel;
extern bool drcUpdate;

#endif
