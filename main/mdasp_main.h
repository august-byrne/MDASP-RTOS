#define AUDIO_SAMPLE_RATE   48000           // 48kHz
#define MINUS_90_DB         0.00003162277f  // -90dB constant
#define BUF_LEN             256             // buffer length
#define BUF_LEN_STEREO     BUF_LEN*2        // buffer length * 2 channels (stereo)
#define BUF_BYTES    BUF_LEN_STEREO*2       // stereo buffer size * 16-bit (2 bytes)
#define I2S_NUM             I2S_NUM_0       // i2s port number
#define I2S_MCK_IO          GPIO_NUM_0      // i2s master clock
#define I2S_BCK_IO          GPIO_NUM_17     // i2s bit clock
#define I2S_WS_IO           GPIO_NUM_16     // i2s word select
#define I2S_DO_IO           GPIO_NUM_18      // i2s data out
#define I2S_DI_IO           GPIO_NUM_19      // i2s data in


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

/* Global Variables */

extern ParametricEQ paramEQModel = {
    .passthrough = false,
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
    .hs_amount = 1.0f,
    .br_amount = 1.0f,
    .ls_amount = 1.0f
    };

extern bool paramEqUpdate = true;  //make into mutex

extern AdvancedCompressor compressorModel = {
    .passthrough = false,
    .pregain = 0.0f,
    .threshold = -24.0f,
    .knee = 30.0f,
    .ratio = 12.0f,
    .attack = 0.003f,
    .release = 0.250f,
    .predelay = 0.006f,
	.releasezone1 = 0.090f,
	.releasezone2 = 0.160f,
    .releasezone3 = 0.420f,
	.releasezone4 = 0.980f,
	.postgain = 0.000f,
	.wet = 1.000f
    };

extern bool drcUpdate = true;  //make into mutex
