/**
 * @file trapezoidal_state_filter.c
 * @author August Byrne (august.byrne@gmail.com)
 * @brief C recreation of state space trapozoidal filter from the technical paper "Simultaneous
 * solving of all outputs of Linear SVF using trapezoidal integration in state space form", and
 * from "Linear Trapezoidal State Variable Filter (SVF) in state increment form: state += val"
 * by © Andrew Simper, Cytomic, 2014-2021, andy@cytomic.com
 * @version 0.2
 * @date 2022-06-02
 * 
 */

#include <math.h>


void dsp_trap_state_gen_f32(float *coeffs, float f, float qFactor, int type) {
    float w = M_PI * f;
    float g0 = tanf(w);
    float k0 = 1 / qFactor;// damp = 1/Q; where Q is the qFactor (like biquads)
    //float gaindb = -5.0f;
    //now using exp10f(gaindb*0.05) with 0.05 for 90dB scale at 16 bit audio
    float A = 0.5623413f;//exp10f(gaindb * 0.05f);// originally 10^(gaindb/40);
    float g = 0.0f;
    float k = 0.0f;
    float m0 = 0.0f;
    float m1 = 0.0f;
    float m2 = 0.0f;
    
    if (type == 0) {        //high:
        g = g0;
        k = k0;
        m0 = 1;
        m1 = 0;
        m2 = 0;
    } else if (type == 1) { //band:
        g = g0;
        k = k0;
        m0 = 0;
        m1 = 1;
        m2 = 0;
    } else if (type == 2) { //low:
        g = g0;
        k = k0;
        m0 = 0;
        m1 = 0;
        m2 = 1;
    } else if (type == 3) { //notch:
        g = g0;
        k = k0;
        m0 = 1;
        m1 = 0;
        m2 = 1;
    } else if (type == 4) { //peak:
        g = g0;
        k = k0;
        m0 = 1;
        m1 = 0;
        m2 = -1;
    } else if (type == 5) { //all:
        g = g0;
        k = k0;
        m0 = 1;
        m1 = -k0;
        m2 = 1;
    } else if (type == 6) { //bell:
        g = g0;
        k = k0/A;
        m0 = 1;
        m1 = k0*A;
        m2 = 1;
    } else if (type == 7) { //low shelf:
        g = g0/0.7498942f;//sqrt(A);
        k = k0;
        m0 = 1;
        m1 = k0*A;
        m2 = A*A;
    } else if (type == 8) { //high shelf:
        g = g0*0.7498942f;//sqrt(A);
        k = k0;
        m0 = A*A;
        m1 = k0*A;
        m2 = 1;
    } else {
        //printf("ERROR in TRAPAZOIDS");
    }
    //shared:
    float gk = g + k;
    float gt0 = 1 / (1 + g * gk);
    float gk0 = gk * gt0;

    //save parameters
    coeffs[0] = g;
    coeffs[1] = gt0;
    coeffs[2] = gk0;
    coeffs[3] = m0;
    coeffs[4] = m1;
    coeffs[5] = m2;
}

void dsp_trap_state_filt_f32_ansi(const float *input, float *output, int len, float *coef, float *w) {
    for (int i = 0 ; i < len ; i++) {
        float t0 = input[i] - w[1]; //vin - ic2eq;
        float v0 = coef[1] * t0 - coef[2] * w[0]; //gt0*t0 - gk0*ic1eq;
        float t1 = coef[0] * v0; //g*v0;
        float v1 = w[0] + t1; //ic1eq + t1;
        float t2 = coef[0] * v1; //g*v1;
        float v2 = w[1] + t2; //ic2eq + t2;
        // state update either
        w[0] = w[0] + 2 * t1; // better precision
        w[1] = w[1] + 2 * t2; // better precision
        // or
        //w[0] = v1 + t1; // lower operation count
        //w[1] = v2 + t2; // lower operation count
        output[i] = coef[3] * v0 + coef[4] * v1 + coef[5] * v2; // output = m0*v0 + m1*v1 + m2*v2;
    }
}


// on the fly state variable filter. No coefficiants pre-made.
// Sine in place of tangent allows for more stability and accuracy.
void dsp_otf_state_filt_f32_sine(const float *input, float *output, int len, int filtType, float f, float qFactor, float *w) {
    
    float w_val = M_PI * f;
    float k = 1 / qFactor; // damp = 1/Q; where Q is the qFactor (like biquads)

    float s1 = sinf(w_val);
    float s2 = sinf(2*w_val);
    float nrm = 1/(2 + k*s2);
    float g0 = (s2)*nrm;
    float g1 = (-2*s1*s1 - k*s2)*nrm;
    float g2 = (2*s1*s1)*nrm;

    float v0, v1, v2, t0, t1, t2;
    
    for (int i = 0 ; i < len ; i++) {
        v0 = input[i];
        t0 = v0 - w[1];
        t1 = g0*t0 + g1*w[0];
        t2 = g2*t0 + g0*w[0];
        v1 = t1 + w[0];
        v2 = t2 + w[1];
        w[0] = w[0] + 2.0f*t1;
        w[1] = w[1] + 2.0f*t2;
        if (filtType == 0) {                // highpass filter
            output[i] = v0 - k*v1 - v2;
        } else if (filtType == 1) {         // bandpass filter
            output[i] = v1;
        } else if (filtType == 2) {         // lowpass filter
            output[i] = v2;
        } else if (filtType == 3) {         // notch filter
            output[i] = v0 - k*v1;          // = high + low
        } else if (filtType == 4) {         // peak filter
            output[i] = v0 - k*v1 - 2*v2;   // = high - low
        }
    }
}