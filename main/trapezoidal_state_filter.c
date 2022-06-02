/**
 * @file trapezoidal_state_filter.c
 * @author August Byrne (august.byrne@gmail.com)
 * @brief C recreation of state space trapozoidal filter from the technical paper "Simultaneous
 * solving of all outputs of Linear SVF using trapezoidal integration in state space form"
 * © Andrew Simper, Cytomic, 2021, andy@cytomic.com
 * @version 0.1
 * @date 2022-06-01
 * 
 */

#include <math.h>


void dsp_trap_state_gen_f32(float *coeffs, float f, float qFactor, int type) { //float qFactor) {
    float w = M_PI * f;
    float g0 = tan(w);
    float k0 = 1 / qFactor;// damp = 1/Q;
    float gaindb = -30.0f;
    float A = log10f(gaindb * 0.25f);//10^(gaindb/40);
    float g = 0.0f;
    float k = 0.0f;
    float m0 = 0.0f;
    float m1 = 0.0f;
    float m2 = 0.0f;
    
    if (type == 0) {
        //high:
        g = g0;
        k = k0;
        m0 = 1;
        m1 = 0;
        m2 = 0;
    } else if (type == 1) {
        //band:
        g = g0;
        k = k0;
        m0 = 0;
        m1 = 1;
        m2 = 0;
    } else if (type == 2) {
        //low:
        g = g0;
        k = k0;
        m0 = 0;
        m1 = 0;
        m2 = 1;
    } else if (type == 3) {
        //notch:
        g = g0;
        k = k0;
        m0 = 1;
        m1 = 0;
        m2 = 1;
    } else if (type == 4) {
        //peak:
        g = g0;
        k = k0;
        m0 = 1;
        m1 = 0;
        m2 = -1;
    } else if (type == 5) {
        //all:
        g = g0;
        k = k0;
        m0 = 1;
        m1 = -k0;
        m2 = 1;
    } else if (type == 6) {
        //bell:
        g = g0;
        k = k0/A;
        m0 = 1;
        m1 = k0*A;
        m2 = 1;
    } else if (type == 7) {
        //low shelf:
        g = g0/sqrt(A);
        k = k0;
        m0 = 1;
        m1 = k0*A;
        m2 = A*A;
    } else if (type == 8) {
        //high shelf:
        g = g0*sqrt(A);
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
// for parallel version also:
//gt1 = g*gt0;
//gk1 = g*gk0;
//gt2 = g*gt1;
//clear:
//ic1eq = 0;
//ic2eq = 0;
//tick parallel (possibly quicker on cpus with large pipelines):
/* t0 = vin - ic2eq
v0 = gt0*t0 - gk0*ic1eq
t1 = gt1*t0 - gk1*ic1eq
t2 = gt2*t0 + gt1*ic1eq
v1 = ic1eq + t1
v2 = ic2eq + t2
// state update either
ic1eq = ic1eq + 2*t1 // better precision
ic2eq = ic2eq + 2*t2 // better precision
// or
ic1eq = v1 + t1 // lower operation count
ic2eq = v2 + t2 // lower operation count */
//tick serial (possibly quicker on cpus with low latencies):


/**
 * @brief KEY
high = v0
band = v1
low = v2
notch = v0 + v2
peak = v0 - v2
all = v0 - k*v1 + v2
 */

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
        //ic1eq = v1 + t1; // lower operation count
        //ic2eq = v2 + t2; // lower operation count
        //parallel core 8* 8+ = 16 ops total
        //serial core 4* 6+ = 10 ops total
        output[i] = coef[3] * v0 + coef[4] * v1 + coef[5] * v2; // output = m0*v0 + m1*v1 + m2*v2;
    }
}
