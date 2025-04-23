#ifndef INC_2024B_CORE_SRC_EDC24BALG0731C_H_
#define INC_2024B_CORE_SRC_EDC24BALG0731C_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// constants
const uint32_t adcpagelen = 512;	//ADC DMA page length (number of int16_t elements) per channel
const uint32_t adcbits = 15;	//number of effective ADC bits
const uint32_t adcsampleratesps = 6400;	//must be 50*2^N, N>=5
const uint32_t fftbinlen = 50 * adcpagelen / adcsampleratesps;	//ensure fftbinlen>=4, do not modify
const uint32_t fftbinpassbandlen = fftbinlen;	//length of the pass band (samples) during fft binning, decrease to improve noise immunity
const uint32_t fftbinnum = 20;	//number of measured harmonics
const uint32_t avgnum = 20;	//length of the moving average window of the measurement results
const uint32_t autorangingcooldownticks = 5;	//number of ticks for the autoranging function to wait before next autoranging condition

int algstat_initialize();
void algstat_tick(int16_t *wfvquan, int16_t *wfiquan,
                  int16_t currenttransformerturns, float *acrmsv, float *acrmsi,
                  float *realpwr, float *pwrfact, float *harmonicrms);

float algstat_postprocthd(float* harmonicrms);

int algstat_autorangingtest(int16_t *wfiquan);

void algstat_resetavgbuf();

#ifdef __cplusplus
}
#endif
#endif // INC_2024B_CORE_SRC_EDC24BALG0731C_H_