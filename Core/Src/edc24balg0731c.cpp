#include "edc24balg0731c.h"

#include "arm_math.h"

// internal states, do not modify
int16_t fftoutputbuf[2 * adcpagelen];
float wfvavgbuf[avgnum];
float wfiavgbuf[avgnum];
float rpwravgbuf[avgnum];
float pfactavgbuf[avgnum];
int32_t fftbinout[fftbinnum];
float fftbinoutconvavgbuf[avgnum * fftbinnum];
float fftbinoutconvavg[fftbinnum];
uint16_t avgbufindex = 0;
uint16_t avgbufcount = 0;
arm_rfft_instance_q15 fftinst;
int32_t autorangingcooldowntickcounter = 0;

//global states
float fullscaleunipolarvoltage = (250.0f * 1.4142f);	//to be calibrated
float fullscaleunipolarcurrent = (6.0f * 1.4142f);	//1-turn, to be calibrated and looked-up for different PGA gains and possibly current transformer turns

int algstat_initialize() {
  //  if (arm_rfft_init_q15(&fftinst, adcpagelen, 0, 1) != ARM_MATH_SUCCESS)
  //    return -1;
  //  return 0;

  //   fixed to 512 points
  if (arm_rfft_init_512_q15(&fftinst, 0, 1) != ARM_MATH_SUCCESS)
    return -1;
  return 0;
}

/*
 * wfvquan (in): voltage waveform data (q15 signed) after Vcom cancellation
 * wfiquan (in): current waveform data (q15 signed) after Vcom cancellation
 * currenttransformerturns (in): number of wired turns for the current
 * transformer acrmsv (out): voltage acrms measurement result acrmsi (out):
 * current acrms measurement result realpwr (out): real power measurement result
 * pwrfact (out): power factor measurement result
 * harmonicrms[fftbinnum] (out): harmonic acrms measurement results
 */
void algstat_tick(int16_t *wfvquan, int16_t *wfiquan,
                  int16_t currenttransformerturns, float *acrmsv, float *acrmsi,
                  float *realpwr, float *pwrfact, float *harmonicrms) {
  if (adcbits < 16) {
    arm_shift_q15(wfvquan, 16 - adcbits, wfvquan, adcpagelen);
    arm_shift_q15(wfiquan, 16 - adcbits, wfiquan, adcpagelen);
  }

  // acrms
  int16_t acrmsvi = 0, acrmsii = 0;
  arm_std_q15(wfvquan, adcpagelen, &acrmsvi);
  arm_std_q15(wfiquan, adcpagelen, &acrmsii);
  // pwr
  int16_t meanv = 0, meani = 0;
  arm_mean_q15(wfvquan, adcpagelen, &meanv);
  arm_mean_q15(wfiquan, adcpagelen, &meani);
  arm_offset_q15(wfvquan, -meanv, wfvquan, adcpagelen);
  arm_offset_q15(wfiquan, -meani, wfiquan, adcpagelen);
  int64_t dotprod;
  arm_dot_prod_q15(wfvquan, wfiquan, adcpagelen, &dotprod);
  // fft
  arm_rfft_q15(&fftinst, wfiquan, fftoutputbuf);
  for (int bini = 0; bini < fftbinnum; bini++) {
    fftbinout[bini] = 0;
  }
  // binning with extended precision
  for (int i = 0; i < adcpagelen; i++) {
    int32_t re = fftoutputbuf[i * 2];
    int32_t im = fftoutputbuf[i * 2 + 1];
    int32_t magsqr = re * re + im * im;
    ((int32_t *)fftoutputbuf)[i] = magsqr;
    int32_t fftbinindex = (i - fftbinlen / 2) / fftbinlen;
    int32_t binpos = ((i - fftbinlen / 2) % fftbinlen) - (fftbinlen / 2);
    if (binpos < 0)
      binpos = -binpos;
    if (binpos <= fftbinpassbandlen / 2 && fftbinindex >= 0 &&
        fftbinindex < fftbinnum) {
      fftbinout[fftbinindex] += magsqr;
    }
  }
  // conversions
  float acrmsvconv = (float)acrmsvi / (1 << (16 - adcbits)) *
                     fullscaleunipolarvoltage / (1 << (adcbits - 1));
  float acrmsiconv = (float)acrmsii / (1 << (16 - adcbits)) *
                     (fullscaleunipolarcurrent / currenttransformerturns) /
                     (1 << (adcbits - 1));
  float realpwrconv = (float)dotprod / adcpagelen /
                      ((1 << (16 - adcbits)) * (1 << (16 - adcbits))) /
                      (1 << (adcbits - 1)) * fullscaleunipolarvoltage /
                      (1 << (adcbits - 1)) *
                      (fullscaleunipolarcurrent / currenttransformerturns);
  float pwrfactconv = realpwrconv / (acrmsvconv * acrmsiconv);
  wfvavgbuf[avgbufindex] = acrmsvconv;
  wfiavgbuf[avgbufindex] = acrmsiconv;
  rpwravgbuf[avgbufindex] = realpwrconv;
  pfactavgbuf[avgbufindex] = pwrfactconv;
  for (int bini = 0; bini < fftbinnum; bini++) {
    float scaled = (1 << (adcbits - 12)) / (sqrtf(6400.0f / adcsampleratesps)) *
                   sqrtf((float)fftbinout[bini] / (adcpagelen / fftbinlen)) /
                   (1 << (adcbits - 1)) *
                   (fullscaleunipolarcurrent / currenttransformerturns);
    fftbinoutconvavgbuf[avgbufindex * fftbinnum + bini] = scaled;
  }
  avgbufindex = (avgbufindex + 1) % avgnum;
  if (avgbufcount < avgnum)
    avgbufcount++;
  float acrmsvavg = 0.0f, acrmsiavg = 0.0f, realpwravg = 0.0f,
        pwrfactavg = 0.0f;
  for (int bini = 0; bini < fftbinnum; bini++) {
    fftbinoutconvavg[bini] = 0.0f;
  }
  for (int i = 0; i < avgbufcount; i++) {
    acrmsvavg += wfvavgbuf[i];
    acrmsiavg += wfiavgbuf[i];
    realpwravg += rpwravgbuf[i];
    pwrfactavg += pfactavgbuf[i];
    for (int bini = 0; bini < fftbinnum; bini++) {
      fftbinoutconvavg[bini] += fftbinoutconvavgbuf[i * fftbinnum + bini];
    }
  }
  acrmsvavg /= avgbufcount;
  acrmsiavg /= avgbufcount;
  realpwravg /= avgbufcount;
  pwrfactavg /= avgbufcount;
  for (int bini = 0; bini < fftbinnum; bini++) {
    fftbinoutconvavg[bini] /= avgbufcount;
  }
  *acrmsv = acrmsvavg;
  *acrmsi = acrmsiavg;
  *realpwr = realpwravg;
  *pwrfact = pwrfactavg;
  for (int bini = 0; bini < fftbinnum; bini++) {
    harmonicrms[bini] = fftbinoutconvavg[bini];
  }
}

/* Calculate THD from harmonic Irms results
 * return value: THD (>0.0f and usually <1.0f)
 */
float algstat_postprocthd(float *harmonicrms) {
  float totalnonfundamentalrms = 0.0f;
  for (int i = 1; i < fftbinnum; i++) {
    totalnonfundamentalrms += harmonicrms[i] * harmonicrms[i];
  }
  return sqrtf(totalnonfundamentalrms) / harmonicrms[0];
}

/* Test for autoranging conditions, must called before calls to algstat_tick
 * wfiquan (in): voltage waveform data (q15 signed) after Vcom cancellation
 * return value: 1 for overrange, -1 for underrange, 0 for normal operation
 */
int algstat_autorangingtest(int16_t *wfiquan) {
  const int16_t overrangethreshold = (int16_t)((1 << (adcbits - 1)) * 0.9f);
  const int16_t underrangethreshold = (int16_t)((1 << adcbits) * 0.15f);
  const int16_t overrangesamplesthreshold = (int16_t)adcpagelen * 0.02f + 1;
  int16_t overrangesamples = 0;
  int16_t minval = wfiquan[0], maxval = wfiquan[0];

  for (int i = 0; i < adcpagelen; i++) {
    if (wfiquan[i] > overrangethreshold || wfiquan[i] < -overrangethreshold) {
      overrangesamples++;
    }
    if (wfiquan[i] > maxval)
      maxval = wfiquan[i];
    if (wfiquan[i] < minval)
      minval = wfiquan[i];
  }
  if (autorangingcooldowntickcounter > 0) {
    autorangingcooldowntickcounter--;
  }
  if (overrangesamples > overrangesamplesthreshold &&
      autorangingcooldowntickcounter == 0) {
    // overrange condition detected
    autorangingcooldowntickcounter = autorangingcooldownticks;
    return 1;
  } else if (maxval - minval < underrangethreshold &&
             autorangingcooldowntickcounter == 0) {
    // underrange condition detected
    autorangingcooldowntickcounter = autorangingcooldownticks;
    return -1;
  }
  return 0;
}

void algstat_resetavgbuf() {
  avgbufindex = 0;
  avgbufcount = 0;
}