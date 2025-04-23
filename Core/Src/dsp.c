#include "edc24balg0731c.h"
#include "gui.h"
#include "printf.h"
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <SEGGER_RTT.h>
#include <string.h>
#include <task.h>

#define ADC_SAMPLE_COUNT (512)
#define ADC_CHANNEL_COUNT (3)
static uint8_t channel;
static uint32_t count;
static TaskHandle_t dsp_task;
static uint8_t adc_buffer_index;

/* Calibration Start*/
float voltage_correction = 2.273595f;
float current_correction[5] = {0.892224f, 0.2977f, 0.127565f,
                               0.0600225f, 0.02909075f,}; // 5 gears
/* Calibration End */

#define RECORD_RAW 0

const static uint32_t sample_max = 32767;
const static uint32_t range_upper = sample_max * 95 / 100 / 2;
const static uint32_t range_lower = sample_max * 25 / 100 / 2;
static size_t up_count = 0;
static size_t down_count = 0;

static int16_t v_samples[ADC_SAMPLE_COUNT * 2];
static int16_t i_samples[ADC_SAMPLE_COUNT * 2];
#if RECORD_RAW
static uint16_t raw_samples[3][ADC_SAMPLE_COUNT * 2];
#endif
static int16_t vcom = 0;

const static uint8_t gears[] = {1, 3, 7, 15, 31};
const static uint32_t pga_gains[] = {DL_OPA_GAIN_N1_P2, DL_OPA_GAIN_N3_P4,
                                     DL_OPA_GAIN_N7_P8, DL_OPA_GAIN_N15_P16,
                                     DL_OPA_GAIN_N31_P32};
static bool gear_locked = false;
static bool gear_changed = false;
static int cur_gear = 0;
const static uint8_t gear_stabilize_cycles = 5;
static uint8_t gear_stabilize_countdown = 0;

static void apply_gear() {
  gear_stabilize_countdown = gear_stabilize_cycles;
  SEGGER_RTT_printf(0, "gear %d\n", gears[cur_gear]);
  DL_OPA_setGain(OPA_0_INST, pga_gains[cur_gear]);
  gear_changed = true;
}

// return val: if geared, true; if already top, false
static void gear_up() {
  if (cur_gear < sizeof(gears) - 1) {
    cur_gear++;
    apply_gear();
  }
}

static void gear_down() {
  if (cur_gear > 0) {
    cur_gear--;
    apply_gear();
  }
}

static void manual_gear(int gear) {
  portENTER_CRITICAL();
  gear_locked = true;
  cur_gear = gear;
  apply_gear();
  portEXIT_CRITICAL();
}

static void auto_gear() { gear_locked = false; }

static void cycle_end(uint8_t buf_idx) {
  portENTER_CRITICAL();
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  if (gear_stabilize_countdown > 0) {
    gear_stabilize_countdown--;
  } else {
    if (!gear_locked) {
      if (up_count > 5) {
        gear_down();
        //        SEGGER_RTT_printf(0, "up %d\n", up_count);
      } else if (down_count < 5) {
        gear_up();
        //        SEGGER_RTT_printf(0, "down %d\n", down_count);
      }
    }
  }
  up_count = down_count = 0;
  portEXIT_CRITICAL();

  if (gear_stabilize_countdown == 0) {
    adc_buffer_index = buf_idx;
    vTaskNotifyGiveFromISR(dsp_task, &pxHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

static void AdcPutSample(uint16_t val) {
  if (channel == 0) {
    vcom = (int16_t)val;
  } else if (channel == 1) {
    v_samples[count] = val - vcom;
  } else if (channel == 2) {
    i_samples[count] = -(val - vcom);

    int up_val = abs((int)val - (int)(sample_max / 2));
    if (up_val > range_upper) {
      up_count++;
    }
    int down_val = abs(val - vcom);
    if (down_val > range_lower) {
      down_count++;
    }
  }
#if RECORD_RAW
  raw_samples[channel][count] = val;
#endif
  channel++;

  if (channel >= ADC_CHANNEL_COUNT) {
    channel = 0;
    count++;
    if (count == ADC_SAMPLE_COUNT) {
      cycle_end(0);
    }
    if (count >= ADC_SAMPLE_COUNT * 2) {
      count = 0;
      cycle_end(1);
    }
  }
}

void ADC12_0_INST_IRQHandler(void) {
  switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
  case DL_ADC12_IIDX_MEM11_RESULT_LOADED:
    for (int i = 0; i < 6; i++) {
      uint32_t data = DL_ADC12_getFIFOData(ADC12_0_INST);
      AdcPutSample(data & 0xffff);
      AdcPutSample(data >> 16);
    }
    break;
  default:
    break;
  }
}

static void AdcStart() {
  DL_TimerG_startCounter(TIMER_0_INST);

  DL_ADC12_enableInterrupt(ADC12_0_INST,
                           DL_ADC12_INTERRUPT_MEM11_RESULT_LOADED);

  /* Setup interrupts on device */
  NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
  NVIC_SetPriority(ADC12_0_INST_INT_IRQN, 3);

  DL_ADC12_startConversion(ADC12_0_INST);
}

void DspTask(void *argument) {
  SEGGER_RTT_printf(0, "dsp\n");
  dsp_task = xTaskGetCurrentTaskHandle();
  algstat_initialize();
  AdcStart();
  uint8_t gear_at_sample_time;
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    gear_at_sample_time = cur_gear;
    if (gear_changed) {
      gear_changed = false;
      algstat_resetavgbuf();
    }

    float acrmsv;
    float acrmsi;
    float realpwr;
    float pwrfact;
    float harmonicrms[fftbinnum];

    algstat_tick(&v_samples[adc_buffer_index * ADC_SAMPLE_COUNT],
                 &i_samples[adc_buffer_index * ADC_SAMPLE_COUNT], 1, &acrmsv,
                 &acrmsi, &realpwr, &pwrfact, harmonicrms);

    float thd = algstat_postprocthd(harmonicrms);
    char buf[50];

    snprintf(buf, sizeof(buf), "NO_CAL G%d U=%.2f I=%.2f P=%.2f \n", cur_gear,
             acrmsv, acrmsi * 1000, realpwr);
    SEGGER_RTT_Write(0, buf, strlen(buf));

    float cur_cor = current_correction[gear_at_sample_time];
    acrmsv *= voltage_correction;
    acrmsi *= cur_cor;
    realpwr *= voltage_correction * cur_cor;

    for (int i = 0; i < fftbinnum; i++) {
      harmonicrms[i] *= cur_cor;
    }

    snprintf(buf, sizeof(buf), "CAL G%d %.2fV %.2fmA %.2fW %.1f%% %.1f%%\n\n",
             cur_gear, acrmsv, acrmsi * 1000, realpwr, pwrfact * 100,
             thd * 100);
    SEGGER_RTT_Write(0, buf, strlen(buf));

    struct GuiEvent evt;
    evt.type = MEASURE_EVENT;
    evt.measure.gear = cur_gear;
    evt.measure.voltage = acrmsv;
    evt.measure.current = acrmsi;
    evt.measure.power = realpwr;
    evt.measure.pf = pwrfact;
    evt.measure.thd = thd;

    memcpy(evt.measure.harmonics, harmonicrms, sizeof(evt.measure.harmonics));

    if (uxQueueMessagesWaiting(guiEventQueue) == 0) {
      xQueueSend(guiEventQueue, &evt, 0);
    }
  }
}

void DspStart() {
  BaseType_t ret =
      xTaskCreate(DspTask, "dsp", 512, NULL, tskIDLE_PRIORITY, NULL);
}