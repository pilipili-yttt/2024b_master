#include "gui.h"
#include "printf.h"
#include <FreeRTOS.h>
#include <SEGGER_RTT.h>
#include <task.h>
#include <ti_msp_dl_config.h>
#include <u8g2.h>

#define SET_RESET(x)                                                           \
  do {                                                                         \
    if (x) {                                                                   \
      DL_GPIO_setPins(GPIO_GRP_0_LCD_RST_PORT, GPIO_GRP_0_LCD_RST_PIN);        \
    } else {                                                                   \
      DL_GPIO_clearPins(GPIO_GRP_0_LCD_RST_PORT, GPIO_GRP_0_LCD_RST_PIN);      \
    }                                                                          \
  } while (0)

#define SET_CS(x)                                                              \
  do {                                                                         \
    if (x) {                                                                   \
      DL_GPIO_setPins(GPIO_GRP_0_LCD_CS_PORT, GPIO_GRP_0_LCD_CS_PIN);          \
    } else {                                                                   \
      DL_GPIO_clearPins(GPIO_GRP_0_LCD_CS_PORT, GPIO_GRP_0_LCD_CS_PIN);        \
      ;                                                                        \
    }                                                                          \
  } while (0)

static u8g2_t u8g2;
static bool dc_pin_state = false;
static TaskHandle_t pending_task;
static uint32_t remaining;
static uint8_t *spi_tx_buffer;

QueueHandle_t guiEventQueue;

void SpiTransmit(uint8_t *buf, uint8_t len, bool is_cmd) {
  pending_task = xTaskGetCurrentTaskHandle();
  while (DL_SPI_isBusy(SPI_0_INST))
    ;

  if (is_cmd)
    DL_SPI_setControllerCommandDataModeConfig(SPI_0_INST, len);

  remaining = len;
  spi_tx_buffer = buf;
  DL_SPI_enableInterrupt(SPI_0_INST, DL_SPI_INTERRUPT_TX_EMPTY);

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void SPI_Controller_transmitCommand(uint8_t *cmd, uint8_t cmdLength) {
  /* SPI must be idle prior to setting a new value of CDMODE */
  while (DL_SPI_isBusy(SPI_0_INST))
    ;

  DL_SPI_setControllerCommandDataModeConfig(SPI_0_INST, cmdLength);

  int i = 0;
  for (i = 0; i < cmdLength; i++) {
    while (DL_SPI_isBusy(SPI_0_INST))
      ;
    DL_SPI_transmitData8(SPI_0_INST, cmd[i]);
  }
}

void SPI_Controller_transmitData(uint8_t *data, uint8_t dataLength) {
  int i = 0;
  for (i = 0; i < dataLength; i++) {
    while (DL_SPI_isBusy(SPI_0_INST))
      ;
    DL_SPI_transmitData8(SPI_0_INST, data[i]);
  }
}

void SPI_0_INST_IRQHandler(void) {
  __asm__("nop");
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  switch (DL_SPI_getPendingInterrupt(SPI_0_INST)) {
  case DL_SPI_IIDX_IDLE:
    // unblock task
    if (pending_task) {
      vTaskNotifyGiveFromISR(pending_task, &pxHigherPriorityTaskWoken);
      pending_task = NULL;
    }
    DL_SPI_disableInterrupt(SPI_0_INST, DL_SPI_INTERRUPT_IDLE);
    break;
  case DL_SPI_IIDX_TX_EMPTY:
    while (remaining) {
      if (!DL_SPI_isTXFIFOFull(SPI_0_INST)) {
        DL_SPI_transmitData8(SPI_0_INST, *spi_tx_buffer);
        spi_tx_buffer++;
        remaining--;
      }
    }
    if (remaining == 0) {
      DL_SPI_disableInterrupt(SPI_0_INST, DL_SPI_INTERRUPT_TX_EMPTY);
      DL_SPI_enableInterrupt(SPI_0_INST, DL_SPI_INTERRUPT_IDLE);
    }
    break;
  default:
    break;
  }

  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
                                  U8X8_UNUSED uint8_t msg,
                                  U8X8_UNUSED uint8_t arg_int,
                                  U8X8_UNUSED void *arg_ptr) {
  switch (msg) {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    break;
  case U8X8_MSG_DELAY_MILLI:
    vTaskDelay(arg_int);
    break;
  case U8X8_MSG_GPIO_CS:
    SET_CS(arg_int);
    break;
  case U8X8_MSG_GPIO_DC:
    //    OLED_SET_DC(arg_int);
    dc_pin_state = arg_int;
    break;
  case U8X8_MSG_GPIO_RESET:
    SET_RESET(arg_int);
    break;
  }
  return 1;
}

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                               void *arg_ptr) {
  switch (msg) {
  case U8X8_MSG_BYTE_SEND:
    SpiTransmit((uint8_t *)arg_ptr, arg_int, dc_pin_state == 0);
    //    if (dc_pin_state == 0) { // DC=0 for command (active low)
    //      SPI_Controller_transmitCommand((uint8_t *)arg_ptr, arg_int);
    //    } else {
    //      SPI_Controller_transmitData((uint8_t *)arg_ptr, arg_int);
    //    }
    break;
  case U8X8_MSG_BYTE_INIT:
    break;
  case U8X8_MSG_BYTE_SET_DC:
    //    OLED_SET_DC(arg_int);
    dc_pin_state = arg_int;
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
    SET_CS(0);
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
    SET_CS(1);
    break;
  default:
    return 0;
  }
  return 1;
}

static char buf[100];
static uint8_t gear_ratios[] = {1, 3, 7, 15, 31};
static uint8_t coil_turns = 1;
const static float coil_calibration[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

static int gui_style = 0;

static void gui_task(void *_) {
  while (1) {
    struct GuiEvent event;
    xQueueReceive(guiEventQueue, &event, portMAX_DELAY);

    switch (event.type) {
    case INIT_EVENT:
      u8g2_DrawFrame(&u8g2, 0, 0, 320, 240);
      u8g2_DrawStr(&u8g2, 20, 110, "INIT_PLEASE_WAIT");
      break;
    case BUTTON_EVENT:
      if (event.button.btn_id == 0) {
        if (event.button.click == Short) {
          if (coil_turns < sizeof(coil_calibration) / sizeof(float)) {
            coil_turns++;
          }
        } else if (event.button.click == Long) {
          gui_style++;
          gui_style %= 2;
          u8g2_ClearDisplay(&u8g2);
        }
      } else if (event.button.btn_id == 1) {
        if (event.button.click == Short) {
          if (coil_turns > 1) {
            coil_turns--;
          }
        } else if (event.button.click == Long) {
        }
      }
      break;
    case MEASURE_EVENT:
      u8g2_SetFontMode(&u8g2, 0);
      uint16_t v_sep;
      if (gui_style == 0) {
        u8g2_SetFont(&u8g2, u8g2_font_inb30_mr);
        v_sep = 36;
      } else {
        u8g2_SetFont(&u8g2, u8g2_font_profont22_mr);
        v_sep = 18;
      }

      uint16_t v_pos = v_sep;
      const uint16_t h_start = 5;

      snprintf(buf, sizeof(buf), "U=%7.2f V", event.measure.voltage);
      if (gui_style == 0) u8g2_DrawStr(&u8g2, h_start, v_pos, buf);
      v_pos += v_sep;

      event.measure.current /= (float)coil_calibration[coil_turns - 1];

      if (event.measure.current < 0.9f) {
        snprintf(buf, sizeof(buf), "I=%7.2fmA", event.measure.current * 1000);
      } else {
        snprintf(buf, sizeof(buf), "I=%7.2f A", event.measure.current);
      }
      if (gui_style == 0) u8g2_DrawStr(&u8g2, h_start, v_pos, buf);
      v_pos += v_sep;

      event.measure.power /= (float)coil_calibration[coil_turns - 1];
      if (event.measure.power < 0)
        event.measure.power = 0;
      if (event.measure.power < 0.9f) {
        snprintf(buf, sizeof(buf), "P=%7.2fmW", event.measure.power * 1000);
      } else {
        snprintf(buf, sizeof(buf), "P=%7.2f W", event.measure.power);
      }
      if (gui_style == 0) u8g2_DrawStr(&u8g2, h_start, v_pos, buf);
      v_pos += v_sep;

      if (isfinite(event.measure.pf)) {
        if (event.measure.pf > 1) {
          event.measure.pf = 1;
        }
        if (event.measure.pf < 0) {
          event.measure.pf = 0;
        }
        snprintf(buf, sizeof(buf), "PF =%7.3f", event.measure.pf);
      } else {
        snprintf(buf, sizeof(buf), "PF =INVALID");
      }
      if (gui_style == 0) u8g2_DrawStr(&u8g2, h_start, v_pos, buf);

      v_pos += v_sep;

      snprintf(buf, sizeof(buf), "N=%2d", coil_turns);
      u8g2_DrawStr(&u8g2, 180, v_pos, buf);
      v_pos += v_sep;
        snprintf(buf, sizeof(buf), "G%d", event.measure.gear);
        u8g2_DrawStr(&u8g2, 180, v_pos, buf);
      if (gui_style == 1) {
        v_pos += v_sep;

        snprintf(buf, sizeof(buf), "THD=%6.1f%%", event.measure.thd * 100);
        u8g2_DrawStr(&u8g2, 180, v_pos, buf);
        for (int i = 0; i < 13; i++) {
          snprintf(buf, sizeof(buf), "H%02d=%7.1fmA", i + 1,
                   event.measure.harmonics[i] * 1000 /
                       (float)coil_calibration[coil_turns - 1]);
          u8g2_DrawStr(&u8g2, 5, (i + 1) * v_sep, buf);
        }

      }

      break;
    }
    u8g2_UpdateDisplay(&u8g2);
  }
}


void gui_init() {
  guiEventQueue = xQueueCreate(3, sizeof(struct GuiEvent));

  // enable SPI interrupt
  NVIC_ClearPendingIRQ(SPI_0_INST_INT_IRQN);
  NVIC_EnableIRQ(SPI_0_INST_INT_IRQN);
  NVIC_SetPriority(SPI_0_INST_INT_IRQN, 3);

  SET_CS(1);
  SET_RESET(1);
  vTaskDelay(100);

  //  u8g2_ssd1322_n
  u8g2_Setup_st75320_jlx320240_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi,
                                 u8x8_stm32_gpio_and_delay);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_SetContrast(&u8g2, 81); // 70~90, tune this value based on screen

  xTaskCreate(gui_task, "gui", 256, NULL, tskIDLE_PRIORITY, NULL);
}
