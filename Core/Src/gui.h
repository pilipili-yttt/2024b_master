#ifndef INC_2024B_CORE_SRC_GUI_H_
#define INC_2024B_CORE_SRC_GUI_H_
#include <stdint.h>

#include <FreeRTOS.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif
void SPI_Controller_transmitCommand(uint8_t *cmd, uint8_t cmdLength);
void SPI_Controller_transmitData(uint8_t *data, uint8_t dataLength);
void gui_init();

enum GuiEventType { INIT_EVENT, MEASURE_EVENT, BUTTON_EVENT };
struct GuiMeasureEvent {
  int gear;
  float current, voltage, power, pf, thd;
  float harmonics[13];
};
enum ButtonClickType { Long, Short, Double, Continuous, Both, Released };
struct GuiButtonEvent {
  int btn_id;
  enum ButtonClickType click;
};
struct GuiEvent {
  enum GuiEventType type;
  union {
    struct GuiMeasureEvent measure;
    struct GuiButtonEvent button;
  };
};

extern QueueHandle_t guiEventQueue;

#ifdef __cplusplus
}
#endif

#endif // INC_2024B_CORE_SRC_GUI_H_
