#include "button.h"
#include "SEGGER_RTT.h"
#include "gpio.h"
#include "gui.h"

namespace button {
struct Button {
  GPIO Port;
  bool CurrentStatus = false;
  int TransitionTime = 0;

  int PressTime = 0;
  bool PendingSingleClick = false;
  int Interval = 0;

  bool isPressed() const { return Port.Read(); }

  Button(GPIO &&gpio) : Port(gpio) {}
};

static Button Btns[NumberOfButtons] = {
    GPIO(GPIO_GRP_0_KEY_LEFT_PORT, GPIO_GRP_0_KEY_LEFT_PIN),
    GPIO(GPIO_GRP_0_KEY_RIGHT_PORT, GPIO_GRP_0_KEY_RIGHT_PIN)};

volatile DetectionMode_t DetectionMode[NumberOfButtons];

const static int DebounceCycles = 5;
const static int LongPressThreshold = 500;
const static int DoubleClickThreshold = 50;
const static int BothPressThreshold = 10;
const static int ContinuousPressInitialDuration = 250;
const static int ContinuousPressInterval = 20;

static void HandleEvent(GuiButtonEvent info) {
  GuiEvent evt;
  evt.type = BUTTON_EVENT;
  evt.button = info;
  volatile BaseType_t t = xQueueSend(guiEventQueue, &evt, 0);
  SEGGER_RTT_printf(0, "btn send %d (id %d, t %d)\n", t,info.btn_id,info.click);
  asm("nop");
}

bool IsPressed(int id) { return Btns[id].isPressed(); }

void Routine() {
  int pressedCount = 0;
  for (int i = 0; i < NumberOfButtons; i++) {
    Button &btn = Btns[i];
    bool pressed = btn.isPressed();
    if (btn.CurrentStatus != pressed) {
      btn.TransitionTime++;
      if (btn.TransitionTime > DebounceCycles) {
        btn.CurrentStatus = pressed;
        btn.TransitionTime = 0;
      }
    }
    if (btn.CurrentStatus)
      pressedCount++;
  }

  for (int i = 0; i < NumberOfButtons; i++) {
    Button &btn = Btns[i];
    if (btn.CurrentStatus) {
      if (btn.PressTime != -1)
        btn.PressTime++;

      btn.Interval = 0;
      if (pressedCount <= 1) // Long press
      {
        if (DetectionMode[i] & RespondContinuousPress) {
          if (btn.PressTime > ContinuousPressInitialDuration &&
              (btn.PressTime - ContinuousPressInitialDuration) %
                      ContinuousPressInterval ==
                  0) {
            HandleEvent({i, Continuous});
          }
        } else {
          if (btn.PressTime > LongPressThreshold) {
            btn.PressTime = -1;
            HandleEvent({i, Long});
          }
        }
      }
    } else {
      // Check if it's a short click
      if (btn.PressTime > 0 &&
          btn.PressTime < (DetectionMode[i] & RespondContinuousPress
                               ? ContinuousPressInitialDuration
                               : LongPressThreshold)) {
        if (DetectionMode[i] & DetectDoubleClick) {
          if (!btn.PendingSingleClick) {
            btn.PendingSingleClick = true;
          } else {
            btn.PendingSingleClick = false;
            HandleEvent({i, Double});
          }
        } else {
          HandleEvent({i, Short});
        }
      } else if (btn.PressTime == 0) {
        if (btn.PendingSingleClick) {
          btn.Interval++;
          if (btn.Interval > DoubleClickThreshold) {
            btn.PendingSingleClick = false;
            HandleEvent({i, Short});
          }
        }
      } else if ((DetectionMode[i] & RespondContinuousPress) &&
                 btn.PressTime > ContinuousPressInitialDuration) {
        HandleEvent({i, Released});
      }
      btn.PressTime = 0;
    }
  }
}
} // namespace button

void ButtonRoutine() { button::Routine(); }