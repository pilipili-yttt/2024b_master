#include "gpio.h"
#include "ti_msp_dl_config.h"

bool GPIO::Read() const {
  return !!DL_GPIO_readPins(this->GPIOx, this->GPIO_Pin);
}
