//
// Created by Radonyl on 2019/7/20.
//

#ifndef CW3160_GPIO_H
#define CW3160_GPIO_H

#include <cstdint>
#include <ti_msp_dl_config.h>

#define DEFINED_GPIO(name) GPIO(name##_GPIO_Port, name##_Pin)

class GPIO {
private:
  GPIO_Regs *GPIOx;
  uint32_t GPIO_Pin;

public:
  bool Read() const;
  GPIO(GPIO_Regs *port, uint32_t pin) : GPIOx(port), GPIO_Pin(pin) {}
};

#endif // CW3160_GPIO_H
