#include <cmsis_compiler.h>
#include <cmsis_os2.h>
#include <main.h>
#include <ti_msp_dl_config.h>

#include "dsp.h"
#include "gui.h"
#include <FreeRTOS.h>
#include <SEGGER_RTT.h>
#include <task.h>
#include "button.h"

void _putchar(char character) { ; } // empty function, use SEGGER RTT to debug

uint32_t SystemCoreClock;

void StartDefaultTask(void *argument) {
  SEGGER_RTT_Init();
  SEGGER_RTT_printf(0, "BOOT! %s\n", __TIME__);
  gui_init();
  DspStart();
  int cnt = 0;
  for (;;) {
    vTaskDelay(2);
    cnt++;
    if (cnt == 500) {
      cnt = 0;
      SEGGER_RTT_printf(0, "%d\n", xPortGetFreeHeapSize());
      DL_GPIO_togglePins(GPIO_GRP_0_LED_B16_PORT, GPIO_GRP_0_LED_B16_PIN);
    }
    ButtonRoutine();
  }
  /* USER CODE END 5 */
}

int main(void) {
  SYSCFG_DL_init();
  SystemCoreClock = CPUCLK_FREQ;

  osKernelInitialize();
  xTaskCreate(StartDefaultTask, "default", 256, NULL, tskIDLE_PRIORITY, NULL);
  osKernelStart();
  while (1) {
  }
}

void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}