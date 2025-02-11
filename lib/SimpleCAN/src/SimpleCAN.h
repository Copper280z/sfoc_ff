#if defined(ARDUINO_ARCH_ESP32)
#include "esp/twai/CAN.h"
#elif defined(HAL_CAN_MODULE_ENABLED)
#include "stm/can/CAN.h"
#elif defined(STM32G4xx) || defined(HAL_FDCAN_MODULE_ENABLED)
#include "stm/fdcan/CAN.h"
#elif defined(ARDUINO_ARCH_GD32)
#include "gd/can2b/CAN.h"
#else
#error "No CAN module is enabled, expecting a define for ARDUINO_ARCH_ESP32 | HAL_CAN_MODULE_ENABLED | HAL_FDCAN_MODULE_ENABLED"
#endif