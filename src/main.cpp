#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Espfc.h>
#include <Kalman.h>
#include <Madgwick.h>
#include <Mahony.h>
#include <printf.h>
#include <blackbox/blackbox.h>
#include <EscDriver.h>
#if defined(ESP8266)
#include <EspWire.h>
#endif
#include <Gps.hpp>
#if defined(ESPFC_ESPNOW)
#include <EspNowRcLink/Receiver.h>
#endif
#ifdef ESPFC_WIFI_ALT
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif
#include "Debug_Espfc.h"

#ifdef ESP32
void IRAM_ATTR serialEventRun(void) {}

RTC_DATA_ATTR static bool g_powerOnRestartDone = false;

static void normalizePowerOnUsbState()
{
#if defined(ESP32S3)
  if(esp_reset_reason() == ESP_RST_POWERON && !g_powerOnRestartDone)
  {
    g_powerOnRestartDone = true;
    ESP.restart();
  }
#endif
}
#endif

Espfc::Espfc espfc;

#if defined(ESPFC_MULTI_CORE)
  #if defined(ESPFC_FREE_RTOS)

    // ESP32 multicore
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <driver/timer.h>

    TaskHandle_t gyroTaskHandle = NULL;
    TaskHandle_t pidTaskHandle = NULL;
    static const timer_group_t TIMER_GROUP = TIMER_GROUP_0;
    static const timer_idx_t TIMER_IDX = TIMER_0;

    bool IRAM_ATTR gyroTimerIsr(void* args)
    {
      BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(gyroTaskHandle, &xHigherPriorityTaskWoken);
      return xHigherPriorityTaskWoken == pdTRUE;
    }

    void gyroTimerInit(bool (*isrCb)(void* args), int interval)
    {
      timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80,
      };
      timer_init(TIMER_GROUP, TIMER_IDX, &config);
      timer_set_counter_value(TIMER_GROUP, TIMER_IDX, 0);
      timer_set_alarm_value(TIMER_GROUP, TIMER_IDX, interval);
      timer_isr_callback_add(TIMER_GROUP, TIMER_IDX, isrCb, nullptr, ESP_INTR_FLAG_IRAM);
      timer_enable_intr(TIMER_GROUP, TIMER_IDX);
      timer_start(TIMER_GROUP, TIMER_IDX);
    }

    void gyroTask(void *pvParameters)
    {
      espfc.begin();
      gyroTimerInit(gyroTimerIsr, espfc.getGyroInterval());
      while(true)
      {
        // Use timeout as fallback: waits for timer ISR, or wakes every 2ms if gyro not detected
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
        espfc.update(true);
      }
    }

    void pidTask(void *pvParameters)
    {
      while(true)
      {
        espfc.updateOther();
      }
    }

    void setup()
    {
      disableCore0WDT();
      // internal task priorities
      // PRO(0): hi-res timer(22), timer(1), event-loop(20), lwip(18/any), wifi(23), wpa(2/any), BT/vhci(23), NimBle(21), BT/other(19,20,22), Eth(15), Mqtt(5/any)
      // APP(1): free
      normalizePowerOnUsbState();
      espfc.load();
      xTaskCreateUniversal(gyroTask, "gyroTask", 8192, NULL, 24, &gyroTaskHandle, 1);
      xTaskCreateUniversal(pidTask,  "pidTask",  8192, NULL,  1, &pidTaskHandle,  0);
      vTaskDelete(NULL); // delete arduino loop task
    }

    void loop()
    {
    }

  #elif defined(ESPFC_MULTI_CORE_RP2040)

    bool core1_separate_stack = true;
    volatile bool setup_done = false;

    // RP2040 multicore
    // TODO: https://emalliab.wordpress.com/2021/04/18/raspberry-pi-pico-arduino-core-and-timers/
    void setup()
    {
      espfc.load();
      espfc.begin();
      setup_done = true;
    }
    void loop()
    {
      espfc.update();
    }
    void setup1()
    {
#if defined(ESP32)
      while(!setup_done);
#endif
    }
    void loop1()
    {
      espfc.updateOther();
    }

  #else
    #error "No RTOS defined for multicore board"
  #endif

#else

  // single core
  void setup()
  {
#if defined(ESP32)
    normalizePowerOnUsbState();
#endif
    espfc.load();
    espfc.begin();
  }
  void loop()
  {
    espfc.update();
    espfc.updateOther();
  }

#endif

#if defined(ESPFC_NATIVE_RUN_MAIN) && !defined(PIO_UNIT_TESTING)
int main()
{
  setup();
  return 0;
}
#endif
