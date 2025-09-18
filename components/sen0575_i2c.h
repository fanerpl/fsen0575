#pragma once

#include "esphome.h"
#include "Wire.h"
#include "DFRobot_RainfallSensor.h"

class SEN0575_I2C : public Component {
 public:
  // Sensory do ESPHome
  Sensor *rain_raw = new Sensor();
  Sensor *rain_1h = new Sensor();
  Sensor *rain_24h = new Sensor();
  Sensor *rain_total = new Sensor();

  DFRobot_RainfallSensor_I2C sensor = DFRobot_RainfallSensor_I2C(&Wire);

  // Buffory do 1h i 24h (zakładamy update co 1 minutę)
  std::deque<float> rain_last_1h;
  std::deque<float> rain_last_24h;
  float accumulated_rain = 0.0;

  void setup() override {
    Wire.begin();
    if (!sensor.begin()) {
      ESP_LOGE("sen0575_i2c", "Sensor init error!");
    } else {
      ESP_LOGD("sen0575_i2c", "Sensor VID: %X, PID: %X, FW: %s", sensor.vid, sensor.pid, sensor.getFirmwareVersion().c_str());
    }
  }

  void loop() override {
    // Odczyt z sensora
    float rain = sensor.getRainfall();  // mm od ostatniego odczytu
    float rain_raw_count = sensor.getRawData(); // liczba przechylnych kubełków (opcjonalnie)

    // Aktualizacja całkowitych opadów
    accumulated_rain += rain;
    rain_raw->publish_state(rain);
    rain_total->publish_state(accumulated_rain);

    // 1h i 24h (przybliżenie: jeden odczyt co minutę)
    rain_last_1h.push_back(rain);
    rain_last_24h.push_back(rain);

    if (rain_last_1h.size() > 60) rain_last_1h.pop_front();        // 1h = 60 minut
    if (rain_last_24h.size() > 1440) rain_last_24h.pop_front();    // 24h = 1440 minut

    float sum_1h = 0;
    for (auto &r : rain_last_1h) sum_1h += r;
    rain_1h->publish_state(sum_1h);

    float sum_24h = 0;
    for (auto &r : rain_last_24h) sum_24h += r;
    rain_24h->publish_state(sum_24h);

    delay(60000);  // loop co minutę
  }

  void reset_total() {
    accumulated_rain = 0.0;
    rain_total->publish_state(0);
  }
};
