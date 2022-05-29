#pragma once
/*
 * Most of the code in this integration is based on the Adafruit_VL53L1X library
 * by Adafruit Industries, which in turn is based on
 * the VL53L1X API provided by ST (STSW-IMG007).
 *
 * For more information about licensing, please view the included LICENSE.txt file
 * in the vl53l1x integration directory.
 */

#include <list>
#include "esphome/components/sensor/sensor.h"
#include "Adafruit_VL53L1X.h"

namespace esphome {
namespace vl53l1x {

class VL53L1XComponent : public PollingComponent, public Adafruit_VL53L1X {
 public:
  VL53L1XComponent();

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void update() override;
  void loop() override;

  static std::list<VL53L1XComponent *> vl53l1x_sensors;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
  int ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus);

  // esphome configuration setters
  void set_distance_sensor(sensor::Sensor *sensor_distance) { s_distance_ = sensor_distance; }
  void set_threshold_sensor(sensor::Sensor *sensor_threshold) { s_threshold_ = sensor_threshold; }
  void set_enable_pin(GPIOPin *enable) { this->enable_pin_ = enable; }
  void set_irg_pin(GPIOPin *irq) { this->irq_pin_ = irq; }
  void set_io_2v8(bool io_2v8) { this->io_2v8_ = io_2v8; }
  void set_long_range(bool long_range) { this->long_range_ = long_range; }
  void set_timing_budget(uint16_t timing_budget) { this->timing_budget_ = timing_budget; }
  void set_offset(int16_t offset) { this->offset_ = offset; }
  void set_threshold(uint16_t thresh_low, uint16_t thresh_high, uint8_t thresh_mode) {
    threshold_low_ = thresh_low;
    threshold_high_ = thresh_high;
    threshold_mode_ = thresh_mode;
  }

 protected:
  static bool enable_pin_setup_complete;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

  /* esphome configuration variables */
  sensor::Sensor *s_distance_{nullptr};
  sensor::Sensor *s_threshold_{nullptr};

  bool io_2v8_ = false;
  bool long_range_ = true;
  uint16_t timing_budget_ = 100;
  int16_t offset_ = 0;
  uint16_t threshold_low_;
  uint16_t threshold_high_;
  uint8_t threshold_mode_;
};

}  // namespace vl53l1x
}  // namespace esphome
