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

#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "Adafruit_VL53L1X.h"

namespace esphome {
namespace vl53l1x {

static const char *const TAG = "vl53l1x";

class VL53L1XSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:

  VL53L1XSensor();
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void update() override;
  //  void loop() override;

  static std::list<VL53L1XSensor *> vl53l1x_sensors;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)


  // esphome configuration setters
  void set_enable_pin(GPIOPin *enable) { this->enable_pin_ = enable; }
  void set_irg_pin(GPIOPin *irq) { this->irq_pin_ = irq; }
  void set_io_2v8(bool io_2v8) { this->io_2v8_ = io_2v8; }
  void set_long_range(bool long_range) { this->long_range_ = long_range; }
  void set_timing_budget(uint16_t timing_budget) { this->timing_budget_ = timing_budget; }
  void set_offset(int16_t offset) { this->offset_ = offset; }

 protected:
  static bool enable_pin_setup_complete;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

  /* esphome configuration variables */
  GPIOPin *enable_pin_{nullptr};
  GPIOPin *irq_pin_{nullptr};
  bool io_2v8_ = false;
  bool long_range_ = true;
  uint16_t timing_budget_ = 100;
  int16_t offset_ = 0;
  Adafruit_VL53L1X tof_device_ = Adafruit_VL53L1X(nullptr, nullptr, nullptr);
};

}  // namespace vl53l1x
}  // namespace esphome
