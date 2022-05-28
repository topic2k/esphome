/*
 * Most of the code in this integration is based on the Adafruit_VL53L1X library
 * by Adafruit Industries, which in turn is based on
 * the VL53L1X API provided by ST (STSW-IMG007).
 *
 * For more information about licensing, please view the included LICENSE.txt file
 * in the vl53l1x integration directory.
 */

#include "vl53l1x_sensor.h"

namespace esphome {
namespace vl53l1x {
uint16_t PAD_I2C_HV_EXTSUP_CONFIG = 0x002E;

std::list<VL53L1XSensor *>
    VL53L1XSensor::vl53l1x_sensors;                     // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
bool VL53L1XSensor::enable_pin_setup_complete = false;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

VL53L1XSensor::VL53L1XSensor() {
  ESP_LOGE(TAG, " - VL53L1XSensor()");
  VL53L1XSensor::vl53l1x_sensors.push_back(this);
}

void VL53L1XSensor::dump_config() {
  LOG_SENSOR("", "VL53L1X", this);
  LOG_UPDATE_INTERVAL(this);
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_PIN("  IRQ Pin: ", this->irq_pin_);
  ESP_LOGCONFIG(TAG, "  Model 0x%X", this->get_sensor_id());
  ESP_LOGCONFIG(TAG, "  io_2v8: %s", this->io_2v8_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  long_range: %s", this->long_range_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  timing_budget: %i", this->timing_budget_);
  ESP_LOGCONFIG(TAG, "  offset: %i", this->offset_);
}

void VL53L1XSensor::setup() {
  if (!esphome::vl53l1x::VL53L1XSensor::enable_pin_setup_complete) {
    for (auto &vl53_sensor : vl53l1x_sensors) {
      if (vl53_sensor->enable_pin_ != nullptr) {
        // Set enable pin as OUTPUT and disable the enable pin to force vl53 to HW Standby mode
        vl53_sensor->enable_pin_->setup();
        vl53_sensor->enable_pin_->pin_mode(gpio::FLAG_OUTPUT);
        vl53_sensor->enable_pin_->digital_write(false);
      }
    }
    esphome::vl53l1x::VL53L1XSensor::enable_pin_setup_complete = true;
  }

  if (this->irq_pin_ != nullptr) {
    this->irq_pin_->setup();
    this->irq_pin_->pin_mode(gpio::FLAG_OUTPUT);
  }

  uint8_t address_to_set = address_;
  if (address_ != this->VL53L1X_DEFAULT_DEVICE_ADDRESS) {
    set_i2c_address(this->VL53L1X_DEFAULT_DEVICE_ADDRESS);
  }

  if (!begin(address_to_set)) {
    ESP_LOGE(TAG, "'%s' - Sensor init failed", this->name_.c_str());
    this->mark_failed();
  }
  if (this->io_2v8_) {
    uint8_t val;
    VL53L1X_Error status;
    status = this->vl53l1x_rd_byte(PAD_I2C_HV_EXTSUP_CONFIG, &val);
    if (status == this->VL53L1X_ERROR_NONE) {
      val = (val & 0xfe) | 0x01;
      this->vl53l1x_wr_byte(PAD_I2C_HV_EXTSUP_CONFIG, val);
    }
  }

  this->set_distance_mode(this->long_range_ ? 2 : 1);
  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  this->vl53l1x_set_timing_budget_in_ms(this->timing_budget_);

  if (this->offset_ != 0) {
    this->vl53l1x_set_offset(this->offset_);
  }

  if (!this->vl53l1x_start_ranging()) {
    ESP_LOGE(TAG, "'%s' - Couldn't start ranging. Error code %i", this->name_.c_str(), this->vl_status);
    this->mark_failed();
  }
}

void VL53L1XSensor::update() {
  int16_t dstnc = 0;

  if (this->is_data_ready()) {
    // new measurement for the taking!
    dstnc = this->get_distance();
    if (dstnc == -1) {
      // something went wrong!
      ESP_LOGE(TAG, "Couldn't get get_distance. Error code %i", this->vl_status);
      return;
    }
    // data is read out, time for another reading!
    this->clear_interrupt();
  }
  publish_state(dstnc / 1000.0);  // convert from mm to m and publish the result
}

// void VL53L1XSensor::loop() {
//  uint16_t range_m = 0;
//  range_m = this->read_range(true);
//  publish_state(range_m);
//}

}  // namespace vl53l1x
}  // namespace esphome
