/*!
 * @file Adafruit_VL53L1X.cpp
 *
 * @mainpage Adafruit VL53L1X time-of-flight sensor
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's VL53L1X driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit VL53L1X breakout: https://www.adafruit.com/product/3967
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_VL53L1X.h"

namespace esphome {
namespace vl53l1x {

/**************************************************************************/
/*!
    @brief  Create a new VL53L1X instance
    @param  shutdown_pin Optional specify pin attached to shutdown
    @param irq_pin Optional specify pin attached to interrupt
*/
/**************************************************************************/
Adafruit_VL53L1X::Adafruit_VL53L1X() {}

/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware
    @param  i2c_addr Optional I2C address the sensor can be found on. Default is
   0x29
    @returns  True if device is set up, false on any failure
*/
/**************************************************************************/
bool Adafruit_VL53L1X::begin(uint8_t i2c_addr) {
  if (enable_pin_ != nullptr) {
    enable_pin_->digital_write(true);
    enable_pin_->digital_write(false);
    delay(5);
    enable_pin_->digital_write(true);
  }
  delay(5);

  vl_status = init_sensor(i2c_addr);
  if (vl_status != VL53L1X_ERROR_NONE) {
    return false;
  }

  if (get_sensor_id() != 0xEACC) {
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Get the sensor ID.
    @returns The sensor ID.
*/
/**************************************************************************/
uint16_t Adafruit_VL53L1X::get_sensor_id(void) {
  uint16_t sensorID = 0;
  vl_status = vl53l1x_get_sensor_id(&sensorID);
  return sensorID;
}

/**************************************************************************/
/*!
    @brief  Get the get_distance.
    @returns The get_distance.
*/
/**************************************************************************/
int16_t Adafruit_VL53L1X::get_distance(void) {
  uint16_t distance;

  vl_status = vl53l1x_get_distance(&distance);
  if (vl_status != VL53L1X_ERROR_NONE) {
    return -1;
  }
  return (int16_t) distance;
}

/**************************************************************************/
/*!
    @brief  Clear the interrupt.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::clear_interrupt(void) {
  vl_status = vl53l1x_clear_interrupt();
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Set the interrupt polarity.
    @param polarity The polarity to set as a boolean.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::set_int_polarity(bool polarity) {
  vl_status = vl53l1x_set_interrupt_polarity(polarity);
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Get the interrupt polarity.
    @returns Polarity as a boolean.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::get_int_polarity(void) {
  uint8_t x = 0;
  vl_status = vl53l1x_get_interrupt_polarity(&x);
  return (bool) x;
}

/**************************************************************************/
/*!
    @brief  Start ranging operations.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::start_ranging(void) {
  vl_status = vl53l1x_start_ranging();
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Stop ranging operations.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::stop_ranging(void) {
  vl_status = vl53l1x_stop_ranging();
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Check status of new data.
    @returns True if new data available, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::is_data_ready(void) {
  uint8_t x = 0;
  vl_status = vl53l1x_check_for_data_ready(&x);
  return (bool) x;
}

/**************************************************************************/
/*!
    @brief  Set the timing budget.
    @param ms Timing budget in milliseconds.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::set_timing_budget_ms(uint16_t ms) {
  vl_status = vl53l1x_set_timing_budget_in_ms(ms);
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Get the timing budget.
    @returns Timing budget in milliseconds.
*/
/**************************************************************************/
uint16_t Adafruit_VL53L1X::get_timing_budget(void) {
  uint16_t ms = 0;

  vl_status = vl53l1x_get_timing_budget_in_ms(&ms);
  if (vl_status == VL53L1X_ERROR_NONE) {
    return ms;
  }
  return 0;
}

bool Adafruit_VL53L1X::set_distance_mode(uint16_t mode) {
  VL53L1X_Error status;
  status = this->vl53l1x_set_distance_mode(mode);
  return (status == VL53L1X::VL53L1X_ERROR_NONE);
}

/*
boolean
Adafruit_VL53L1X::GetRangingMeasurementData(VL53L1_RangingMeasurementData_t
*ranging) { Status = VL53L1_GetRangingMeasurementData(pMyDevice, ranging);
  return (Status == VL53L1_ERROR_NONE);
}
*/

}  // namespace vl53l1x
}  // namespace esphome
