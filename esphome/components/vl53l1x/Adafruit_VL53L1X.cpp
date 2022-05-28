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
  if (_shutdown_pin != nullptr) {
    _shutdown_pin->pin_mode(gpio::FLAG_OUTPUT);
    _shutdown_pin->digital_write(HIGH);
    _shutdown_pin->digital_write(LOW);
    delay(5);
    _shutdown_pin->digital_write(HIGH);
  }
  delay(5);

  vl_status = InitSensor(i2c_addr);
  if (vl_status != VL53L1X_ERROR_NONE) {
    return false;
  }

  if (sensorID() != 0xEACC) {
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
uint16_t Adafruit_VL53L1X::sensorID(void) {
  uint16_t sensorID = 0;
  vl_status = VL53L1X_GetSensorId(&sensorID);
  return sensorID;
}

/**************************************************************************/
/*!
    @brief  Get the distance.
    @returns The distance.
*/
/**************************************************************************/
int16_t Adafruit_VL53L1X::distance(void) {
  uint16_t distance;

  vl_status = VL53L1X_GetDistance(&distance);
  if (vl_status != VL53L1X_ERROR_NONE) {
    return -1;
  }
  return (int16_t)distance;
}

/**************************************************************************/
/*!
    @brief  Clear the interrupt.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::clearInterrupt(void) {
  vl_status = VL53L1X_ClearInterrupt();
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Set the interrupt polarity.
    @param polarity The polarity to set as a boolean.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::setIntPolarity(bool polarity) {
  vl_status = VL53L1X_SetInterruptPolarity(polarity);
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Get the interrupt polarity.
    @returns Polarity as a boolean.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::getIntPolarity(void) {
  uint8_t x = 0;
  vl_status = VL53L1X_GetInterruptPolarity(&x);
  return (bool)x;
}

/**************************************************************************/
/*!
    @brief  Start ranging operations.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::startRanging(void) {
  vl_status = VL53L1X_StartRanging();
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Stop ranging operations.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::stopRanging(void) {
  vl_status = VL53L1X_StopRanging();
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Check status of new data.
    @returns True if new data available, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::dataReady(void) {
  uint8_t x = 0;
  vl_status = VL53L1X_CheckForDataReady(&x);
  return (bool)x;
}

/**************************************************************************/
/*!
    @brief  Set the timing budget.
    @param ms Timing budget in milliseconds.
    @returns True if successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_VL53L1X::setTimingBudget(uint16_t ms) {
  vl_status = VL53L1X_SetTimingBudgetInMs(ms);
  return (vl_status == VL53L1X_ERROR_NONE);
}

/**************************************************************************/
/*!
    @brief  Get the timing budget.
    @returns Timing budget in milliseconds.
*/
/**************************************************************************/
uint16_t Adafruit_VL53L1X::getTimingBudget(void) {
  uint16_t ms = 0;

  vl_status = VL53L1X_GetTimingBudgetInMs(&ms);
  if (vl_status == VL53L1X_ERROR_NONE) {
    return ms;
  }
  return 0;
}

/*

}

boolean Adafruit_VL53L1X::SetDistanceMode(VL53L1_DistanceModes mode) {
  Status = VL53L1_SetDistanceMode(pMyDevice, mode );
  return (Status == VL53L1_ERROR_NONE);
}

boolean
Adafruit_VL53L1X::GetRangingMeasurementData(VL53L1_RangingMeasurementData_t
*ranging) { Status = VL53L1_GetRangingMeasurementData(pMyDevice, ranging);
  return (Status == VL53L1_ERROR_NONE);
}
*/

}  // namespace vl53l1x
}  // namespace esphome
