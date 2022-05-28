#pragma once
/*!
 * @file Adafruit_VL53L1X.h

  This is a library for the Adafruit VL53L1X Sensor Breakout

  Designed specifically to work with the VL53L1X sensor from Adafruit
  ----> https://www.adafruit.com/products/3967

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef ADAFRUIT_VL53L1X_H
#define ADAFRUIT_VL53L1X_H

#include "esphome/core/gpio.h"
#include "esphome/components/i2c/i2c.h"
#include "vl53l1x_class.h"

namespace esphome {
namespace vl53l1x {

    const uint8_t VL53L1X_I2C_ADDR = 0x29;              ///< Default sensor I2C address

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with VL53L1X
   time-of-flight sensor chips
*/
/**************************************************************************/
class Adafruit_VL53L1X : public VL53L1X {
public:
  Adafruit_VL53L1X(GPIOPin *shutdown_pin, GPIOPin *irq_pin, uint8_t i2c_adr);

  bool begin(uint8_t i2c_addr = VL53L1X_I2C_ADDR);
  uint16_t sensorID(void);

  bool startRanging(void);
  bool stopRanging(void);
  bool setTimingBudget(uint16_t ms);
  uint16_t getTimingBudget(void);

  bool dataReady(void);
  int16_t distance(void);

  bool clearInterrupt(void);
  bool setIntPolarity(bool polarity);
  bool getIntPolarity(void);

  /*
  boolean SetDistanceMode(VL53L1_DistanceModes mode);
  boolean SetMeasurementTimingBudgetMicroSeconds(uint32_t budget);
  boolean SetInterMeasurementPeriodMilliSeconds(uint32_t period);
  boolean StartMeasurement(void);
  boolean WaitMeasurementDataReady(void);
  boolean GetRangingMeasurementData(VL53L1_RangingMeasurementData_t *ranging);
  */

  VL53L1X_ERROR vl_status; /**< VL53L1X API Error Status */

private:
  GPIOPin *_irq_pin{nullptr};
  GPIOPin *_shutdown_pin{nullptr};

};

}  // namespace vl53l1x
}  // namespace esphome

#endif
