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

#include "esphome/core/gpio.h"
#include "esphome/components/i2c/i2c.h"
#include "vl53l1x_class.h"

namespace esphome {
namespace vl53l1x {

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with VL53L1X
   time-of-flight sensor chips
*/
/**************************************************************************/
class Adafruit_VL53L1X : public VL53L1X {
 public:
  Adafruit_VL53L1X();

  bool begin(uint8_t i2c_addr = VL53L1X::VL53L1X_DEFAULT_DEVICE_ADDRESS);
  uint16_t get_sensor_id(void);

  bool start_ranging(void);
  bool stop_ranging(void);
  bool set_timing_budget_ms(uint16_t ms);
  uint16_t get_timing_budget(void);

  bool is_data_ready(void);
  int16_t get_distance(void);

  bool clear_interrupt(void);
  bool set_int_polarity(bool polarity);
  bool get_int_polarity(void);

  bool set_distance_mode(uint16_t mode);
  /*
    boolean SetMeasurementTimingBudgetMicroSeconds(uint32_t budget);
    boolean SetInterMeasurementPeriodMilliSeconds(uint32_t period);
    boolean StartMeasurement(void);
    boolean WaitMeasurementDataReady(void);
    boolean GetRangingMeasurementData(VL53L1_RangingMeasurementData_t *ranging);
    */
};

}  // namespace vl53l1x
}  // namespace esphome
