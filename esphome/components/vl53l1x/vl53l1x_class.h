#pragma once
/*******************************************************************************
 Copyright Ã‚Â© 2018, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "vl53l1x_error_codes.h"

namespace esphome {
namespace vl53l1x {

static const char *const TAG = "vl53l1x";

/**
 *  @brief defines SW Version
 */
using VL53L1X_Version = struct {
  uint8_t major;     /*!< major number */
  uint8_t minor;     /*!< minor number */
  uint8_t build;     /*!< build number */
  uint32_t revision; /*!< revision number */
};

/* Classes -------------------------------------------------------------------*/
/** Class representing a VL53L1X sensor component
 */
class VL53L1X : public i2c::I2CDevice, public VL53L1XErrorCodes {  // NOLINT(cppcoreguidelines-virtual-class-destructor)
 public:
  static const uint8_t VL53L1X_IMPLEMENTATION_VER_MAJOR = 1;
  static const uint8_t VL53L1X_IMPLEMENTATION_VER_MINOR = 0;
  static const uint8_t VL53L1X_IMPLEMENTATION_VER_SUB = 1;
  static const uint32_t VL53L1X_IMPLEMENTATION_VER_REVISION = 0000;

  static const uint16_t VL53L1X_I2C_BUS_DEVICE_ADDRESS = 0x0001;
  static const uint16_t VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
  static const uint16_t VL53L1X_VHV_CONFIG_INIT = 0x000B;
  static const uint16_t ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS = 0x0016;
  static const uint16_t ALGO_CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS = 0x0018;
  static const uint16_t ALGO_CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS = 0x001A;
  static const uint16_t ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x001E;
  static const uint16_t MM_CONFIG_INNER_OFFSET_MM = 0x0020;
  static const uint16_t MM_CONFIG_OUTER_OFFSET_MM = 0x0022;
  static const uint16_t PAD_I2C_HV_EXTSUP_CONFIG = 0x002E;
  static const uint16_t GPIO_HV_MUX_CTRL = 0x0030;
  static const uint16_t GPIO_TIO_HV_STATUS = 0x0031;
  static const uint16_t SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0046;
  static const uint16_t PHASECAL_CONFIG_TIMEOUT_MACROP = 0x004B;
  static const uint16_t RANGE_CONFIG_TIMEOUT_MACROP_A_HI = 0x005E;
  static const uint16_t RANGE_CONFIG_VCSEL_PERIOD_A = 0x0060;
  static const uint16_t RANGE_CONFIG_VCSEL_PERIOD_B = 0x0063;
  static const uint16_t RANGE_CONFIG_TIMEOUT_MACROP_B_HI = 0x0061;
  static const uint16_t RANGE_CONFIG_TIMEOUT_MACROP_B_LO = 0x0062;
  static const uint16_t RANGE_CONFIG_SIGMA_THRESH = 0x0064;
  static const uint16_t RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066;
  static const uint16_t RANGE_CONFIG_VALID_PHASE_HIGH = 0x0069;
  static const uint16_t VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD = 0x006C;
  static const uint16_t SYSTEM_THRESH_HIGH = 0x0072;
  static const uint16_t SYSTEM_THRESH_LOW = 0x0074;
  static const uint16_t SD_CONFIG_WOI_SD0 = 0x0078;
  static const uint16_t SD_CONFIG_INITIAL_PHASE_SD0 = 0x007A;
  static const uint16_t ROI_CONFIG_USER_ROI_CENTRE_SPAD = 0x007F;
  static const uint16_t ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080;
  static const uint16_t SYSTEM_SEQUENCE_CONFIG = 0x0081;
  static const uint16_t VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD = 0x0082;
  static const uint16_t SYSTEM_INTERRUPT_CLEAR = 0x0086;
  static const uint16_t SYSTEM_MODE_START = 0x0087;
  static const uint16_t VL53L1X_RESULT_RANGE_STATUS = 0x0089;
  static const uint16_t VL53L1X_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x008C;
  static const uint16_t RESULT_AMBIENT_COUNT_RATE_MCPS_SD = 0x0090;
  static const uint16_t VL53L1X_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096;
  static const uint16_t VL53L1X_RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 = 0x0098;
  static const uint16_t VL53L1X_RESULT_OSC_CALIBRATE_VAL = 0x00DE;
  static const uint16_t VL53L1X_FIRMWARE_SYSTEM_STATUS = 0x00E5;
  static const uint16_t VL53L1X_IDENTIFICATION_MODEL_ID = 0x010F;
  static const uint16_t VL53L1X_ROI_CONFIG_MODE_ROI_CENTRE_SPAD = 0x013E;

  static const uint8_t VL53L1X_DEFAULT_DEVICE_ADDRESS = 0x29;

  VL53L1X_Error vl_status; /**< VL53L1X API Error Status */

  /*** Interface Methods ***/
  /*** High level API ***/
  /**
   * @brief       PowerOn the sensor
   * @return      void
   */
  /* turns on the sensor */
  virtual void vl53l1x_on() {
    if (irq_pin_ != nullptr) {
      irq_pin_->digital_write(true);
    }
    delay(10);
  }

  /**
   * @brief       PowerOff the sensor
   * @return      void
   */
  /* turns off the sensor */
  virtual void vl53l1x_off() {
    if (irq_pin_ != nullptr) {
      irq_pin_->digital_write(false);
    }
    delay(10);
  }

  /**
   * @brief       Initialize the sensor with default values
   * @return      0 on Success
   */

  VL53L1X_Error init_sensor(uint8_t address) {
    VL53L1X_Error status = 0;
    uint8_t sensorState = 0;
    vl53l1x_off();
    vl53l1x_on();

    if (address != VL53L1X_DEFAULT_DEVICE_ADDRESS) {
      status = vl53l1x_set_i2c_address(address);
    }
    while (!sensorState && !status) {
      status = vl53l1x_boot_state(&sensorState);
      delay(2);
    }
    if (!status) {
      status = vl53l1x_sensor_init();
    }
    return status;
  }

  /**
   *
   * @brief One time device initialization
   * @param void
   * @return     0 on success,  @a #CALIBRATION_WARNING if failed
   */
  int init() { return vl53l1x_sensor_init(); }

  /* Read function of the ID device */
  int read_id() {
    uint16_t sensorId;
    vl53l1x_get_sensor_id(&sensorId);
    if (sensorId == 0xEEAC)
      return 0;
    return -1;
  }

  /**
   * @brief Get ranging result and only that
   * @param pRange_mm  Pointer to range get_distance
   * @return           0 on success
   */
  int get_distance(uint32_t *piData) {
    int status;
    uint16_t distance;
    status = vl53l1x_get_distance(&distance);
    *piData = (uint32_t) distance;
    return status;
  }

  /* VL53L1X_api.h functions */

  /**
   * @brief This function returns the SW driver version
   */
  VL53L1X_Error vl53l1x_get_sw_version(VL53L1X_Version *version);

  /**
   * @brief This function sets the sensor I2C address used in case multiple devices application, default address 0x52
   */
  VL53L1X_Error vl53l1x_set_i2c_address(uint8_t new_address);

  /**
   * @brief This function loads the 135 bytes default values to initialize the sensor.
   * @param dev Device address
   * @return 0:success, != 0:failed
   */
  VL53L1X_Error vl53l1x_sensor_init();

  /**
   * @brief This function clears the interrupt, to be called after a ranging data reading
   * to arm the interrupt for the next data ready event.
   */
  VL53L1X_Error vl53l1x_clear_interrupt();

  /**
   * @brief This function programs the interrupt polarity\n
   * 1=active high (default), 0=active low
   */
  VL53L1X_Error vl53l1x_set_interrupt_polarity(uint8_t new_polarity);

  /**
   * @brief This function returns the current interrupt polarity\n
   * 1=active high (default), 0=active low
   */
  VL53L1X_Error vl53l1x_get_interrupt_polarity(uint8_t *interrupt_polarity);

  /**
   * @brief This function starts the ranging get_distance operation\n
   * The ranging operation is continuous. The clear interrupt has to be done after each get data to allow the interrupt
   * to raise when the next data is ready\n 1=active high (default), 0=active low, use SetInterruptPolarity() to change
   * the interrupt polarity if required.
   */
  VL53L1X_Error vl53l1x_start_ranging();

  /**
   * @brief This function stops the ranging.
   */
  VL53L1X_Error vl53l1x_stop_ranging();

  /**
   * @brief This function checks if the new ranging data is available by polling the dedicated register.
   * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
   */
  VL53L1X_Error vl53l1x_check_for_data_ready(uint8_t *is_data_ready);

  /**
   * @brief This function programs the timing budget in ms.
   * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
   */
  VL53L1X_Error vl53l1x_set_timing_budget_in_ms(uint16_t timing_budget_in_ms);

  /**
   * @brief This function returns the current timing budget in ms.
   */
  VL53L1X_Error vl53l1x_get_timing_budget_in_ms(uint16_t *timing_budget);

  /**
   * @brief This function programs the get_distance mode (1=short, 2=long(default)).
   * Short mode max get_distance is limited to 1.3 m but better ambient immunity.\n
   * Long mode can range up to 4 m in the dark with 200 ms timing budget.
   */
  VL53L1X_Error vl53l1x_set_distance_mode(uint16_t distance_mode);

  /**
   * @brief This function returns the current get_distance mode (1=short, 2=long).
   */
  VL53L1X_Error vl53l1x_get_distance_mode(uint16_t *distance_mode);

  /**
   * @brief This function programs the Intermeasurement period in ms\n
   * Intermeasurement period must be >/= timing budget. This condition is not checked by the API,
   * the customer has the duty to check the condition. Default = 100 ms
   */
  VL53L1X_Error vl53l1x_set_inter_measurement_in_ms(uint16_t inter_meas_ms);

  /**
   * @brief This function returns the Intermeasurement period in ms.
   */
  VL53L1X_Error vl53l1x_get_inter_measurement_in_ms(uint16_t *p_im);

  /**
   * @brief This function returns the boot state of the device (1:booted, 0:not booted)
   */
  VL53L1X_Error vl53l1x_boot_state(uint8_t *state);

  /**
   * @brief This function returns the sensor id, sensor Id must be 0xEEAC
   */
  VL53L1X_Error vl53l1x_get_sensor_id(uint16_t *sensor_id);

  /**
   * @brief This function returns the get_distance measured by the sensor in mm
   */
  VL53L1X_Error vl53l1x_get_distance(uint16_t *distance);

  /**
   * @brief This function returns the returned signal per SPAD in kcps/SPAD.
   * With kcps stands for Kilo Count Per Second
   */
  VL53L1X_Error vl53l1x_get_signal_per_spad(uint16_t *signalPerSp);

  /**
   * @brief This function returns the ambient per SPAD in kcps/SPAD
   */
  VL53L1X_Error vl53l1x_get_ambient_per_spad(uint16_t *amb_per_sp);

  /**
   * @brief This function returns the returned signal in kcps.
   */
  VL53L1X_Error vl53l1x_get_signal_rate(uint16_t *signalRate);

  /**
   * @brief This function returns the current number of enabled SPADs
   */
  VL53L1X_Error vl53l1x_get_spad_nb(uint16_t *sp_nb);

  /**
   * @brief This function returns the ambient rate in kcps
   */
  VL53L1X_Error vl53l1x_get_ambient_rate(uint16_t *amb_rate);

  /**
   * @brief This function returns the ranging status error \n
   * (0:no error, 1:sigma failed, 2:signal failed, ..., 7:wrap-around)
   */
  VL53L1X_Error vl53l1x_get_range_status(uint8_t *range_status);

  /**
   * @brief This function programs the offset correction in mm
   * @param offset_value:the offset correction value to program in mm
   */
  VL53L1X_Error vl53l1x_set_offset(int16_t offset_value);

  /**
   * @brief This function returns the programmed offset correction value in mm
   */
  VL53L1X_Error vl53l1x_get_offset(int16_t *offset);

  /**
   * @brief This function programs the xtalk correction value in cps (Count Per Second).\n
   * This is the number of photons reflected back from the cover glass in cps.
   */
  VL53L1X_Error vl53l1x_set_xtalk(uint16_t XtalkValue);

  /**
   * @brief This function returns the current programmed xtalk correction value in cps
   */
  VL53L1X_Error vl53l1x_get_xtalk(uint16_t *Xtalk);

  /**
   * @brief This function programs the threshold detection mode\n
   * Example:\n
   * VL53L1X_SetDistanceThreshold(dev,100,300,0,1): Below 100 \n
   * VL53L1X_SetDistanceThreshold(dev,100,300,1,1): Above 300 \n
   * VL53L1X_SetDistanceThreshold(dev,100,300,2,1): Out of window \n
   * VL53L1X_SetDistanceThreshold(dev,100,300,3,1): In window \n
   * @param   dev : device address
   * @param  	ThreshLow(in mm) : the threshold under which one the device raises an interrupt if Window = 0
   * @param 	thresh_high(in mm) :  the threshold above which one the device raises an interrupt if Window = 1
   * @param   window detection mode : 0=below, 1=above, 2=out, 3=in
   * @param   int_on_no_target = 1 (No longer used - just use 1)
   */
  VL53L1X_Error vl53l1x_set_distance_threshold(uint16_t ThreshLow, uint16_t thresh_high, uint8_t window,
                                               uint8_t int_on_no_target = 0);

  /**
   * @brief This function returns the window detection mode (0=below; 1=above; 2=out; 3=in)
   */
  VL53L1X_Error vl53l1x_get_distance_threshold_window(uint16_t *window);

  /**
   * @brief This function returns the low threshold in mm
   */
  VL53L1X_Error vl53l1x_get_distance_threshold_low(uint16_t *low);

  /**
   * @brief This function returns the high threshold in mm
   */
  VL53L1X_Error vl53l1x_get_distance_threshold_high(uint16_t *high);

  /**
   * @brief This function programs the ROI (Region of Interest)\n
   * The ROI position is centered, only the ROI size can be reprogrammed.\n
   * The smallest acceptable ROI size = 4\n
   * @param roi_x:ROI Width; Y=ROI Height
   */
  VL53L1X_Error vl53l1x_set_roi(uint16_t roi_x, uint16_t roi_y);

  /**
   *@brief This function returns width X and height Y
   */
  VL53L1X_Error vl53l1x_get_roi_xy(uint16_t *roi_x, uint16_t *roi_y);

  /**
   *@brief This function programs the new user ROI center, please to be aware that there is no check in this function.
   *if the ROI center vs ROI size is out of border the ranging function return error #13
   */
  VL53L1X_Error vl53l1x_set_roi_center(uint8_t roi_center);

  /**
   *@brief This function returns the current user ROI center
   */
  VL53L1X_Error vl53l1x_get_roi_center(uint8_t *roi_center);

  /**
   * @brief This function programs a new signal threshold in kcps (default=1024 kcps\n
   */
  VL53L1X_Error vl53l1x_set_signal_threshold(uint16_t signal);

  /**
   * @brief This function returns the current signal threshold in kcps
   */
  VL53L1X_Error vl53l1x_get_signal_threshold(uint16_t *signal);

  /**
   * @brief This function programs a new sigma threshold in mm (default=15 mm)
   */
  VL53L1X_Error vl53l1x_set_sigma_threshold(uint16_t sigma);

  /**
   * @brief This function returns the current sigma threshold in mm
   */
  VL53L1X_Error vl53l1x_get_sigma_threshold(uint16_t *signal);

  /**
   * @brief This function performs the temperature calibration.
   * It is recommended to call this function any time the temperature might have changed by more than 8 deg C
   * without sensor ranging activity for an extended period.
   */
  VL53L1X_Error vl53l1x_start_temperature_update();

  /* VL53L1X_calibration.h functions */

  /**
   * @brief This function performs the offset calibration.\n
   * The function returns the offset value found and programs the offset compensation into the device.
   * @param target_dist_in_mm target get_distance in mm, ST recommended 100 mm
   * Target reflectance = grey17%
   * @return 0:success, !=0: failed
   * @return offset pointer contains the offset found in mm
   */
  int8_t vl53l1x_calibrate_offset(uint16_t target_dist_in_mm, int16_t *offset);

  /**
   * @brief This function performs the xtalk calibration.\n
   * The function returns the xtalk value found and programs the xtalk compensation to the device
   * @param target_dist_in_mm target get_distance in mm\n
   * The target get_distance : the distance where the sensor start to "under range"\n
   * due to the influence of the photons reflected back from the cover glass becoming strong\n
   * It's also called inflection point\n
   * Target reflectance = grey 17%
   * @return 0: success, !=0: failed
   * @return xtalk pointer contains the xtalk value found in cps (number of photons in count per second)
   */
  int8_t vl53l1x_calibrate_xtalk(uint16_t target_dist_in_mm, uint16_t *xtalk);

 protected:
  /* Write and read functions from I2C */

  VL53L1X_Error vl53l1x_wr_byte(uint16_t index, uint8_t data);
  VL53L1X_Error vl53l1x_wr_word(uint16_t index, uint16_t data);
  VL53L1X_Error vl53l1x_wr_dword(uint16_t index, uint32_t data);
  VL53L1X_Error vl53l1x_rd_byte(uint16_t index, uint8_t *data);
  VL53L1X_Error vl53l1x_rd_word(uint16_t index, uint16_t *data);
  VL53L1X_Error vl53l1x_rd_dword(uint16_t index, uint32_t *data);
  VL53L1X_Error vl53l1x_update_byte(uint16_t index, uint8_t and_data, uint8_t or_data);

  VL53L1X_Error vl53l1x_write_multi(uint16_t index, uint8_t *pdata, uint32_t count);
  VL53L1X_Error vl53l1x_read_multi(uint16_t index, uint8_t *pdata, uint32_t count);

  VL53L1X_Error vl53l1x_i2c_write(uint16_t register_addr, const uint8_t *pbuffer, uint16_t num_byte_to_write);
  VL53L1X_Error vl53l1x_i2c_read(uint16_t register_addr, uint8_t *pbuffer, uint16_t num_byte_to_read);
  VL53L1X_Error vl53l1x_get_tick_count(uint32_t *ptick_count_ms);
  VL53L1X_Error vl53l1x_wait_us(int32_t wait_us);
  VL53L1X_Error vl53l1x_wait_ms(int32_t wait_ms);

  VL53L1X_Error vl53l1x_wait_value_mask_ex(uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask,
                                           uint32_t poll_delay_ms);

  GPIOPin *irq_pin_{nullptr};
  GPIOPin *enable_pin_{nullptr};
};

}  // namespace vl53l1x
}  // namespace esphome
