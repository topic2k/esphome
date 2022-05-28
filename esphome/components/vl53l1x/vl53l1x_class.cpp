/**
 ******************************************************************************
 * @file    vl53l0x_class.cpp
 * @author  IMG
 * @version V0.0.1
 * @date    14-December-2018
 * @brief   Implementation file for the VL53L1X driver class
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "vl53l1x_class.h"

namespace esphome {
namespace vl53l1x {

//<editor-fold desc="const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {">
const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
    0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
    0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use
             SetInterruptPolarity() */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
    0x00, /* 0x32 : not user-modifiable */
    0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */
    0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */
    0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */
    0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3a : not user-modifiable */
    0x00, /* 0x3b : not user-modifiable */
    0x00, /* 0x3c : not user-modifiable */
    0x00, /* 0x3d : not user-modifiable */
    0xff, /* 0x3e : not user-modifiable */
    0x00, /* 0x3f : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */
    0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */
    0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */
    0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window,
             0x20-> New sample ready , TBC */
    0x0b, /* 0x47 : not user-modifiable */
    0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */
    0x02, /* 0x4a : not user-modifiable */
    0x0a, /* 0x4b : not user-modifiable */
    0x21, /* 0x4c : not user-modifiable */
    0x00, /* 0x4d : not user-modifiable */
    0x00, /* 0x4e : not user-modifiable */
    0x05, /* 0x4f : not user-modifiable */
    0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */
    0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */
    0xc8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */
    0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */
    0xff, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */
    0x00, /* 0x5a : not user-modifiable */
    0x08, /* 0x5b : not user-modifiable */
    0x00, /* 0x5c : not user-modifiable */
    0x00, /* 0x5d : not user-modifiable */
    0x01, /* 0x5e : not user-modifiable */
    0xcc, /* 0x5f : not user-modifiable */
    0x0f, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */
    0xf1, /* 0x62 : not user-modifiable */
    0x0d, /* 0x63 : not user-modifiable */
    0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm */
    0x68, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
    0x80, /* 0x67 : Min count Rate LSB */
    0x08, /* 0x68 : not user-modifiable */
    0xb8, /* 0x69 : not user-modifiable */
    0x00, /* 0x6a : not user-modifiable */
    0x00, /* 0x6b : not user-modifiable */
    0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
    0x00, /* 0x6d : Intermeasurement period */
    0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */
    0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x75 : distance threshold low LSB */
    0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */
    0x0f, /* 0x78 : not user-modifiable */
    0x0d, /* 0x79 : not user-modifiable */
    0x0e, /* 0x7a : not user-modifiable */
    0x0e, /* 0x7b : not user-modifiable */
    0x00, /* 0x7c : not user-modifiable */
    0x00, /* 0x7d : not user-modifiable */
    0x02, /* 0x7e : not user-modifiable */
    0xc7, /* 0x7f : ROI center, use SetROI() */
    0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
    0x9B, /* 0x81 : not user-modifiable */
    0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */
    0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */
    0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
    0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after
             VL53L1X_init() call, put 0x40 in location 0x87 */
};
//</editor-fold>

/* VL53L1X_api.h functions */

VL53L1X_Error VL53L1X::vl53l1x_get_sw_version(VL53L1X_Version *version) {
  VL53L1X_Error Status = 0;

  version->major = VL53L1X::VL53L1X_IMPLEMENTATION_VER_MAJOR;
  version->minor = VL53L1X::VL53L1X_IMPLEMENTATION_VER_MINOR;
  version->build = VL53L1X::VL53L1X_IMPLEMENTATION_VER_SUB;
  version->revision = VL53L1X::VL53L1X_IMPLEMENTATION_VER_REVISION;
  return Status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_i2c_address(uint8_t new_address) {
  VL53L1X_Error status = 0;
  status = this->vl53l1x_wr_byte(VL53L1X::VL53L1X_I2C_BUS_DEVICE_ADDRESS, new_address);
  set_i2c_address(new_address);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_sensor_init() {
  VL53L1X_Error status = 0;
  uint8_t addr = 0x00, tmp = 0;

  for (addr = 0x2D; addr <= 0x87; addr++) {
    status = this->vl53l1x_wr_byte(addr, VL51L1X_DEFAULT_CONFIGURATION[addr - 0x2D]);
  }
  status = this->vl53l1x_start_ranging();
  while (tmp == 0) {
    status = this->vl53l1x_check_for_data_ready(&tmp);
  }
  tmp = 0;
  status = this->vl53l1x_clear_interrupt();
  status = this->vl53l1x_stop_ranging();
  status = this->vl53l1x_wr_byte(VL53L1X::VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
  status = this->vl53l1x_wr_byte(VL53L1X::VL53L1X_VHV_CONFIG_INIT, 0); /* start VHV from the previous temperature */
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_clear_interrupt() {
  VL53L1X_Error status = 0;

  status = this->vl53l1x_wr_byte(VL53L1X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_interrupt_polarity(uint8_t new_polarity) {
  uint8_t temp;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_rd_byte(VL53L1X::GPIO_HV_MUX_CTRL, &temp);
  temp = temp & 0xEF;
  status = this->vl53l1x_wr_byte(VL53L1X::GPIO_HV_MUX_CTRL, temp | (!(new_polarity & 1)) << 4);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_interrupt_polarity(uint8_t *interrupt_polarity) {
  uint8_t temp;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_rd_byte(VL53L1X::GPIO_HV_MUX_CTRL, &temp);
  temp = temp & 0x10;
  *interrupt_polarity = !(temp >> 4);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_start_ranging() {
  return this->vl53l1x_wr_byte(VL53L1X::SYSTEM_MODE_START, 0x40); /* Enable VL53L1X */
}

VL53L1X_Error VL53L1X::vl53l1x_stop_ranging() {
  return this->vl53l1x_wr_byte(VL53L1X::SYSTEM_MODE_START, 0x00); /* Disable VL53L1X */
}

VL53L1X_Error VL53L1X::vl53l1x_check_for_data_ready(uint8_t *is_data_ready) {
  uint8_t temp;
  uint8_t int_pol;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_get_interrupt_polarity(&int_pol);
  status = this->vl53l1x_rd_byte(VL53L1X::GPIO_TIO_HV_STATUS, &temp);
  /* Read in the register to check if a new value is available */
  if (status == 0) {
    if ((temp & 1) == int_pol) {
      *is_data_ready = 1;
    } else {
      *is_data_ready = 0;
    }
  }
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_timing_budget_in_ms(uint16_t timing_budget_in_ms) {
  uint16_t distance_mode;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_get_distance_mode(&distance_mode);
  if (distance_mode == 0) {
    return 1;
  } else if (distance_mode == 1) {
    /* Short DistanceMode */
    switch (timing_budget_in_ms) {
      case 15: /* only available in short distance mode */
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x01D);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x0027);
        break;
      case 20:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x0051);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x006E);
        break;
      case 33:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x00D6);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x006E);
        break;
      case 50:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x1AE);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x01E8);
        break;
      case 100:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x02E1);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x0388);
        break;
      case 200:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x03E1);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x0496);
        break;
      case 500:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x0591);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x05C1);
        break;
      default:
        status = 1;
        break;
    }
  } else {
    switch (timing_budget_in_ms) {
      case 20:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x001E);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x0022);
        break;
      case 33:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x0060);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x006E);
        break;
      case 50:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x00AD);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x00C6);
        break;
      case 100:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x01CC);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x01EA);
        break;
      case 200:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x02D9);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x02F8);
        break;
      case 500:
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x048F);
        this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x04A4);
        break;
      default:
        status = 1;
        break;
    }
  }
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_timing_budget_in_ms(uint16_t *timing_budget) {
  uint16_t temp;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_rd_word(VL53L1X::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, &temp);
  switch (temp) {
    case 0x001D:
      *timing_budget = 15;
      break;
    case 0x0051:
    case 0x001E:
      *timing_budget = 20;
      break;
    case 0x00D6:
    case 0x0060:
      *timing_budget = 33;
      break;
    case 0x1AE:
    case 0x00AD:
      *timing_budget = 50;
      break;
    case 0x02E1:
    case 0x01CC:
      *timing_budget = 100;
      break;
    case 0x03E1:
    case 0x02D9:
      *timing_budget = 200;
      break;
    case 0x0591:
    case 0x048F:
      *timing_budget = 500;
      break;
    default:
      *timing_budget = 0;
      break;
  }
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_distance_mode(uint16_t distance_mode) {
  uint16_t timing_budget;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_get_timing_budget_in_ms(&timing_budget);
  switch (distance_mode) {
    case 1:
      status = this->vl53l1x_wr_byte(VL53L1X::PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14);
      status = this->vl53l1x_wr_byte(VL53L1X::RANGE_CONFIG_VCSEL_PERIOD_A, 0x07);
      status = this->vl53l1x_wr_byte(VL53L1X::RANGE_CONFIG_VCSEL_PERIOD_B, 0x05);
      status = this->vl53l1x_wr_byte(VL53L1X::RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
      status = this->vl53l1x_wr_word(VL53L1X::SD_CONFIG_WOI_SD0, 0x0705);
      status = this->vl53l1x_wr_word(VL53L1X::SD_CONFIG_INITIAL_PHASE_SD0, 0x0606);
      break;
    case 2:
      status = this->vl53l1x_wr_byte(VL53L1X::PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0A);
      status = this->vl53l1x_wr_byte(VL53L1X::RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F);
      status = this->vl53l1x_wr_byte(VL53L1X::RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D);
      status = this->vl53l1x_wr_byte(VL53L1X::RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8);
      status = this->vl53l1x_wr_word(VL53L1X::SD_CONFIG_WOI_SD0, 0x0F0D);
      status = this->vl53l1x_wr_word(VL53L1X::SD_CONFIG_INITIAL_PHASE_SD0, 0x0E0E);
      break;
    default:
      break;
  }
  status = this->vl53l1x_set_timing_budget_in_ms(timing_budget);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_distance_mode(uint16_t *distance_mode) {
  uint8_t temp_dm, status = 0;

  status = this->vl53l1x_rd_byte(VL53L1X::PHASECAL_CONFIG_TIMEOUT_MACROP, &temp_dm);
  if (temp_dm == 0x14) {
    *distance_mode = 1;
  }
  if (temp_dm == 0x0A) {
    *distance_mode = 2;
  }
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_inter_measurement_in_ms(uint16_t inter_meas_ms) {
  uint16_t clock_pll;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_rd_word(VL53L1X::VL53L1X_RESULT_OSC_CALIBRATE_VAL, &clock_pll);
  clock_pll = clock_pll & 0x3FF;
  this->vl53l1x_wr_dword(VL53L1X::VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD, (uint32_t) (clock_pll * inter_meas_ms * 1.075));
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_inter_measurement_in_ms(uint16_t *inter_ms) {
  uint16_t clock_pll;
  VL53L1X_Error status = 0;
  uint32_t tmp;

  status = this->vl53l1x_rd_dword(VL53L1X::VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD, &tmp);
  *inter_ms = (uint16_t) tmp;
  status = this->vl53l1x_rd_word(VL53L1X::VL53L1X_RESULT_OSC_CALIBRATE_VAL, &clock_pll);
  clock_pll = clock_pll & 0x3FF;
  *inter_ms = (uint16_t) (*inter_ms / (clock_pll * 1.065));
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_boot_state(uint8_t *state) {
  VL53L1X_Error status = 0;
  uint8_t tmp = 0;

  status = vl53l1x_rd_byte(VL53L1X_FIRMWARE_SYSTEM_STATUS, &tmp);
  *state = tmp;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_sensor_id(uint16_t *sensor_id) {
  VL53L1X_Error status = 0;
  uint16_t tmp = 0;

  status = this->vl53l1x_rd_word(VL53L1X::VL53L1X_IDENTIFICATION_MODEL_ID, &tmp);
  *sensor_id = tmp;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_distance(uint16_t *distance) {
  VL53L1X_Error status = 0;
  uint16_t tmp = 0;

  status = this->vl53l1x_rd_word(VL53L1X::VL53L1X_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp);
  *distance = tmp;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_signal_per_spad(uint16_t *signalRate) {
  VL53L1X_Error status = 0;
  uint16_t sp_nb = 1, signal;

  status = this->vl53l1x_rd_word(VL53L1X::VL53L1X_RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &signal);
  status = this->vl53l1x_rd_word(VL53L1X::VL53L1X_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &sp_nb);
  *signalRate = (uint16_t) (2000.0 * signal / sp_nb);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_ambient_per_spad(uint16_t *amb_per_sp) {
  VL53L1X_Error status = 0;
  uint16_t ambient_rate, sp_nb = 1;

  status = this->vl53l1x_rd_word(VL53L1X::RESULT_AMBIENT_COUNT_RATE_MCPS_SD, &ambient_rate);
  status = this->vl53l1x_rd_word(VL53L1X::VL53L1X_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &sp_nb);
  *amb_per_sp = (uint16_t) (2000.0 * ambient_rate / sp_nb);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_signal_rate(uint16_t *signal) {
  VL53L1X_Error status = 0;
  uint16_t tmp;

  status = this->vl53l1x_rd_word(VL53L1X::VL53L1X_RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &tmp);
  *signal = tmp * 8;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_spad_nb(uint16_t *sp_nb) {
  VL53L1X_Error status = 0;
  uint16_t tmp;

  status = this->vl53l1x_rd_word(VL53L1X::VL53L1X_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &tmp);
  *sp_nb = tmp >> 8;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_ambient_rate(uint16_t *amb_rate) {
  VL53L1X_Error status = 0;
  uint16_t tmp;

  status = this->vl53l1x_rd_word(VL53L1X::RESULT_AMBIENT_COUNT_RATE_MCPS_SD, &tmp);
  *amb_rate = tmp * 8;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_range_status(uint8_t *range_status) {
  VL53L1X_Error status = 0;
  uint8_t rg_st;

  status = this->vl53l1x_rd_byte(VL53L1X::VL53L1X_RESULT_RANGE_STATUS, &rg_st);
  rg_st = rg_st & 0x1F;
  switch (rg_st) {
    case 9:
      rg_st = 0;
      break;
    case 6:
      rg_st = 1;
      break;
    case 4:
      rg_st = 2;
      break;
    case 8:
      rg_st = 3;
      break;
    case 5:
      rg_st = 4;
      break;
    case 3:
      rg_st = 5;
      break;
    case 19:
      rg_st = 6;
      break;
    case 7:
      rg_st = 7;
      break;
    case 12:
      rg_st = 9;
      break;
    case 18:
      rg_st = 10;
      break;
    case 22:
      rg_st = 11;
      break;
    case 23:
      rg_st = 12;
      break;
    case 13:
      rg_st = 13;
      break;
    default:
      rg_st = 255;
      break;
  }
  *range_status = rg_st;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_offset(int16_t offset_value) {
  VL53L1X_Error status = 0;
  int16_t temp;

  temp = (offset_value * 4);
  this->vl53l1x_wr_word(VL53L1X::ALGO_PART_TO_PART_RANGE_OFFSET_MM, (uint16_t) temp);
  this->vl53l1x_wr_word(VL53L1X::MM_CONFIG_INNER_OFFSET_MM, 0x0);
  this->vl53l1x_wr_word(VL53L1X::MM_CONFIG_OUTER_OFFSET_MM, 0x0);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_offset(int16_t *offset) {
  VL53L1X_Error status = 0;
  uint16_t temp;

  status = this->vl53l1x_rd_word(VL53L1X::ALGO_PART_TO_PART_RANGE_OFFSET_MM, &temp);
  temp = temp << 3;
  *offset = (int16_t) (temp);
  *offset = *offset / 32;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_xtalk(uint16_t XtalkValue) {
  /* XTalkValue in count per second to avoid float type */
  VL53L1X_Error status = 0;

  status = this->vl53l1x_wr_word(VL53L1X::ALGO_CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, 0x0000);
  status = this->vl53l1x_wr_word(VL53L1X::ALGO_CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, 0x0000);
  status = this->vl53l1x_wr_word(VL53L1X::ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, (XtalkValue << 9) / 1000);
  /* * << 9 (7.9 format) and /1000 to convert cps to kpcs */
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_xtalk(uint16_t *Xtalk) {
  VL53L1X_Error status = 0;
  uint16_t tmp;

  status = this->vl53l1x_rd_word(VL53L1X::ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, &tmp);
  *Xtalk = (tmp * 1000) >> 9; /* * 1000 to convert kcps to cps and >> 9 (7.9 format) */
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_distance_threshold(uint16_t thresh_low, uint16_t thresh_high, uint8_t window,
                                                      uint8_t int_on_no_target) {
  VL53L1X_Error status = 0;
  uint8_t temp = 0;

  status = this->vl53l1x_rd_byte(VL53L1X::SYSTEM_INTERRUPT_CONFIG_GPIO, &temp);
  temp = temp & 0x47;
  if (int_on_no_target == 0) {
    status = this->vl53l1x_wr_byte(VL53L1X::SYSTEM_INTERRUPT_CONFIG_GPIO, (temp | (window & 0x07)));
  } else {
    status = this->vl53l1x_wr_byte(VL53L1X::SYSTEM_INTERRUPT_CONFIG_GPIO, ((temp | (window & 0x07)) | 0x40));
  }
  status = this->vl53l1x_wr_word(VL53L1X::SYSTEM_THRESH_HIGH, thresh_high);
  status = this->vl53l1x_wr_word(VL53L1X::SYSTEM_THRESH_LOW, thresh_low);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_distance_threshold_window(uint16_t *window) {
  VL53L1X_Error status = 0;
  uint8_t tmp;

  status = this->vl53l1x_rd_byte(VL53L1X::SYSTEM_INTERRUPT_CONFIG_GPIO, &tmp);
  *window = (uint16_t) (tmp & 0x7);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_distance_threshold_low(uint16_t *low) {
  VL53L1X_Error status = 0;
  uint16_t tmp;

  status = this->vl53l1x_rd_word(VL53L1X::SYSTEM_THRESH_LOW, &tmp);
  *low = tmp;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_distance_threshold_high(uint16_t *high) {
  VL53L1X_Error status = 0;
  uint16_t tmp;

  status = this->vl53l1x_rd_word(VL53L1X::SYSTEM_THRESH_HIGH, &tmp);
  *high = tmp;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_roi(uint16_t roi_x, uint16_t roi_y) {
  uint8_t optical_center;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_rd_byte(VL53L1X::VL53L1X_ROI_CONFIG_MODE_ROI_CENTRE_SPAD, &optical_center);
  if (roi_x > 16) {
    roi_x = 16;
  }
  if (roi_y > 16) {
    roi_y = 16;
  }
  if (roi_x > 10 || roi_y > 10) {
    optical_center = 199;
  }
  status = this->vl53l1x_wr_byte(VL53L1X::ROI_CONFIG_USER_ROI_CENTRE_SPAD, optical_center);
  status = this->vl53l1x_wr_byte(VL53L1X::ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, (roi_y - 1) << 4 | (roi_x - 1));
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_roi_xy(uint16_t *roi_x, uint16_t *roi_y) {
  VL53L1X_Error status = 0;
  uint8_t tmp;

  status = this->vl53l1x_rd_byte(VL53L1X::ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &tmp);
  *roi_x = ((uint16_t) tmp & 0x0F) + 1;
  *roi_y = (((uint16_t) tmp & 0xF0) >> 4) + 1;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_roi_center(uint8_t roi_center) {
  VL53L1X_Error status = 0;

  status = this->vl53l1x_wr_byte(VL53L1X::ROI_CONFIG_USER_ROI_CENTRE_SPAD, roi_center);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_roi_center(uint8_t *roi_center) {
  VL53L1X_Error status = 0;

  uint8_t tmp;
  status = this->vl53l1x_rd_byte(VL53L1X::ROI_CONFIG_USER_ROI_CENTRE_SPAD, &tmp);
  *roi_center = tmp;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_signal_threshold(uint16_t Signal) {
  VL53L1X_Error status = 0;

  this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, Signal >> 3);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_signal_threshold(uint16_t *signal) {
  VL53L1X_Error status = 0;
  uint16_t tmp;

  status = this->vl53l1x_rd_word(VL53L1X::RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
  *signal = tmp << 3;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_set_sigma_threshold(uint16_t Sigma) {
  VL53L1X_Error status = 0;

  if (Sigma > (0xFFFF >> 2)) {
    return 1;
  }
  /* 16 bits register 14.2 format */
  status = this->vl53l1x_wr_word(VL53L1X::RANGE_CONFIG_SIGMA_THRESH, Sigma << 2);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_get_sigma_threshold(uint16_t *sigma) {
  VL53L1X_Error status = 0;
  uint16_t tmp;

  status = this->vl53l1x_rd_word(VL53L1X::RANGE_CONFIG_SIGMA_THRESH, &tmp);
  *sigma = tmp >> 2;
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_start_temperature_update() {
  VL53L1X_Error status = 0;
  uint8_t tmp = 0;
  status = this->vl53l1x_wr_byte(VL53L1X::VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x81); /* full VHV */
  status = this->vl53l1x_wr_byte(VL53L1X::VL53L1X_VHV_CONFIG_INIT, 0x92);
  status = this->vl53l1x_start_ranging();
  while (tmp == 0) {
    status = this->vl53l1x_check_for_data_ready(&tmp);
  }
  tmp = 0;
  status = this->vl53l1x_clear_interrupt();
  status = this->vl53l1x_stop_ranging();
  status = this->vl53l1x_wr_byte(VL53L1X::VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
  status = this->vl53l1x_wr_byte(VL53L1X::VL53L1X_VHV_CONFIG_INIT, 0); /* start VHV from the previous temperature */
  return status;
}

/* VL53L1X_calibration.h functions */

int8_t VL53L1X::vl53l1x_calibrate_offset(uint16_t target_dist_in_mm, int16_t *offset) {
  uint8_t cnt = 0, tmp;
  int16_t average_distance = 0;
  uint16_t distance;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_wr_word(VL53L1X::ALGO_PART_TO_PART_RANGE_OFFSET_MM, 0x0);
  status = this->vl53l1x_wr_word(VL53L1X::MM_CONFIG_INNER_OFFSET_MM, 0x0);
  status = this->vl53l1x_wr_word(VL53L1X::MM_CONFIG_OUTER_OFFSET_MM, 0x0);
  status = this->vl53l1x_start_ranging(); /* Enable VL53L1X sensor */
  for (cnt = 0; cnt < 50; cnt++) {
    while (tmp == 0) {
      status = this->vl53l1x_check_for_data_ready(&tmp);
    }
    tmp = 0;
    status = this->vl53l1x_get_distance(&distance);
    status = this->vl53l1x_clear_interrupt();
    average_distance = average_distance + distance;
  }
  status = this->vl53l1x_stop_ranging();
  average_distance = average_distance / 50;
  *offset = target_dist_in_mm - average_distance;
  status = this->vl53l1x_wr_word(VL53L1X::ALGO_PART_TO_PART_RANGE_OFFSET_MM, *offset * 4);
  return status;
}

int8_t VL53L1X::vl53l1x_calibrate_xtalk(uint16_t target_dist_in_mm, uint16_t *xtalk) {
  uint8_t cnt, tmp = 0;
  float average_signal_rate = 0;
  float average_distance = 0;
  float average_spad_nb = 0;
  uint16_t distance = 0, spad_num;
  uint16_t signal_rate;
  VL53L1X_Error status = 0;

  status = this->vl53l1x_wr_word(VL53L1X::ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, 0);
  status = this->vl53l1x_start_ranging();
  for (cnt = 0; cnt < 50; cnt++) {
    while (tmp == 0) {
      status = vl53l1x_check_for_data_ready(&tmp);
    }
    tmp = 0;
    status = this->vl53l1x_get_signal_rate(&signal_rate);
    status = this->vl53l1x_get_distance(&distance);
    status = this->vl53l1x_clear_interrupt();
    average_distance = average_distance + distance;
    status = this->vl53l1x_get_spad_nb(&spad_num);
    average_spad_nb = average_spad_nb + spad_num;
    average_signal_rate = average_signal_rate + signal_rate;
  }
  status = this->vl53l1x_stop_ranging();
  average_distance = average_distance / 50;
  average_spad_nb = average_spad_nb / 50;
  average_signal_rate = average_signal_rate / 50;
  /* Calculate Xtalk value */
  *xtalk = (uint16_t) (512 * (average_signal_rate * (1 - (average_distance / target_dist_in_mm))) / average_spad_nb);
  status = this->vl53l1x_wr_word(VL53L1X::ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, *xtalk);
  return status;
}

/* Write and read functions from I2C */

VL53L1X_Error VL53L1X::vl53l1x_write_multi(uint16_t index, uint8_t *pdata, uint32_t count) {
  int status;

  status = this->vl53l1x_i2c_write(index, pdata, (uint16_t) count);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_read_multi(uint16_t index, uint8_t *pdata, uint32_t count) {
  int status;

  status = this->vl53l1x_i2c_read(index, pdata, (uint16_t) count);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_wr_byte(uint16_t index, uint8_t data) {
  int status;

  status = this->vl53l1x_i2c_write(index, &data, 1);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_wr_word(uint16_t index, uint16_t data) {
  int status;
  uint8_t buffer[2];

  buffer[0] = data >> 8;
  buffer[1] = data & 0x00FF;
  status = this->vl53l1x_i2c_write(index, (uint8_t *) buffer, 2);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_wr_dword(uint16_t index, uint32_t data) {
  int status;
  uint8_t buffer[4];

  buffer[0] = (data >> 24) & 0xFF;
  buffer[1] = (data >> 16) & 0xFF;
  buffer[2] = (data >> 8) & 0xFF;
  buffer[3] = (data >> 0) & 0xFF;
  status = this->vl53l1x_i2c_write(index, (uint8_t *) buffer, 4);
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_rd_byte(uint16_t index, uint8_t *data) {
  int status;

  status = this->vl53l1x_i2c_read(index, data, 1);
  if (status) {
    return -1;
  }
  return 0;
}

VL53L1X_Error VL53L1X::vl53l1x_rd_word(uint16_t index, uint16_t *data) {
  int status;
  uint8_t buffer[2] = {0, 0};

  status = this->vl53l1x_i2c_read(index, buffer, 2);
  if (!status) {
    *data = (buffer[0] << 8) + buffer[1];
  }
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_rd_dword(uint16_t index, uint32_t *data) {
  int status;
  uint8_t buffer[4] = {0, 0, 0, 0};

  status = this->vl53l1x_i2c_read(index, buffer, 4);
  if (!status) {
    *data = ((uint32_t) buffer[0] << 24) + ((uint32_t) buffer[1] << 16) + ((uint32_t) buffer[2] << 8) +
            (uint32_t) buffer[3];
  }
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_update_byte(uint16_t index, uint8_t and_data, uint8_t or_data) {
  int status;
  uint8_t buffer = 0;

  /* read data direct onto buffer */
  status = this->vl53l1x_i2c_read(index, &buffer, 1);
  if (!status) {
    buffer = (buffer & and_data) | or_data;
    status = this->vl53l1x_i2c_write(index, &buffer, (uint16_t) 1);
  }
  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_i2c_write(uint16_t register_addr, const uint8_t *pbuffer, uint16_t num_byte_to_write) {
  uint16_t idx;
  uint8_t buffer[2 + num_byte_to_write];

  buffer[0] = (uint8_t) (register_addr >> 8);
  buffer[1] = (uint8_t) (register_addr & 0xFF);
  for (idx = 0; idx < num_byte_to_write; idx++) {
    buffer[idx + 2] = pbuffer[idx];
  }
  write(buffer, 2 + num_byte_to_write, true);
  return 0;
}

VL53L1X_Error VL53L1X::vl53l1x_i2c_read(uint16_t register_addr, uint8_t *pbuffer, uint16_t num_byte_to_read) {
  int status = 0;
  do {
    uint8_t buffer[2];
    buffer[0] = (uint8_t) (register_addr >> 8);
    buffer[1] = (uint8_t) (register_addr & 0xFF);
    status = write(buffer, 2, false);
  } while (status != 0);
  read(pbuffer, num_byte_to_read);

  return 0;
}

VL53L1X_Error VL53L1X::vl53l1x_get_tick_count(uint32_t *ptick_count_ms) {
  /* Returns current tick count in [ms] */
  VL53L1X_Error status = VL53L1X::VL53L1X_ERROR_NONE;
  //*ptick_count_ms = timeGetTime();
  *ptick_count_ms = 0;

  return status;
}

VL53L1X_Error VL53L1X::vl53l1x_wait_us(int32_t wait_us) {
  delay(wait_us / 1000);
  return VL53L1X::VL53L1X_ERROR_NONE;
}

VL53L1X_Error VL53L1X::vl53l1x_wait_ms(int32_t wait_ms) {
  delay(wait_ms);
  return VL53L1X::VL53L1X_ERROR_NONE;
}

VL53L1X_Error VL53L1X::vl53l1x_wait_value_mask_ex(uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask,
                                                  uint32_t poll_delay_ms) {
  VL53L1X_Error status = VL53L1X::VL53L1X_ERROR_NONE;
  uint32_t start_time_ms = 0;
  uint32_t current_time_ms = 0;
  uint32_t polling_time_ms = 0;
  uint8_t byte_value = 0;
  uint8_t found = 0;

  /* calculate time limit in absolute time */
  this->vl53l1x_get_tick_count(&start_time_ms);

  /* wait until value is found, timeout reached on error occurred */
  while ((status == VL53L1X::VL53L1X_ERROR_NONE) && (polling_time_ms < timeout_ms) && (found == 0)) {
    if (status == VL53L1X::VL53L1X_ERROR_NONE) {
      status = this->vl53l1x_rd_byte(index, &byte_value);
    }

    if ((byte_value & mask) == value) {
      found = 1;
    }

    if (status == VL53L1X::VL53L1X_ERROR_NONE && found == 0 && poll_delay_ms > 0) {
      status = this->vl53l1x_wait_ms(poll_delay_ms);
    }

    /* Update polling time (Compare difference rather than absolute to
     negate 32bit wrap around issue) */
    this->vl53l1x_get_tick_count(&current_time_ms);
    polling_time_ms = current_time_ms - start_time_ms;
  }

  if (found == 0 && status == VL53L1X::VL53L1X_ERROR_NONE) {
    status = VL53L1X::VL53L1X_ERROR_TIME_OUT;
  }

  return status;
}

}  // namespace vl53l1x
}  // namespace esphome
