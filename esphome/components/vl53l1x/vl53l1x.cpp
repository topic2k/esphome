/*
 * Most of the code in this integration is based on the Adafruit_VL53L1X library
 * by Adafruit Industries, which in turn is based on
 * the VL53L1X API provided by ST (STSW-IMG007).
 *
 * For more information about licensing, please view the included LICENSE.txt file
 * in the vl53l1x integration directory.
 */

#include "vl53l1x.h"

namespace esphome {
namespace vl53l1x {

std::list<VL53L1XComponent *>
    VL53L1XComponent::vl53l1x_sensors;                     // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
bool VL53L1XComponent::enable_pin_setup_complete = false;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

VL53L1XComponent::VL53L1XComponent() { VL53L1XComponent::vl53l1x_sensors.push_back(this); }

void VL53L1XComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "VL53L1X config:");
  LOG_UPDATE_INTERVAL(this);
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_PIN("  IRQ Pin: ", this->irq_pin_);
  ESP_LOGCONFIG(TAG, "  Model 0x%X", this->get_sensor_id());
  ESP_LOGCONFIG(TAG, "  io_2v8: %s", this->io_2v8_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  long_range: %s", this->long_range_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  timing_budget: %i", this->timing_budget_);
  ESP_LOGCONFIG(TAG, "  offset: %i", this->offset_);
  if (this->s_distance_ != nullptr) {
    LOG_SENSOR("", "distance sensor", this->s_distance_);
  }
  if (this->s_threshold_ != nullptr) {
    LOG_SENSOR("", "threshold sensor", this->s_threshold_);
    ESP_LOGCONFIG(TAG, "  threshold low: %i mm", this->threshold_low_);
    ESP_LOGCONFIG(TAG, "  threshold high: %i mm", this->threshold_high_);
    ESP_LOGCONFIG(TAG, "  threshold mode: %i", this->threshold_mode_);
  }
}

void VL53L1XComponent::setup() {
  if (!esphome::vl53l1x::VL53L1XComponent::enable_pin_setup_complete) {
    for (auto &vl53_sensor : vl53l1x_sensors) {
      if (vl53_sensor->enable_pin_ != nullptr) {
        // Set enable pin as OUTPUT and disable the enable pin to force vl53 to HW Standby mode
        vl53_sensor->enable_pin_->setup();
        vl53_sensor->enable_pin_->pin_mode(gpio::FLAG_OUTPUT);
        vl53_sensor->enable_pin_->digital_write(false);
      }
    }
    esphome::vl53l1x::VL53L1XComponent::enable_pin_setup_complete = true;
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
    ESP_LOGE(TAG, "'%s' - Sensor init failed", this->s_distance_->get_name().c_str());
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

  if (this->s_threshold_ != nullptr) {
    ESP_LOGD(TAG, "'%s' - setting threshold mode", this->s_threshold_->get_name().c_str());
    this->vl53l1x_set_distance_threshold(this->threshold_low_, this->threshold_high_, this->threshold_mode_, 1);
  }

  this->vl53l1x_set_roi(8, 16);

  if (!this->start_ranging()) {
    ESP_LOGE(TAG, "'%s' - Couldn't start ranging. Error code %i", this->s_distance_->get_name().c_str(),
             this->vl_status);
    this->mark_failed();
  }
}

void VL53L1XComponent::update() {
//  int16_t distance = 0;
//
//  if (this->is_data_ready()) {
//    // new measurement for the taking!
//    distance = this->get_distance();
//    if (distance == -1) {
//      // something went wrong!
//      ESP_LOGE(TAG, "Couldn't get get_distance. Error code %i", this->vl_status);
//      return;
//    }
//    // data is read out, time for another reading!
//    this->clear_interrupt();
//  }
//  if (this->s_distance_ != nullptr) {
//    this->s_distance_->publish_state(distance);
//  }
//
//  if (this->s_threshold_ != nullptr) {
//    this->s_threshold_->publish_state((bool) distance != 0);
//  }
}

#define NOBODY                    0
#define SOMEONE                   1
#define LEFT                      0
#define RIGHT                     1
#define DISTANCES_ARRAY_SIZE                         10   // nb of samples
#define DIST_THRESHOLD                               1600  // mm
//int MAX_DISTANCE = 2400; // mm
//int MIN_DISTANCE = 0; // mm
//int FRONT_ZONE_CENTER = 175; // was 167, see UM2555 on st.com, centre = 175 has better return signal rate for the ROI #1
//int BACK_ZONE_CENTER = 231;

void VL53L1XComponent::loop() {
  if (!this->is_data_ready()) {
    return;
  }

  VL53L1X_Error status = 0;
  int8_t error;
  uint8_t byteData, sensorState=0;
  uint16_t wordData;
  uint16_t Distance, Signal;
  uint8_t range_status;
  int PplCounter;
  int MAX_DISTANCE = 1000; // mm
  int MIN_DISTANCE = 0; // mm
  int FRONT_ZONE_CENTER = 175; // was 167, see UM2555 on st.com, centre = 175 has better return signal rate for the ROI #1
  int BACK_ZONE_CENTER = 231;
  int center[2] = {FRONT_ZONE_CENTER, BACK_ZONE_CENTER}; /* these are the spad center of the 2 4*16 zones */
  int Zone = 0;

  status += vl53l1x_get_range_status(&range_status);
  status += vl53l1x_get_distance(&Distance);
  status += vl53l1x_get_signal_per_spad(&Signal);
  status += vl53l1x_clear_interrupt(); /* clear interrupt has to be called to enable next interrupt*/
  if (status != 0) {
    ESP_LOGE(TAG, "Error in operating the device");
    return;
  }
  status = vl53l1x_set_roi_center(center[Zone]);
  if (status != 0) {
    ESP_LOGE(TAG, "Error in chaning the center of the ROI");
    return;
  }
  // check the status of the ranging. In case of error, lets assume the distance is the max of the use case
  // Value RangeStatus string Comment
  // 0 VL53L1_RANGESTATUS_RANGE_VALID Ranging measurement is valid
  // 1 VL53L1_RANGESTATUS_SIGMA_FAIL Raised if sigma estimator check is above the internal defined threshold
  // 2 VL53L1_RANGESTATUS_SIGNAL_FAIL Raised if signal value is below the internal defined threshold
  // 4 VL53L1_RANGESTATUS_OUTOFBOUNDS_ FAIL Raised when phase is out of bounds
  // 5 VL53L1_RANGESTATUS_HARDWARE_FAIL Raised in case of HW or VCSEL failure
  // 7 VL53L1_RANGESTATUS_WRAP_TARGET_ FAIL Wrapped target, not matching phases
  // 8 VL53L1_RANGESTATUS_PROCESSING_ FAIL Internal algorithm underflow or overflow
  // 14 VL53L1_RANGESTATUS_RANGE_INVALID The reported range is invalid
  if ((range_status == 0) || (range_status == 4) || (range_status == 7)) {
    if (Distance <= MIN_DISTANCE) // wraparound case see the explanation at the constants definition place
      Distance = MAX_DISTANCE + MIN_DISTANCE;
  }
  else {
    // severe error cases
    Distance = MAX_DISTANCE;
  }
  // inject the new ranged distance in the people counting algorithm
    PplCounter = ProcessPeopleCountingData(Distance, Zone, range_status);
    ESP_LOGD(TAG, "People: %i", PplCounter);
    ESP_LOGD(TAG, "Zone: %i, Distance: %i, Signal: %i", Zone, Distance, Signal);
    Zone++;
    Zone = Zone % 2;
}


int VL53L1XComponent::ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus) {
  static int PathTrack[] = {0,0,0,0};
  static int PathTrackFillingSize = 1; // init this to 1 as we start from state where nobody is any of the zones
  static int LeftPreviousStatus = NOBODY;
  static int RightPreviousStatus = NOBODY;
  static int PeopleCount = 0;
  static uint16_t Distances[2][DISTANCES_ARRAY_SIZE];
  static uint8_t DistancesTableSize[2] = {0,0};

  uint16_t MinDistance;
  uint8_t i;

#ifdef TRACE_PPC
#define TIMES_WITH_NO_EVENT 10// was 40
  static uint32_t trace_count = TIMES_WITH_NO_EVENT;  // replace by 0 if you want to trace the first TIMES_WITH_NO_EVENT values
#endif

  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

  // Add just picked distance to the table of the corresponding zone
  if (DistancesTableSize[zone] < DISTANCES_ARRAY_SIZE) {
    Distances[zone][DistancesTableSize[zone]] = Distance;
    DistancesTableSize[zone] ++;
  }
  else {
    for (i=1; i<DISTANCES_ARRAY_SIZE; i++)
      Distances[zone][i-1] = Distances[zone][i];
    Distances[zone][DISTANCES_ARRAY_SIZE-1] = Distance;
  }

  // pick up the min distance
  MinDistance = Distances[zone][0];
  if (DistancesTableSize[zone] >= 2) {
    for (i=1; i<DistancesTableSize[zone]; i++) {
      if (Distances[zone][i] < MinDistance)
        MinDistance = Distances[zone][i];
    }
  }

  if (MinDistance < DIST_THRESHOLD) {
    // Someone is in !
    CurrentZoneStatus = SOMEONE;
  }

  // left zone
  if (zone == LEFT) {
    if (CurrentZoneStatus != LeftPreviousStatus) {
      // event in left zone has occured
      AnEventHasOccured = 1;

      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  }
  // right zone
  else {

    if (CurrentZoneStatus != RightPreviousStatus) {

      // event in left zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
      }
      // need to left right zone as well ...
      if (LeftPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }

#ifdef TRACE_PPC
  // print debug data only when someone is within the field of view
  trace_count++;
  if ((CurrentZoneStatus == SOMEONE) || (LeftPreviousStatus == SOMEONE) || (RightPreviousStatus == SOMEONE))
    trace_count = 0;

  if (trace_count < TIMES_WITH_NO_EVENT)
//    printf ("%d,%d,%d,%d,%d\n", zone, Distance, MinDistance, RangeStatus, PeopleCount);
#endif

    // if an event has occured
    if (AnEventHasOccured) {
      if (PathTrackFillingSize < 4) {
        PathTrackFillingSize ++;
      }

      // if nobody anywhere lets check if an exit or entry has happened
      if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {

        // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
        if (PathTrackFillingSize == 4) {
          // check exit or entry. no need to check PathTrack[0] == 0 , it is always the case

          if ((PathTrack[1] == 1)  && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
            // This an entry
            PeopleCount ++;
            // reset the table filling size in case an entry or exit just found
            DistancesTableSize[0] = 0;
            DistancesTableSize[1] = 0;
            ESP_LOGD(TAG, "Walk In, People Count = %i", PeopleCount);
          } else if ((PathTrack[1] == 2)  && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
            // This an exit
            PeopleCount --;
            // reset the table filling size in case an entry or exit just found
            DistancesTableSize[0] = 0;
            DistancesTableSize[1] = 0;
            ESP_LOGD(TAG, "Walk Out, People Count = %i", PeopleCount);
          } else {
            // reset the table filling size also in case of unexpected path
            DistancesTableSize[0] = 0;
            DistancesTableSize[1] = 0;
            ESP_LOGD(TAG, "Wrong path");
          }
        }

        PathTrackFillingSize = 1;
      }
      else {
        // update PathTrack
        // example of PathTrack update
        // 0
        // 0 1
        // 0 1 3
        // 0 1 3 1
        // 0 1 3 3
        // 0 1 3 2 ==> if next is 0 : check if exit
        PathTrack[PathTrackFillingSize-1] = AllZonesCurrentStatus;
      }

      //#ifdef TRACE_PPC
      //    if (AnEventHasOccured) {
      //      for (int j=0; j<PathTrackFillingSize; j++)
      //        printf ("%d ", PathTrack[j]);
      //    }
      //    printf("\n");
      //#endif
    }

  // output debug data to main host machine
  return(PeopleCount);
}


}  // namespace vl53l1x
}  // namespace esphome
