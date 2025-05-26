#include "roode.h"

namespace esphome {
namespace roode {
void Roode::dump_config() {
  ESP_LOGCONFIG(TAG, "Roode:");
  ESP_LOGCONFIG(TAG, "  Sample size: %d", samples);
  LOG_UPDATE_INTERVAL(this);
  entry->dump_config();
  exit->dump_config();
}

void Roode::setup() {
  ESP_LOGI(SETUP, "Booting Roode %s", VERSION);
  if (version_sensor != nullptr) {
    version_sensor->publish_state(VERSION);
  }
  ESP_LOGI(SETUP, "Using sampling with sampling size: %d", samples);

  if (this->distanceSensor->is_failed()) {
    this->mark_failed();
    ESP_LOGE(TAG, "Roode cannot be setup without a valid VL53L1X sensor");
    return;
  }

  calibrate_zones();
}

void Roode::update() {
  if (distance_entry != nullptr) {
    distance_entry->publish_state(entry->getDistance());
  }
  if (distance_exit != nullptr) {
    distance_exit->publish_state(exit->getDistance());
  }
}

void Roode::loop() {
  // unsigned long start = micros();
  this->current_zone->readDistance(distanceSensor);
  // uint16_t samplingDistance = sampling(this->current_zone);
  path_tracking(this->current_zone);
  handle_sensor_status();
  this->current_zone = this->current_zone == this->entry ? this->exit : this->entry;
  // ESP_LOGI("Experimental", "Entry zone: %d, exit zone: %d",
  // entry->getDistance(Roode::distanceSensor, Roode::sensor_status),
  // exit->getDistance(Roode::distanceSensor, Roode::sensor_status)); unsigned
  // long end = micros(); unsigned long delta = end - start; ESP_LOGI("Roode
  // loop", "loop took %lu microseconds", delta);
}

bool Roode::handle_sensor_status() {
  bool check_status = false;
  if (last_sensor_status != sensor_status && sensor_status == VL53L1_ERROR_NONE) {
    if (status_sensor != nullptr) {
      status_sensor->publish_state(sensor_status);
    }
    check_status = true;
  }
  if (sensor_status < 28 && sensor_status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Ranging failed with an error. status: %d", sensor_status);
    status_sensor->publish_state(sensor_status);
    check_status = false;
  }

  last_sensor_status = sensor_status;
  sensor_status = VL53L1_ERROR_NONE;
  return check_status;
}

void Roode::path_tracking(Zone *zone) {
  static int PathTrack[] = {0, 0, 0, 0};
  static int PathTrackFillingSize = 1;  // init this to 1 as we start from state
                                        // where nobody is any of the zones
  static int LeftPreviousStatus = NOBODY;
  static int RightPreviousStatus = NOBODY;
  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

  // PathTrack algorithm
  if (zone->getMinDistance() < zone->threshold->max && zone->getMinDistance() > zone->threshold->min) {
    // Someone is in the sensing area
    CurrentZoneStatus = SOMEONE;
    if (presence_sensor != nullptr) {
      presence_sensor->publish_state(true);
    }
  }

  // left zone
  if (zone == (this->invert_direction_ ? this->exit : this->entry)) {
    if (CurrentZoneStatus != LeftPreviousStatus) {
      // event in left zone has occured
      AnEventHasOccured = 1;

      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE) {
        // event in right zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  }
  // right zone
  else {
    if (CurrentZoneStatus != RightPreviousStatus) {
      // event in right zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
      }
      // need to check left zone as well ...
      if (LeftPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }

  // if an event has occured
  if (AnEventHasOccured) {
    ESP_LOGD(TAG, "Event has occured, AllZonesCurrentStatus: %d", AllZonesCurrentStatus);
    if (PathTrackFillingSize < 4) {
      PathTrackFillingSize++;
    }

    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {
      ESP_LOGD(TAG, "Nobody anywhere, AllZonesCurrentStatus: %d", AllZonesCurrentStatus);
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1
      // 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is
        // always the case

        if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          // This an exit
          ESP_LOGI("Roode pathTracking", "Exit detected.");

          this->updateCounter(-1);
          if (entry_exit_event_sensor != nullptr) {
            entry_exit_event_sensor->publish_state("Exit");
          }
        } else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an entry
          ESP_LOGI("Roode pathTracking", "Entry detected.");
          this->updateCounter(1);
          if (entry_exit_event_sensor != nullptr) {
            entry_exit_event_sensor->publish_state("Entry");
          }
        }
      }

      PathTrackFillingSize = 1;
    } else {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      PathTrack[PathTrackFillingSize - 1] = AllZonesCurrentStatus;
    }
  }
  if (presence_sensor != nullptr) {
    if (CurrentZoneStatus == NOBODY && LeftPreviousStatus == NOBODY && RightPreviousStatus == NOBODY) {
      // nobody is in the sensing area
      presence_sensor->publish_state(false);
    }
  }
}
void Roode::updateCounter(int delta) {
  if (this->people_counter == nullptr) {
    return;
  }
  auto next = this->people_counter->state + (float) delta;
  ESP_LOGI(TAG, "Updating people count: %d", (int) next);
  auto call = this->people_counter->make_call();
  call.set_value(next);
  call.perform();
}
void Roode::recalibration() { calibrate_zones(); }

const RangingMode *Roode::determine_raning_mode(uint16_t average_entry_zone_distance,
                                                uint16_t average_exit_zone_distance) {
  uint16_t min = average_entry_zone_distance < average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  uint16_t max = average_entry_zone_distance > average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  if (min <= short_distance_threshold) {
    return Ranging::Short;
  }
  if (max > short_distance_threshold && min <= medium_distance_threshold) {
    return Ranging::Medium;
  }
  if (max > medium_distance_threshold && min <= medium_long_distance_threshold) {
    return Ranging::Long;
  }
  if (max > medium_long_distance_threshold && min <= long_distance_threshold) {
    return Ranging::Longer;
  }
  return Ranging::Longest;
}

void Roode::calibrate_zones() {
  ESP_LOGI(SETUP, "Calibrating sensor zones");

  entry->reset_roi(orientation_ == Parallel ? 167 : 195);
  exit->reset_roi(orientation_ == Parallel ? 231 : 60);

  calibrateDistance();

  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  entry->calibrateThreshold(distanceSensor, number_attempts);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->calibrateThreshold(distanceSensor, number_attempts);

  publish_sensor_configuration(entry, exit, true);
  App.feed_wdt();
  publish_sensor_configuration(entry, exit, false);
  ESP_LOGI(SETUP, "Finished calibrating sensor zones");
}

void Roode::calibrateDistance() {
  auto *const initial = distanceSensor->get_ranging_mode_override().value_or(Ranging::Longest);
  distanceSensor->set_ranging_mode(initial);

  entry->calibrateThreshold(distanceSensor, number_attempts);
  exit->calibrateThreshold(distanceSensor, number_attempts);

  if (distanceSensor->get_ranging_mode_override().has_value()) {
    return;
  }
  auto *mode = determine_raning_mode(entry->threshold->idle, exit->threshold->idle);
  if (mode != initial) {
    distanceSensor->set_ranging_mode(mode);
  }
}

void Roode::publish_sensor_configuration(Zone *entry, Zone *exit, bool isMax) {
  if (isMax) {
    if (max_threshold_entry_sensor != nullptr) {
      max_threshold_entry_sensor->publish_state(entry->threshold->max);
    }

    if (max_threshold_exit_sensor != nullptr) {
      max_threshold_exit_sensor->publish_state(exit->threshold->max);
    }
  } else {
    if (min_threshold_entry_sensor != nullptr) {
      min_threshold_entry_sensor->publish_state(entry->threshold->min);
    }
    if (min_threshold_exit_sensor != nullptr) {
      min_threshold_exit_sensor->publish_state(exit->threshold->min);
    }
  }

  if (entry_roi_height_sensor != nullptr) {
    entry_roi_height_sensor->publish_state(entry->roi->height);
  }
  if (entry_roi_width_sensor != nullptr) {
    entry_roi_width_sensor->publish_state(entry->roi->width);
  }

  if (exit_roi_height_sensor != nullptr) {
    exit_roi_height_sensor->publish_state(exit->roi->height);
  }
  if (exit_roi_width_sensor != nullptr) {
    exit_roi_width_sensor->publish_state(exit->roi->width);
  }
}

// Service method implementations
void Roode::set_sampling_size_service(uint8_t size) {
  if (size > 0 && size <= 10) {
    samples = size;
    entry->set_max_samples(size);
    exit->set_max_samples(size);
    ESP_LOGI(TAG, "Sampling size set to: %d", size);
  } else {
    ESP_LOGW(TAG, "Invalid sampling size: %d. Must be between 1 and 10", size);
  }
}

void Roode::set_max_threshold_percentage(uint8_t percentage) {
  entry->threshold->set_max_percentage(percentage);
  exit->threshold->set_max_percentage(percentage);
  entry->threshold->max = (entry->threshold->idle * percentage) / 100;
  exit->threshold->max = (exit->threshold->idle * percentage) / 100;
  ESP_LOGI(TAG, "Max threshold percentage set to: %d%%", percentage);
}

void Roode::set_min_threshold_percentage(uint8_t percentage) {
  entry->threshold->set_min_percentage(percentage);
  exit->threshold->set_min_percentage(percentage);
  entry->threshold->min = (entry->threshold->idle * percentage) / 100;
  exit->threshold->min = (exit->threshold->idle * percentage) / 100;
  ESP_LOGI(TAG, "Min threshold percentage set to: %d%%", percentage);
}

void Roode::set_max_threshold_exact(uint16_t threshold_mm) {
  entry->threshold->set_max(threshold_mm);
  exit->threshold->set_max(threshold_mm);
  ESP_LOGI(TAG, "Max threshold set to: %dmm", threshold_mm);
}

void Roode::set_min_threshold_exact(uint16_t threshold_mm) {
  entry->threshold->set_min(threshold_mm);
  exit->threshold->set_min(threshold_mm);
  ESP_LOGI(TAG, "Min threshold set to: %dmm", threshold_mm);
}

void Roode::set_entry_max_threshold_percentage(uint8_t percentage) {
  entry->threshold->set_max_percentage(percentage);
  entry->threshold->max = (entry->threshold->idle * percentage) / 100;
  ESP_LOGI(TAG, "Entry max threshold percentage set to: %d%%", percentage);
}

void Roode::set_entry_min_threshold_percentage(uint8_t percentage) {
  entry->threshold->set_min_percentage(percentage);
  entry->threshold->min = (entry->threshold->idle * percentage) / 100;
  ESP_LOGI(TAG, "Entry min threshold percentage set to: %d%%", percentage);
}

void Roode::set_exit_max_threshold_percentage(uint8_t percentage) {
  exit->threshold->set_max_percentage(percentage);
  exit->threshold->max = (exit->threshold->idle * percentage) / 100;
  ESP_LOGI(TAG, "Exit max threshold percentage set to: %d%%", percentage);
}

void Roode::set_exit_min_threshold_percentage(uint8_t percentage) {
  exit->threshold->set_min_percentage(percentage);
  exit->threshold->min = (exit->threshold->idle * percentage) / 100;
  ESP_LOGI(TAG, "Exit min threshold percentage set to: %d%%", percentage);
}

void Roode::set_entry_max_threshold_exact(uint16_t threshold_mm) {
  entry->threshold->set_max(threshold_mm);
  ESP_LOGI(TAG, "Entry max threshold set to: %dmm", threshold_mm);
}

void Roode::set_entry_min_threshold_exact(uint16_t threshold_mm) {
  entry->threshold->set_min(threshold_mm);
  ESP_LOGI(TAG, "Entry min threshold set to: %dmm", threshold_mm);
}

void Roode::set_exit_max_threshold_exact(uint16_t threshold_mm) {
  exit->threshold->set_max(threshold_mm);
  ESP_LOGI(TAG, "Exit max threshold set to: %dmm", threshold_mm);
}

void Roode::set_exit_min_threshold_exact(uint16_t threshold_mm) {
  exit->threshold->set_min(threshold_mm);
  ESP_LOGI(TAG, "Exit min threshold set to: %dmm", threshold_mm);
}

void Roode::set_roi_size(uint8_t width, uint8_t height) {
  set_entry_roi_size(width, height);
  set_exit_roi_size(width, height);
  ESP_LOGI(TAG, "ROI size set to: %dx%d", width, height);
}

void Roode::set_entry_roi_size(uint8_t width, uint8_t height) {
  entry->roi_override->width = width;
  entry->roi_override->height = height;
  entry->roi->width = width;
  entry->roi->height = height;
  ESP_LOGI(TAG, "Entry ROI size set to: %dx%d", width, height);
}

void Roode::set_exit_roi_size(uint8_t width, uint8_t height) {
  exit->roi_override->width = width;
  exit->roi_override->height = height;
  exit->roi->width = width;
  exit->roi->height = height;
  ESP_LOGI(TAG, "Exit ROI size set to: %dx%d", width, height);
}

void Roode::set_entry_roi_center(uint8_t center) {
  entry->roi_override->center = center;
  entry->roi->center = center;
  ESP_LOGI(TAG, "Entry ROI center set to: %d", center);
}

void Roode::set_exit_roi_center(uint8_t center) {
  exit->roi_override->center = center;
  exit->roi->center = center;
  ESP_LOGI(TAG, "Exit ROI center set to: %d", center);
}

void Roode::set_calibration_attempts(int attempts) {
  if (attempts > 0 && attempts <= 100) {
    number_attempts = attempts;
    ESP_LOGI(TAG, "Calibration attempts set to: %d", attempts);
  } else {
    ESP_LOGW(TAG, "Invalid calibration attempts: %d. Must be between 1 and 100", attempts);
  }
}

void Roode::reset_people_counter() {
  if (people_counter != nullptr) {
    auto call = people_counter->make_call();
    call.set_value(0);
    call.perform();
    ESP_LOGI(TAG, "People counter reset to 0");
  }
}

void Roode::adjust_people_counter(int adjustment) {
  if (people_counter != nullptr) {
    auto next = people_counter->state + (float) adjustment;
    auto call = people_counter->make_call();
    call.set_value(next);
    call.perform();
    ESP_LOGI(TAG, "People counter adjusted by %d to %d", adjustment, (int) next);
  }
}

// Getter method implementations for percentage calculations
uint8_t Roode::get_entry_max_threshold_percentage() const {
  if (entry->threshold->max_percentage.has_value()) {
    return entry->threshold->max_percentage.value();
  }
  return (entry->threshold->max * 100) / entry->threshold->idle;
}

uint8_t Roode::get_entry_min_threshold_percentage() const {
  if (entry->threshold->min_percentage.has_value()) {
    return entry->threshold->min_percentage.value();
  }
  return (entry->threshold->min * 100) / entry->threshold->idle;
}

uint8_t Roode::get_exit_max_threshold_percentage() const {
  if (exit->threshold->max_percentage.has_value()) {
    return exit->threshold->max_percentage.value();
  }
  return (exit->threshold->max * 100) / exit->threshold->idle;
}

uint8_t Roode::get_exit_min_threshold_percentage() const {
  if (exit->threshold->min_percentage.has_value()) {
    return exit->threshold->min_percentage.value();
  }
  return (exit->threshold->min * 100) / exit->threshold->idle;
}
}  // namespace roode
}  // namespace esphome