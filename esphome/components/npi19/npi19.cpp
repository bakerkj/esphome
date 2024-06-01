#include "npi19.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

// https://www.amphenol-sensors.com/hubfs/I2C%20NPI-19%20product%20application%20Note.pdf
// https://forum.arduino.cc/t/code-help-to-receive-data-from-i2c-vacuum-pressure-sensor/1025416/14

namespace esphome {
namespace npi19 {

static const char *const TAG = "npi19";

static const uint8_t READ_COMMAND = 0xAC;

void NPI19Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up NPI19...");

  uint16_t temperature_raw(0);
  uint16_t pressure_raw(0);

  // startup device delay
  delay(10);

  i2c::ErrorCode err = this->read_(temperature_raw, pressure_raw);
  if (err != i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "    I2C Communication Failed...");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "    Success...");
}

void NPI19Component::dump_config() {
  ESP_LOGCONFIG(TAG, "NPI19:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
}

float NPI19Component::get_setup_priority() const { return setup_priority::DATA; }

i2c::ErrorCode NPI19Component::read_(uint16_t &temperature_raw, uint16_t &pressure_raw) {
  i2c::ErrorCode w_err = write(&READ_COMMAND, sizeof(READ_COMMAND), true);
  if (w_err != i2c::ERROR_OK) {
    return w_err;
  }

  uint8_t response[4] = {0x00, 0x00, 0x00, 0x00};

  // read 4 bytes from senesor
  i2c::ErrorCode r_err = this->read(response, 4);

  if (r_err != i2c::ERROR_OK) {
    return r_err;
  }

  // extract top 6 bits of first byte and all bits of second byte for pressure
  pressure_raw = ((response[0] & 0x3F) << 8) | (response[1] & 0xFF);

  // extract all bytes of 3rd byte and top 3 bits of fourth byte for temperature
  temperature_raw = (((uint16_t) response[2] & 0xFF) << 3) | ((response[3] & 0xE0) >> 5);

  return r_err;
}

float NPI19Component::convert_temperature_(uint16_t temperature_raw) {
  /*
   * Correspondance with Amphenol confirmed the appropriate equation for computing temperature is:
   * T (°C) =(((((Th*8)+Tl)/2048)*200)-50), where Th is the high (third) byte and Tl is the low (fourth) byte.
   *
   * Tl is actually the upper 3 bits of the fourth data byte; the first 5 (LSBs) must be masked out.
   *
   * Note that although the NPI-19 I2C Series has a temperature output, we don’t specify its accuracy on
   * the published data sheet.  For this reason, the sensor should not be used as a calibrated temperature
   * reading; it’s only intended for curve fitting data during compensation.
   */
  const float temperature_bits_span_ = 2048;
  const float temperature_max_ = 150;
  const float temperature_min_ = -50;
  const float temperature_span_ = temperature_max_ - temperature_min_;

  float temperature = (temperature_raw * temperature_span_ / temperature_bits_span_) + temperature_min_;

  return temperature;
}

void NPI19Component::update() {
  this->set_timeout(50, [this]() {
    uint16_t temperature_raw(0);
    uint16_t pressure_raw(0);

    i2c::ErrorCode err = this->read_(temperature_raw, pressure_raw);

    if (err != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "I2C Communication Failed");
      this->status_set_warning();
      return;
    }

    float temperature = convert_temperature_(temperature_raw);

    ESP_LOGD(TAG, "Got temperature=%.1f°C  pressure=%draw", temperature, pressure_raw);

    if (this->temperature_sensor_ != nullptr)
      this->temperature_sensor_->publish_state(temperature);
    if (this->pressure_sensor_ != nullptr)
      this->pressure_sensor_->publish_state(pressure_raw);

    this->status_clear_warning();
  });
}

}  // namespace npi19
}  // namespace esphome
