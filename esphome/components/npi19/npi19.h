#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace npi19 {

/// This class implements support for the npi19 pressure and temperature i2c sensors.
class NPI19Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_pressure_sensor(sensor::Sensor *pressure_sensor) { pressure_sensor_ = pressure_sensor; }

  float get_setup_priority() const override;
  void setup() override;
  void dump_config() override;
  void update() override;

 protected:
  i2c::ErrorCode read_(uint16_t &temperature_raw, uint16_t &pressure_raw);
  float convert_temperature_(uint16_t temperature_raw);
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *pressure_sensor_{nullptr};
};

}  // namespace npi19
}  // namespace esphome
