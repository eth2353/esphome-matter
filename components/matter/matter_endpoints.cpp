#include "esphome/core/defines.h"
#ifdef USE_MATTER

#include "esphome/core/log.h"
#include "matter_actions.h"
#include "matter_component.h"

#include <cmath>

static const char *const TAG = "matter";

namespace esphome::matter {

bool MatterComponent::create_endpoints_(esp_matter::node_t *node) {
  for (auto &sw : this->on_off_switches_) {
    esp_matter::endpoint::on_off_light_switch::config_t sw_config;
    esp_matter::endpoint_t *ep =
        esp_matter::endpoint::on_off_light_switch::create(
            node, &sw_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (ep == nullptr) {
      ESP_LOGE(TAG, "Failed to create on_off_switch endpoint");
      return false;
    }
    sw.endpoint_id = esp_matter::endpoint::get_id(ep);
    sw.ref->endpoint_id = sw.endpoint_id;
    ESP_LOGD(TAG, "On/Off switch endpoint created: id=%u", sw.endpoint_id);
  }

  for (auto &sw : this->dimmer_switches_) {
    esp_matter::endpoint::dimmer_switch::config_t sw_config;
    esp_matter::endpoint_t *ep = esp_matter::endpoint::dimmer_switch::create(
        node, &sw_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (ep == nullptr) {
      ESP_LOGE(TAG, "Failed to create dimmer_switch endpoint");
      return false;
    }
    sw.endpoint_id = esp_matter::endpoint::get_id(ep);
    sw.ref->endpoint_id = sw.endpoint_id;
    ESP_LOGD(TAG, "Dimmer switch endpoint created: id=%u", sw.endpoint_id);
  }

#ifdef USE_SENSOR
  for (auto &ts : this->temperature_sensors_) {
    esp_matter::endpoint::temperature_sensor::config_t ts_config;
    esp_matter::endpoint_t *ep =
        esp_matter::endpoint::temperature_sensor::create(
            node, &ts_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (ep == nullptr) {
      ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint");
      return false;
    }
    ts.endpoint_id = esp_matter::endpoint::get_id(ep);
    ts.ref->endpoint_id = ts.endpoint_id;
    ESP_LOGD(TAG, "Temperature sensor endpoint created: id=%u", ts.endpoint_id);
  }
#endif

#ifdef USE_LIGHT
  for (auto *ml : this->lights_) {
    esp_matter::endpoint_t *ep = nullptr;
    if (ml->dimmable) {
      esp_matter::endpoint::dimmable_light::config_t light_config;
      ep = esp_matter::endpoint::dimmable_light::create(
          node, &light_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    } else {
      esp_matter::endpoint::on_off_light::config_t light_config;
      ep = esp_matter::endpoint::on_off_light::create(
          node, &light_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    }
    if (ep == nullptr) {
      ESP_LOGE(TAG, "Failed to create %s endpoint",
               ml->dimmable ? "dimmable_light" : "on_off_light");
      return false;
    }
    ml->endpoint_id = esp_matter::endpoint::get_id(ep);
    ml->ref->endpoint_id = ml->endpoint_id;
    ESP_LOGD(TAG, "%s endpoint created: id=%u",
             ml->dimmable ? "Dimmable light" : "On/Off light", ml->endpoint_id);
  }
#endif

#ifdef USE_CLIMATE
  for (auto *mc : this->climates_) {
    auto traits = mc->climate->get_traits();

    esp_matter::endpoint::room_air_conditioner::config_t config;

    const bool supports_heat =
        traits.supports_mode(climate::CLIMATE_MODE_HEAT) ||
        traits.supports_mode(climate::CLIMATE_MODE_HEAT_COOL);

    const bool supports_cool =
        traits.supports_mode(climate::CLIMATE_MODE_COOL) ||
        traits.supports_mode(climate::CLIMATE_MODE_HEAT_COOL);

    const bool supports_auto =
        traits.supports_mode(climate::CLIMATE_MODE_AUTO) ||
        traits.supports_mode(climate::CLIMATE_MODE_HEAT_COOL);

    if (!supports_heat && !supports_cool) {
      ESP_LOGE(TAG, "Climate endpoint must support heating or cooling");
      return false;
    }

    const bool supports_fan_modes = traits.get_supports_fan_modes();

    ESP_LOGI(TAG, "Climate fan support: %s",
         YESNO(traits.get_supports_fan_modes()));

    ESP_LOGI(TAG, "Fan AUTO: %s",
             YESNO(traits.supports_fan_mode(climate::CLIMATE_FAN_AUTO)));

    ESP_LOGI(TAG, "Fan LOW: %s",
             YESNO(traits.supports_fan_mode(climate::CLIMATE_FAN_LOW)));

    ESP_LOGI(TAG, "Fan MEDIUM: %s",
             YESNO(traits.supports_fan_mode(climate::CLIMATE_FAN_MEDIUM)));

    ESP_LOGI(TAG, "Fan HIGH: %s",
             YESNO(traits.supports_fan_mode(climate::CLIMATE_FAN_HIGH)));

    // Matter ControlSequenceOfOperation
    //
    // 0 = Cooling Only
    // 2 = Heating Only
    // 4 = Cooling and Heating
    if (supports_heat && supports_cool) {
      config.thermostat.control_sequence_of_operation = 4;
    } else if (supports_heat) {
      config.thermostat.control_sequence_of_operation = 2;
    } else {
      config.thermostat.control_sequence_of_operation = 0;
    }

    // Features MUST be configured before thermostat::create().
    config.thermostat.feature_flags = 0;

    if (supports_heat) {
      config.thermostat.feature_flags |=
          esp_matter::cluster::thermostat::feature::heating::get_id();

      if (!std::isnan(mc->climate->target_temperature)) {
        config.thermostat.features.heating.occupied_heating_setpoint =
            static_cast<int16_t>(
                std::lroundf(mc->climate->target_temperature * 100.0f));
      }
    }

    if (supports_cool) {
      config.thermostat.feature_flags |=
          esp_matter::cluster::thermostat::feature::cooling::get_id();

      if (!std::isnan(mc->climate->target_temperature)) {
        config.thermostat.features.cooling.occupied_cooling_setpoint =
            static_cast<int16_t>(
                std::lroundf(mc->climate->target_temperature * 100.0f));
      }
    }

    // Auto requires both Heating and Cooling.
    if (supports_auto && supports_heat && supports_cool) {
      config.thermostat.feature_flags |=
          esp_matter::cluster::thermostat::feature::auto_mode::get_id();
    }

    // Initial current temperature.
    if (!std::isnan(mc->climate->current_temperature)) {
      config.thermostat.local_temperature =
          nullable<int16_t>(static_cast<int16_t>(
              std::lroundf(mc->climate->current_temperature * 100.0f)));
    }

    ESP_LOGD(
        TAG,
        "Creating thermostat: heat=%s cool=%s auto=%s features=0x%08" PRIX32,
        YESNO(supports_heat), YESNO(supports_cool), YESNO(supports_auto),
        config.thermostat.feature_flags);

    esp_matter::endpoint_t *ep =
        esp_matter::endpoint::room_air_conditioner::create(
            node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);

    if (ep == nullptr) {
      ESP_LOGE(TAG, "Failed to create Room Air Conditioner endpoint");
      return false;
    }

    if (supports_fan_modes) {
      esp_matter::cluster::fan_control::config_t fan_config;

      const bool fan_auto = traits.supports_fan_mode(climate::CLIMATE_FAN_AUTO);

      const bool fan_low = traits.supports_fan_mode(climate::CLIMATE_FAN_LOW);

      const bool fan_medium =
          traits.supports_fan_mode(climate::CLIMATE_FAN_MEDIUM);

      const bool fan_high = traits.supports_fan_mode(climate::CLIMATE_FAN_HIGH);

      ESP_LOGD(TAG, "Fan modes: auto=%s low=%s medium=%s high=%s",
               YESNO(fan_auto), YESNO(fan_low), YESNO(fan_medium),
               YESNO(fan_high));

      //
      // Matter FanModeSequence:
      //
      // 0 = Off / Low / Medium / High
      // 1 = Off / Low / High
      // 2 = Off / Low / Medium / High / Auto
      // 3 = Off / Low / High / Auto
      // 4 = Off / High / Auto
      // 5 = Off / High
      //
      if (fan_low && fan_medium && fan_high && fan_auto) {
        fan_config.fan_mode_sequence = 2;
      } else if (fan_low && fan_high && fan_auto) {
        fan_config.fan_mode_sequence = 3;
      } else if (fan_high && fan_auto) {
        fan_config.fan_mode_sequence = 4;
      } else if (fan_low && fan_medium && fan_high) {
        fan_config.fan_mode_sequence = 0;
      } else if (fan_low && fan_high) {
        fan_config.fan_mode_sequence = 1;
      } else {
        fan_config.fan_mode_sequence = 5;
      }

      //
      // Initial FanMode.
      //
      // Matter FanMode:
      // 0 Off
      // 1 Low
      // 2 Medium
      // 3 High
      // 4 On
      // 5 Auto
      //
      if (mc->climate->fan_mode.has_value()) {
        switch (*mc->climate->fan_mode) {
        case climate::CLIMATE_FAN_LOW:
          fan_config.fan_mode = 1;
          break;

        case climate::CLIMATE_FAN_MEDIUM:
          fan_config.fan_mode = 2;
          break;

        case climate::CLIMATE_FAN_HIGH:
          fan_config.fan_mode = 3;
          break;

        case climate::CLIMATE_FAN_ON:
          fan_config.fan_mode = 4;
          break;

        case climate::CLIMATE_FAN_AUTO:
          fan_config.fan_mode = 5;
          break;

        default:
          fan_config.fan_mode = 5;
          break;
        }
      }

      auto *fan_cluster = esp_matter::cluster::fan_control::create(
          ep, &fan_config, esp_matter::CLUSTER_FLAG_SERVER);

      if (fan_cluster == nullptr) {
        ESP_LOGE(TAG, "Failed to create FanControl cluster");
        return false;
      }

      ESP_LOGI(
          TAG,
          "FanControl cluster added to native Room Air Conditioner endpoint");
    }

    mc->endpoint_id = esp_matter::endpoint::get_id(ep);
    mc->ref->endpoint_id = mc->endpoint_id;

    ESP_LOGD(TAG, "Room Air Conditioner endpoint created: id=%u", mc->endpoint_id);
  }
#endif

  register_client_request_callbacks();

  return true;
}

#ifdef USE_CLIMATE

static uint8_t climate_mode_to_matter_mode(climate::ClimateMode mode) {
  using Mode = chip::app::Clusters::Thermostat::SystemModeEnum;

  switch (mode) {
  case climate::CLIMATE_MODE_OFF:
    return chip::to_underlying(Mode::kOff);

  case climate::CLIMATE_MODE_COOL:
    return chip::to_underlying(Mode::kCool);

  case climate::CLIMATE_MODE_HEAT:
    return chip::to_underlying(Mode::kHeat);

  case climate::CLIMATE_MODE_AUTO:
  case climate::CLIMATE_MODE_HEAT_COOL:
    return chip::to_underlying(Mode::kAuto);

  case climate::CLIMATE_MODE_FAN_ONLY:
    return chip::to_underlying(Mode::kFanOnly);

  case climate::CLIMATE_MODE_DRY:
    return chip::to_underlying(Mode::kDry);

  default:
    return chip::to_underlying(Mode::kOff);
  }
}

static uint8_t climate_fan_mode_to_matter(climate::ClimateFanMode mode) {
  switch (mode) {
  case climate::CLIMATE_FAN_OFF:
    return 0;

  case climate::CLIMATE_FAN_LOW:
    return 1;

  case climate::CLIMATE_FAN_MEDIUM:
    return 2;

  case climate::CLIMATE_FAN_HIGH:
    return 3;

  case climate::CLIMATE_FAN_ON:
    return 4;

  case climate::CLIMATE_FAN_AUTO:
    return 5;

  default:
    // Modes such as QUIET/MIDDLE/etc. have no clean
    // basic Matter FanMode equivalent.
    return 5;
  }
}

void MatterClimate::push_state_to_matter() {
  const uint16_t eid = this->endpoint_id;

  ESP_LOGI(TAG, "push_state_to_matter: fan_mode has_value=%s",
           YESNO(this->climate->fan_mode.has_value()));

  if (this->climate->fan_mode.has_value()) {
    ESP_LOGI(TAG, "Current ESPHome fan mode=%u",
             static_cast<unsigned>(*this->climate->fan_mode));
  }

  ESP_LOGI(TAG, "Matter climate endpoint=%u fan_mode=%s",
           this->endpoint_id,
           this->climate->fan_mode.has_value() ? "present" : "missing");

  const float current_temperature = this->climate->current_temperature;

  const float target_temperature = this->climate->target_temperature;

  const auto mode = this->climate->mode;

  const uint8_t matter_mode = climate_mode_to_matter_mode(mode);

  const bool has_fan_mode = this->climate->fan_mode.has_value();

  const uint8_t matter_fan_mode =
      has_fan_mode ? climate_fan_mode_to_matter(*this->climate->fan_mode) : 0;

  chip::DeviceLayer::SystemLayer().ScheduleLambda([eid, current_temperature,

                                                   target_temperature, mode,
                                                   matter_mode, has_fan_mode,
                                                   matter_fan_mode]() {
    using namespace chip::app::Clusters;

    //
    // System mode
    //
    esp_matter_attr_val_t mode_val = esp_matter_enum8(matter_mode);

    esp_matter::attribute::update(
        eid, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &mode_val);

    //
    // Current temperature
    //
    const bool current_null = std::isnan(current_temperature);

    int16_t current_raw =
        current_null
            ? 0
            : static_cast<int16_t>(std::lroundf(current_temperature * 100.0f));

    esp_matter_attr_val_t current_val = esp_matter_nullable_int16(
        current_null ? nullable<int16_t>() : nullable<int16_t>(current_raw));

    esp_matter::attribute::update(eid, Thermostat::Id,
                                  Thermostat::Attributes::LocalTemperature::Id,
                                  &current_val);

    if (std::isnan(target_temperature))
      return;

    int16_t target_raw =
        static_cast<int16_t>(std::lroundf(target_temperature * 100.0f));

    esp_matter_attr_val_t target_val = esp_matter_int16(target_raw);

    if (mode == climate::CLIMATE_MODE_HEAT) {
      esp_matter::attribute::update(
          eid, Thermostat::Id,
          Thermostat::Attributes::OccupiedHeatingSetpoint::Id, &target_val);
    } else {
      //
      // COOL, AUTO and the usual AC modes use the
      // cooling setpoint for this first implementation.

      //
      esp_matter::attribute::update(
          eid, Thermostat::Id,
          Thermostat::Attributes::OccupiedCoolingSetpoint::Id, &target_val);
    }

    if (has_fan_mode) {
      esp_matter_attr_val_t fan_val = esp_matter_enum8(matter_fan_mode);

      esp_err_t fan_err = esp_matter::attribute::update(
          eid, FanControl::Id, FanControl::Attributes::FanMode::Id, &fan_val);

      ESP_LOGI(TAG,
               "FanControl push: endpoint=%u Matter=%u result=%s",
               eid, matter_fan_mode, esp_err_to_name(fan_err));
    }
  });
}

void MatterClimate::apply_matter_update(uint32_t cluster_id,
                                        uint32_t attribute_id,
                                        esp_matter_attr_val_t val) {

  using namespace chip::app::Clusters;

  if (cluster_id == Thermostat::Id) {
    //
    // HVAC mode
    //
    if (attribute_id == Thermostat::Attributes::SystemMode::Id) {

      auto matter_mode = static_cast<Thermostat::SystemModeEnum>(val.val.u8);

      climate::ClimateMode new_mode;

      switch (matter_mode) {
      case Thermostat::SystemModeEnum::kOff:
        new_mode = climate::CLIMATE_MODE_OFF;
        break;

      case Thermostat::SystemModeEnum::kCool:
        new_mode = climate::CLIMATE_MODE_COOL;
        break;

      case Thermostat::SystemModeEnum::kHeat:
        new_mode = climate::CLIMATE_MODE_HEAT;
        break;

      case Thermostat::SystemModeEnum::kAuto:
        new_mode = climate::CLIMATE_MODE_AUTO;
        break;

      case Thermostat::SystemModeEnum::kFanOnly:
        new_mode = climate::CLIMATE_MODE_FAN_ONLY;
        break;

      case Thermostat::SystemModeEnum::kDry:
        new_mode = climate::CLIMATE_MODE_DRY;
        break;

      default:
        return;
      }

      auto traits = this->climate->get_traits();

      //
      // OFF is mandatory in ESPHome; reject Matter modes the
      // underlying climate doesn't advertise.
      //
      if (new_mode != climate::CLIMATE_MODE_OFF &&
          !traits.supports_mode(new_mode)) {

        // AUTO can reasonably map to HEAT_COOL.
        if (new_mode == climate::CLIMATE_MODE_AUTO &&
            traits.supports_mode(climate::CLIMATE_MODE_HEAT_COOL)) {
          new_mode = climate::CLIMATE_MODE_HEAT_COOL;
        } else {
          return;
        }
      }

      if (this->climate->mode == new_mode)
        return;

      auto call = this->climate->make_call();
      call.set_mode(new_mode);
      call.perform();
      return;
    }

    //
    // Cooling setpoint
    //
    if (attribute_id == Thermostat::Attributes::OccupiedCoolingSetpoint::Id) {

      const float temperature = static_cast<float>(val.val.i16) / 100.0f;

      if (!std::isnan(this->climate->target_temperature) &&
          std::fabs(this->climate->target_temperature - temperature) < 0.005f)
        return;

      auto call = this->climate->make_call();
      call.set_target_temperature(temperature);
      call.perform();
      return;
    }

    //
    // Heating setpoint
    //
    if (attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id) {

      const float temperature = static_cast<float>(val.val.i16) / 100.0f;

      if (!std::isnan(this->climate->target_temperature) &&
          std::fabs(this->climate->target_temperature - temperature) < 0.005f)
        return;

      auto call = this->climate->make_call();
      call.set_target_temperature(temperature);
      call.perform();
    }
  }

  if (cluster_id == FanControl::Id) {
    ESP_LOGI(TAG,
           "Matter FanControl update: attribute=0x%08" PRIX32
           " value=%u",
           attribute_id,
           val.val.u8);

    if (attribute_id == FanControl::Attributes::FanMode::Id) {
      const uint8_t matter_fan_mode = val.val.u8;

      climate::ClimateFanMode new_fan_mode;

      switch (matter_fan_mode) {
      case 0:
        new_fan_mode = climate::CLIMATE_FAN_OFF;
        break;

      case 1:
        new_fan_mode = climate::CLIMATE_FAN_LOW;
        break;

      case 2:
        new_fan_mode = climate::CLIMATE_FAN_MEDIUM;
        break;

      case 3:
        new_fan_mode = climate::CLIMATE_FAN_HIGH;
        break;

      case 4:
        new_fan_mode = climate::CLIMATE_FAN_ON;
        break;

      case 5:
        new_fan_mode = climate::CLIMATE_FAN_AUTO;
        break;

      default:
        return;
      }

      auto traits = this->climate->get_traits();

      if (!traits.supports_fan_mode(new_fan_mode))
        return;

      if (this->climate->fan_mode.has_value() &&
          *this->climate->fan_mode == new_fan_mode)
        return;

      ESP_LOGD(TAG, "Matter fan mode update: %u", matter_fan_mode);

      auto call = this->climate->make_call();
      call.set_fan_mode(new_fan_mode);
      call.perform();

      return;
    }
  }
}

#endif // USE_CLIMATE

#ifdef USE_LIGHT
  // Mirrors the current ESPHome light state to the Matter attributes.
  // Runs on the main loop; the attribute writes hop to the Matter thread.
  void MatterLight::push_state_to_matter() {
    uint16_t eid = this->endpoint_id;
    bool dim = this->dimmable;
    bool on = this->light->remote_values.is_on();
    float brightness = this->light->remote_values.get_brightness();
    auto level = static_cast<uint8_t>(std::lroundf(brightness * 254.0f));
    level = level < 1 ? 1 : level;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([eid, dim, on, level]() {
      using namespace chip::app::Clusters;
      esp_matter_attr_val_t on_val = esp_matter_bool(on);
      esp_matter::attribute::update(eid, OnOff::Id,
                                    OnOff::Attributes::OnOff::Id, &on_val);
      if (dim) {
        esp_matter_attr_val_t level_val =
            esp_matter_nullable_uint8(nullable<uint8_t>(level));
        esp_matter::attribute::update(
            eid, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id,
            &level_val);
      }
    });
  }

  // Applies a Matter-side attribute change to the ESPHome light. Runs on the
  // main loop (deferred from the Matter thread). Values that already match the
  // light's state are ignored, which also breaks the mirror echo loop.
  void MatterLight::apply_matter_update(
      uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t val) {
    using namespace chip::app::Clusters;
    if (cluster_id == OnOff::Id &&
        attribute_id == OnOff::Attributes::OnOff::Id) {
      bool on = val.val.b;
      if (this->light->remote_values.is_on() == on)
        return;
      auto call = this->light->make_call();
      call.set_state(on);
      // The Matter side already ramps CurrentLevel during transitions; a second
      // ESPHome-side transition would double-smooth every change.
      call.set_transition_length(0);
      call.perform();
    } else if (this->dimmable && cluster_id == LevelControl::Id &&
               attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
      uint8_t level = val.val.u8;
      if (level < 1 || level > 254)
        return; // null or out of spec range
      float brightness = level / 254.0f;
      if (std::fabs(this->light->remote_values.get_brightness() - brightness) <
          (0.5f / 254.0f))
        return;
      auto call = this->light->make_call();
      call.set_brightness(brightness);
      call.set_transition_length(0);
      call.perform();
    }
  }
#endif // USE_LIGHT

  esp_err_t endpoint_attribute_update_cb(
      esp_matter::attribute::callback_type_t type, uint16_t endpoint_id,
      uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val,
      void *priv_data) {

    if (type != esp_matter::attribute::POST_UPDATE ||
        global_matter_component == nullptr)
      return ESP_OK;

#ifdef USE_LIGHT
    if (auto *ml =
            global_matter_component->get_light_by_endpoint(endpoint_id)) {

      esp_matter_attr_val_t val_copy = *val;

      global_matter_component->defer_to_main_loop(
          [ml, cluster_id, attribute_id, val_copy]() {
            ml->apply_matter_update(cluster_id, attribute_id, val_copy);
          });

      return ESP_OK;
    }
#endif

#ifdef USE_CLIMATE
    if (auto *mc =
            global_matter_component->get_climate_by_endpoint(endpoint_id)) {

      esp_matter_attr_val_t val_copy = *val;

      global_matter_component->defer_to_main_loop(
          [mc, cluster_id, attribute_id, val_copy]() {
            mc->apply_matter_update(cluster_id, attribute_id, val_copy);
          });

      return ESP_OK;
    }
#endif

    return ESP_OK;
  }

  // Wires ESPHome entities to Matter attributes. Must run after
  // esp_matter::start(). Client switch endpoints have no wiring here: their
  // behaviour comes from matter.* actions in YAML automations.
  void MatterComponent::register_endpoint_callbacks_() {
#ifdef USE_SENSOR
    for (const auto &ts : this->temperature_sensors_) {
      uint16_t eid = ts.endpoint_id;
      ts.sensor->add_on_state_callback([eid](float value) {
        // Matter spec: MeasuredValue = temperature in °C * 100, nullable int16
        // (valid range -273.15 °C .. 327.67 °C). Out-of-range or NaN reports
        // null.
        bool is_null = std::isnan(value) || value < -273.15f || value > 327.67f;
        int16_t raw =
            is_null ? 0 : static_cast<int16_t>(lroundf(value * 100.0f));
        // Attribute updates must run in the Matter thread (same pattern as the
        // esp-matter sensors example).
        chip::DeviceLayer::SystemLayer().ScheduleLambda([eid, raw, is_null]() {
          using namespace chip::app::Clusters;
          esp_matter_attr_val_t val = esp_matter_nullable_int16(
              is_null ? nullable<int16_t>() : nullable<int16_t>(raw));
          esp_matter::attribute::update(
              eid, TemperatureMeasurement::Id,
              TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
        });
      });
    }
#endif

#ifdef USE_LIGHT
    for (auto *ml : this->lights_) {
      ml->light->add_remote_values_listener(ml);
      ml->push_state_to_matter(); // initial sync so controllers read the real
                                  // state
    }
#endif

#ifdef USE_CLIMATE
    for (auto *mc : this->climates_) {
      mc->climate->add_on_state_callback(
          [mc](climate::Climate &) { mc->push_state_to_matter(); });

      mc->push_state_to_matter();
    }
#endif
  }

} // namespace esphome::matter

#endif // USE_MATTER
