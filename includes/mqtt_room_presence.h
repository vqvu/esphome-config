#pragma once
#include <cctype>

#include "esphome.h"

namespace mqtt_room_presence {

std::string normalize_uuid(const std::string& data) {
  std::string result = data;
  std::transform(
      result.begin(), result.end(), result.begin(),
      [](unsigned char c){ return std::tolower(c); });
  return result;
}

std::string uuid_to_string(const esphome::esp32_ble_tracker::ESPBTUUID& uuid) {
  // The ESPBTUUID string formatter and parser assume little endian even though
  // the data is stored in big endian format. We reverse the bytes to convince
  // it to print the correct string.
  // See https://github.com/esphome/issues/issues/2445
  auto raw_uuid = uuid.as_128bit().get_uuid();
  uint8_t fixed_uuid_data[ESP_UUID_LEN_128];
  for (size_t i = 0; i < ESP_UUID_LEN_128; i++) {
    fixed_uuid_data[i] = raw_uuid.uuid.uuid128[ESP_UUID_LEN_128 - i - 1];
  }
  return normalize_uuid(esphome::esp32_ble_tracker::ESPBTUUID::from_raw(fixed_uuid_data).to_string());
} 

std::string make_ibeacon_device_id(
    const std::string& uuid, uint16_t major, uint16_t minor) {
  return "iBeacon:" + uuid + "_" + esphome::to_string(major) + "_" + esphome::to_string(minor);
}

std::string to_device_id_f(esphome::esp32_ble_tracker::ESPBLEiBeacon& ibeacon) {
  return make_ibeacon_device_id(
      uuid_to_string(ibeacon.get_uuid()),
      ibeacon.get_major(),
      ibeacon.get_minor());
}

class TrackedDevice : public esphome::Component {
public:
  explicit TrackedDevice(
      const std::string& device_id,
      const std::string& room_slug,
      const std::string& mqtt_prefix,
      const std::map<std::string, esphome::sensor::Sensor*>& distance_sensors_map,
      esphome::sensor::Sensor *rssi_sensor)
      : device_id(device_id),
	device_topic(mqtt_prefix + "/devices/" + device_id + "/rooms/" + room_slug),
	rooms_topic(mqtt_prefix + "/rooms/" + room_slug),
	distance_sensors_map(distance_sensors_map),
	rssi_sensor(rssi_sensor),
	last_rssi_at_1m(0),
        last_published_millis(0) {};

  void record_distance(float distance) {
    for (auto& x : this->distance_sensors_map) {
      x.second->publish_state(distance);
    }
  }

  void record_rssi(float rssi) {
    rssi_sensor->publish_state(rssi);
  }

  void record_rssi_at_1m(int8_t rssi_at_1m) {
    this->last_rssi_at_1m = rssi_at_1m;
  }

  void publish(mqtt::MQTTClientComponent *mqtt_client) {
    uint32_t now_millis = millis();
    uint32_t last_published_millis = this->last_published_millis;
    if (last_published_millis > 0 && now_millis - last_published_millis < 3000) {
      return;
    }

    auto device_id = this->device_id;
    auto distance = this->distance_sensors_map["distance"]->get_state();
    auto rssi = this->rssi_sensor->get_state();
    auto rssi_at_1m = this->last_rssi_at_1m;

    auto room_payload = [=](JsonObject root) {
      root["id"] = device_id;
      root["distance"] = distance;
      root["rssi"] = rssi;
      root["rssi@1m"] = rssi_at_1m;
    };

    ESP_LOGD("mqtt_room_presence", "'%s': Publishing device state to MQTT", device_id.c_str());
    for (auto& x : this->distance_sensors_map) {
      mqtt_client->publish(
          this->device_topic + "/" + x.first,
	  esphome::to_string(x.second->get_state()));
    }
    mqtt_client->publish(this->device_topic + "/rssi", esphome::to_string(rssi));
    mqtt_client->publish(this->device_topic + "/rssi@1m", esphome::to_string(rssi_at_1m));
    mqtt_client->publish_json(this->rooms_topic, room_payload);
    this->last_published_millis = now_millis;
  }

private:
  std::string device_id;
  std::string device_topic;
  std::string rooms_topic;
  std::map<std::string, esphome::sensor::Sensor*> distance_sensors_map;
  esphome::sensor::Sensor *rssi_sensor;

  int8_t last_rssi_at_1m;
  uint32_t last_published_millis;
};

TrackedDevice* to_tracked_device(void* tracked_device) {
  return static_cast<TrackedDevice*>(tracked_device);
}

}

