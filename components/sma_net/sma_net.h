#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace esphome {
namespace sma_net {

struct CInfoSpotEntry {
  uint32_t spot_cidx;  // 0-basiert über alle SPOT-Kanäle
  uint8_t cidx;
  uint16_t ctype;
  uint16_t ntype;
  std::string name;
  std::string unit;
  sensor::Sensor *sensor_ref{nullptr};
  text_sensor::TextSensor *text_sensor_ref{nullptr};

  union {
    struct {
      float gain;
      float offset;
    } analog;
    struct {
      float gain;
    } counter;
    struct {
      uint16_t sizet;
      char *text;
    } status;
  } data;
};

struct SensorBinding {
  sensor::Sensor *sensor{nullptr};
  bool is_fast{false};
};

struct TextSensorBinding {
  text_sensor::TextSensor *sensor{nullptr};
};

class SmaNetSensor : public sensor::Sensor {};
class SmaNetTextSensor : public text_sensor::TextSensor {};

class SmaNetComponent : public PollingComponent, public uart::UARTDevice {
 public:
  void set_debug(bool debug) { this->debug_ = debug; }
  void set_slow_factor(uint32_t slow_factor) { this->slow_factor_ = (slow_factor < 2) ? 2 : slow_factor; }

  void register_sensor(const std::string &channel_name, sensor::Sensor *sensor, bool is_fast);
  void register_text_sensor(const std::string &channel, text_sensor::TextSensor *sensor);

  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;

 protected:
  enum class RequestState : uint8_t {
    IDLE,
    WAITING_GETDATA,
  };

  static float read_f32_le_(const uint8_t *p);
  static uint32_t read_u32_le_(const uint8_t *p);
  static bool has_bytes_(size_t len, size_t off, size_t need);
  static uint16_t read_u16_le_(const uint8_t *p);
  static std::string cstr_fixed_(const uint8_t *p, size_t len);

  static std::vector<uint8_t> unescape_frame_(const std::vector<uint8_t> &in);
  static void log_payload_hex_(const uint8_t *data, size_t len, const char *tag = "sma_payload");
  static uint16_t crc16_sma_(const uint8_t *data, size_t len);

  void send_command_frame_(uint16_t dst, uint16_t src, uint8_t ctrl, uint8_t pkt, uint8_t cmd,
                           const uint8_t *payload, size_t payload_len);
  void request_followup_fragment_(uint16_t src, uint16_t dst, uint8_t cmd, uint8_t pkt);

  void send_sync_();
  void send_getdata_(uint16_t cmask, uint8_t channel);
  void send_getcinfo_();
  void read_uart_frames_();
  void parse_frame_(const std::vector<uint8_t> &frame);
  void dispatch_cmd_payload_(uint8_t cmd, const uint8_t *data, size_t len);
  void parse_cdata_payload_(const uint8_t *data, size_t len);
  void parse_cinfo_payload_(const uint8_t *data, size_t len);
  void publish_data_timestamp_(uint32_t ts);
  bool find_spot_cidx_(const std::string &name, uint8_t &channel_no) const;
  void queue_getdata_request_(uint8_t channel);
  void send_next_queued_getdata_();

  std::unordered_map<std::string, SensorBinding> sensors_by_channel_name_;
  std::unordered_map<std::string, TextSensorBinding> text_sensors_by_channel_name_;
  std::vector<CInfoSpotEntry> cinfo_spot_channels_;

  bool debug_{false};

  RequestState request_state_{RequestState::IDLE};
  uint32_t sync_sent_at_ms_{0};
  uint32_t poll_cycle_{0};
  uint32_t slow_factor_{6};
  std::vector<uint8_t> pending_getdata_channels_{};

  bool in_frame_{false};
  std::vector<uint8_t> frame_buffer_{};

  // Debug-/Scaffold-Status für Reassemblierung
  bool rx_fragment_series_active_{false};
  uint8_t rx_last_pkt_counter_{0};
  uint8_t rx_last_cmd_{0};
  uint16_t rx_last_src_{0};
  uint16_t rx_last_dst_{0};

  std::vector<uint8_t> rx_reassembly_buffer_{};
  uint8_t rx_crc_retry_count_{0};

  uint32_t setup_started_ms_{0};
  bool transfer_session_active_{false};
  uint32_t transfer_session_started_ms_{0};

  uint8_t transfer_last_rx_pkt_{0};
  uint32_t transfer_last_rx_ms_{0};
  uint8_t transfer_last_stall_pkt_{0xFF};
  uint8_t transfer_stall_retry_count_{0};

  bool pending_frag_request_{false};
  uint8_t pending_frag_pkt_{0};
  uint32_t pending_frag_due_ms_{0};
  uint32_t last_frag_req_sent_ms_{0};
};

}  // namespace sma_net
}  // namespace esphome
