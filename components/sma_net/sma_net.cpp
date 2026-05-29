#include "sma_net.h"

#include "esphome/core/log.h"

#include <ctime>
#include <cstring>

namespace esphome {
namespace sma_net {

static const char *const TAG = "sma_net";
static constexpr uint32_t CINFO_SESSION_TIMEOUT_MS = 60000;
static constexpr uint32_t SYNC_TO_GETDATA_DELAY_MS = 30;
static constexpr size_t TRANSFER_REASSEMBLY_RESERVE = 70000;
static constexpr uint32_t CINFO_ACK_SPACING_MS = 0;
static constexpr uint32_t CINFO_FRAGMENT_STALL_WARN_MS = 1500;
static constexpr uint8_t CINFO_STALL_RETRY_MAX = 3;

static constexpr uint16_t SMA_ADDR_DEFAULT = 0x0000;
static constexpr uint16_t SMA_ADDR_GETDATA_SRC = 0x0002;
static constexpr uint8_t SMA_CTRL_DEFAULT = 0x00;
static constexpr uint8_t SMA_CTRL_SYNC = 0x80;
static constexpr uint8_t CMD_GETCINFO = 0x09;
static constexpr uint8_t CMD_SYNC = 0x0A;
static constexpr uint8_t CMD_GETDATA = 0x0B;
static constexpr uint8_t CHANNEL_ALL = 0x00;

static constexpr size_t CDATA_HDR_CTYPE_OFF = 0;
static constexpr size_t CDATA_HDR_CIDX_OFF = 2;
static constexpr size_t CDATA_HDR_RECORD_COUNT_OFF = 3;
static constexpr size_t CDATA_HDR_TIMESTAMP_OFF = 5;
static constexpr size_t CDATA_HDR_TIME_BASE_OFF = 9;
static constexpr size_t CDATA_HDR_LEN = 13;

static constexpr size_t CINFO_ENTRY_CIDX_OFF = 0;
static constexpr size_t CINFO_ENTRY_CTYPE_OFF = 1;
static constexpr size_t CINFO_ENTRY_NTYPE_OFF = 3;
static constexpr size_t CINFO_ENTRY_NFILL_OFF = 5;
static constexpr size_t CINFO_ENTRY_NAME_OFF = 7;
static constexpr size_t CINFO_ENTRY_NAME_LEN = 16;
static constexpr size_t CINFO_ENTRY_HEAD_LEN = 23;
static constexpr size_t CINFO_TAIL_ANALOG_LEN = 16;
static constexpr size_t CINFO_TAIL_COUNTER_LEN = 12;
static constexpr size_t CINFO_TAIL_STATUS_MIN_LEN = 2;

static constexpr uint16_t NTYPE_U32 = 0x0102;
static constexpr uint16_t NTYPE_FLOAT = 0x0104;
static constexpr uint16_t CTYPE_SPOT = 0x0900;
static constexpr uint16_t CTYPE_FORMAT_MASK = 0x000F;
static constexpr uint16_t CTYPE_FORMAT_ANALOG = 0x0001;
static constexpr uint16_t CTYPE_FORMAT_COUNTER = 0x0004;
static constexpr uint16_t CTYPE_FORMAT_STATUS = 0x0008;
static constexpr uint16_t CTYPE_SPOT_ANALOG = CTYPE_SPOT | CTYPE_FORMAT_ANALOG;
static constexpr uint16_t CTYPE_SPOT_COUNTER = CTYPE_SPOT | CTYPE_FORMAT_COUNTER;
static constexpr uint16_t CTYPE_SPOT_STATUS = CTYPE_SPOT | CTYPE_FORMAT_STATUS;
static constexpr uint16_t CTYPE_SPOT_ALL = CTYPE_SPOT | CTYPE_FORMAT_MASK;

static const uint16_t CRC16_POLY_REV = 0x8408;

static std::vector<uint8_t> escape_frame_inner_(const std::vector<uint8_t> &in) {
  std::vector<uint8_t> out;
  out.reserve(in.size() + 8);
  for (uint8_t b : in) {
    if (b == 0x7E || b == 0x7D || b == 0x11 || b == 0x12 || b == 0x13) {
      out.push_back(0x7D);
      out.push_back(static_cast<uint8_t>(b ^ 0x20));
    } else {
      out.push_back(b);
    }
  }
  return out;
}

void SmaNetComponent::register_sensor(const std::string &channel_name, sensor::Sensor *sensor, bool is_fast) {
  auto &binding = this->sensors_by_channel_name_[channel_name];
  binding.sensor = sensor;
  binding.is_fast = is_fast;
}

void SmaNetComponent::register_text_sensor(const std::string &channel, text_sensor::TextSensor *sensor) {
  auto &binding = this->text_sensors_by_channel_name_[channel];
  binding.sensor = sensor;
}

void SmaNetComponent::setup() {
  this->frame_buffer_.reserve(800);
  // Große Reserve verhindert teure Reallocs mitten im laufenden
  // Fragment-Handshake (z. B. große Transfers wie CINFO).
  this->rx_reassembly_buffer_.reserve(TRANSFER_REASSEMBLY_RESERVE);
  this->setup_started_ms_ = millis();

  // CINFO einmalig direkt beim Start abrufen.
  this->send_getcinfo_();
  if (this->debug_) {
    ESP_LOGD(TAG, "transfer session started (cinfo one-shot request)");
  }
}

void SmaNetComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "SMA Net:");
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Debug: %s", YESNO(this->debug_));
  ESP_LOGCONFIG(TAG, "  Slow factor: %u", (unsigned) this->slow_factor_);
  ESP_LOGCONFIG(TAG, "  Registered CInfo sensors: %u", (unsigned) this->sensors_by_channel_name_.size());
  this->check_uart_settings(19200);
}

void SmaNetComponent::update() {
  // Während laufender Fragment-Session kein neues CData-Polling starten.
  if (this->rx_fragment_series_active_ || this->transfer_session_active_) {
    this->request_state_ = RequestState::IDLE;
    return;
  }

  this->pending_getdata_channels_.clear();

  const bool is_slow_cycle = (this->slow_factor_ <= 1) || ((this->poll_cycle_ % this->slow_factor_) == 0);
  this->poll_cycle_++;

  if (is_slow_cycle) {
    this->queue_getdata_request_(CHANNEL_ALL);
  } else {
    for (const auto &it : this->sensors_by_channel_name_) {
      const auto &name = it.first;
      const auto &binding = it.second;
      if (!binding.is_fast || binding.sensor == nullptr) {
        continue;
      }

      uint8_t channel_no = 0;
      if (!this->find_spot_cidx_(name, channel_no)) {
        if (this->debug_) {
          ESP_LOGW(TAG, "fast poll skipped: channel '%s' not found in CINFO map", name.c_str());
        }
        continue;
      }
      this->queue_getdata_request_(channel_no);
    }
  }

  if (this->pending_getdata_channels_.empty()) {
    if (this->debug_) {
      ESP_LOGV(TAG, "poll cycle has no queued GETDATA channels");
    }
    this->request_state_ = RequestState::IDLE;
    return;
  }

  this->send_sync_();
  this->sync_sent_at_ms_ = millis();
  this->request_state_ = RequestState::WAITING_GETDATA;
}

void SmaNetComponent::loop() {
  const uint32_t now = millis();

  if (this->transfer_session_active_ && (now - this->transfer_session_started_ms_) > CINFO_SESSION_TIMEOUT_MS) {
    this->transfer_session_active_ = false;
    this->rx_fragment_series_active_ = false;
    this->rx_reassembly_buffer_.clear();
    this->pending_frag_request_ = false;
    if (this->debug_) {
      ESP_LOGW(TAG, "transfer session timeout after %u ms", (unsigned) CINFO_SESSION_TIMEOUT_MS);
    }
  }

  if (this->rx_fragment_series_active_ &&
      (now - this->transfer_last_rx_ms_) > CINFO_FRAGMENT_STALL_WARN_MS) {
    const uint8_t stall_pkt = this->pending_frag_pkt_;
    const bool same_stall_pkt = (stall_pkt == this->transfer_last_stall_pkt_);
    this->transfer_last_stall_pkt_ = stall_pkt;
    this->transfer_stall_retry_count_ = same_stall_pkt ? (this->transfer_stall_retry_count_ + 1) : 1;

    const uint16_t approx_missing = stall_pkt;
    ESP_LOGW(TAG,
             "transfer fragment stalled: last_rx_pkt=%u last_req_pkt=%u silence=%u ms reasm=%u approx_missing=%u retry=%u/%u",
             (unsigned) this->transfer_last_rx_pkt_,
             (unsigned) stall_pkt,
             (unsigned) (now - this->transfer_last_rx_ms_),
             (unsigned) this->rx_reassembly_buffer_.size(),
             (unsigned) approx_missing,
             (unsigned) this->transfer_stall_retry_count_,
             (unsigned) CINFO_STALL_RETRY_MAX);

    if (this->transfer_stall_retry_count_ <= CINFO_STALL_RETRY_MAX) {
      this->request_followup_fragment_(this->rx_last_src_, this->rx_last_dst_, this->rx_last_cmd_, stall_pkt);
      this->last_frag_req_sent_ms_ = now;
      ESP_LOGW(TAG, "fragment stall retry sent for cmd=0x%02X pkt=%u", (unsigned) this->rx_last_cmd_, (unsigned) stall_pkt);
      this->transfer_last_rx_ms_ = now;
    } else {
      ESP_LOGW(TAG, "fragment stall retry limit reached for cmd=0x%02X pkt=%u -> abort session",
               (unsigned) this->rx_last_cmd_, (unsigned) stall_pkt);
      this->transfer_session_active_ = false;
      this->rx_fragment_series_active_ = false;
      this->pending_frag_request_ = false;
      this->rx_reassembly_buffer_.clear();
      this->transfer_stall_retry_count_ = 0;
      this->transfer_last_rx_ms_ = now;
    }
  }

  if (this->pending_frag_request_ && millis() >= this->pending_frag_due_ms_) {
    this->request_followup_fragment_(this->rx_last_src_, this->rx_last_dst_,
                                     this->rx_last_cmd_, this->pending_frag_pkt_);
    this->pending_frag_request_ = false;
    this->last_frag_req_sent_ms_ = millis();
  }

  if (this->request_state_ == RequestState::WAITING_GETDATA) {
    if (!this->transfer_session_active_ && (millis() - this->sync_sent_at_ms_) >= SYNC_TO_GETDATA_DELAY_MS) {
      this->send_next_queued_getdata_();
    }
  }
  this->read_uart_frames_();
}

bool SmaNetComponent::find_spot_cidx_(const std::string &name, uint8_t &channel_no) const {
  for (const auto &channel : this->cinfo_spot_channels_) {
    if (channel.name == name) {
      if (channel.spot_cidx > 0xFF) {
        return false;
      }
      channel_no = static_cast<uint8_t>(channel.spot_cidx);
      return true;
    }
  }
  return false;
}

void SmaNetComponent::queue_getdata_request_(uint8_t channel) {
  this->pending_getdata_channels_.push_back(channel);
}

void SmaNetComponent::send_next_queued_getdata_() {
  if (this->pending_getdata_channels_.empty()) {
    this->request_state_ = RequestState::IDLE;
    return;
  }

  const uint8_t channel = this->pending_getdata_channels_.front();
  this->pending_getdata_channels_.erase(this->pending_getdata_channels_.begin());

  uint16_t cmask = CTYPE_SPOT_ALL;
  uint8_t req_cidx = channel;
  if (channel != CHANNEL_ALL) {
    bool found_ctype = false;
    for (const auto &mapped_channel : this->cinfo_spot_channels_) {
      if (mapped_channel.spot_cidx == channel) {
        cmask = mapped_channel.ctype;
        req_cidx = mapped_channel.cidx;
        found_ctype = true;
        break;
      }
    }
    if (!found_ctype && this->debug_) {
      ESP_LOGW(TAG,
               "single-channel GETDATA fallback mask: channel=%u not found in CINFO map, using 0x%04X",
               (unsigned) channel,
               (unsigned) CTYPE_SPOT_ALL);
    }
  }

  this->send_getdata_(cmask, req_cidx);

  if (this->pending_getdata_channels_.empty()) {
    this->request_state_ = RequestState::IDLE;
  } else {
    this->request_state_ = RequestState::WAITING_GETDATA;
    this->sync_sent_at_ms_ = millis();
  }
}

float SmaNetComponent::read_f32_le_(const uint8_t *p) {
  union {
    uint32_t u;
    float f;
  } v{};
  v.u = (uint32_t(p[0])) |
        (uint32_t(p[1]) << 8) |
        (uint32_t(p[2]) << 16) |
        (uint32_t(p[3]) << 24);
  return v.f;
}

uint32_t SmaNetComponent::read_u32_le_(const uint8_t *p) {
  return (uint32_t(p[0])) |
         (uint32_t(p[1]) << 8) |
         (uint32_t(p[2]) << 16) |
         (uint32_t(p[3]) << 24);
}

uint16_t SmaNetComponent::read_u16_le_(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

std::string SmaNetComponent::cstr_fixed_(const uint8_t *p, size_t len) {
  if (p == nullptr || len == 0) return "";
  size_t used = 0;
  while (used < len && p[used] != 0x00) {
    used++;
  }
  while (used > 0 && p[used - 1] == 0x20) {
    used--;
  }
  return std::string(reinterpret_cast<const char *>(p), used);
}

bool SmaNetComponent::has_bytes_(size_t len, size_t off, size_t need) {
  return off + need <= len;
}

std::vector<uint8_t> SmaNetComponent::unescape_frame_(const std::vector<uint8_t> &in) {
  std::vector<uint8_t> out;
  out.reserve(in.size());
  if (in.size() < 2) return out;

  for (size_t i = 1; i + 1 < in.size(); ++i) {
    uint8_t b = in[i];
    if (b == 0x7D && (i + 1) < (in.size() - 1)) {
      ++i;
      out.push_back(static_cast<uint8_t>(in[i] ^ 0x20));
    } else {
      out.push_back(b);
    }
  }
  return out;
}

void SmaNetComponent::log_payload_hex_(const uint8_t *data, size_t len, const char *tag) {
  if (data == nullptr || len == 0) return;

  char line[3 * 64 + 1];
  for (size_t base = 0; base < len; base += 64) {
    const size_t chunk = ((len - base) > 64) ? 64 : (len - base);
    char *p = line;
    for (size_t i = 0; i < chunk; i++) {
      std::snprintf(p, 4, "%02X ", data[base + i]);
      p += 3;
    }
    *p = '\0';
    ESP_LOGV(tag, "%03u..%03u/%03u: %s", (unsigned) base,
             (unsigned) (base + chunk - 1), (unsigned) len, line);
  }
}

void SmaNetComponent::send_sync_() {
  std::time_t sync_time = std::time(nullptr);
  if (std::tm *local_tm = std::localtime(&sync_time)) {
    sync_time = std::mktime(local_tm);
  } else if (this->debug_) {
    ESP_LOGW(TAG, "sync time: localtime unavailable, fallback to utc=%u", (unsigned) sync_time);
  }

  const uint32_t ts = static_cast<uint32_t>(sync_time);
  const uint8_t payload[] = {
      static_cast<uint8_t>(ts & 0xFF),
      static_cast<uint8_t>((ts >> 8) & 0xFF),
      static_cast<uint8_t>((ts >> 16) & 0xFF),
      static_cast<uint8_t>((ts >> 24) & 0xFF),
  };
  this->send_command_frame_(
      SMA_ADDR_DEFAULT,
      SMA_ADDR_DEFAULT,
      SMA_CTRL_SYNC,
      0x00,
      CMD_SYNC,
      payload,
      sizeof(payload));
}

void SmaNetComponent::publish_data_timestamp_(uint32_t ts) {
  auto it = this->text_sensors_by_channel_name_.find("Data_Timestamp");
  if (it == this->text_sensors_by_channel_name_.end() || it->second.sensor == nullptr) {
    return;
  }

  if (ts == 0) {
    it->second.sensor->publish_state(std::string("invalid"));
    return;
  }

  time_t raw = static_cast<time_t>(ts);
  struct tm *tm_utc = std::gmtime(&raw);
  if (tm_utc == nullptr) {
    it->second.sensor->publish_state(std::string("invalid"));
    return;
  }

  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                tm_utc->tm_year + 1900, tm_utc->tm_mon + 1, tm_utc->tm_mday,
                tm_utc->tm_hour, tm_utc->tm_min, tm_utc->tm_sec);
  it->second.sensor->publish_state(std::string(buf));
}

void SmaNetComponent::send_getdata_(uint16_t cmask, uint8_t channel) {
  const uint8_t payload[] = {
      static_cast<uint8_t>(cmask & 0xFF),
      static_cast<uint8_t>((cmask >> 8) & 0xFF),
      channel,
  };
  this->transfer_session_active_ = true;
  this->transfer_session_started_ms_ = millis();
  this->transfer_last_rx_ms_ = this->transfer_session_started_ms_;
  this->transfer_last_rx_pkt_ = 0xFF;
  this->send_command_frame_(
      SMA_ADDR_DEFAULT,
      SMA_ADDR_GETDATA_SRC,
      SMA_CTRL_DEFAULT,
      0x00,
      CMD_GETDATA,
      payload,
      sizeof(payload));
}

void SmaNetComponent::send_getcinfo_() {
  // Aus den Referenz-Captures "Start device detection":
  // erster CINFO-Request ist cmd=CMD_GETCINFO, pkt=0x00 mit Payload 44 22.
  // Danach liefert der WR mit pkt=0xFF und wir fragen Folgefragmente (pkt jeweils gleich zurück) an.
  static const uint8_t payload[] = {0x44, 0x22};
  this->transfer_session_active_ = true;
  this->transfer_session_started_ms_ = millis();
  this->transfer_last_rx_ms_ = this->transfer_session_started_ms_;
  this->transfer_last_rx_pkt_ = 0xFF;
  this->send_command_frame_(
      SMA_ADDR_DEFAULT,
      SMA_ADDR_GETDATA_SRC,
      SMA_CTRL_DEFAULT,
      0x00,
      CMD_GETCINFO,
      payload,
      sizeof(payload));

  this->pending_frag_pkt_ = 0x00;

  if (this->debug_) {
    ESP_LOGD(TAG, "cinfo request sent (cmd=0x%02X, pkt=0, payload=44 22)", CMD_GETCINFO);
  }
}

void SmaNetComponent::request_followup_fragment_(uint16_t src, uint16_t dst, uint8_t cmd, uint8_t pkt) {
  this->send_command_frame_(
      src,
      dst,
      SMA_CTRL_DEFAULT,
      pkt,
      cmd,
      nullptr,
      0);
  this->pending_frag_pkt_ = pkt;

  if (this->debug_) {
    ESP_LOGV(TAG, "frag request: dst=0x%04X src=0x%04X cmd=0x%02X pkt=%u",
             src, dst, cmd, (unsigned) pkt);
  }
}

uint16_t SmaNetComponent::crc16_sma_(const uint8_t *data, size_t len) {
  uint16_t fcs = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    fcs ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (fcs & 0x0001) {
        fcs = (fcs >> 1) ^ CRC16_POLY_REV;
      } else {
        fcs >>= 1;
      }
    }
  }
  return fcs;
}

void SmaNetComponent::send_command_frame_(uint16_t dst, uint16_t src, uint8_t ctrl, uint8_t pkt, uint8_t cmd,
                                          const uint8_t *payload, size_t payload_len) {
  std::vector<uint8_t> inner;
  inner.reserve(11 + payload_len + 2);

  inner.push_back(0xFF);
  inner.push_back(0x03);
  inner.push_back(0x40);
  inner.push_back(0x41);

  inner.push_back(static_cast<uint8_t>(dst & 0xFF));
  inner.push_back(static_cast<uint8_t>((dst >> 8) & 0xFF));
  inner.push_back(static_cast<uint8_t>(src & 0xFF));
  inner.push_back(static_cast<uint8_t>((src >> 8) & 0xFF));

  inner.push_back(ctrl);
  inner.push_back(pkt);
  inner.push_back(cmd);

  if (payload != nullptr && payload_len > 0) {
    inner.insert(inner.end(), payload, payload + payload_len);
  }

  const uint16_t fcs = static_cast<uint16_t>(~crc16_sma_(inner.data(), inner.size()));
  inner.push_back(static_cast<uint8_t>(fcs & 0xFF));
  inner.push_back(static_cast<uint8_t>((fcs >> 8) & 0xFF));

  const std::vector<uint8_t> inner_escaped = escape_frame_inner_(inner);

  std::vector<uint8_t> frame;
  frame.reserve(inner_escaped.size() + 2);
  frame.push_back(0x7E);
  frame.insert(frame.end(), inner_escaped.begin(), inner_escaped.end());
  frame.push_back(0x7E);

  this->write_array(frame.data(), frame.size());
  this->flush();

  if (this->debug_) {
    ESP_LOGV(TAG, "tx frame: src=0x%04X dst=0x%04X cmd=0x%02X pkt=%u ctrl=0x%02X payload=%u",
             src, dst, cmd, (unsigned) pkt, ctrl, (unsigned) payload_len);
  }
}

void SmaNetComponent::read_uart_frames_() {
  while (this->available()) {
    uint8_t b;
    this->read_byte(&b);

    if (!this->in_frame_) {
      if (b == 0x7E) {
        this->frame_buffer_.clear();
        this->frame_buffer_.push_back(b);
        this->in_frame_ = true;
      }
      continue;
    }

    this->frame_buffer_.push_back(b);

    if (b == 0x7E && this->frame_buffer_.size() > 1) {
      this->parse_frame_(this->frame_buffer_);
      this->frame_buffer_.clear();
      this->in_frame_ = false;
      continue;
    }

    if (this->frame_buffer_.size() > 768) {
      ESP_LOGW(TAG, "frame_too_large");
      this->frame_buffer_.clear();
      this->in_frame_ = false;
    }
  }
}

void SmaNetComponent::parse_frame_(const std::vector<uint8_t> &frame) {
  std::vector<uint8_t> p = unescape_frame_(frame);
  if (p.size() < 14) return;
  if (!(p[0] == 0xFF && p[1] == 0x03 && p[2] == 0x40 && p[3] == 0x41)) return;

  const uint8_t pkt = p[9];
  const uint8_t cmd = p[10];
  const uint16_t dst = static_cast<uint16_t>(p[4]) | (static_cast<uint16_t>(p[5]) << 8);
  const uint16_t src = static_cast<uint16_t>(p[6]) | (static_cast<uint16_t>(p[7]) << 8);

  const size_t data_offset = 11;
  size_t data_len = 0;
  if (p.size() >= data_offset + 2) {
    data_len = p.size() - data_offset - 2;  // ohne CRC
  }

  // RX-CRC/FCS-Check auf dem kompletten HDLC-Inhalt (Header+Payload+FCS)
  // ohne Start/Stop-Flag. Entspricht SMANet/PPP-FCS (YASDI: PPPGOODFCS16=0xF0B8).
  const uint16_t fcs_raw = crc16_sma_(p.data(), p.size());
  if (fcs_raw != 0xF0B8) {
    if (this->debug_) {
      const uint16_t rx_fcs = static_cast<uint16_t>(p[p.size() - 2]) |
                              (static_cast<uint16_t>(p[p.size() - 1]) << 8);
      ESP_LOGW(TAG, "drop frame: crc mismatch raw=0x%04X expected=0xF0B8 rx=0x%04X len=%u cmd=0x%02X pkt=%u",
               fcs_raw, rx_fcs, (unsigned) p.size(), cmd, (unsigned) pkt);
    }

    const bool same_series = this->rx_fragment_series_active_ &&
                             this->rx_last_cmd_ == cmd &&
                             this->rx_last_src_ == src &&
                             this->rx_last_dst_ == dst;
    if (same_series && this->rx_crc_retry_count_ < 2) {
      this->rx_crc_retry_count_++;
      if (this->debug_) {
        ESP_LOGW(TAG,
                 "frag retry (%u/2): re-request pkt=%u src=0x%04X dst=0x%04X cmd=0x%02X",
                 (unsigned) this->rx_crc_retry_count_,
                 (unsigned) this->pending_frag_pkt_,
                 src, dst, cmd);
      }
      this->request_followup_fragment_(src, dst, cmd, this->pending_frag_pkt_);
    } else if (same_series && this->rx_crc_retry_count_ >= 2) {
      ESP_LOGW(TAG, "frag retry exhausted: abort series cmd=0x%02X src=0x%04X dst=0x%04X",
               cmd, src, dst);
      this->rx_fragment_series_active_ = false;
      this->rx_reassembly_buffer_.clear();
    }
    return;
  }

  this->rx_crc_retry_count_ = 0;

  if (this->debug_) {
    ESP_LOGV(TAG, "frame ok: src=0x%04X dst=0x%04X cmd=0x%02X pkt=%u len=%u payload=%u",
             src, dst, cmd, (unsigned) pkt, (unsigned) p.size(), (unsigned) data_len);
  }

  // Fragment-Progress generisch tracken (für Stall-/Retry-Handling).
  if (pkt > 0) {
    this->transfer_last_rx_ms_ = millis();
    this->transfer_last_rx_pkt_ = pkt;
    this->transfer_stall_retry_count_ = 0;
    if (this->debug_) {
      ESP_LOGV(TAG, "fragment progress: cmd=0x%02X pkt=%u approx_missing_after_this=%u",
               cmd, (unsigned) pkt, (unsigned) (pkt - 1));
    }
  }

  // Reassembly (PktCnt-basiert): >0 Folgefragmente ausstehend, 0 = letztes Fragment.
  if (pkt > 0) {
    if (!this->rx_fragment_series_active_ || this->rx_last_cmd_ != cmd ||
        this->rx_last_src_ != src || this->rx_last_dst_ != dst) {
      this->rx_fragment_series_active_ = true;
      this->rx_reassembly_buffer_.clear();
      if (this->rx_reassembly_buffer_.capacity() < TRANSFER_REASSEMBLY_RESERVE) {
        this->rx_reassembly_buffer_.reserve(TRANSFER_REASSEMBLY_RESERVE);
      }
      if (this->debug_) {
        ESP_LOGV(TAG,
                 "frag series start: src=0x%04X dst=0x%04X cmd=0x%02X first_pkt=%u payload=%u",
                 src, dst, cmd, (unsigned) pkt, (unsigned) data_len);
        ESP_LOGV(TAG, "transfer reassembly capacity=%u", (unsigned) this->rx_reassembly_buffer_.capacity());
      }
    } else if (this->debug_ && pkt >= this->rx_last_pkt_counter_) {
      ESP_LOGW(TAG,
               "frag counter non-descending: prev=%u now=%u src=0x%04X dst=0x%04X cmd=0x%02X",
               (unsigned) this->rx_last_pkt_counter_, (unsigned) pkt, src, dst, cmd);
    }

    if (data_len > 0) {
      this->rx_reassembly_buffer_.insert(this->rx_reassembly_buffer_.end(),
                                         p.begin() + data_offset,
                                         p.begin() + data_offset + data_len);
    }
    // Folgefragment generisch anfordern.
    this->request_followup_fragment_(src, dst, cmd, pkt);
    this->last_frag_req_sent_ms_ = millis();
  } else if (pkt == 0) {
    const bool had_active_series = this->rx_fragment_series_active_ && this->rx_last_cmd_ == cmd &&
                                   this->rx_last_src_ == src && this->rx_last_dst_ == dst;
    const uint8_t *final_data = nullptr;
    size_t final_len = 0;

    if (had_active_series) {
      if (data_len > 0) {
        this->rx_reassembly_buffer_.insert(this->rx_reassembly_buffer_.end(),
                                           p.begin() + data_offset,
                                           p.begin() + data_offset + data_len);
      }
      if (this->debug_) {
        ESP_LOGV(TAG,
                 "frag series end: src=0x%04X dst=0x%04X cmd=0x%02X last_payload=%u total=%u",
                 src, dst, cmd, (unsigned) data_len, (unsigned) this->rx_reassembly_buffer_.size());
      }
      if (!this->rx_reassembly_buffer_.empty()) {
        final_data = this->rx_reassembly_buffer_.data();
        final_len = this->rx_reassembly_buffer_.size();
      }
    } else {
      // Einheitlicher Abschluss: auch Single-Fragment endet bei pkt==0.
      if (data_len > 0) {
        final_data = p.data() + data_offset;
        final_len = data_len;
      }
    }

    if (final_data != nullptr && final_len > 0) {
      this->dispatch_cmd_payload_(cmd, final_data, final_len);
    }

    // pkt==0 signalisiert immer das Ende der aktuellen Antwort.
    this->transfer_session_active_ = false;
    this->pending_frag_request_ = false;
    if (this->debug_) {
      ESP_LOGV(TAG, "transfer session complete (cmd=0x%02X): total_payload=%u", cmd, (unsigned) final_len);
    }

    if (had_active_series) {
      this->rx_fragment_series_active_ = false;
      this->rx_reassembly_buffer_.clear();
    }
  }

  this->rx_last_pkt_counter_ = pkt;
  this->rx_last_cmd_ = cmd;
  this->rx_last_src_ = src;
  this->rx_last_dst_ = dst;

}

void SmaNetComponent::dispatch_cmd_payload_(uint8_t cmd, const uint8_t *data, size_t len) {
  switch (cmd) {
    case CMD_GETDATA:
      this->parse_cdata_payload_(data, len);
      break;
    case CMD_GETCINFO:
      this->parse_cinfo_payload_(data, len);
      break;
    default:
      if (this->debug_) {
        ESP_LOGD(TAG, "unhandled cmd=0x%02X payload_len=%u", cmd, (unsigned) len);
      }
      break;
  }
}

void SmaNetComponent::parse_cinfo_payload_(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) return;

  if (this->debug_) {
    log_payload_hex_(data, len, "sma_cinfo");
  }

  for (auto &channel : this->cinfo_spot_channels_) {
    if ((channel.ctype & CTYPE_FORMAT_MASK) == CTYPE_FORMAT_STATUS && channel.data.status.text != nullptr) {
      free(channel.data.status.text);
      channel.data.status.text = nullptr;
      channel.data.status.sizet = 0;
    }
  }
  this->cinfo_spot_channels_.clear();
  this->cinfo_spot_channels_.reserve(80);

  size_t offset = 0;
  uint32_t spot_cidx = 0;
  while (offset < len) {
    if ((len - offset) < CINFO_ENTRY_HEAD_LEN) {
      ESP_LOGW(TAG, "cinfo parse stop: short header at offset=%u remaining=%u", (unsigned) offset, (unsigned) (len - offset));
      break;
    }

    const uint8_t cidx = data[offset + CINFO_ENTRY_CIDX_OFF];
    const uint16_t ctype = read_u16_le_(data + offset + CINFO_ENTRY_CTYPE_OFF);
    const uint16_t ntype = read_u16_le_(data + offset + CINFO_ENTRY_NTYPE_OFF);
    const uint16_t nfill = read_u16_le_(data + offset + CINFO_ENTRY_NFILL_OFF);
    const std::string name = cstr_fixed_(data + offset + CINFO_ENTRY_NAME_OFF, CINFO_ENTRY_NAME_LEN);
    const uint16_t ctype_format = static_cast<uint16_t>(ctype & CTYPE_FORMAT_MASK);

    size_t next_chan_offset = offset;
    const size_t p = offset + CINFO_ENTRY_HEAD_LEN;
    if (ctype_format == CTYPE_FORMAT_ANALOG) {
      if ((len - p) < CINFO_TAIL_ANALOG_LEN) {
        ESP_LOGW(TAG, "cinfo parse stop: short analog tail at offset=%u ctype=0x%04X", (unsigned) offset, ctype);
        break;
      }
      next_chan_offset = p + CINFO_TAIL_ANALOG_LEN;
    } else if (ctype_format == CTYPE_FORMAT_COUNTER) {
      if ((len - p) < CINFO_TAIL_COUNTER_LEN) {
        ESP_LOGW(TAG, "cinfo parse stop: short counter tail at offset=%u ctype=0x%04X", (unsigned) offset, ctype);
        break;
      }
      next_chan_offset = p + CINFO_TAIL_COUNTER_LEN;
    } else if (ctype_format == CTYPE_FORMAT_STATUS) {
      if ((len - p) < CINFO_TAIL_STATUS_MIN_LEN) {
        ESP_LOGW(TAG, "cinfo parse stop: short status size at offset=%u", (unsigned) offset);
        break;
      }
      const uint16_t sizet = read_u16_le_(data + p);
      if ((len - (p + 2)) < sizet) {
        ESP_LOGW(TAG, "cinfo parse stop: short status data at offset=%u need=%u", (unsigned) offset, (unsigned) sizet);
        break;
      }
      next_chan_offset = p + 2 + sizet;
    } else {
      ESP_LOGW(TAG,
               "cinfo parse stop: unknown ctype=0x%04X offset=%u ntype=0x%04X nfill=%u",
               ctype, (unsigned) offset, ntype, nfill);
      break;
    }

    if (ctype != CTYPE_SPOT_ANALOG && ctype != CTYPE_SPOT_COUNTER && ctype != CTYPE_SPOT_STATUS) {
      offset = next_chan_offset;
      continue;
    }

    CInfoSpotEntry parsed_channel{};
    parsed_channel.spot_cidx = spot_cidx;
    parsed_channel.cidx = cidx;
    parsed_channel.ctype = ctype;
    parsed_channel.ntype = ntype;
    parsed_channel.name = name;
    parsed_channel.unit = "";
    parsed_channel.sensor_ref = nullptr;

    if (ctype == CTYPE_SPOT_ANALOG) {
      parsed_channel.unit = cstr_fixed_(data + p, 8);
      parsed_channel.data.analog.gain = read_f32_le_(data + p + 8);
      parsed_channel.data.analog.offset = read_f32_le_(data + p + 12);
    } else if (ctype == CTYPE_SPOT_COUNTER) {
      parsed_channel.unit = cstr_fixed_(data + p, 8);
      parsed_channel.data.counter.gain = read_f32_le_(data + p + 8);
    } else if (ctype == CTYPE_SPOT_STATUS) {
      parsed_channel.data.status.sizet = 0;
      parsed_channel.data.status.text = nullptr;

      const uint16_t sizet = read_u16_le_(data + p);
      parsed_channel.data.status.sizet = sizet;
      if (sizet > 0) {
        parsed_channel.data.status.text = static_cast<char *>(malloc(static_cast<size_t>(sizet) + 1));
        if (parsed_channel.data.status.text == nullptr) {
          ESP_LOGW(TAG, "cinfo parse stop: status malloc failed offset=%u size=%u", (unsigned) offset, (unsigned) sizet);
          break;
        }
        std::memcpy(parsed_channel.data.status.text, data + p + 2, sizet);
        parsed_channel.data.status.text[sizet] = '\0';
      }
    }

    std::string dbg = parsed_channel.name;
    if (!parsed_channel.unit.empty()) {
      dbg += " [" + parsed_channel.unit + "]";
    }

    std::string details;

    if (parsed_channel.ctype == CTYPE_SPOT_ANALOG) {
      if (parsed_channel.data.analog.gain != 1.0f) {
        char gain_buf[48];
        std::snprintf(gain_buf, sizeof(gain_buf), "gain=%.6g", parsed_channel.data.analog.gain);
        details += gain_buf;
      }
      if (parsed_channel.data.analog.offset != 0.0f) {
        if (!details.empty()) details += " ";
        char offset_buf[48];
        std::snprintf(offset_buf, sizeof(offset_buf), "offset=%.6g", parsed_channel.data.analog.offset);
        details += offset_buf;
      }
    } else if (parsed_channel.ctype == CTYPE_SPOT_COUNTER) {
      if (parsed_channel.data.counter.gain != 1.0f) {
        char gain_buf[48];
        std::snprintf(gain_buf, sizeof(gain_buf), "gain=%.6g", parsed_channel.data.counter.gain);
        details += gain_buf;
      }
    } else if (parsed_channel.ctype == CTYPE_SPOT_STATUS) {
      std::string status_joined;
      if (parsed_channel.data.status.text != nullptr && parsed_channel.data.status.sizet > 0) {
        status_joined.reserve(parsed_channel.data.status.sizet + 8);
        for (size_t i = 0; i < parsed_channel.data.status.sizet; i++) {
          const char c = parsed_channel.data.status.text[i];
          if (c == '\0') {
            if (!status_joined.empty() && status_joined.back() != ',') {
              status_joined.push_back(',');
              status_joined.push_back(' ');
            }
          } else {
            status_joined.push_back(c);
          }
        }
        while (!status_joined.empty() &&
               (status_joined.back() == ' ' || status_joined.back() == ',')) {
          status_joined.pop_back();
        }
      }
      if (!status_joined.empty()) {
        details = "states=[" + status_joined + "]";
      }
    }

    if (!details.empty()) {
      dbg += ": ";
      dbg += details;
    }

    const bool is_status_channel = (parsed_channel.ctype == CTYPE_SPOT_STATUS);
    bool matched = false;
    auto sensor_binding_it = this->sensors_by_channel_name_.find(name);
    auto text_binding_it = this->text_sensors_by_channel_name_.find(name);

    if (is_status_channel) {
      if (text_binding_it != this->text_sensors_by_channel_name_.end() && text_binding_it->second.sensor != nullptr) {
        parsed_channel.text_sensor_ref = text_binding_it->second.sensor;
        dbg += " -> TextSensor \"" + text_binding_it->second.sensor->get_name() + "\"";
        matched = true;
      } else if (sensor_binding_it != this->sensors_by_channel_name_.end() && sensor_binding_it->second.sensor != nullptr) {
        ESP_LOGW(TAG,
                 "cinfo mapping skipped: status channel '%s' requires TextSensor (Sensor binding ignored)",
                 name.c_str());
      }
    } else {
      if (sensor_binding_it != this->sensors_by_channel_name_.end() && sensor_binding_it->second.sensor != nullptr) {
        parsed_channel.sensor_ref = sensor_binding_it->second.sensor;
        dbg += " -> Sensor \"" + sensor_binding_it->second.sensor->get_name() + "\"";
        if (sensor_binding_it->second.is_fast) {
          dbg += " (fast)";
        }
        matched = true;
      } else if (text_binding_it != this->text_sensors_by_channel_name_.end() && text_binding_it->second.sensor != nullptr) {
        ESP_LOGW(TAG,
                 "cinfo mapping skipped: numeric channel '%s' requires Sensor (TextSensor binding ignored)",
                 name.c_str());
      }
    }
    ESP_LOGI(TAG, "%s", dbg.c_str());

    if (!matched) {
      if (parsed_channel.ctype == CTYPE_SPOT_STATUS && parsed_channel.data.status.text != nullptr) {
        free(parsed_channel.data.status.text);
        parsed_channel.data.status.text = nullptr;
      }
      spot_cidx++;
      offset = next_chan_offset;
      continue;
    }

    this->cinfo_spot_channels_.emplace_back(std::move(parsed_channel));
    spot_cidx++;
    offset = next_chan_offset;
  }

  if (this->debug_) {
    const uint32_t matched = static_cast<uint32_t>(this->cinfo_spot_channels_.size());
    ESP_LOGD(TAG,
             "cinfo parse complete: total_spot_channels=%u matched_sensors=%u registered_sensors=%u",
             (unsigned) this->cinfo_spot_channels_.size(),
             (unsigned) matched,
             (unsigned) this->sensors_by_channel_name_.size());
  }
}

void SmaNetComponent::parse_cdata_payload_(const uint8_t *data, size_t len) {
  if (data == nullptr || len < CDATA_HDR_LEN) return;

  // 13-Byte-Nutzdatenkopf (CMDGETDATA):
  // 0..2  : Übertragungsmaske (ctype LE + channel_index)
  // 3..4  : Anzahl Datensätze (u16 LE)
  // 5..8  : Speicherzeitpunkt 1 (u32 LE, Unix-Zeit)
  // 9..12 : Zeitbasis 1 (u32 LE, Sekunden)
  const uint16_t ctype = read_u16_le_(data + CDATA_HDR_CTYPE_OFF);
  const uint8_t cidx = data[CDATA_HDR_CIDX_OFF];
  const uint16_t record_count = read_u16_le_(data + CDATA_HDR_RECORD_COUNT_OFF);
  const uint32_t cdata_ts = read_u32_le_(data + CDATA_HDR_TIMESTAMP_OFF);
  const uint32_t time_base_s = read_u32_le_(data + CDATA_HDR_TIME_BASE_OFF);

  if (this->debug_) {
    ESP_LOGV(TAG,
             "cdata header: mask_type=0x%04X mask_index=%u records=%u ts=%u base_s=%u",
             ctype,
             (unsigned) cidx,
             (unsigned) record_count,
             (unsigned) cdata_ts,
             (unsigned) time_base_s);
    log_payload_hex_(data, len, "sma_cdata");
  }

  const bool is_all = (cidx == CHANNEL_ALL && ctype == CTYPE_SPOT_ALL);
  for (const auto &channel : this->cinfo_spot_channels_) {
    const bool single_match = (channel.ctype == ctype && channel.cidx == cidx);
    if (!is_all && !single_match) continue;

    const size_t value_offset = is_all
                                    ? (CDATA_HDR_LEN + (static_cast<size_t>(channel.spot_cidx) * 4))
                                    : CDATA_HDR_LEN;

    if (has_bytes_(len, value_offset, 4)) {
      const uint32_t raw_u32 = read_u32_le_(data + value_offset);
      const bool is_float = (channel.ntype == NTYPE_FLOAT);
      const bool is_u32 = (channel.ntype == NTYPE_U32);

      if (channel.ctype == CTYPE_SPOT_STATUS) {
        if (channel.text_sensor_ref != nullptr) {
          std::string state = "unknown";
          if (channel.data.status.text != nullptr && channel.data.status.sizet > 0) {
            size_t idx = 0;
            size_t pos = 0;
            while (pos < channel.data.status.sizet) {
              size_t start = pos;
              while (pos < channel.data.status.sizet && channel.data.status.text[pos] != '\0') pos++;
              if (idx == raw_u32) {
                state.assign(channel.data.status.text + start, pos - start);
                break;
              }
              idx++;
              pos++;
            }
          }
          channel.text_sensor_ref->publish_state(state);
        }
      } else if (is_float || is_u32) {
        float base_value = is_float ? read_f32_le_(data + value_offset) : static_cast<float>(raw_u32);
        float value = base_value;

        if (channel.ctype == CTYPE_SPOT_ANALOG) {
          value = base_value * channel.data.analog.gain + channel.data.analog.offset;
        } else if (channel.ctype == CTYPE_SPOT_COUNTER) {
          value = base_value * channel.data.counter.gain;
        }

        if (channel.sensor_ref != nullptr) {
          channel.sensor_ref->publish_state(value);
        }
      } else if (this->debug_) {
        ESP_LOGW(TAG,
                 "cdata value skipped: %s unsupported ntype=0x%04X ctype=0x%04X",
                 channel.name.c_str(),
                 channel.ntype,
                 channel.ctype);
      }
    }

    if (!is_all) break;
  }

  if (this->pending_getdata_channels_.empty()) {
    this->publish_data_timestamp_(cdata_ts);
  }
}

}  // namespace sma_net
}  // namespace esphome
