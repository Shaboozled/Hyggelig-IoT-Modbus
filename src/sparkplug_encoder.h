#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>

namespace sparkplug {

struct Buffer {
  uint8_t data[256]{};
  size_t size = 0;
  bool valid = true;

  void bytes(const void* value, size_t length) {
    if (!valid || length > sizeof(data) - size) { valid = false; return; }
    memcpy(data + size, value, length);
    size += length;
  }

  void varint(uint64_t value) {
    while (value >= 128) {
      const uint8_t next = (value & 127) | 128;
      bytes(&next, 1);
      value >>= 7;
    }
    const uint8_t last = value;
    bytes(&last, 1);
  }

  void integer(uint8_t field, uint64_t value) {
    varint(field << 3);
    varint(value);
  }

  void blob(uint8_t field, const void* value, size_t length) {
    varint((field << 3) | 2);
    varint(length);
    bytes(value, length);
  }

  void metric(const Buffer& value) {
    if (!value.valid) { valid = false; return; }
    blob(2, value.data, value.size);
  }
};

inline Buffer payload(uint64_t timestamp, int sequence) {
  Buffer message;
  message.integer(1, timestamp);
  if (sequence >= 0) message.integer(3, sequence);
  return message;
}

inline Buffer named(const char* name, uint64_t timestamp, uint8_t type) {
  Buffer metric;
  metric.blob(1, name, strlen(name));
  metric.integer(3, timestamp);
  metric.integer(4, type);
  return metric;
}

inline void add_bd_seq(Buffer& message, uint64_t timestamp, uint8_t seq) {
  auto metric = named("bdSeq", timestamp, 4);
  metric.integer(11, seq);
  message.metric(metric);
}

inline void add_rebirth_control(Buffer& message, uint64_t timestamp) {
  auto metric = named("Node Control/Rebirth", timestamp, 11);
  metric.integer(14, 0);
  message.metric(metric);
}

inline void add_double(Buffer& message, const char* name, uint8_t alias,
                       uint64_t timestamp, double value) {
  Buffer metric;
  if (name) metric = named(name, timestamp, 10);
  else metric.integer(3, timestamp);
  metric.integer(2, alias);
  if (std::isfinite(value)) {
    static_assert(sizeof(double) == 8, "Protobuf Double requires 64-bit doubles");
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    metric.varint((13 << 3) | 1);
    for (unsigned i = 0; i < 8; ++i) {
      const uint8_t byte = bits >> (i * 8);
      metric.bytes(&byte, 1);
    }
  } else {
    metric.integer(7, 1);
  }
  message.metric(metric);
}

struct Reader {
  const uint8_t* data;
  size_t remaining;

  bool varint(uint64_t& value) {
    value = 0;
    for (unsigned shift = 0; shift <= 63 && remaining; shift += 7) {
      const uint8_t byte = *data++;
      --remaining;
      if (shift == 63 && byte > 1) return false;
      value |= uint64_t(byte & 127) << shift;
      if (!(byte & 128)) return true;
    }
    return false;
  }

  bool take(uint64_t length, Reader& out) {
    if (length > remaining) return false;
    out = {data, size_t(length)};
    data += length;
    remaining -= length;
    return true;
  }

  bool skip(uint64_t wire) {
    uint64_t value = 0;
    Reader ignored{};
    if (wire == 0) return varint(value);
    if (wire == 1) return take(8, ignored);
    if (wire == 2) return varint(value) && take(value, ignored);
    if (wire == 5) return take(4, ignored);
    return false;
  }
};

inline bool is_rebirth_metric(Reader metric) {
  constexpr char name[] = "Node Control/Rebirth";
  bool match = false;
  bool enabled = false;
  while (metric.remaining) {
    uint64_t key, value;
    if (!metric.varint(key) || key == 0) return false;
    if (key == ((1 << 3) | 2)) {
      Reader text{};
      if (!metric.varint(value) || !metric.take(value, text)) return false;
      match = text.remaining == sizeof(name) - 1 && !memcmp(text.data, name, sizeof(name) - 1);
    } else if (key == (14 << 3)) {
      if (!metric.varint(value)) return false;
      enabled = value == 1;
    } else if (!metric.skip(key & 7)) return false;
  }
  return match && enabled;
}

inline bool is_rebirth(const uint8_t* bytes, size_t length) {
  Reader message{bytes, length};
  bool rebirth = false;
  while (message.remaining) {
    uint64_t key, size;
    if (!message.varint(key) || key == 0) return false;
    if (key == ((2 << 3) | 2)) {
      Reader metric{};
      if (!message.varint(size) || !message.take(size, metric)) return false;
      rebirth |= is_rebirth_metric(metric);
    } else if (!message.skip(key & 7)) return false;
  }
  return rebirth;
}

}  // namespace sparkplug
