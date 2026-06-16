#ifndef BIG_BERTHA_HARDWARE_BRINGUP__MINI_MSGPACK_HPP_
#define BIG_BERTHA_HARDWARE_BRINGUP__MINI_MSGPACK_HPP_

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace mini_msgpack {

// -- Marker bytes --
constexpr uint8_t NIL = 0xC0;
constexpr uint8_t FLOAT64 = 0xCB;

// -- Encoder --

class Packer {
public:
  explicit Packer(std::vector<uint8_t> & buf) : buf_(buf) {}

  void pack_nil() { buf_.push_back(NIL); }

  void pack_int(int64_t v) {
    if (v >= 0 && v <= 127) {
      buf_.push_back(static_cast<uint8_t>(v));
    } else {
      throw std::runtime_error("mini_msgpack: large ints not implemented");
    }
  }

  void pack_float(double v) {
    buf_.push_back(FLOAT64);
    uint64_t bits;
    static_assert(sizeof(bits) == sizeof(v));
    std::memcpy(&bits, &v, sizeof(bits));
    for (int i = 7; i >= 0; --i) {
      buf_.push_back(static_cast<uint8_t>(bits >> (i * 8)));
    }
  }

  void pack_str(const std::string & s) {
    size_t len = s.size();
    if (len > 31) {
      throw std::runtime_error("mini_msgpack: long strings not implemented");
    }
    buf_.push_back(static_cast<uint8_t>(0xA0 | len));
    buf_.insert(buf_.end(), s.begin(), s.end());
  }

  void pack_array(size_t n) {
    if (n > 15) {
      throw std::runtime_error("mini_msgpack: large arrays not implemented");
    }
    buf_.push_back(static_cast<uint8_t>(0x90 | n));
  }

  template<typename... Args>
  void pack_array(int64_t v, Args... args) {
    // no-op, use explicit pack_array() + pack_*
  }

private:
  std::vector<uint8_t> & buf_;
};

// -- Decoder --

class Unpacker {
public:
  explicit Unpacker(const std::vector<uint8_t> & buf) : buf_(buf), pos_(0) {}

  struct Value {
    enum Type { NIL, INT, FLOAT, STR, ARRAY } type;
    int64_t int_val;
    double float_val;
    std::string str_val;
    size_t array_len;
  };

  Value next() {
    if (pos_ >= buf_.size()) {
      throw std::runtime_error("mini_msgpack: unexpected end of data");
    }
    uint8_t b = buf_[pos_++];
    if (b == NIL) {
      return {Value::NIL, 0, 0.0, "", 0};
    }
    if (b == FLOAT64) {
      uint64_t bits = 0;
      for (int i = 0; i < 8; ++i) {
        bits = (bits << 8) | read_byte();
      }
      double val;
      std::memcpy(&val, &bits, sizeof(val));
      return {Value::FLOAT, 0, val, "", 0};
    }
    if ((b & 0x80) == 0) {
      return {Value::INT, static_cast<int64_t>(b), 0.0, "", 0};
    }
    if ((b & 0xE0) == 0xA0) {
      size_t len = b & 0x1F;
      std::string s;
      for (size_t i = 0; i < len; ++i) {
        s.push_back(static_cast<char>(read_byte()));
      }
      return {Value::STR, 0, 0.0, s, 0};
    }
    if ((b & 0xF0) == 0x90) {
      return {Value::ARRAY, 0, 0.0, "", static_cast<size_t>(b & 0x0F)};
    }
    throw std::runtime_error("mini_msgpack: unsupported type byte: " +
                             std::to_string(b));
  }

  Value expect(Value::Type t) {
    Value v = next();
    if (v.type != t) {
      throw std::runtime_error("mini_msgpack: type mismatch");
    }
    return v;
  }

  uint8_t read_byte() {
    if (pos_ >= buf_.size()) {
      throw std::runtime_error("mini_msgpack: unexpected end of data");
    }
    return buf_[pos_++];
  }

  bool done() const { return pos_ >= buf_.size(); }

private:
  const std::vector<uint8_t> & buf_;
  size_t pos_;
};

}  // namespace mini_msgpack

#endif  // BIG_BERTHA_HARDWARE_BRINGUP__MINI_MSGPACK_HPP_
