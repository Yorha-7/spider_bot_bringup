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
    } else if (v >= 128 && v <= 255) {
      buf_.push_back(0xCC);
      buf_.push_back(static_cast<uint8_t>(v));
    } else if (v >= 256 && v <= 65535) {
      buf_.push_back(0xCD);
      buf_.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
      buf_.push_back(static_cast<uint8_t>(v & 0xFF));
    } else if (v >= -32 && v <= -1) {
      buf_.push_back(static_cast<uint8_t>(static_cast<int8_t>(v)));
    } else if (v >= -128 && v <= -33) {
      buf_.push_back(0xD0);
      buf_.push_back(static_cast<uint8_t>(static_cast<int8_t>(v)));
    } else if (v >= -32768 && v <= -129) {
      buf_.push_back(0xD1);
      buf_.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
      buf_.push_back(static_cast<uint8_t>(v & 0xFF));
    } else {
      buf_.push_back(0xD2);
      buf_.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
      buf_.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
      buf_.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
      buf_.push_back(static_cast<uint8_t>(v & 0xFF));
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
    if (len <= 31) {
      buf_.push_back(static_cast<uint8_t>(0xA0 | len));
    } else if (len <= 255) {
      buf_.push_back(0xD9);
      buf_.push_back(static_cast<uint8_t>(len));
    } else if (len <= 65535) {
      buf_.push_back(0xDA);
      buf_.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
      buf_.push_back(static_cast<uint8_t>(len & 0xFF));
    } else {
      buf_.push_back(0xDB);
      for (int i = 3; i >= 0; --i) {
        buf_.push_back(static_cast<uint8_t>((len >> (i * 8)) & 0xFF));
      }
    }
    buf_.insert(buf_.end(), s.begin(), s.end());
  }

  void pack_array(size_t n) {
    if (n <= 15) {
      buf_.push_back(static_cast<uint8_t>(0x90 | n));
    } else if (n <= 65535) {
      buf_.push_back(0xDC);
      buf_.push_back(static_cast<uint8_t>((n >> 8) & 0xFF));
      buf_.push_back(static_cast<uint8_t>(n & 0xFF));
    } else {
      buf_.push_back(0xDD);
      for (int i = 3; i >= 0; --i) {
        buf_.push_back(static_cast<uint8_t>((n >> (i * 8)) & 0xFF));
      }
    }
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
    if ((b & 0x80) == 0) {
      return {Value::INT, static_cast<int64_t>(b), 0.0, "", 0};
    }
    if ((b & 0xE0) == 0xE0) {
      return {Value::INT, static_cast<int64_t>(static_cast<int8_t>(b)),
              0.0, "", 0};
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
    if (b >= 0xCC && b <= 0xCF) {
      uint64_t val = 0;
      int bytes = 1 << (b - 0xCC);
      for (int i = 0; i < bytes; ++i) {
        val = (val << 8) | read_byte();
      }
      return {Value::INT, static_cast<int64_t>(val), 0.0, "", 0};
    }
    if (b >= 0xD0 && b <= 0xD3) {
      int64_t val = 0;
      int bytes = 1 << (b - 0xD0);
      bool negative = (buf_[pos_] & 0x80) != 0;
      if (negative) val = -1;
      for (int i = 0; i < bytes; ++i) {
        val = (val << 8) | read_byte();
      }
      return {Value::INT, val, 0.0, "", 0};
    }
    if ((b & 0xE0) == 0xA0) {
      size_t len = b & 0x1F;
      std::string s;
      for (size_t i = 0; i < len; ++i) {
        s.push_back(static_cast<char>(read_byte()));
      }
      return {Value::STR, 0, 0.0, s, 0};
    }
    if (b == 0xD9) {
      size_t len = read_byte();
      std::string s;
      for (size_t i = 0; i < len; ++i) {
        s.push_back(static_cast<char>(read_byte()));
      }
      return {Value::STR, 0, 0.0, s, 0};
    }
    if (b == 0xDA) {
      size_t len = (static_cast<size_t>(read_byte()) << 8) | read_byte();
      std::string s;
      for (size_t i = 0; i < len; ++i) {
        s.push_back(static_cast<char>(read_byte()));
      }
      return {Value::STR, 0, 0.0, s, 0};
    }
    if (b == 0xDB) {
      size_t len = 0;
      for (int i = 0; i < 4; ++i) {
        len = (len << 8) | read_byte();
      }
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
