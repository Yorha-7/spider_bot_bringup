#include "big_bertha_hardware_bringup/router_bridge.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "big_bertha_hardware_bringup/mini_msgpack.hpp"

// -----------------------------------------------------------------------
// RouterBridge
// -----------------------------------------------------------------------

RouterBridge::RouterBridge(const std::string & sock_path)
: sock_path_(sock_path), sock_(-1), next_id_(1) {
  connect();
}

RouterBridge::~RouterBridge() { close(); }

void RouterBridge::connect() {
  if (sock_ >= 0) ::close(sock_);
  sock_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (sock_ < 0) {
    throw std::runtime_error("RouterBridge: socket() failed");
  }
  struct sockaddr_un addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  size_t path_len = std::min(sock_path_.size(), sizeof(addr.sun_path) - 1);
  std::memcpy(addr.sun_path, sock_path_.data(), path_len);
  addr.sun_path[path_len] = '\0';
  if (::connect(sock_, reinterpret_cast<struct sockaddr *>(&addr),
                sizeof(addr)) < 0) {
    ::close(sock_);
    sock_ = -1;
    throw std::runtime_error("RouterBridge: connect(" + sock_path_ + ") failed");
  }
  next_id_ = 1;
}

void RouterBridge::send_raw(const std::vector<uint8_t> & data) {
  const uint8_t * ptr = data.data();
  size_t remaining = data.size();
  while (remaining > 0) {
    ssize_t n = ::write(sock_, ptr, remaining);
    if (n < 0) {
      close();
      connect();
      n = ::write(sock_, ptr, remaining);
      if (n < 0) {
        throw std::runtime_error("RouterBridge: write failed after reconnect");
      }
    }
    ptr += n;
    remaining -= static_cast<size_t>(n);
  }
}

std::vector<uint8_t> RouterBridge::recv_raw() {
  std::vector<uint8_t> buf;
  buf.reserve(1024);

  struct pollfd pfd;
  pfd.fd = sock_;
  pfd.events = POLLIN;

  // Wait up to 5s for the first byte of the response
  {
    int ret;
    do {
      ret = poll(&pfd, 1, 5000);
    } while (ret < 0 && errno == EINTR);

    if (ret < 0) {
      throw std::runtime_error("RouterBridge: poll() failed");
    }
    if (ret == 0) {
      throw std::runtime_error("RouterBridge: timeout waiting for response");
    }
  }

  // Read all chunks with a short gap timeout
  while (true) {
    uint8_t chunk[8192];
    ssize_t n;
    do {
      n = ::read(sock_, chunk, sizeof(chunk));
    } while (n < 0 && errno == EINTR);

    if (n > 0) {
      buf.insert(buf.end(), chunk, chunk + n);
    } else if (n == 0) {
      break;
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // Spurious poll wakeup — poll again
      int ret;
      do {
        ret = poll(&pfd, 1, 10);
      } while (ret < 0 && errno == EINTR);
      if (ret <= 0 || !(pfd.revents & POLLIN)) break;
      continue;
    } else {
      throw std::runtime_error("RouterBridge: read error");
    }

    // Try parsing — if complete, return immediately
    try {
      mini_msgpack::Unpacker chk(buf);
      while (!chk.done()) { chk.next(); }
      return buf;
    } catch (const std::runtime_error &) {
      // Incomplete — wait up to 10ms for more
    }

    int ret;
    do {
      ret = poll(&pfd, 1, 10);
    } while (ret < 0 && errno == EINTR);
    if (ret <= 0 || !(pfd.revents & POLLIN)) break;
  }

  if (buf.empty()) {
    throw std::runtime_error("RouterBridge: no data received");
  }
  return buf;
}

std::string RouterBridge::call(const std::string & method,
                               const std::string & args) {
  // Reuse the same socket across calls for low latency
  if (sock_ < 0) {
    connect();
  }

  std::vector<uint8_t> buf;
  mini_msgpack::Packer pk(buf);
  pk.pack_array(4);       // [0, id, method, args...]
  pk.pack_int(0);         // request type
  pk.pack_int(next_id_++);
  pk.pack_str(method);
  if (args.empty()) {
    pk.pack_array(0);     // no args
  } else {
    pk.pack_array(1);     // one string arg
    pk.pack_str(args);
  }

  try {
    send_raw(buf);
  } catch (const std::runtime_error & e) {
    close();
    throw;
  }

  std::vector<uint8_t> resp;
  try {
    resp = recv_raw();
  } catch (const std::runtime_error &) {
    // recv failed — socket may have stale data; reconnect on next call
    close();
    throw;
  }

  if (resp.empty()) {
    close();
    throw std::runtime_error("RouterBridge: empty response");
  }

  mini_msgpack::Unpacker up(resp);
  try {
    up.expect(mini_msgpack::Unpacker::Value::ARRAY);  // [type, id, error, result]
    up.next();  // skip type
    up.next();  // skip id
    mini_msgpack::Unpacker::Value err_val = up.next();
    if (err_val.type != mini_msgpack::Unpacker::Value::NIL) {
      std::string err_msg = "RPC error";
      if (err_val.type == mini_msgpack::Unpacker::Value::STR) {
        err_msg = err_val.str_val;
      } else if (err_val.type == mini_msgpack::Unpacker::Value::ARRAY) {
        up.next();  // skip error code
        mini_msgpack::Unpacker::Value msg_val = up.next();
        if (msg_val.type == mini_msgpack::Unpacker::Value::STR) {
          err_msg = msg_val.str_val;
        }
      }
      throw std::runtime_error("RouterBridge RPC error: " + err_msg);
    }
    mini_msgpack::Unpacker::Value result = up.next();
    if (result.type == mini_msgpack::Unpacker::Value::NIL) {
      return "";
    }
    if (result.type == mini_msgpack::Unpacker::Value::STR) {
      return result.str_val;
    }
    return "";
  } catch (const std::runtime_error & e) {
    std::string hex;
    for (size_t i = 0; i < resp.size(); ++i) {
      char buf8[4];
      std::snprintf(buf8, sizeof(buf8), "%02x", resp[i]);
      hex += buf8;
    }
    throw std::runtime_error(std::string(e.what()) + " [raw: " + hex + "]");
  }
}

void RouterBridge::close() {
  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
}

// -----------------------------------------------------------------------
// MockBridge
// -----------------------------------------------------------------------

std::string MockBridge::call(const std::string & method,
                             const std::string & args) {
  (void)args;
  if (method == "read_imu") {
    return "0.0 0.0 0.0 0.0 0.0 9.81";
  }
  return "ok";
}

void MockBridge::close() {}
