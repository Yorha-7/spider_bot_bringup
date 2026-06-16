#include "big_bertha_hardware_bringup/router_bridge.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
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
  // set a 1 second timeout on the socket
  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  while (true) {
    uint8_t chunk[4096];
    ssize_t n = ::read(sock_, chunk, sizeof(chunk));
    if (n < 0) {
      if (buf.empty()) {
        throw std::runtime_error("RouterBridge: read timeout with no data");
      }
      break;  // partial data received, try to parse what we have
    }
    if (n == 0) {
      break;  // EOF
    }
    buf.insert(buf.end(), chunk, chunk + n);
    // Try to parse: if Unpacker succeeds and we've consumed the whole buffer, we're done
    try {
      mini_msgpack::Unpacker chk(buf);
      while (!chk.done()) {
        chk.next();
      }
      break;  // successfully parsed complete msgpack
    } catch (const std::runtime_error &) {
      continue;  // incomplete, need more data
    }
  }
  return buf;
}

std::string RouterBridge::call(const std::string & method,
                               const std::string & args) {
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

  if (sock_ < 0) {
    connect();
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
  } catch (const std::runtime_error & e) {
    close();
    throw;
  }

  if (resp.empty()) {
    close();
    throw std::runtime_error("RouterBridge: empty response");
  }

  mini_msgpack::Unpacker up(resp);
  up.expect(mini_msgpack::Unpacker::Value::ARRAY);  // [type, id, error, result]
  up.next();  // skip type
  up.next();  // skip id
  mini_msgpack::Unpacker::Value err_val = up.next();
  if (err_val.type != mini_msgpack::Unpacker::Value::NIL) {
    std::string err = err_val.type == mini_msgpack::Unpacker::Value::STR
                          ? err_val.str_val
                          : "RPC error";
    throw std::runtime_error("RouterBridge RPC error: " + err);
  }
  mini_msgpack::Unpacker::Value result = up.next();
  if (result.type == mini_msgpack::Unpacker::Value::NIL) {
    return "";
  }
  return result.str_val;
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
