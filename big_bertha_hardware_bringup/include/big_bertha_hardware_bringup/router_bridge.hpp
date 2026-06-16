#ifndef BIG_BERTHA_HARDWARE_BRINGUP__ROUTER_BRIDGE_HPP_
#define BIG_BERTHA_HARDWARE_BRINGUP__ROUTER_BRIDGE_HPP_

#include <cstdint>
#include <string>
#include <vector>

class BridgeInterface {
public:
  virtual ~BridgeInterface() = default;
  virtual std::string call(const std::string & method,
                           const std::string & args = "") = 0;
  virtual void close() = 0;
};

class RouterBridge : public BridgeInterface {
public:
  explicit RouterBridge(const std::string & sock_path);
  ~RouterBridge() override;

  std::string call(const std::string & method,
                   const std::string & args = "") override;
  void close() override;

private:
  void connect();
  void send_raw(const std::vector<uint8_t> & data);
  std::vector<uint8_t> recv_raw();

  std::string sock_path_;
  int sock_;
  int next_id_;
};

class MockBridge : public BridgeInterface {
public:
  std::string call(const std::string & method,
                   const std::string & args = "") override;
  void close() override;
};

#endif  // BIG_BERTHA_HARDWARE_BRINGUP__ROUTER_BRIDGE_HPP_
