#ifndef CANOPEN_BRIDGE_CANOPEN_INTERFACE_HPP
#define CANOPEN_BRIDGE_CANOPEN_INTERFACE_HPP

#include <cstdint>
#include <stdexcept>
#include <string>

namespace canopen_bridge
{

/// Thrown by the CAN layer on a bad/missing response or a bus timeout.
/// The node catches this to drive RCLCPP_ERROR logging and an error state.
class CanIoError : public std::runtime_error
{
public:
  explicit CanIoError(const std::string & msg)
  : std::runtime_error(msg) {}
};

/// Abstract CANopen transport.
///
/// The bridge node depends only on this interface, so the mock below can be
/// swapped for a real SocketCAN / ros2_canopen backend without changing any
/// node logic.
class CanopenInterface
{
public:
  virtual ~CanopenInterface() = default;

  /// Write `value` to object dictionary entry [index:subindex] (RPDO path).
  /// Throws CanIoError if the object is not writable / does not exist.
  virtual void sendRPDO(uint16_t index, uint8_t subindex, int value) = 0;

  /// Read object dictionary entry [index:subindex] (TPDO path).
  /// Throws CanIoError on timeout / invalid response / unknown object.
  virtual int readTPDO(uint16_t index, uint8_t subindex) = 0;
};

/// In-memory mock of a velocity-controlled CANopen device.
///
/// Object dictionary it models:
///   RPDO  0x2000:0x00  commanded velocity (int16, scaled)
///   TPDO  0x3000:0x00  actual velocity    (int16, scaled)
///   TPDO  0x3000:0x01  device status      (0 = OK)
///   TPDO  0x3000:0x02  error code         (0 = no error)
///
/// The actual velocity lags the command by a fixed slip factor to make the
/// feedback loop look physically plausible (e.g. cmd 1500 -> actual ~1450).
class MockCanopenInterface : public CanopenInterface
{
public:
  MockCanopenInterface() = default;

  void sendRPDO(uint16_t index, uint8_t subindex, int value) override;
  int readTPDO(uint16_t index, uint8_t subindex) override;

  /// Demo/test hook: when enabled, readTPDO randomly simulates a bus timeout
  /// so the node's error-handling path can be exercised at runtime.
  void setFaultInjection(bool enabled) { fault_injection_ = enabled; }

private:
  int16_t commanded_velocity_ {0};
  bool fault_injection_ {false};
};

}  // namespace canopen_bridge

#endif  // CANOPEN_BRIDGE_CANOPEN_INTERFACE_HPP
