#include "canopen_interface.hpp"

#include <cmath>
#include <random>
#include <sstream>
#include <string>

namespace canopen_bridge
{
namespace
{
constexpr uint16_t kRpdoIndex = 0x2000;
constexpr uint16_t kTpdoIndex = 0x3000;
constexpr uint8_t kSubVelocity = 0x00;
constexpr uint8_t kSubStatus = 0x01;
constexpr uint8_t kSubError = 0x02;

// Simulated mechanical/response lag: 1500 -> 1450 (matches the spec example).
constexpr double kSlipFactor = 0.9666;

std::string to_hex(uint16_t index, uint8_t subindex)
{
  std::ostringstream ss;
  ss << "0x" << std::hex << std::uppercase << index
     << ":0x" << static_cast<int>(subindex);
  return ss.str();
}
}  // namespace

void MockCanopenInterface::sendRPDO(uint16_t index, uint8_t subindex, int value)
{
  if (index != kRpdoIndex || subindex != kSubVelocity) {
    throw CanIoError("sendRPDO: object " + to_hex(index, subindex) +
                     " is not a writable RPDO entry");
  }
  commanded_velocity_ = static_cast<int16_t>(value);
}

int MockCanopenInterface::readTPDO(uint16_t index, uint8_t subindex)
{
  if (fault_injection_) {
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(rng) < 0.05) {
      throw CanIoError("readTPDO: simulated bus timeout (no response within window)");
    }
  }

  if (index != kTpdoIndex) {
    throw CanIoError("readTPDO: unknown index " + to_hex(index, subindex));
  }

  switch (subindex) {
    case kSubVelocity:
      return static_cast<int16_t>(std::lround(commanded_velocity_ * kSlipFactor));
    case kSubStatus:
      return 0;  // 0 = OK
    case kSubError:
      return 0;  // 0 = no error
    default:
      throw CanIoError("readTPDO: unknown subindex in " + to_hex(index, subindex));
  }
}

}  // namespace canopen_bridge
