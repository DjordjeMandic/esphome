#pragma once

#ifdef USE_ARDUINO
#ifdef USE_REMOTE_TRANSMITTER
#include "esphome/components/remote_base/midea_protocol.h"
#include "esphome/components/midea_ir/midea_data.h"

namespace esphome {
namespace midea {

using remote_base::RemoteTransmitterBase;
using IrData = remote_base::MideaData;
using IrFollowMeData = midea_ir::FollowMeData;
using IrSpecialData = midea_ir::SpecialData;

class IrTransmitter {
 public:
  void set_transmitter(RemoteTransmitterBase *transmitter) { this->transmitter_ = transmitter; }
  void transmit(IrData &data) {
    data.finalize();
    auto transmit = this->transmitter_->transmit();
    remote_base::MideaProtocol().encode(transmit.get_data(), data);
    transmit.perform();
  }

 protected:
  RemoteTransmitterBase *transmitter_{nullptr};
};

}  // namespace midea
}  // namespace esphome

#endif
#endif  // USE_ARDUINO
