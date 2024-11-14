#pragma once

#ifdef USE_ARDUINO
#ifdef USE_REMOTE_TRANSMITTER
#include "esphome/components/remote_base/midea_protocol.h"

namespace esphome {
namespace midea {

using remote_base::RemoteTransmitterBase;
using IrData = remote_base::MideaData;

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
