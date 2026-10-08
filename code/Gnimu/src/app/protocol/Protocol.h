#pragma once

#include "../Config.h"
#include "RaceBox.h"

namespace gnimu {

// The protocol class for config::kProtocol, chosen at compile time with no
// virtual dispatch. A Protocol value without a specialisation here fails the
// build.
template <config::Protocol P>
struct ProtocolFor;
template <>
struct ProtocolFor<config::Protocol::RaceBox> {
  using type = RaceBoxProtocol;
};

using Protocol = ProtocolFor<config::kProtocol>::type;

}  // namespace gnimu
