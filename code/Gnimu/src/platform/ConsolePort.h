#pragma once

namespace gnimu {

// Opens the serial console and waits long enough for a host to be listening,
// so boot messages aren't lost. Implemented once per platform.
void consoleOpen();

}  // namespace gnimu
