#pragma once
// GPIO abstraction.
//
// Backend: wiringPi (Orange Pi). Pin numbers in config.hpp are wiringPi numbers
// (verify with `gpio readall` on the target board). Compile with USE_WIRINGPI
// (set automatically by CMake when libwiringPi is found); without it the Mock
// backend is used so the project still builds on a dev machine.

#include <memory>

namespace r3d::gpio {

enum class Dir { Output, Input };

// Abstract GPIO controller. Pins are referenced by the wiringPi number from config.
class IGpio {
public:
    virtual ~IGpio() = default;
    virtual bool setup() = 0;                 // initialize the backend (e.g. wiringPiSetup)
    virtual void set_mode(int pin, Dir d, bool initial_high) = 0;
    virtual void write(int pin, bool high) = 0;
    virtual bool read(int pin) = 0;
    virtual void cleanup() = 0;               // turn everything off, release
    virtual const char* backend_name() const = 0;
};

// Factory: returns WiringPiGpio when compiled with USE_WIRINGPI, otherwise the
// logging MockGpio (for host development).
std::unique_ptr<IGpio> make_gpio();

} // namespace r3d::gpio
