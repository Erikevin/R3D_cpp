#pragma once
// GPIO abstraction with pluggable backends.
//
// The Python code used RPi.GPIO with BOARD numbering. On Jetson that API is not
// available, so we wrap the character-device (libgpiod) interface, with a classic
// sysfs fallback and a logging mock for host development.
//
// Select at compile time: -DR3D_GPIO_BACKEND=LIBGPIOD|SYSFS|MOCK

#include <memory>
#include <string>

namespace r3d::gpio {

enum class Dir { Output, Input };

// Abstract GPIO controller. Pins are referenced by the BOARD number from config.
class IGpio {
public:
    virtual ~IGpio() = default;
    virtual bool setup() = 0;                 // configure all known pins
    virtual void set_mode(int board_pin, Dir d, bool initial_high) = 0;
    virtual void write(int board_pin, bool high) = 0;
    virtual bool read(int board_pin) = 0;
    virtual void cleanup() = 0;               // turn everything off, release
    virtual const char* backend_name() const = 0;
};

// Factory: returns the backend selected at compile time (or MOCK if libgpiod
// is unavailable at runtime).
std::unique_ptr<IGpio> make_gpio();

} // namespace r3d::gpio
