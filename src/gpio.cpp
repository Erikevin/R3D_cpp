// GPIO backend implementations.
//
// WiringPi backend (Orange Pi) selected at compile time via USE_WIRINGPI;
// without it the logging MockGpio is used so the project builds on a host.

#include "r3d/gpio.hpp"

#include <string>
#include <unordered_map>
#include <vector>

#include "r3d/logger.hpp"

#ifdef USE_WIRINGPI
#include <wiringPi.h>
#endif

namespace r3d::gpio {

// ===========================================================================
// MOCK backend — logs every action, drives no hardware. For dev machines.
// ===========================================================================
class MockGpio : public IGpio {
public:
    bool setup() override {
        log::info("GPIO(mock): setup");
        return true;
    }
    void set_mode(int pin, Dir d, bool initial) override {
        (void)d;
        states_[pin] = initial;
        log::debug(std::string("GPIO(mock): pin ") + std::to_string(pin) +
                   " -> " + (initial ? "HIGH" : "LOW"));
    }
    void write(int pin, bool high) override {
        states_[pin] = high;
        log::debug(std::string("GPIO(mock): write pin ") + std::to_string(pin) +
                   " = " + (high ? "HIGH" : "LOW"));
    }
    bool read(int pin) override { return states_.count(pin) ? states_[pin] : false; }
    void cleanup() override { log::info("GPIO(mock): cleanup"); }
    const char* backend_name() const override { return "mock"; }

private:
    std::unordered_map<int, bool> states_;
};

// ===========================================================================
// WIRINGPI backend — Orange Pi GPIO via wiringPi.
// ===========================================================================
#ifdef USE_WIRINGPI
class WiringPiGpio : public IGpio {
public:
    bool setup() override {
        if (wiringPiSetup() == -1) {
            log::error("GPIO(wiringPi): wiringPiSetup() failed — "
                       "need root and libwiringPi installed");
            return false;
        }
        initialized_ = true;
        return true;
    }

    void set_mode(int pin, Dir d, bool initial) override {
        if (!initialized_) return;
        if (d == Dir::Output) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, initial ? HIGH : LOW);
            outputs_.push_back(pin);
        } else {
            pinMode(pin, INPUT);
        }
    }

    void write(int pin, bool high) override {
        if (!initialized_) return;
        digitalWrite(pin, high ? HIGH : LOW);
    }

    bool read(int pin) override {
        if (!initialized_) return false;
        return digitalRead(pin) == HIGH;
    }

    void cleanup() override {
        if (!initialized_) return;
        for (int pin : outputs_) digitalWrite(pin, LOW);
        outputs_.clear();
    }

    const char* backend_name() const override { return "wiringPi"; }

private:
    bool initialized_ = false;
    std::vector<int> outputs_;   // output pins to turn off on cleanup
};
#endif

// ---- factory ---------------------------------------------------------------
std::unique_ptr<IGpio> make_gpio() {
#ifdef USE_WIRINGPI
    return std::make_unique<WiringPiGpio>();
#else
    return std::make_unique<MockGpio>();
#endif
}

} // namespace r3d::gpio
