// GPIO backend implementations.

#include "r3d/gpio.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <unordered_map>

#include "r3d/config.hpp"
#include "r3d/logger.hpp"

#if defined(R3D_GPIO_BACKEND_LIBGPIOD) && defined(__has_include)
#  if __has_include(<gpiod.h>)
#    define R3D_HAVE_LIBGPIOD 1
#  endif
#endif

namespace r3d::gpio {

// ---- shared helpers --------------------------------------------------------
static const config::PinMap* find_pin(int board_pin) {
    for (const auto& p : config::GPIO_MAP) {
        if (p.board == board_pin) return &p;
    }
    return nullptr;
}

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
// SYSFS backend — classic /sys/class/gpio export. Works on most Jetson images
// that still expose the sysfs interface.
// ===========================================================================
class SysfsGpio : public IGpio {
public:
    bool setup() override { return true; }

    void set_mode(int pin, Dir d, bool initial) override {
        const auto* m = find_pin(pin);
        if (!m) { log::error("GPIO(sysfs): no map for BOARD pin " + std::to_string(pin)); return; }
        int gpio = static_cast<int>(m->line); // sysfs uses the global gpio number
        export_pin(gpio);
        set_dir(gpio, d);
        if (d == Dir::Output) write(pin, initial);
    }

    void write(int pin, bool high) override {
        const auto* m = find_pin(pin);
        if (!m) return;
        write_file(std::string(config::SYSFS_GPIO_BASE) + "/gpio" + std::to_string(m->line) + "/value",
                   high ? "1" : "0");
    }

    bool read(int pin) override {
        const auto* m = find_pin(pin);
        if (!m) return false;
        std::ifstream f(std::string(config::SYSFS_GPIO_BASE) + "/gpio" + std::to_string(m->line) + "/value");
        std::string v; f >> v;
        return v == "1";
    }

    void cleanup() override {}
    const char* backend_name() const override { return "sysfs"; }

private:
    static void export_pin(int gpio) {
        std::ifstream exists(std::string(config::SYSFS_GPIO_BASE) + "/gpio" + std::to_string(gpio) + "/value");
        if (exists.good()) return; // already exported
        write_file(std::string(config::SYSFS_GPIO_BASE) + "/export", std::to_string(gpio));
        ::usleep(config::GPIO_EXPORT_SETTLE_MS * 1000);
    }
    static void set_dir(int gpio, Dir d) {
        write_file(std::string(config::SYSFS_GPIO_BASE) + "/gpio" + std::to_string(gpio) + "/direction",
                   d == Dir::Output ? "out" : "in");
    }
    static void write_file(const std::string& path, const std::string& val) {
        std::ofstream f(path);
        if (f) f << val;
    }
};

// ===========================================================================
// LIBGPIOD backend — modern character-device interface (preferred on Jetson).
// ===========================================================================
#ifdef R3D_HAVE_LIBGPIOD
#include <gpiod.h>

class LibgpiodGpio : public IGpio {
public:
    ~LibgpiodGpio() override {
        for (auto& [_, line] : lines_) {
            if (line) gpiod_line_release(line);
        }
        if (chip_) gpiod_chip_close(chip_);
    }

    bool setup() override {
        // Open the chip referenced by the first mapped pin.
        chip_ = gpiod_chip_open_by_name(config::GPIO_MAP[0].chip);
        if (!chip_) {
            log::error(std::string("GPIO: cannot open ") + config::GPIO_MAP[0].chip +
                       " — " + std::strerror(errno));
            return false;
        }
        return true;
    }

    void set_mode(int pin, Dir d, bool initial) override {
        const auto* m = find_pin(pin);
        if (!m || !chip_) return;
        struct gpiod_line* line = gpiod_chip_get_line(chip_, m->line);
        if (!line) return;
        if (d == Dir::Output) {
            gpiod_line_request_output(line, config::GPIO_CONSUMER, initial ? 1 : 0);
        } else {
            gpiod_line_request_input(line, config::GPIO_CONSUMER);
        }
        lines_[pin] = line;
    }

    void write(int pin, bool high) override {
        auto it = lines_.find(pin);
        if (it == lines_.end() || !it->second) return;
        gpiod_line_set_value(it->second, high ? 1 : 0);
    }

    bool read(int pin) override {
        auto it = lines_.find(pin);
        if (it == lines_.end() || !it->second) return false;
        return gpiod_line_get_value(it->second) == 1;
    }

    void cleanup() override {
        for (auto& [_, line] : lines_) {
            if (line) { gpiod_line_set_value(line, 0); gpiod_line_release(line); }
        }
        lines_.clear();
    }

    const char* backend_name() const override { return "libgpiod"; }

private:
    struct gpiod_chip* chip_ = nullptr;
    std::unordered_map<int, struct gpiod_line*> lines_;
};
#endif

// ---- factory ---------------------------------------------------------------
std::unique_ptr<IGpio> make_gpio() {
#if defined(R3D_GPIO_BACKEND_LIBGPIOD) && defined(R3D_HAVE_LIBGPIOD)
    return std::make_unique<LibgpiodGpio>();
#elif defined(R3D_GPIO_BACKEND_SYSFS)
    return std::make_unique<SysfsGpio>();
#else
    // Default to mock when no hardware backend is selected/available.
    return std::make_unique<MockGpio>();
#endif
}

} // namespace r3d::gpio
