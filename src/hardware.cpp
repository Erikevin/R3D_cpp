// Hardware layer: GPIO-backed LEDs + scan blink thread.

#include "r3d/hardware.hpp"

#include <chrono>
#include <thread>

#include "r3d/config.hpp"
#include "r3d/gpio.hpp"
#include "r3d/logger.hpp"

namespace r3d {

Hardware::~Hardware() { exit_clean(); }

bool Hardware::init() {
    log::info("\n========================");
    log::info("[硬件初始化开始]");
    bool ok = true;
    try {
        gpio_ = gpio::make_gpio();
        log::info(std::string("步骤0: GPIO backend = ") + gpio_->backend_name());

        if (!gpio_->setup()) {
            log::error("[硬件初始化 -> 失败！] GPIO setup");
            return false;
        }

        log::info("步骤3: 初始化绿灯引脚 " + std::to_string(config::GREEN_LED));
        gpio_->set_mode(config::GREEN_LED, gpio::Dir::Output, false);

        log::info("步骤4: 初始化红灯引脚 " + std::to_string(config::RED_LED));
        gpio_->set_mode(config::RED_LED, gpio::Dir::Output, false);

        log::info("步骤5: 初始化扫描引脚 " + std::to_string(config::PWM_SCAN));
        gpio_->set_mode(config::PWM_SCAN, gpio::Dir::Output, true);

        log::info("[硬件初始化 -> 全部成功]");
    } catch (const std::exception& e) {
        log::error(std::string("[硬件初始化 -> 失败！] ") + e.what());
        ok = false;
    }
    log::info("========================");
    return ok;
}

void Hardware::green(bool value) {
    if (gpio_) gpio_->write(config::GREEN_LED, value);
}

void Hardware::red(bool value) {
    if (gpio_) gpio_->write(config::RED_LED, value);
}

void Hardware::blink_task() {
    // Toggle the scan/green pin every 500 ms while scanning, then leave it HIGH.
    while (is_scanning_) {
        gpio_->write(config::PWM_SCAN, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(config::BLINK_HALF_PERIOD_MS));
        if (!is_scanning_) break;
        gpio_->write(config::PWM_SCAN, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(config::BLINK_HALF_PERIOD_MS));
    }
    gpio_->write(config::PWM_SCAN, true);
}

void Hardware::start_scan_pwm() {
    is_scanning_ = true;
    if (blink_thread_.joinable()) return; // already running
    blink_thread_ = std::thread([this] { blink_task(); });
}

void Hardware::stop_scan_pwm() {
    is_scanning_ = false;
    if (blink_thread_.joinable()) blink_thread_.join();
    if (gpio_) gpio_->write(config::PWM_SCAN, true);
}

void Hardware::exit_clean() {
    is_scanning_ = false;
    if (blink_thread_.joinable()) blink_thread_.join();
    if (gpio_) {
        gpio_->write(config::GREEN_LED, false);
        gpio_->write(config::RED_LED, false);
        gpio_->write(config::PWM_SCAN, false);
        gpio_->cleanup();
    }
}

} // namespace r3d
