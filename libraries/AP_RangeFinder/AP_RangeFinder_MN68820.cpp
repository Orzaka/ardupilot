/*
  Backend driver skeleton for MN68820 ToF rangefinder (I2C)
*/

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MN68820_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_MN68820.h"
#include "AP_RangeFinder_MN68820_image.h"

extern const AP_HAL::HAL& hal;

static const uint8_t MEASUREMENT_TIME_MS = 50; // adjust later
static const uint8_t REG_STATUS_FLAG = 0x08;   // status port for boot/patch flow
static const uint8_t REG_RESET      = 0xE0;    // reset/status
static const uint8_t REG_INT_EN     = 0xE2;    // interrupt enable (optional)
static const uint8_t REG_INT_STAT   = 0xE1;    // interrupt status (optional)
static const uint8_t REG_TIMESTAMP  = 0x24;    // example data register
static const uint8_t REG_DATA_READY = 0x20;    // contents/status to indicate result ready


AP_RangeFinder_MN68820::AP_RangeFinder_MN68820(RangeFinder::RangeFinder_State &_state,
                                               AP_RangeFinder_Params &_params,
                                               AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev))
{
}

AP_RangeFinder_Backend *AP_RangeFinder_MN68820::detect(RangeFinder::RangeFinder_State &_state,
                                                       AP_RangeFinder_Params &_params,
                                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_MN68820 *sensor = NEW_NOTHROW AP_RangeFinder_MN68820(_state, _params, std::move(dev));
    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    auto *sem = sensor->dev->get_semaphore();
    sem->take_blocking();
    const bool ok = sensor->probe() && sensor->init();
    sem->give();

    if (!ok) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_RangeFinder_MN68820::probe()
{
    // If the device answers on reset register with expected bit, consider present
    uint8_t val = 0;
    if (!read_register(REG_RESET, val)) {
        return false;
    }
    return true;
}

bool AP_RangeFinder_MN68820::init()
{
    // reset
    if (!write_register(REG_RESET, 0x81)) {
        return false;
    }
    hal.scheduler->delay(100);

    // wait CPU ready (0x41)
    {
        uint32_t start = AP_HAL::millis();
        uint8_t v = 0;
        while (AP_HAL::millis() - start < 100) {
            if (!read_register(REG_RESET, v)) {
                return false;
            }
            if (v == 0x41) { break; }
            hal.scheduler->delay(1);
        }
        if (v != 0x41) {
            return false;
        }
    }

    // download/patch flow
    if (!download_init()) {
        return false;
    }
    if (!set_ram_addr(0x0000)) {
        return false;
    }
    if (!write_ram_image()) {
        return false;
    }
    if (!ram_remap_and_reset()) {
        return false;
    }

    // register timer callback after successful initialization
    dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_MN68820::timer, void));

    return true;
}

bool AP_RangeFinder_MN68820::read_distance_mm(uint16_t &reading_mm)
{
    // Example data-read flow: check a result-ready flag then read 2 bytes distance
    uint8_t contents = 0;
    if (!read_register(REG_DATA_READY, contents)) {
        return false;
    }
    if (contents != 0x55) { // require data ready
        return false;
    }

    uint8_t buf[2] = {};
    uint8_t reg = 0x22; // assume distance high/low at 0x22..0x23 (adjust when spec available)
    if (!dev->transfer(&reg, 1, buf, 2)) {
        return false;
    }
    reading_mm = (uint16_t(buf[0]) << 8) | buf[1];
    return true;
}

void AP_RangeFinder_MN68820::update()
{
    // nothing to do - its all done in the timer()
}

void AP_RangeFinder_MN68820::timer()
{
    uint16_t range_mm = 0;
    if (read_distance_mm(range_mm)) {
        WITH_SEMAPHORE(_sem);
        state.distance_m = range_mm * 0.001f;
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else {
        WITH_SEMAPHORE(_sem);
        if (AP_HAL::millis() - state.last_reading_ms > 200) {
            set_status(RangeFinder::Status::NoData);
        }
    }
}

// ----- I2C helpers -----
bool AP_RangeFinder_MN68820::read_register(uint8_t reg, uint8_t &value)
{
    return dev->transfer(&reg, 1, &value, 1);
}

bool AP_RangeFinder_MN68820::read_register16(uint8_t reg, uint16_t &value)
{
    uint8_t buf[2] {};
    if (!dev->transfer(&reg, 1, buf, 2)) {
        return false;
    }
    value = (uint16_t(buf[0]) << 8) | buf[1];
    return true;
}

bool AP_RangeFinder_MN68820::write_register(uint8_t reg, uint8_t value)
{
    const uint8_t tx[2] = {reg, value};
    return dev->transfer(tx, 2, nullptr, 0);
}

bool AP_RangeFinder_MN68820::write_register16(uint8_t reg, uint16_t value)
{
    const uint8_t tx[3] = {reg, uint8_t(value >> 8), uint8_t(value & 0xFF)};
    return dev->transfer(tx, 3, nullptr, 0);
}

// ----- MN68820 boot/patch helpers -----
bool AP_RangeFinder_MN68820::status_read_ok()
{
    uint8_t buf[3] = {0};
    // read 3 status bytes from 0x08 port
    uint8_t reg = REG_STATUS_FLAG;
    if (!dev->transfer(&reg, 1, buf, 3)) {
        return false;
    }
    return (buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0xFF);
}

bool AP_RangeFinder_MN68820::download_init()
{
    const uint8_t cmd[4] = {0x14, 0x01, 0x29, 0xC1};
    if (!dev->transfer(cmd, 4, nullptr, 0)) {
        return false;
    }
    uint32_t start = AP_HAL::millis();
    while (AP_HAL::millis() - start < 10) {
        if (status_read_ok()) { return true; }
        hal.scheduler->delay(1);
    }
    return false;
}

bool AP_RangeFinder_MN68820::set_ram_addr(uint16_t addr)
{
    uint8_t cmd[5] = {0};
    cmd[0] = 0x43;
    cmd[1] = 0x02;
    cmd[2] = uint8_t(addr & 0xFF);
    cmd[3] = uint8_t((addr >> 8) & 0xFF);
    cmd[4] = (uint8_t)((cmd[0] + cmd[1] + cmd[2] + cmd[3]) ^ 0xFF);
    if (!dev->transfer(cmd, 5, nullptr, 0)) {
        return false;
    }
    uint32_t start = AP_HAL::millis();
    while (AP_HAL::millis() - start < 10) {
        if (status_read_ok()) { return true; }
        hal.scheduler->delay(1);
    }
    return false;
}

bool AP_RangeFinder_MN68820::write_ram_image()
{
    // firmware image is provided in AP_RangeFinder_MN68820_image.h
    const uint8_t *image = mn68820_firmware_image;
    const uint16_t image_size = mn68820_firmware_image_size;
    if (image == nullptr || image_size == 0) {
        return false;
    }

    if (image_size % 16 != 0) {
        // expect 16-byte blocks
        return false;
    }

    uint8_t cmd[19] = {0};
    cmd[0] = 0x41; // write RAM
    cmd[1] = 0x10; // 16 bytes per segment

    const uint16_t segments = image_size / 16;
    for (uint16_t i = 0; i < segments; i++) {
        for (uint16_t j = 0; j < 16; j++) {
            cmd[2 + j] = image[i * 16 + j];
        }
        uint8_t cksum = 0;
        for (uint8_t k = 0; k < 18; k++) {
            cksum += cmd[k];
        }
        cmd[18] = cksum ^ 0xFF;
        if (!dev->transfer(cmd, 19, nullptr, 0)) {
            return false;
        }
        uint32_t start = AP_HAL::millis();
        while (AP_HAL::millis() - start < 10) {
            if (status_read_ok()) { break; }
            hal.scheduler->delay(1);
        }
        if (!status_read_ok()) {
            return false;
        }
    }
    return true;
}

bool AP_RangeFinder_MN68820::ram_remap_and_reset()
{
    const uint8_t cmd[3] = {0x11, 0x00, 0xEE};
    if (!dev->transfer(cmd, 3, nullptr, 0)) {
        return false;
    }
    hal.scheduler->delay(500);

    // wait CPU ready (0x41)
    uint32_t start = AP_HAL::millis();
    uint8_t v = 0;
    while (AP_HAL::millis() - start < 1000) {
        if (!read_register(REG_RESET, v)) {
            return false;
        }
        if (v == 0x41) {
            return true;
        }
        hal.scheduler->delay(1);
    }
    return false;
}

#endif // AP_RANGEFINDER_MN68820_ENABLED


