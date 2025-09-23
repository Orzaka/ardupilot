/*
  Backend driver skeleton for MN68820 ToF rangefinder (I2C)
*/

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MN68820_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_MN68820.h"
#include "AP_RangeFinder_MN68820_image.h"
#include <GCS_MAVLink/GCS.h>

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
    // According to datasheet: write 0x81 to reset register, then check if it returns 0x41
    // Based on STM32F103 implementation with exact same logic
    
    // Use both console and GCS for debugging
    hal.console->printf("MN68820: Probing device...\n");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Probing device...");
    
    // Check if I2C device is valid
    if (!dev) {
        hal.console->printf("MN68820: ERROR - I2C device is null!\n");
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: I2C device is null");
        return false;
    }
    
    hal.console->printf("MN68820: I2C device OK, starting probe...\n");
    
    // First, try a simple I2C read to see if device responds
    uint8_t test_data[1];
    if (!dev->transfer(nullptr, 0, test_data, 1)) {
        hal.console->printf("MN68820: ERROR - I2C device not responding\n");
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: I2C device not responding");
        return false;
    }
    hal.console->printf("MN68820: I2C device responding, continuing probe...\n");
    
    uint8_t regval = 0;
    uint32_t tick = 0;

    // Step 1: Write reset command 0x81 (equivalent to I2C_MN68820_Write(0xE0, &regval, 1))
    regval = 0x81;
    hal.console->printf("MN68820: Writing reset command 0x81 to register 0x%02X\n", REG_RESET);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Writing reset command 0x81 to register 0x%02X", REG_RESET);
    if (!write_register(REG_RESET, regval)) {
        hal.console->printf("MN68820: ERROR - Failed to write reset command during probe\n");
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to write reset command during probe");
        return false;
    }
    hal.console->printf("MN68820: Reset command written successfully\n");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Reset command written successfully");
    
    // Step 2: Wait for device to process reset command (equivalent to HAL_Delay(100))
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Waiting 100ms for device to process reset...");
    hal.scheduler->delay(100);

    // Step 3: Wait for CPU ready with exact same logic as STM32F103 implementation
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Starting CPU ready check loop...");
    while (1) { // wait CPU ready
        if (!read_register(REG_RESET, regval)) {
            // Only log every 10th attempt to avoid spam
            if ((tick + 1) % 10 == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Failed to read reset register (attempt %lu)", (unsigned long)(tick + 1));
            }
            // Continue trying instead of returning false immediately
        } else {
            // Only log every 10th attempt to avoid spam
            if ((tick + 1) % 10 == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Read register 0x%02X = 0x%02X (attempt %lu)", REG_RESET, regval, (unsigned long)(tick + 1));
            }
        }

        if (regval == 0x41) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Probe successful, CPU ready (0x41) after %lu attempts", (unsigned long)(tick + 1));
            return true;
        }
        
        tick++;
        if (tick >= 100) {  // Same timeout as STM32F103 implementation (100 attempts)
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Probe timeout after 100 attempts, last value was 0x%02X", regval);
            return false;
        }
        hal.scheduler->delay(1);  // Equivalent to HAL_Delay(1) - 1ms delay
    }
}

bool AP_RangeFinder_MN68820::init()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Starting initialization");
    
    // Device should already be reset and CPU ready from probe()
    // Just verify CPU is still ready
    uint8_t v = 0;
    if (!read_register(REG_RESET, v)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to read reset register during init");
        return false;
    }
    if (v != 0x41) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: CPU not ready during init, got 0x%02X", v);
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: CPU ready (0x41)");

    // download/patch flow
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Starting download/patch flow");
    if (!download_init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Download init failed");
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Download init successful");
    
    if (!set_ram_addr(0x0000)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to set RAM address");
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: RAM address set to 0x0000");
    
    if (!write_ram_image()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to write firmware image");
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Firmware image written successfully");
    
    if (!ram_remap_and_reset()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: RAM remap and reset failed");
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: RAM remap and reset successful");

    // register timer callback after successful initialization
    dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_MN68820::timer, void));

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Initialization completed successfully");
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

// ----- MN68820 specific I2C functions (based on STM32F103 implementation) -----
bool AP_RangeFinder_MN68820::mn68820_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    // Equivalent to I2C_MN68820_Read(reg, data, len) from STM32F103
    if (!dev->transfer(&reg, 1, data, len)) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MN68820: I2C read failed at reg 0x%02X", reg);
        return false;
    }
    return true;
}

bool AP_RangeFinder_MN68820::mn68820_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    // Equivalent to I2C_MN68820_Write(reg, data, len) from STM32F103
    if (!dev->transfer(&reg, 1, nullptr, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MN68820: I2C register select failed at reg 0x%02X", reg);
        return false;
    }
    
    if (!dev->transfer(data, len, nullptr, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MN68820: I2C write failed at reg 0x%02X", reg);
        return false;
    }
    return true;
}

bool AP_RangeFinder_MN68820::mn68820_write_direct(uint8_t reg, const uint8_t *data, uint8_t len)
{
    // Direct write without register selection (for commands)
    uint8_t cmd[len + 1];
    cmd[0] = reg;
    memcpy(&cmd[1], data, len);
    
    if (!dev->transfer(cmd, len + 1, nullptr, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MN68820: I2C direct write failed at reg 0x%02X", reg);
        return false;
    }
    return true;
}

// ----- MN68820 boot/patch helpers -----
bool AP_RangeFinder_MN68820::status_read_ok()
{
    uint8_t buf[3] = {0};
    // read 3 status bytes from 0x08 port (equivalent to STM32F103 implementation)
    if (!mn68820_read(REG_STATUS_FLAG, buf, 3)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to read status register");
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MN68820: Status bytes: 0x%02X 0x%02X 0x%02X", buf[0], buf[1], buf[2]);
    
    // Check for expected status: 0x00 0x00 0xFF (from STM32F103 implementation)
    // Also check for alternative status patterns that might indicate success
    if (buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0xFF) {
        return true;
    }
    
    // Check if we're getting 0x41 0x00 0x00 which might be a different success indicator
    // This could indicate the command was received but not yet processed
    if (buf[0] == 0x41 && buf[1] == 0x00 && buf[2] == 0x00) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MN68820: Got 0x41 status, might be processing...");
        return false;  // Not ready yet, but not an error
    }
    
    return false;
}

bool AP_RangeFinder_MN68820::download_init()
{
    // Use the working command from STM32F103 implementation
    const uint8_t cmd[4] = {0x14, 0x01, 0x29, 0xC1};
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Sending download init command: 0x%02X 0x%02X 0x%02X 0x%02X", cmd[0], cmd[1], cmd[2], cmd[3]);
    
    // Send command to register 0x08 (equivalent to I2C_MN68820_Write(0x08, cmd_buf, 4))
    if (!mn68820_write(REG_STATUS_FLAG, cmd, 4)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to send download init command");
        return false;
    }
    
    // Add a small delay before checking status (like STM32F103 implementation)
    hal.scheduler->delay(1);
    
    // Wait for status confirmation - following STM32F103 implementation exactly
    uint32_t start = AP_HAL::millis();
    while (AP_HAL::millis() - start < 10) {  // 10ms timeout like STM32F103 implementation
        if (status_read_ok()) { 
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Download init status OK");
            return true; 
        }
        hal.scheduler->delay(1);
    }
    
    // If first attempt failed, try again with longer timeout
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MN68820: First attempt failed, retrying...");
    start = AP_HAL::millis();
    while (AP_HAL::millis() - start < 50) {  // Extended timeout for retry
        if (status_read_ok()) { 
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Download init status OK on retry");
            return true; 
        }
        hal.scheduler->delay(1);
    }
    
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Download init timeout after retry");
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
    
    // Send RAM address command (equivalent to I2C_MN68820_Write(0x08, cmd, 5))
    if (!mn68820_write(REG_STATUS_FLAG, cmd, 5)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to send RAM address command");
        return false;
    }
    
    uint32_t start = AP_HAL::millis();
    while (AP_HAL::millis() - start < 10) {
        if (status_read_ok()) { 
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MN68820: RAM address set to 0x%04X", addr);
            return true; 
        }
        hal.scheduler->delay(1);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: RAM address set timeout");
    return false;
}

bool AP_RangeFinder_MN68820::write_ram_image()
{
    // firmware image is provided in AP_RangeFinder_MN68820_image.h
    const uint8_t *image = mn68820_firmware_image;
    const uint16_t image_size = mn68820_firmware_image_size;
    if (image == nullptr || image_size == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Firmware image not available");
        return false;
    }

    if (image_size % 16 != 0) {
        // expect 16-byte blocks
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Invalid firmware image size: %u bytes", image_size);
        return false;
    }
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Writing firmware image (%u bytes, %u segments)", 
                  image_size, image_size / 16);

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
        // Send RAM write command (equivalent to I2C_MN68820_Write(0x08, cmd, 19))
        if (!mn68820_write(REG_STATUS_FLAG, cmd, 19)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to write segment %u/%u", i+1, segments);
            return false;
        }
        uint32_t start = AP_HAL::millis();
        while (AP_HAL::millis() - start < 10) {
            if (status_read_ok()) { break; }
            hal.scheduler->delay(1);
        }
        if (!status_read_ok()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Segment %u/%u write timeout", i+1, segments);
            return false;
        }
        
        // 每 10 個段顯示一次進度
        if ((i + 1) % 10 == 0 || i == segments - 1) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Firmware write progress: %u/%u segments", i+1, segments);
        }
    }
    return true;
}

bool AP_RangeFinder_MN68820::ram_remap_and_reset()
{
    const uint8_t cmd[3] = {0x11, 0x00, 0xEE};
    // Send RAM remap command (equivalent to I2C_MN68820_Write(0x08, cmd, 3))
    if (!mn68820_write(REG_STATUS_FLAG, cmd, 3)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to send RAM remap command");
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: RAM remap command sent, waiting 500ms");
    hal.scheduler->delay(500);

    // wait CPU ready (0x41)
    uint32_t start = AP_HAL::millis();
    uint8_t v = 0;
    while (AP_HAL::millis() - start < 1000) {
        if (!read_register(REG_RESET, v)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Failed to read reset register during final CPU ready check");
            return false;
        }
        if (v == 0x41) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MN68820: Final CPU ready check passed");
            return true;
        }
        hal.scheduler->delay(1);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MN68820: Final CPU ready check timeout, got 0x%02X", v);
    return false;
}

#endif // AP_RANGEFINDER_MN68820_ENABLED


