#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MN68820_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>
#include "AP_RangeFinder_MN68820_image.h"

class AP_RangeFinder_MN68820 : public AP_RangeFinder_Backend
{
public:
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void update(void) override;

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override { return MAV_DISTANCE_SENSOR_LASER; }

private:
    AP_RangeFinder_MN68820(RangeFinder::RangeFinder_State &_state,
                           AP_RangeFinder_Params &_params,
                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    __INITFUNC__ bool init();
    bool probe();
    bool read_distance_mm(uint16_t &reading_mm);
    void timer();

    // I2C helpers
    bool read_register(uint8_t reg, uint8_t &value);
    bool read_register16(uint8_t reg, uint16_t &value);
    bool write_register(uint8_t reg, uint8_t value);
    bool write_register16(uint8_t reg, uint16_t value);

    // MN68820 boot/patch helpers (from reference test code)
    bool status_read_ok();
    bool download_init();
    bool set_ram_addr(uint16_t addr);
    bool write_ram_image();
    bool ram_remap_and_reset();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
};

#endif // AP_RANGEFINDER_MN68820_ENABLED


