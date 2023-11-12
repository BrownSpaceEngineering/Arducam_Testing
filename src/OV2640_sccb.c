/*
 * OV2640_sccb.c
 * Created: 12/30/21
 * Author: Brown Space Engineering
 */

#include "OV2640_regs.h"
#include "OV2640_sccb.h"


// TODO: return a status code (check the io_write/read & i2c_* calls!)
/**
 * Reads a sensor register with an 8-bit address over I2C.
 * @param reg_id the ID of the register to write.
 * @param reg_data_buf a buffer in which the register contents will be loaded by the function.
 */
void OV2640_sccb_read_8bit_reg(uint8_t reg_id, unsigned char *reg_data_buf) {
    // Mask the register ID to specify a read
    uint8_t reg_id_masked = reg_id | 0x01;
    struct io_descriptor *io;
    i2c_m_sync_get_io_descriptor(&I2C_0, &io);
    i2c_m_sync_enable(&I2C_0);
    i2c_m_sync_set_slaveaddr(&I2C_0, OV2640_I2C_ADDR, I2C_M_SEVEN);
    io_write(io, &reg_id_masked, 1);
    io_read(io, reg_data_buf, 1);
}

// TODO: return a status code (check the io_write/read & i2c_* calls!)
/**
 * Reads a sensor register with an 8-bit address over I2C.
 * @param reg_id the ID of the register to write.
 * @param reg_data_buf a buffer in which the register contents will be loaded by the function.
 */
 // FIXME: `rdSensorReg16_8` writes 0x78 instead of the sensor address at the start and then writes 0x79 at the end --
 // WHY? and how do we do the equivalent thing using the I2C stuff here?
void OV2640_sccb_read_16bit_reg(uint16_t reg_id, unsigned char *reg_data_buf) {
    // Mask the register ID to specify a read
    uint16_t reg_id_masked = reg_id | 0x01;
    struct io_descriptor *io;
    i2c_m_sync_get_io_descriptor(&I2C_0, &io);
    i2c_m_sync_enable(&I2C_0);
    // TODO: figure out why the sample code uses 0x78
    i2c_m_sync_set_slaveaddr(&I2C_0, 0x78, I2C_M_SEVEN);
    io_write(io, (uint8_t *) &reg_id_masked, 2);
    io_read(io, reg_data_buf, 1);
}

// TODO: return a status code (check the io_writes & i2c_* calls!)
/**
 * Writes a byte to a sensor register with an 8-bit address over I2C.
 * @param reg_id the ID of the register to write.
 * @param reg_data the data (byte) to write to the register.
 */
void OV2640_sccb_write_8bit_reg(uint8_t reg_id, uint8_t reg_data) {
    int32_t err;
    struct io_descriptor *io;
    err = i2c_m_sync_get_io_descriptor(&I2C_0, &io);
    if (err != ERR_NONE) {
        // TODO: error
    }
    err = i2c_m_sync_enable(&I2C_0);
    if (err != ERR_NONE) {
        // TODO: error
    }
    err = i2c_m_sync_set_slaveaddr(&I2C_0, OV2640_I2C_ADDR, I2C_M_SEVEN);
    if (err != ERR_NONE) {
        // TODO: error
    }
    // FIXME: getting error code 96:
    // https://asf.microchip.com/docs/latest/samd21/html/group__group__sam0__utils__status__codes.html
    err = io_write(io, &reg_id, 1);
    if (err != ERR_NONE) {
        // TODO: error
    }
    err = io_write(io, &reg_data, 1);
    if (err != ERR_NONE) {
        // TODO: error
    }
}

// TODO: return a status code (check the io_writes & i2c_* calls!)
/**
 * Writes a byte to a sensor register with a 16-bit address over I2C.
 * @param reg_id the ID of the register to write.
 * @param reg_data the data (byte) to write to the register.
 */
void OV2640_sccb_write_16bit_reg(uint16_t reg_id, uint8_t reg_data) {
    struct io_descriptor *io;
    i2c_m_sync_get_io_descriptor(&I2C_0, &io);
    i2c_m_sync_enable(&I2C_0);
    i2c_m_sync_set_slaveaddr(&I2C_0, OV2640_I2C_ADDR, I2C_M_SEVEN);
    // TODO: check this is okay (using length = 2)
    // Do we want to be using i2c_m_sync_cmd_[read/write]?
    // May be an issue for this one b/c we need *two* sensor addr bytes
    io_write(io, (uint8_t *) &reg_id, 2);
    io_write(io, &reg_data, 1);
}

// TODO: status code
void OV2640_sccb_write_8bit_reg_array(const struct sensor_reg* reglist) {
    int err = 0;
    unsigned int reg_addr = 0;
    unsigned int reg_val = 0;
    const struct sensor_reg *next = reglist;
    while ((reg_addr != 0xff) | (reg_val != 0xff))
    {
        reg_addr =next->reg;
        reg_val = next->val;
        // err = OV2640_sccb_write_8bit_reg(reg_addr, reg_val);
        OV2640_sccb_write_8bit_reg(reg_addr, reg_val);
        delay_ms(10);
        next++;
    }
    // return error
}