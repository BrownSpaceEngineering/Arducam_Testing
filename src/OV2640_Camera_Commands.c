/*
 * OV2640_Camera_Commands.c
 * Created: 12/29/21
 * Author: Brown Space Engineering
 */

#include "OV2640_Camera_Commands.h"
#include "OV2640_regs.h"

// How many times to attempt SPI and camera detection before giving up
#define OV2640_DETECTION_ATTEMPTS 10
// The bits to send to the test register
#define OV2640_DETECTION_BITS 0x55
// Detection return values
#define OV2640_DETECTION_SUCCESS 1
#define OV2640_DETECTION_FAILURE 0

#define CS_LOW() gpio_set_pin_level(OV2640_CS, 0)
#define CS_HIGH() gpio_set_pin_level(OV2640_CS, 1)

// Private functions
uint8_t OV2640_spi_test(void);
uint8_t OV2640_module_detect(void);
void OV2640_reset_firmware(void);
void OV2640_arducam_init(void);
void OV2640_write_spi_reg(int address, int value);
uint8_t OV2640_read_spi_reg(int address);
uint8_t OV2640_spi_transceive(uint8_t send);
void OV2640_i2c_sccb_reg_write_8_8(uint8_t reg_id, uint8_t *reg_data);
void OV2640_i2c_sccb_reg_write_16_8(uint16_t reg_id, uint8_t *reg_data);


// TODO: status code
void OV2640_init(void) {
    // Configure CS pin
    gpio_set_pin_direction(OV2640_CS, GPIO_DIRECTION_OUT);
    gpio_set_pin_pull_mode(OV2640_CS, GPIO_PULL_OFF);
    gpio_set_pin_level(OV2640_CS, true);  // neutral high

    // Initialize Serial Camera Control Bus (SCCB)
    // TODO (I^2C awaits...)

    // Verify SPI connection and OV2640 self-identification
    uint8_t spi_success = OV2640_spi_test();
    if (!spi_success) {
        // TODO: error
    }
    // TODO: this call is used in the sample code just for identifying which
    // camera model is connected. Since we know it's always going to be the
    // OV2640, this is essentially just serving as a sanity check -- is this
    // necessary since we're already doing an SPI check above?
    uint8_t module_detect_success = OV2640_module_detect();
    if (!module_detect_success) {
        // TODO: error
    }

    // Reset firmware
    // TODO: error check
    OV2640_reset_firmware();

    // Perform camera initialization
    // TODO: error check
    OV2640_arducam_init();
}

// TODO: return a status code
void OV2640_set_resolution(uint8_t resolution) {
    // TODO: check these calls
    switch (resolution) {
        case OV2640_160x120:
            wrSensorRegs8_8(OV2640_160x120_JPEG);
            break;
        case OV2640_320x240:
            wrSensorRegs8_8(OV2640_320x240_JPEG);
            break;
        case OV2640_640x480:
            wrSensorRegs8_8(OV2640_640x480_JPEG);
            break;
        case OV2640_1024x768:
            wrSensorRegs8_8(OV2640_1024x768_JPEG);
            break;
        default:
            // TODO: some sort of error?
            break;
    }
}

/**
 * Tests the SPI connection by setting the test register on the OV2640.
 * @return 1 if the test succeeds, 0 if it fails
 */
uint8_t OV2640_spi_test(void) {
    unsigned char temp;
    uint8_t attempts = 0;
    while (attempts < OV2640_DETECTION_ATTEMPTS) {
        // TODO: check these calls
        OV2640_write_spi_reg(ARDUCHIP_TEST1, OV2640_DETECTION_BITS);
        temp = OV2640_read_spi_reg(ARDUCHIP_TEST1);
        if (temp != OV2640_DETECTION_BITS) {
            // Error -- wait and try again
            attempts++;
            delay_ms(1000);
        } else {
            // Success
            return OV2640_DETECTION_SUCCESS;
        }
    }
    return OV2640_DETECTION_FAILURE;
}

/**
 * Attempts to identify the OV2640 camera module.
 * @return 1 if the test succeeds, 0 if it fails
 */
uint8_t OV2640_module_detect(void) {
    unsigned char vid, pid;
    uint8_t attempts = 0;
    while (attempts < OV2640_DETECTION_ATTEMPTS) {
        // TODO: check these calls
        OV2640_i2c_sccb_reg_write_8_8(0xff, 0x01);
        rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
        rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
        // TODO: this is straight from the OV2640 sample code, but clearly
        // something is awry here because ((pid != 0x41) || ( pid != 0x42)) is
        // tautological
        if ((vid != 0x26) && ((pid != 0x41) || ( pid != 0x42))) {
            attempts++;
        } else {
            return OV2640_DETECTION_SUCCESS;
        }
    }
    return OV2640_DETECTION_FAILURE;
}

// TODO: return a status code
/**
 * Performs a firmware reset on the camera (call at startup).
 */
void OV2640_reset_firmware(void) {
    // TODO: check these calls
    OV2640_write_spi_reg(0x07, 0x80);
    delay_ms(100);
    OV2640_write_spi_reg(0x07, 0x00);
    delay_ms(100);
}

// TODO: return a status code
/**
 * Performs the ArduCAM initialization sequence, including configuring the
 * default resolution. We hardcode the image format (JPEG) and color encoding
 * (YUV422). Call at startup.
 */
void OV2640_arducam_init(void) {
    // TODO: check these calls
    // These are black-box values pulled from the ArduCAM sample code
    OV2640_i2c_sccb_reg_write_8_8(0xff, 0x01);
    OV2640_i2c_sccb_reg_write_8_8(0x12, 0x80);
    wrSensorRegs8_8(OV2640_JPEG_INIT);
    // TODO: there may be more efficient color encodings (e.g., YUV411) that we
    // may want to explore -- YUV422 is just the default used in the ArduCAM
    // sample code
    wrSensorRegs8_8(OV2640_YUV422);
    wrSensorRegs8_8(OV2640_JPEG);
    // More magic numbers from the ArduCAM code
    OV2640_i2c_sccb_reg_write_8_8(0xff, 0x01);
    OV2640_i2c_sccb_reg_write_8_8(0x15, 0x00);
    wrSensorRegs8_8(OV2640_DEFAULT_RESOLUTION);
}

// TODO: return a status code
/**
 * Writes an OV2640 register over SPI.
 * @param address the address to write.
 * @param value the value to write.
 */
void OV2640_write_spi_reg(int address, int value) {
    address = address | 0x80;  // set MSB 1 to indicate write
    CS_LOW();
    OV2640_spi_transceive(address);
    OV2640_spi_transceive(value);
    CS_HIGH();
}

// TODO: return a status code
/**
 * Reads an OV2640 register over SPI.
 * @param address the address to write.
 * @return the value in the specified register, or 0 if an error occurred (this
 *         should be changed to a more robust error-handling approach)
 */
uint8_t OV2640_read_spi_reg(int address) {
    uint8_t value;
    address = address & 0x7F;  // set MSB 0 to indicate read
    CS_LOW();
    OV2640_spi_transceive(address);
    value = OV2640_spi_transceive(0x00);  // dummy byte for read tick
    CS_HIGH();
    return value;
}

// TODO: status code
// TODO: this should probably be made into an OS-wide SPI helper function
/**
 * Sends and receives one byte (each way) on the SPI bus.
 * @param send the byte to send.
 * @return the received byte, or 0 if an error occurred (this should be
 *         changed to a more robust error-handling approach)
 */
uint8_t OV2640_spi_transceive(uint8_t send) {
    unsigned char res;
    struct spi_xfer xfer;
    xfer.size = 1;
    xfer.txbuf = (unsigned char*) &send;
    xfer.rxbuf = &res;
    spi_m_sync_enable(&SPI_0);  // Forgetting this yields error -20
    int32_t bytes_read = spi_m_sync_transfer(&SPI_0, &xfer);
    if (bytes_read == 1) {
        return *(xfer.rxbuf);
    } else {
        // TODO: produce a helpful error
        // Somewhere account for different status codes (e.g., ERR_BUSY == -4)
        return 0x00;
    }
}

// TODO: implement!
void OV2640_i2c_sccb_reg_multiwrite_8_8();

// TODO: return a status code (check the io_writes & i2c_* calls!)
/**
 * Writes a sensor register with an 8-bit address over I2C.
 * @param reg_id the ID of the register to write.
 * @param reg_data the data to write to the register.
 */
void OV2640_i2c_sccb_reg_write_8_8(uint8_t reg_id, uint8_t *reg_data) {
    struct io_descriptor io;
    i2c_m_sync_get_io_descriptor(&I2C_0, &io);
    i2c_m_sync_enable(&I2C_0);
    i2c_m_sync_set_slaveaddr(&I2C_0, OV2640_I2C_ADDR, I2C_M_SEVEN);
    io_write(&io, reg_id, 1);
    io_write(&io, reg_data, 1);
}

// TODO: return a status code (check the io_writes & i2c_* calls!)
/**
 * Writes a sensor register with a 16-bit address over I2C.
 * @param reg_id the ID of the register to write.
 * @param reg_data the data to write to the register.
 */
void OV2640_i2c_sccb_reg_write_16_8(uint16_t reg_id, uint8_t *reg_data) {
    struct io_descriptor io;
    i2c_m_sync_get_io_descriptor(&I2C_0, &io);
    i2c_m_sync_enable(&I2C_0);
    i2c_m_sync_set_slaveaddr(&I2C_0, OV2640_I2C_ADDR, I2C_M_SEVEN);
    // TODO: check this is okay (using length = 2)
    // Do we want to be using i2c_m_sync_cmd_[read/write]?
    // May be an issue for this one b/c we need *two* sensor addr bytes
    io_write(&io, reg_id, 2);
    io_write(&io, reg_data, 1);
}
