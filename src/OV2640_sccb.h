/*
 * OV2640_sccb_bus.h
 * Contains functions for communicating with the OV2640's Serial Camera Control
 * Bus (SCCB) over I2C.
 *
 * Created: 12/30/21
 * Author: Brown Space Engineering
 */
#ifndef PVDX_ARDUCAM_DRIVER_OV2640_SCCB_H
#define PVDX_ARDUCAM_DRIVER_OV2640_SCCB_H

#include <atmel_start.h>

// TODO: all of these need status codes
void OV2640_sccb_start_tx(void);
void OV2640_sccb_stop_tx(void);
void OV2640_sccb_send_noack(void);
void OV2640_sccb_send_ack(void);
uint8_t OV2640_sccb_write(uint8_t byte);
uint8_t OV2640_sccb_read(void);

#endif //PVDX_ARDUCAM_DRIVER_OV2640_SCCB_H
