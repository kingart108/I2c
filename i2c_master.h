/******************************************************************************
 * FILE   : i2c_master.h
 * DEVICE : TMS320F2800157  (C2000, 100 MHz, CCS driverlib)
 * ROLE   : I2C Master — polls 15 slaves for health every 100 ms
 *
 * PROTOCOL (both directions, bare I2C — no sub-address):
 *   WRITE  (master → slave, 2 bytes):
 *           [0xA5]  [0x01]
 *            CMD     SUBCMD
 *
 *   READ   (master ← slave, 2 bytes, separate transaction):
 *           [0x5A]  [STATUS]
 *            ACK     bit0=CPU  bit1=PERIPH  bit2=WDG  (0x07 = all OK)
 *
 * SLAVE ADDRESSES : 0x10 … 0x1E  (change SLAVE_BASE_ADDR if needed)
 * I2C BUS SPEED   : 100 kHz standard mode
 * SYSCLK          : 100 MHz  (DEVICE_SYSCLK_FREQ from device.h)
 *
 * RECOVERY STRATEGY:
 *   Soft reset  — I2C module IRS=0 then IRS=1
 *   Hard reset  — 9 SCL pulses via GPIO to free a locked slave,
 *                 followed by soft reset
 *   Per-slave   — any slave that fails is skipped; others continue
 *   Timeout     — every blocking wait has a µs counter; never hangs
 ******************************************************************************/

#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include "driverlib.h"
#include "device.h"

/*---------------------------------------------------------------------------
 * Bus / timing
 *---------------------------------------------------------------------------*/
#define I2C_BUS_HZ          100000UL        /* 100 kHz                      */
#define NUM_SLAVES          15U             /* slaves on the bus             */
#define SLAVE_BASE_ADDR     0x10U           /* 0x10 … 0x1E                  */

/*---------------------------------------------------------------------------
 * Health protocol
 *---------------------------------------------------------------------------*/
#define HEALTH_CMD          0xA5U
#define HEALTH_SUBCMD       0x01U
#define HEALTH_ACK          0x5AU
#define HEALTH_STATUS_FULL  0x07U           /* CPU+PERIPH+WDG all OK        */

/*---------------------------------------------------------------------------
 * Timeouts (µs) — generous for 100 kHz; one bit ≈ 10 µs
 *---------------------------------------------------------------------------*/
#define TIMEOUT_BUS_FREE_US     5000U       /* wait for BB to clear         */
#define TIMEOUT_BYTE_TX_US      500U        /* wait for XRDY / ARDY per byte*/
#define TIMEOUT_BYTE_RX_US      500U        /* wait for RRDY per byte       */
#define TIMEOUT_STOP_US         1000U       /* wait for STP to clear        */

/*---------------------------------------------------------------------------
 * Per-slave health record
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint16_t addr;          /* 7-bit I2C address                            */
    uint16_t statusByte;    /* last STATUS byte from slave                  */
    uint16_t healthy;       /* 1 = fully healthy, 0 = fault / no-response  */
    uint16_t lastError;     /* 0=OK  1=TIMEOUT  2=NACK  3=BAD_RESP         */
    uint32_t pollCount;     /* total polls attempted                        */
    uint32_t failCount;     /* total polls that failed                      */
} SlaveRecord_t;

/*---------------------------------------------------------------------------
 * Error codes
 *---------------------------------------------------------------------------*/
#define I2CM_OK             0U
#define I2CM_ERR_TIMEOUT    1U
#define I2CM_ERR_NACK       2U
#define I2CM_ERR_BAD_RESP   3U
#define I2CM_ERR_BUS_BUSY   4U

/*---------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/
void     Master_I2C_Init(void);
void     Master_I2C_InitSlaveTable(SlaveRecord_t tbl[NUM_SLAVES]);
void     Master_I2C_PollAllSlaves(SlaveRecord_t tbl[NUM_SLAVES]);

#endif /* I2C_MASTER_H */
