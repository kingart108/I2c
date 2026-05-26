/******************************************************************************
 * FILE   : i2c_slave.h
 * DEVICE : TMS320F2800157
 * ROLE   : I2C Slave — responds to master health request via ISR
 *
 * SLAVE ISR STATE MACHINE:
 *
 *   State 0 – IDLE
 *     AAS interrupt fires (master addressed us)
 *     SDIR=0 → master is writing  → go to State 1
 *     SDIR=1 → master is reading  → load reply, go to State 3
 *
 *   State 1 – RECEIVING CMD BYTE
 *     RRDY fires → read byte, store as cmd[0]
 *     Go to State 2
 *
 *   State 2 – RECEIVING SUBCMD BYTE
 *     RRDY fires → read byte, store as cmd[1]
 *     Validate command, prepare health reply in g_txBuf[]
 *     SCD fires → write transaction done → go to State 0
 *
 *   State 3 – TRANSMITTING
 *     XRDY fires → write next byte from g_txBuf[]
 *     SCD fires  → transaction done, reset TX pointer → go to State 0
 *
 * DOUBLE-BUFFER TRAP (F280015x specific):
 *   The TX path has a data register (I2CDXR) and a shift register (I2CXSR).
 *   When master reads, AAS fires, we load byte 0 into I2CDXR.
 *   Hardware immediately copies it to I2CXSR and fires XRDY again.
 *   We must load byte 1 now.  On STOP/NACK the extra pre-loaded byte
 *   must be discarded — we do this by soft-resetting after SCD.
 *
 * IMPORTANT — the slave uses I2C_INT_ADDR_TARGET (AAS) + I2C_INT_RX_DATA_RDY
 * (RRDY) + I2C_INT_STOP_CONDITION (SCD) + I2C_INT_TX_DATA_RDY (XRDY).
 * Do NOT enable I2C_INT_NO_ACK on slave — master sends NACK to end a read;
 * that NACK is normal and must not trigger an error path.
 ******************************************************************************/

#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include "driverlib.h"
#include "device.h"

/*---------------------------------------------------------------------------
 * Slave 7-bit address — must match master's SLAVE_BASE_ADDR + index
 * Each physical board has its own address set at compile time or via GPIO
 *---------------------------------------------------------------------------*/
#ifndef MY_SLAVE_ADDR
#define MY_SLAVE_ADDR       0x10U           /* override per board           */
#endif

/*---------------------------------------------------------------------------
 * Health protocol (same values as master header)
 *---------------------------------------------------------------------------*/
#define HEALTH_CMD          0xA5U
#define HEALTH_SUBCMD       0x01U
#define HEALTH_ACK          0x5AU
#define HEALTH_STATUS_OK    0x07U           /* CPU=1, PERIPH=1, WDG=1       */

/*---------------------------------------------------------------------------
 * STATUS byte bit positions
 *---------------------------------------------------------------------------*/
#define STATUS_BIT_CPU      (1U << 0)
#define STATUS_BIT_PERIPH   (1U << 1)
#define STATUS_BIT_WDG      (1U << 2)

/*---------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/
void Slave_I2C_Init(void);
void Slave_I2C_UpdateHealth(uint16_t cpuOK,
                            uint16_t periphOK,
                            uint16_t wdgOK);

/* PIE interrupt handler — wire to I2CA_INT in PIE vector table */
__interrupt void i2cA_ISR(void);

/*---------------------------------------------------------------------------
 * Shared health state (written by application, read by ISR)
 *---------------------------------------------------------------------------*/
extern volatile uint16_t g_healthStatus;    /* STATUS byte to send master  */

#endif /* I2C_SLAVE_H */
