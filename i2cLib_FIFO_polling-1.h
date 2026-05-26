//#############################################################################
//
// FILE:   i2cLib_FIFO_polling.h
//
// TITLE:  C28x-I2C Library header - FIFO Polling (Fixed + Multi-Slave Health Poll)
//
// FIXES APPLIED:
//   1. Stray semicolon in I2C_ControllerReceiver() while() -> infinite loop fixed
//   2. I2CCNT now set BEFORE START condition (was set after)
//   3. STOP now sent AFTER draining RX FIFO (was before)
//   4. Added ARDY + BB wait after STOP for clean bus release
//   5. Added settle delay between RX mode switch and repeated START
//
// NEW:
//   - Health command/response defines
//   - SlaveHealthStatus struct per slave
//   - I2C_SendHealthRequest() and I2C_ReadHealthResponse() prototypes
//   - I2C_PollAllSlaves() for master polling 15 slaves
//
//#############################################################################

#ifndef I2CLIB_FIFO_POLLING_H
#define I2CLIB_FIFO_POLLING_H

#include "device.h"

//-----------------------------------------------------------------------------
// Error codes
//-----------------------------------------------------------------------------
#define SUCCESS                     0x0000U
#define ERROR_BUS_BUSY              0x1000U
#define ERROR_NACK_RECEIVED         0x2000U
#define ERROR_ARBITRATION_LOST      0x3000U
#define ERROR_STOP_NOT_READY        0x5555U
#define ERROR_TIMEOUT               0x6000U
#define ERROR_BAD_RESPONSE          0x7000U

//-----------------------------------------------------------------------------
// I2C bus / FIFO constants
//-----------------------------------------------------------------------------
#define MAX_BUFFER_SIZE             64U
#define I2C_FIFO_LEVEL              16U
#define MAX_7_BIT_ADDRESS           127U
#define MAX_10_BIT_ADDRESS          1023U

//-----------------------------------------------------------------------------
// Health protocol defines
//
// Master sends:  [HEALTH_REQ_CMD] [HEALTH_REQ_SUBCMD]   (2 bytes)
// Slave replies: [HEALTH_ACK]     [STATUS_BYTE]         (2 bytes)
//
// STATUS_BYTE bit map:
//   Bit 0  : CPU OK        (1 = healthy)
//   Bit 1  : Peripherals OK (1 = healthy)
//   Bit 2  : Watchdog OK   (1 = healthy)
//   Bit 3-7: Reserved (0)
//
// A fully healthy slave returns STATUS_BYTE = 0x07
//-----------------------------------------------------------------------------
#define HEALTH_REQ_CMD              0xA5U   // Health request command byte
#define HEALTH_REQ_SUBCMD           0x01U   // Sub-command: "report health"
#define HEALTH_ACK                  0x5AU   // Expected first byte from slave
#define HEALTH_STATUS_OK            0x07U   // All 3 status bits set = healthy

#define HEALTH_CMD_BYTES            2U      // Bytes master sends  (cmd + subcmd)
#define HEALTH_RESP_BYTES           2U      // Bytes master reads  (ack + status)

//-----------------------------------------------------------------------------
// Number of slaves on the bus
//-----------------------------------------------------------------------------
#define NUM_SLAVES                  15U

//-----------------------------------------------------------------------------
// Slave 7-bit addresses: 0x10 … 0x1E  (adjust to your actual addressing)
//-----------------------------------------------------------------------------
#define SLAVE_BASE_ADDR             0x10U

//-----------------------------------------------------------------------------
// Per-slave health status
//-----------------------------------------------------------------------------
typedef struct
{
    uint16_t address;           // 7-bit I2C address of this slave
    uint16_t lastStatus;        // Raw STATUS_BYTE from last response
    uint16_t commError;         // Last communication error code (SUCCESS = 0)
    uint16_t healthOK;          // 1 = healthy, 0 = fault or comms failure
    uint32_t pollCount;         // Total number of times this slave was polled
    uint32_t failCount;         // Total number of failed polls
} SlaveHealthStatus;

//-----------------------------------------------------------------------------
// I2C handle (unchanged fields kept, unused fields removed for clarity)
//-----------------------------------------------------------------------------
struct I2CHandle
{
    uint32_t  base;
    uint16_t  TargetAddr;           // 7-bit or 10-bit target address
    uint32_t *pControlAddr;         // Pointer to register/control address word
    uint16_t  NumOfAddrBytes;       // Number of address bytes (1 or 2)
    uint16_t *pTX_MsgBuffer;        // Pointer to TX data buffer
    uint16_t *pRX_MsgBuffer;        // Pointer to RX data buffer
    uint16_t  NumOfDataBytes;       // Payload bytes to TX or RX
    uint16_t  NumOfAttempts;        // Retry attempts before giving up
    uint16_t  Delay_us;             // Polling delay between retries (µs)
    uint16_t  WriteCycleTime_in_us; // Write cycle time for EEPROM-type targets
};

//-----------------------------------------------------------------------------
// Core library prototypes
//-----------------------------------------------------------------------------
uint16_t I2CBusScan(uint32_t base, uint16_t *pAvailableI2C_targets);

uint16_t I2C_TransmitTargetAddress_ControlBytes(struct I2CHandle *I2C_Params);

uint16_t I2C_ControllerTransmitter(struct I2CHandle *I2C_Params);
uint16_t I2C_ControllerReceiver(struct I2CHandle *I2C_Params);

uint16_t checkBusStatus(uint32_t base);
uint16_t waitBusRelease(uint32_t base, uint16_t timeout_us);
uint16_t handleNACK(uint32_t base);

//-----------------------------------------------------------------------------
// Multi-slave health poll prototypes
//-----------------------------------------------------------------------------
uint16_t I2C_SendHealthRequest(struct I2CHandle *I2C_Params);
uint16_t I2C_ReadHealthResponse(struct I2CHandle *I2C_Params,
                                SlaveHealthStatus *pSlaveStatus);
void     I2C_PollAllSlaves(uint32_t base,
                           SlaveHealthStatus slaveTable[NUM_SLAVES]);

//-----------------------------------------------------------------------------
// Externals expected from application
//-----------------------------------------------------------------------------
extern uint16_t TX_MsgBuffer[MAX_BUFFER_SIZE];
extern uint16_t RX_MsgBuffer[MAX_BUFFER_SIZE];
extern uint16_t status;

#endif // I2CLIB_FIFO_POLLING_H
