//#############################################################################
//
// FILE:   i2cLib_FIFO_polling.c
//
// TITLE:  C28x-I2C Library - FIFO Polling (Fixed + Multi-Slave Health Poll)
//
// FIXES vs. original TI reference:
//   FIX-1  Stray ';' after while() in I2C_ControllerReceiver() removed
//          → was causing infinite loop, holding SCL low, BB stuck at 1
//   FIX-2  I2CCNT now set BEFORE I2C_TransmitTargetAddress_ControlBytes()
//          → was set after START, causing data count mismatch
//   FIX-3  RX FIFO drained BEFORE I2C_sendStopCondition() in receiver
//          → was draining after STOP, causing data loss
//   FIX-4  Added ARDY + BB wait after every STOP condition
//          → prevents "ERROR_BUS_BUSY" on the very next transaction
//   FIX-5  Added 10 µs settle between RX mode switch and repeated START
//   FIX-6  handleNACK() now waits for STOP to complete after sending it
//   FIX-7  Renamed I2C_TransmittargetAddress_ControlBytes for consistency
//
// NEW:
//   I2C_SendHealthRequest()  – sends 2-byte health command to one slave
//   I2C_ReadHealthResponse() – reads 2-byte health reply from one slave
//   I2C_PollAllSlaves()      – polls all 15 slaves sequentially, fills table
//
//#############################################################################

#include "i2cLib_FIFO_polling.h"

//=============================================================================
// Internal helper: wait for bus to become free (BB=0, STP=0)
// Returns SUCCESS or ERROR_TIMEOUT
//=============================================================================
uint16_t waitBusRelease(uint32_t base, uint16_t timeout_us)
{
    uint16_t elapsed = 0U;

    // Wait for ARDY (register access ready) – confirms STOP propagated
    while(!(I2C_getStatus(base) & I2C_STS_REG_ACCESS_RDY))
    {
        DEVICE_DELAY_US(1U);
        if(++elapsed >= timeout_us)
        {
            return ERROR_TIMEOUT;
        }
    }

    // Wait for STP bit cleared by hardware
    elapsed = 0U;
    while(I2C_getStopConditionStatus(base))
    {
        DEVICE_DELAY_US(1U);
        if(++elapsed >= timeout_us)
        {
            return ERROR_TIMEOUT;
        }
    }

    // Wait for BB (bus busy) to deassert
    elapsed = 0U;
    while(I2C_isBusBusy(base))
    {
        DEVICE_DELAY_US(1U);
        if(++elapsed >= timeout_us)
        {
            return ERROR_TIMEOUT;
        }
    }

    // Clear residual status flags
    I2C_clearStatus(base, I2C_STS_NO_ACK     |
                          I2C_STS_ARB_LOST   |
                          I2C_STS_REG_ACCESS_RDY |
                          I2C_STS_STOP_CONDITION);
    return SUCCESS;
}

//=============================================================================
// checkBusStatus – quick pre-transaction check
//=============================================================================
uint16_t checkBusStatus(uint32_t base)
{
    if(I2C_isBusBusy(base))
    {
        return ERROR_BUS_BUSY;
    }
    if(I2C_getStopConditionStatus(base))
    {
        return ERROR_STOP_NOT_READY;
    }
    return SUCCESS;
}

//=============================================================================
// handleNACK – FIX-6: wait for STOP to finish after sending it
//=============================================================================
uint16_t handleNACK(uint32_t base)
{
    if(I2C_getStatus(base) & I2C_STS_NO_ACK)
    {
        I2C_clearStatus(base, I2C_STS_NO_ACK);
        I2C_sendStopCondition(base);

        // Wait for STOP to complete (was missing in original)
        uint16_t i;
        for(i = 0U; i < 200U; i++)
        {
            DEVICE_DELAY_US(1U);
            if(!I2C_getStopConditionStatus(base))
            {
                break;
            }
        }
        return ERROR_NACK_RECEIVED;
    }
    return SUCCESS;
}

//=============================================================================
// I2CBusScan – probe all addresses, return list of responding slaves
//=============================================================================
uint16_t I2CBusScan(uint32_t base, uint16_t *pAvailableI2C_targets)
{
    uint16_t probeTargetAddress, i = 0U;

    I2C_disableInterrupt(base, (I2C_INT_ADDR_TARGET   |
                                I2C_INT_STOP_CONDITION |
                                I2C_INT_ARB_LOST       |
                                I2C_INT_NO_ACK));

    for(probeTargetAddress = 1U;
        probeTargetAddress <= MAX_10_BIT_ADDRESS;
        probeTargetAddress++)
    {
        if(checkBusStatus(base) != SUCCESS)
        {
            ESTOP0;
            return ERROR_BUS_BUSY;
        }

        I2C_setConfig(base, (I2C_CONTROLLER_SEND_MODE | I2C_REPEAT_MODE));

        if(probeTargetAddress > MAX_7_BIT_ADDRESS)
        {
            I2C_setAddressMode(base, I2C_ADDR_MODE_10BITS);
        }
        else
        {
            I2C_setAddressMode(base, I2C_ADDR_MODE_7BITS);
        }

        I2C_setTargetAddress(base, probeTargetAddress);
        I2C_sendStartCondition(base);

        while(!(I2C_getStatus(base) & I2C_STS_REG_ACCESS_RDY));

        I2C_sendStopCondition(base);

        // FIX-4: use waitBusRelease instead of bare spin loops
        waitBusRelease(base, 500U);

        if(!(I2C_getStatus(base) & I2C_STS_NO_ACK))
        {
            pAvailableI2C_targets[i++] = probeTargetAddress;
        }

        I2C_clearStatus(base, I2C_STS_NO_ACK     |
                              I2C_STS_ARB_LOST   |
                              I2C_STS_REG_ACCESS_RDY |
                              I2C_STS_STOP_CONDITION);
    }

    I2C_setConfig(base, I2C_CONTROLLER_SEND_MODE);
    I2C_setAddressMode(base, I2C_ADDR_MODE_7BITS);
    I2C_enableInterrupt(base, (I2C_INT_ADDR_TARGET   |
                               I2C_INT_STOP_CONDITION |
                               I2C_INT_ARB_LOST       |
                               I2C_INT_NO_ACK));
    return SUCCESS;
}

//=============================================================================
// I2C_TransmitTargetAddress_ControlBytes
// Sends: START + slave-address + control/register bytes
// Leaves bus in REPEAT mode so caller can either continue TX or do repeated
// START for RX.
//
// FIX-7: renamed from I2C_TransmittargetAddress_ControlBytes (typo)
//=============================================================================
uint16_t I2C_TransmitTargetAddress_ControlBytes(struct I2CHandle *I2C_Params)
{
    uint16_t status;
    uint16_t attemptCount = 1U;
    uint32_t base = I2C_Params->base;

    // Wait for bus to be free, retry up to NumOfAttempts times
    do
    {
        status = checkBusStatus(base);
        if(status == SUCCESS)
        {
            break;
        }
        DEVICE_DELAY_US(I2C_Params->Delay_us);
        attemptCount++;
    }
    while(attemptCount <= I2C_Params->NumOfAttempts);

    if(status != SUCCESS)
    {
        return status;
    }

    // Set controller to send mode, keep REPEAT so STOP is not auto-generated
    I2C_setConfig(base, (I2C_CONTROLLER_SEND_MODE | I2C_REPEAT_MODE));

    if(I2C_Params->TargetAddr > MAX_7_BIT_ADDRESS)
    {
        I2C_setAddressMode(base, I2C_ADDR_MODE_10BITS);
    }
    else
    {
        I2C_setAddressMode(base, I2C_ADDR_MODE_7BITS);
    }

    I2C_setTargetAddress(base, I2C_Params->TargetAddr);

    // Load control/address bytes into TX FIFO before issuing START
    int16_t  i;
    uint32_t temp = *(I2C_Params->pControlAddr);
    for(i = (int16_t)I2C_Params->NumOfAddrBytes - 1; i >= 0; i--)
    {
        I2C_putData(base, (temp >> ((uint16_t)i * 8U)) & 0xFFU);
    }

    I2C_sendStartCondition(base);
    DEVICE_DELAY_US(150U);          // Let address phase settle

    // Check for NACK on address
    status = handleNACK(base);
    if(status != SUCCESS)
    {
        return status;
    }

    // Poll until control bytes leave the FIFO
    attemptCount = 1U;
    uint16_t maxWait = (uint16_t)(9U * (I2C_Params->NumOfAddrBytes + 2U));
    while(I2C_getTxFIFOStatus(base) != I2C_FIFO_TX0 &&
          attemptCount <= maxWait)
    {
        status = handleNACK(base);
        if(status != SUCCESS)
        {
            return status;
        }
        attemptCount++;
        DEVICE_DELAY_US(I2C_Params->Delay_us);
    }

    return SUCCESS;
}

//=============================================================================
// I2C_ControllerTransmitter
// Full write transaction: START + addr + ctrl bytes + data bytes + STOP
//
// FIX-2: I2CCNT set BEFORE calling I2C_TransmitTargetAddress_ControlBytes
//=============================================================================
uint16_t I2C_ControllerTransmitter(struct I2CHandle *I2C_Params)
{
    uint16_t status;
    uint16_t attemptCount;
    uint16_t i, count, buff_pos;
    uint32_t base = I2C_Params->base;

    I2C_disableFIFO(base);
    I2C_enableFIFO(base);

    // FIX-2: set total byte count BEFORE start is generated
    I2C_setDataCount(base, (I2C_Params->NumOfAddrBytes +
                            I2C_Params->NumOfDataBytes));

    status = I2C_TransmitTargetAddress_ControlBytes(I2C_Params);
    if(status != SUCCESS)
    {
        return status;
    }

    // Chunk data into 16-byte FIFO loads
    uint16_t numFullChunks  = I2C_Params->NumOfDataBytes / I2C_FIFO_LEVEL;
    uint16_t remainingBytes = I2C_Params->NumOfDataBytes % I2C_FIFO_LEVEL;

    count    = 0U;
    buff_pos = 0U;

    while(count < numFullChunks)
    {
        for(i = 0U; i < I2C_FIFO_LEVEL; i++)
        {
            I2C_putData(base, I2C_Params->pTX_MsgBuffer[buff_pos++]);
        }

        attemptCount = 1U;
        uint16_t maxWait = (uint16_t)(9U * (I2C_FIFO_LEVEL + 2U));
        while(I2C_getTxFIFOStatus(base) != I2C_FIFO_TX0 &&
              attemptCount <= maxWait)
        {
            status = handleNACK(base);
            if(status != SUCCESS)
            {
                return status;
            }
            attemptCount++;
            DEVICE_DELAY_US(I2C_Params->Delay_us);
        }
        count++;
    }

    // Remaining bytes (< 16)
    for(i = 0U; i < remainingBytes; i++)
    {
        I2C_putData(base, I2C_Params->pTX_MsgBuffer[buff_pos++]);
    }

    if(remainingBytes > 0U)
    {
        attemptCount = 1U;
        uint16_t maxWait = (uint16_t)(9U * (remainingBytes + 2U));
        while(I2C_getTxFIFOStatus(base) != I2C_FIFO_TX0 &&
              attemptCount <= maxWait)
        {
            status = handleNACK(base);
            if(status != SUCCESS)
            {
                return status;
            }
            attemptCount++;
            DEVICE_DELAY_US(I2C_Params->Delay_us);
        }
    }

    // Generate STOP
    I2C_sendStopCondition(base);

    // FIX-4: proper ARDY + BB wait after STOP
    status = waitBusRelease(base, 500U);

    return status;
}

//=============================================================================
// I2C_ControllerReceiver
// Full read transaction: START + addr + ctrl bytes + repeated-START + RX + STOP
//
// FIX-1: removed stray ';' from while() at end
// FIX-3: drain RX FIFO BEFORE sending STOP
// FIX-4: proper ARDY + BB wait after STOP
// FIX-5: settle delay between mode switch and repeated START
//=============================================================================
uint16_t I2C_ControllerReceiver(struct I2CHandle *I2C_Params)
{
    uint16_t status;
    uint16_t attemptCount;
    uint16_t i, count, buff_pos;
    uint32_t base = I2C_Params->base;

    I2C_disableFIFO(base);
    I2C_enableFIFO(base);

    // Phase 1: send target address + register/control bytes (write phase)
    status = I2C_TransmitTargetAddress_ControlBytes(I2C_Params);
    if(status != SUCCESS)
    {
        return status;
    }

    uint16_t numFullChunks  = I2C_Params->NumOfDataBytes / I2C_FIFO_LEVEL;
    uint16_t remainingBytes = I2C_Params->NumOfDataBytes % I2C_FIFO_LEVEL;

    // Phase 2: switch to receive mode and issue repeated START
    // FIX-5: brief settle before repeated START
    I2C_setConfig(base, (I2C_CONTROLLER_RECEIVE_MODE | I2C_REPEAT_MODE));
    DEVICE_DELAY_US(10U);
    I2C_setDataCount(base, I2C_Params->NumOfDataBytes);
    I2C_sendStartCondition(base);

    count    = 0U;
    buff_pos = 0U;

    // Read full 16-byte chunks
    while(count < numFullChunks)
    {
        status = handleNACK(base);
        if(status != SUCCESS)
        {
            return status;
        }

        attemptCount = 1U;
        uint16_t maxWait = (uint16_t)(9U * (I2C_FIFO_LEVEL + 2U));
        while(I2C_getRxFIFOStatus(base) != I2C_FIFO_RXFULL &&
              attemptCount <= maxWait)
        {
            DEVICE_DELAY_US(I2C_Params->Delay_us);
            attemptCount++;
        }

        // FIX-3: drain FIFO BEFORE STOP
        for(i = 0U; i < I2C_FIFO_LEVEL; i++)
        {
            I2C_Params->pRX_MsgBuffer[buff_pos++] = I2C_getData(base);
        }
        count++;
    }

    // Wait for remaining bytes to arrive
    if(remainingBytes > 0U)
    {
        attemptCount = 1U;
        uint16_t maxWait = (uint16_t)(9U * (remainingBytes + 2U));
        while(I2C_getRxFIFOStatus(base) != (I2C_RxFIFOLevel)remainingBytes &&
              attemptCount <= maxWait)
        {
            DEVICE_DELAY_US(I2C_Params->Delay_us);
            attemptCount++;
        }

        // FIX-3: drain remaining bytes BEFORE STOP
        for(i = 0U; i < remainingBytes; i++)
        {
            I2C_Params->pRX_MsgBuffer[buff_pos++] = I2C_getData(base);
        }
    }

    // Check NACK before sending STOP
    status = handleNACK(base);
    if(status != SUCCESS)
    {
        return status;
    }

    // Generate STOP — after all data is safely read
    I2C_sendStopCondition(base);

    I2C_disableFIFO(base);

    // FIX-1 + FIX-4: proper wait, no stray semicolon
    status = waitBusRelease(base, 500U);

    return status;
}

//=============================================================================
// I2C_SendHealthRequest
//
// Sends a 2-byte health command to one slave:
//   Byte 0: HEALTH_REQ_CMD   (0xA5)
//   Byte 1: HEALTH_REQ_SUBCMD (0x01)
//
// Uses I2C_ControllerTransmitter internally.
// NumOfAddrBytes = 0 (no register address, pure command)
//=============================================================================
uint16_t I2C_SendHealthRequest(struct I2CHandle *I2C_Params)
{
    uint32_t  dummyCtrlAddr = 0U;

    // Health command payload
    I2C_Params->pTX_MsgBuffer[0] = HEALTH_REQ_CMD;
    I2C_Params->pTX_MsgBuffer[1] = HEALTH_REQ_SUBCMD;

    // No register/address bytes — we send the command directly
    I2C_Params->NumOfAddrBytes  = 0U;
    I2C_Params->NumOfDataBytes  = HEALTH_CMD_BYTES;
    I2C_Params->pControlAddr    = &dummyCtrlAddr;

    return I2C_ControllerTransmitter(I2C_Params);
}

//=============================================================================
// I2C_ReadHealthResponse
//
// Reads 2-byte health response from one slave and validates it:
//   Byte 0: must be HEALTH_ACK (0x5A)
//   Byte 1: STATUS_BYTE – bits 0-2 must all be 1 for full health
//
// Fills pSlaveStatus->healthOK, ->lastStatus, ->commError
//=============================================================================
uint16_t I2C_ReadHealthResponse(struct I2CHandle *I2C_Params,
                                SlaveHealthStatus *pSlaveStatus)
{
    uint16_t  status;
    uint32_t  dummyCtrlAddr = 0U;

    I2C_Params->NumOfAddrBytes = 0U;
    I2C_Params->NumOfDataBytes = HEALTH_RESP_BYTES;
    I2C_Params->pControlAddr   = &dummyCtrlAddr;

    // Clear RX buffer slots before read
    I2C_Params->pRX_MsgBuffer[0] = 0U;
    I2C_Params->pRX_MsgBuffer[1] = 0U;

    status = I2C_ControllerReceiver(I2C_Params);

    pSlaveStatus->commError = status;

    if(status != SUCCESS)
    {
        pSlaveStatus->healthOK    = 0U;
        pSlaveStatus->lastStatus  = 0U;
        pSlaveStatus->failCount++;
        return status;
    }

    // Validate ACK byte
    if(I2C_Params->pRX_MsgBuffer[0] != HEALTH_ACK)
    {
        pSlaveStatus->healthOK   = 0U;
        pSlaveStatus->lastStatus = I2C_Params->pRX_MsgBuffer[0];
        pSlaveStatus->commError  = ERROR_BAD_RESPONSE;
        pSlaveStatus->failCount++;
        return ERROR_BAD_RESPONSE;
    }

    // Store and evaluate status byte
    pSlaveStatus->lastStatus = I2C_Params->pRX_MsgBuffer[1];
    pSlaveStatus->healthOK   = (pSlaveStatus->lastStatus == HEALTH_STATUS_OK)
                               ? 1U : 0U;

    if(!pSlaveStatus->healthOK)
    {
        pSlaveStatus->failCount++;
    }

    return SUCCESS;
}

//=============================================================================
// I2C_PollAllSlaves
//
// Sequentially polls all NUM_SLAVES (15) slaves on the I2C bus.
// For each slave:
//   1. Sends health request  (2-byte write)
//   2. Short inter-frame gap
//   3. Reads health response (2-byte read)
//   4. Updates slaveTable[] entry
//
// slaveTable must be pre-initialised (typically at startup) with each
// entry's .address field set.  Call I2C_InitSlaveTable() once to do that.
//
// This function is designed to be called from your main polling loop or
// a periodic timer ISR on the master.
//=============================================================================
void I2C_PollAllSlaves(uint32_t base,
                       SlaveHealthStatus slaveTable[NUM_SLAVES])
{
    static uint16_t  txBuf[MAX_BUFFER_SIZE];
    static uint16_t  rxBuf[MAX_BUFFER_SIZE];

    struct I2CHandle handle;
    handle.base          = base;
    handle.pTX_MsgBuffer = txBuf;
    handle.pRX_MsgBuffer = rxBuf;
    handle.NumOfAttempts = 3U;
    handle.Delay_us      = 10U;

    uint16_t s;
    for(s = 0U; s < NUM_SLAVES; s++)
    {
        SlaveHealthStatus *pSlave = &slaveTable[s];

        handle.TargetAddr = pSlave->address;
        pSlave->pollCount++;

        // ---- Step 1: Send health request ----
        uint16_t txStatus = I2C_SendHealthRequest(&handle);

        if(txStatus != SUCCESS)
        {
            pSlave->commError = txStatus;
            pSlave->healthOK  = 0U;
            pSlave->failCount++;
            // Bus recovery: short wait then continue to next slave
            DEVICE_DELAY_US(200U);
            continue;
        }

        // ---- Step 2: Inter-frame gap (give slave time to prepare reply) ----
        // 500 µs is conservative; reduce if your slave ISR is faster
        DEVICE_DELAY_US(500U);

        // ---- Step 3: Read health response ----
        I2C_ReadHealthResponse(&handle, pSlave);

        // ---- Step 4: Inter-slave gap ----
        DEVICE_DELAY_US(100U);
    }
}

//=============================================================================
// I2C_InitSlaveTable
//
// Call once at startup to zero-initialise the slave health table and
// assign sequential I2C addresses starting from SLAVE_BASE_ADDR.
//=============================================================================
void I2C_InitSlaveTable(SlaveHealthStatus slaveTable[NUM_SLAVES])
{
    uint16_t s;
    for(s = 0U; s < NUM_SLAVES; s++)
    {
        slaveTable[s].address    = (uint16_t)(SLAVE_BASE_ADDR + s);
        slaveTable[s].lastStatus = 0U;
        slaveTable[s].commError  = 0U;
        slaveTable[s].healthOK   = 0U;
        slaveTable[s].pollCount  = 0UL;
        slaveTable[s].failCount  = 0UL;
    }
}
