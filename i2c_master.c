/******************************************************************************
 * FILE   : i2c_master.c
 * DEVICE : TMS320F2800157
 * ROLE   : I2C Master — health-poll 15 slaves every 100 ms
 *
 * DESIGN RULES (why each decision was made):
 *
 *  1. I2CCNT is loaded BEFORE STT (start).  If loaded after, the hardware
 *     byte counter is wrong from the first clock edge.
 *
 *  2. Every polling loop also checks ARDY (register-access ready) so a NACK
 *     that pulls ARDY high will never leave us spinning on XRDY forever.
 *
 *  3. STOP is sent only after ARDY confirms the last byte left the shift
 *     register.  Sending it earlier can abort the byte mid-transmission.
 *
 *  4. After STOP we wait for: STP bit clear → BB bit clear.
 *     Jumping straight to the next slave while BB=1 causes ERROR_BUS_BUSY.
 *
 *  5. Write and Read are two SEPARATE transactions (START … STOP each).
 *     The slave ISR on the F280015x handles a write and a read as distinct
 *     events.  A combined write+restart+read is supported by the hardware
 *     but requires the slave to prepare TX data inside the AAS interrupt
 *     before the repeated-START address arrives — unreliable under ISR
 *     latency; two transactions is cleaner and safer.
 *
 *  6. Soft reset:  I2C_disableModule / I2C_enableModule.
 *     Clears BB, STP, XSMT, XRDY without touching GPIO — bus recovers
 *     in < 1 µs.  Used after any NACK or timeout.
 *
 *  7. Hard (9-clock) recovery: mux SCL to GPIO output, pulse it 9 times,
 *     then issue a STOP manually (SDA low→high while SCL high), then
 *     return SCL to I2C mux and do a soft reset.  Frees any slave that
 *     is stuck mid-byte holding SDA low.
 *
 *  8. GPIO pins assumed:  GPIO28 = SDA,  GPIO29 = SCL  (F280015x LP default)
 *     Change I2C_GPIO_SDA / I2C_GPIO_SCL if your board differs.
 ******************************************************************************/

#include "i2c_master.h"

/*===========================================================================
 * Private constants
 *===========================================================================*/
#define I2C_GPIO_SDA        28U     /* GPIO28 = SDAA on F280015x LP          */
#define I2C_GPIO_SCL        29U     /* GPIO29 = SCLA on F280015x LP          */

/* Bit-bang timings for 9-clock recovery (half-period at ~10 kHz) */
#define BB_HALF_PERIOD_US   50U

/*===========================================================================
 * Private prototypes
 *===========================================================================*/
static uint16_t waitBusFree(void);
static uint16_t waitArdy(void);
static uint16_t waitXrdy(void);
static uint16_t waitRrdy(void);
static uint16_t waitStopDone(void);
static uint16_t checkNack(void);
static void     softReset(void);
static void     hardBusRecovery(void);

static uint16_t i2c_writeTwoBytes(uint16_t addr,
                                  uint16_t byte0,
                                  uint16_t byte1);
static uint16_t i2c_readTwoBytes(uint16_t addr,
                                 uint16_t *byte0,
                                 uint16_t *byte1);

/*===========================================================================
 * Master_I2C_Init
 *
 * Call once from main() before any other I2C function.
 * Configures GPIO, clock dividers, and module options.
 *===========================================================================*/
void Master_I2C_Init(void)
{
    /*-----------------------------------------------------------------------
     * GPIO: open-drain, pull-up enabled, async qualification, mux to I2C
     *-----------------------------------------------------------------------*/
    GPIO_setPinConfig(GPIO_28_I2CA_SDA);
    GPIO_setDirectionMode(I2C_GPIO_SDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(I2C_GPIO_SDA, GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_OD);
    GPIO_setQualificationMode(I2C_GPIO_SDA, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_29_I2CA_SCL);
    GPIO_setDirectionMode(I2C_GPIO_SCL, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(I2C_GPIO_SCL, GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_OD);
    GPIO_setQualificationMode(I2C_GPIO_SCL, GPIO_QUAL_ASYNC);

    /*-----------------------------------------------------------------------
     * I2C module: controller, 100 kHz, 50 % duty, 7-bit, free-run in debug
     *-----------------------------------------------------------------------*/
    I2C_disableModule(I2CA_BASE);

    I2C_initController(I2CA_BASE,
                       DEVICE_SYSCLK_FREQ,
                       I2C_BUS_HZ,
                       I2C_DUTYCYCLE_50);

    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_setOwnAddress(I2CA_BASE, 0x00U);          /* master has no own addr  */
    I2C_setAddressMode(I2CA_BASE, I2C_ADDR_MODE_7BITS);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);
    I2C_disableLoopback(I2CA_BASE);

    /* No FIFO — plain DR registers; simpler and sufficient for 2-byte frames */
    I2C_disableFIFO(I2CA_BASE);

    /* Allow I2C to free-run when CCS halts the CPU (no false STOP) */
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_FREE_RUN);

    /* No interrupts on master side — we poll status flags */
    I2C_disableInterrupt(I2CA_BASE, 0xFFFFU);
    I2C_clearStatus(I2CA_BASE,
                    I2C_STS_ARB_LOST   |
                    I2C_STS_NO_ACK     |
                    I2C_STS_REG_ACCESS_RDY |
                    I2C_STS_STOP_CONDITION);

    I2C_enableModule(I2CA_BASE);
}

/*===========================================================================
 * Master_I2C_InitSlaveTable
 *
 * Zero-fill the table and assign sequential addresses.
 * Call once before the main loop.
 *===========================================================================*/
void Master_I2C_InitSlaveTable(SlaveRecord_t tbl[NUM_SLAVES])
{
    uint16_t i;
    for(i = 0U; i < NUM_SLAVES; i++)
    {
        tbl[i].addr       = (uint16_t)(SLAVE_BASE_ADDR + i);
        tbl[i].statusByte = 0U;
        tbl[i].healthy    = 0U;
        tbl[i].lastError  = I2CM_OK;
        tbl[i].pollCount  = 0UL;
        tbl[i].failCount  = 0UL;
    }
}

/*===========================================================================
 * Master_I2C_PollAllSlaves
 *
 * For each of the 15 slaves:
 *   1. Send 2-byte health request  [0xA5][0x01]
 *   2. Wait 500 µs  (slave ISR needs time to load reply)
 *   3. Read  2-byte health response [ACK][STATUS]
 *   4. Update SlaveRecord_t entry
 *
 * Designed to be called every 100 ms from a CPU-timer ISR flag or bare loop.
 * Each slave takes ≈ 0.4 ms (write) + 0.5 ms gap + 0.4 ms (read) = ~1.3 ms.
 * Total for 15 slaves ≈ 20 ms, comfortably inside the 100 ms window.
 *===========================================================================*/
void Master_I2C_PollAllSlaves(SlaveRecord_t tbl[NUM_SLAVES])
{
    uint16_t i;
    uint16_t ack, status;
    uint16_t txErr, rxErr;

    for(i = 0U; i < NUM_SLAVES; i++)
    {
        tbl[i].pollCount++;

        /*--------------------------------------------------------------------
         * Step 1 — Write: send health request
         *--------------------------------------------------------------------*/
        txErr = i2c_writeTwoBytes(tbl[i].addr, HEALTH_CMD, HEALTH_SUBCMD);

        if(txErr != I2CM_OK)
        {
            tbl[i].lastError  = txErr;
            tbl[i].healthy    = 0U;
            tbl[i].failCount++;
            /* Give bus time to settle before trying next slave */
            DEVICE_DELAY_US(500U);
            continue;
        }

        /*--------------------------------------------------------------------
         * Step 2 — Inter-frame gap: let slave ISR prepare the reply buffer.
         * 500 µs is generous; tighten once you've measured your slave ISR
         * response time with a logic analyser.
         *--------------------------------------------------------------------*/
        DEVICE_DELAY_US(500U);

        /*--------------------------------------------------------------------
         * Step 3 — Read: fetch health response
         *--------------------------------------------------------------------*/
        rxErr = i2c_readTwoBytes(tbl[i].addr, &ack, &status);

        if(rxErr != I2CM_OK)
        {
            tbl[i].lastError  = rxErr;
            tbl[i].healthy    = 0U;
            tbl[i].failCount++;
            DEVICE_DELAY_US(200U);
            continue;
        }

        /*--------------------------------------------------------------------
         * Step 4 — Validate and record
         *--------------------------------------------------------------------*/
        if(ack != HEALTH_ACK)
        {
            tbl[i].lastError  = I2CM_ERR_BAD_RESP;
            tbl[i].statusByte = ack;
            tbl[i].healthy    = 0U;
            tbl[i].failCount++;
        }
        else
        {
            tbl[i].lastError  = I2CM_OK;
            tbl[i].statusByte = status;
            tbl[i].healthy    = (status == HEALTH_STATUS_FULL) ? 1U : 0U;
            if(!tbl[i].healthy)
            {
                tbl[i].failCount++;
            }
        }

        /* Brief inter-slave gap: bus glitch protection */
        DEVICE_DELAY_US(100U);
    }
}

/*===========================================================================
 * i2c_writeTwoBytes  (private)
 *
 * Sends:  START [addr+W] ACK [byte0] ACK [byte1] ACK STOP
 *
 * Sequence that matches F280015x hardware rules:
 *   a) Load I2CCNT before setting STT
 *   b) Set I2CMDR: IRS | MST | TRX | STT  (no STP yet, no RM)
 *   c) Wait XRDY (or ARDY for early NACK), write byte 0
 *   d) Wait XRDY (or ARDY), write byte 1
 *   e) Wait ARDY — shift register emptied, safe to STOP
 *   f) Set STP bit
 *   g) Wait STP=0, BB=0
 *===========================================================================*/
static uint16_t i2c_writeTwoBytes(uint16_t addr,
                                  uint16_t byte0,
                                  uint16_t byte1)
{
    uint16_t err;

    /* --- Pre-condition: bus must be free --- */
    err = waitBusFree();
    if(err != I2CM_OK)
    {
        hardBusRecovery();
        err = waitBusFree();
        if(err != I2CM_OK) return err;
    }

    /* --- Set target address and byte count BEFORE start --- */
    I2C_setTargetAddress(I2CA_BASE, addr);
    I2C_setDataCount(I2CA_BASE, 2U);          /* MUST be set before STT     */

    /* --- Configure: master, transmit, no repeat mode, then START --- */
    /* Writing I2CMDR in one shot is the recommended approach on C2000;
     * it avoids partial-field glitches on a live bus.                  */
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(I2CA_BASE);        /* STT bit — HW clears it     */

    /* --- Byte 0 --- */
    err = waitXrdy();                         /* also escapes on ARDY/NACK  */
    if(err != I2CM_OK) { softReset(); return err; }
    if(checkNack() != I2CM_OK) { softReset(); return I2CM_ERR_NACK; }

    I2C_putData(I2CA_BASE, byte0);

    /* --- Byte 1 --- */
    err = waitXrdy();
    if(err != I2CM_OK) { softReset(); return err; }
    if(checkNack() != I2CM_OK) { softReset(); return I2CM_ERR_NACK; }

    I2C_putData(I2CA_BASE, byte1);

    /* --- Wait for ARDY: last byte fully shifted out --- */
    err = waitArdy();
    if(err != I2CM_OK) { softReset(); return err; }
    if(checkNack() != I2CM_OK) { softReset(); return I2CM_ERR_NACK; }

    /* --- Generate STOP --- */
    I2C_sendStopCondition(I2CA_BASE);

    err = waitStopDone();
    if(err != I2CM_OK) { softReset(); return err; }

    return I2CM_OK;
}

/*===========================================================================
 * i2c_readTwoBytes  (private)
 *
 * Sends:  START [addr+R] ACK  ← [byte0] ACK ← [byte1] NACK  STOP
 *
 * Key detail: STP bit is set BEFORE STT in receive mode when I2CCNT is used.
 * The hardware will auto-send NACK+STOP after it has clocked in I2CCNT bytes.
 * This is the correct non-repeat-mode receive flow for F280015x.
 *===========================================================================*/
static uint16_t i2c_readTwoBytes(uint16_t addr,
                                 uint16_t *byte0,
                                 uint16_t *byte1)
{
    uint16_t err;

    /* --- Pre-condition: bus must be free --- */
    err = waitBusFree();
    if(err != I2CM_OK)
    {
        hardBusRecovery();
        err = waitBusFree();
        if(err != I2CM_OK) return err;
    }

    /* --- Set address, count, config, then START+STP together --- */
    I2C_setTargetAddress(I2CA_BASE, addr);
    I2C_setDataCount(I2CA_BASE, 2U);

    /* Receive mode, non-repeat: hardware sends NACK+STOP automatically
     * when byte counter reaches 0.  Set STP here so it takes effect.   */
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_RECEIVE_MODE);
    I2C_sendStopCondition(I2CA_BASE);         /* arm auto-STOP              */
    I2C_sendStartCondition(I2CA_BASE);        /* START after STP is armed   */

    /* --- Read byte 0 --- */
    err = waitRrdy();
    if(err != I2CM_OK) { softReset(); return err; }
    if(checkNack() != I2CM_OK) { softReset(); return I2CM_ERR_NACK; }

    *byte0 = I2C_getData(I2CA_BASE);

    /* --- Read byte 1 --- */
    err = waitRrdy();
    if(err != I2CM_OK) { softReset(); return err; }

    *byte1 = I2C_getData(I2CA_BASE);
    /* NACK+STOP are now auto-generated by hardware */

    /* --- Wait for STOP to complete --- */
    err = waitStopDone();
    if(err != I2CM_OK) { softReset(); return err; }

    return I2CM_OK;
}

/*===========================================================================
 * LOW-LEVEL WAIT HELPERS
 * All have hard µs timeouts — no infinite loops anywhere.
 *===========================================================================*/

/* Wait for BB=0 and STP=0 (bus free) */
static uint16_t waitBusFree(void)
{
    uint16_t t;
    for(t = 0U; t < TIMEOUT_BUS_FREE_US; t++)
    {
        if(!I2C_isBusBusy(I2CA_BASE) &&
           !I2C_getStopConditionStatus(I2CA_BASE))
        {
            return I2CM_OK;
        }
        DEVICE_DELAY_US(1U);
    }
    return I2CM_ERR_TIMEOUT;
}

/* Wait for ARDY (register-access ready — last byte shifted out) */
static uint16_t waitArdy(void)
{
    uint16_t t;
    for(t = 0U; t < TIMEOUT_BYTE_TX_US; t++)
    {
        if(I2C_getStatus(I2CA_BASE) & I2C_STS_REG_ACCESS_RDY)
        {
            return I2CM_OK;
        }
        if(I2C_getStatus(I2CA_BASE) & I2C_STS_NO_ACK)
        {
            return I2CM_ERR_NACK;
        }
        DEVICE_DELAY_US(1U);
    }
    return I2CM_ERR_TIMEOUT;
}

/* Wait for XRDY=1 (TX data register ready for next byte)
 * Also exits if ARDY fires first (means NACK received — don't hang on XRDY) */
static uint16_t waitXrdy(void)
{
    uint16_t t;
    uint16_t sts;
    for(t = 0U; t < TIMEOUT_BYTE_TX_US; t++)
    {
        sts = I2C_getStatus(I2CA_BASE);
        if(sts & I2C_STS_NO_ACK)
        {
            return I2CM_ERR_NACK;
        }
        if(sts & I2C_STS_TX_DATA_RDY)
        {
            return I2CM_OK;
        }
        if(sts & I2C_STS_REG_ACCESS_RDY)
        {
            /* ARDY before XRDY means we were NACKed or count expired */
            return I2CM_ERR_NACK;
        }
        DEVICE_DELAY_US(1U);
    }
    return I2CM_ERR_TIMEOUT;
}

/* Wait for RRDY=1 (RX data register has a new byte) */
static uint16_t waitRrdy(void)
{
    uint16_t t;
    uint16_t sts;
    for(t = 0U; t < TIMEOUT_BYTE_RX_US; t++)
    {
        sts = I2C_getStatus(I2CA_BASE);
        if(sts & I2C_STS_NO_ACK)
        {
            return I2CM_ERR_NACK;
        }
        if(sts & I2C_STS_RX_DATA_RDY)
        {
            return I2CM_OK;
        }
        DEVICE_DELAY_US(1U);
    }
    return I2CM_ERR_TIMEOUT;
}

/* Wait for STP bit clear AND BB clear (transaction fully ended on bus) */
static uint16_t waitStopDone(void)
{
    uint16_t t;
    for(t = 0U; t < TIMEOUT_STOP_US; t++)
    {
        if(!I2C_getStopConditionStatus(I2CA_BASE) &&
           !I2C_isBusBusy(I2CA_BASE))
        {
            /* Clear residual status flags */
            I2C_clearStatus(I2CA_BASE,
                            I2C_STS_ARB_LOST       |
                            I2C_STS_NO_ACK         |
                            I2C_STS_REG_ACCESS_RDY |
                            I2C_STS_STOP_CONDITION);
            return I2CM_OK;
        }
        DEVICE_DELAY_US(1U);
    }
    return I2CM_ERR_TIMEOUT;
}

/* Check and clear NACK without blocking */
static uint16_t checkNack(void)
{
    if(I2C_getStatus(I2CA_BASE) & I2C_STS_NO_ACK)
    {
        I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK);
        return I2CM_ERR_NACK;
    }
    return I2CM_OK;
}

/*===========================================================================
 * RECOVERY FUNCTIONS
 *===========================================================================*/

/* softReset — clears all internal state, leaves GPIO mux as I2C.
 * Sends STOP first so a slave that is mid-byte gets a clean abort,
 * then resets the module.                                            */
static void softReset(void)
{
    uint16_t t;

    /* Attempt a STOP before resetting */
    I2C_sendStopCondition(I2CA_BASE);
    for(t = 0U; t < 500U; t++)
    {
        if(!I2C_getStopConditionStatus(I2CA_BASE)) break;
        DEVICE_DELAY_US(1U);
    }

    /* Reset: IRS=0 clears BB, XSMT, XRDY, all status flags */
    I2C_disableModule(I2CA_BASE);
    DEVICE_DELAY_US(10U);
    I2C_enableModule(I2CA_BASE);

    /* Clear any flags set during reset */
    I2C_clearStatus(I2CA_BASE,
                    I2C_STS_ARB_LOST       |
                    I2C_STS_NO_ACK         |
                    I2C_STS_REG_ACCESS_RDY |
                    I2C_STS_STOP_CONDITION);
}

/* hardBusRecovery — used when BB is stuck and softReset is not enough.
 *
 * A slave can hold SDA low mid-byte if the master lost power or reset
 * while clocking.  The only cure is to clock SCL 9 times (max one full
 * byte + ACK) with SDA released, then manually create a STOP condition.
 *
 * Steps:
 *   1. Remux SCL to GPIO output, keep SDA as GPIO input (released)
 *   2. Toggle SCL 9 times — frees any stuck slave shift register
 *   3. Manually create STOP: SCL high, SDA low→high
 *   4. Remux both pins back to I2C peripheral
 *   5. Soft reset the I2C module
 */
static void hardBusRecovery(void)
{
    uint16_t i;

    /*-- 1. Switch SCL to GPIO output, SDA to GPIO input (open/release) ----*/
    GPIO_setPinConfig(GPIO_28_GPIO28);                  /* SDA = GPIO         */
    GPIO_setDirectionMode(I2C_GPIO_SDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(I2C_GPIO_SDA,
                      GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_OD);

    GPIO_setPinConfig(GPIO_29_GPIO29);                  /* SCL = GPIO output  */
    GPIO_setDirectionMode(I2C_GPIO_SCL, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(I2C_GPIO_SCL,
                      GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_OD);
    GPIO_writePin(I2C_GPIO_SCL, 1U);

    /*-- 2. Pulse SCL 9 times, checking if SDA released each cycle ----------*/
    for(i = 0U; i < 9U; i++)
    {
        GPIO_writePin(I2C_GPIO_SCL, 0U);
        DEVICE_DELAY_US(BB_HALF_PERIOD_US);
        GPIO_writePin(I2C_GPIO_SCL, 1U);
        DEVICE_DELAY_US(BB_HALF_PERIOD_US);

        /* If SDA is released (high) the slave is free */
        if(GPIO_readPin(I2C_GPIO_SDA) == 1U) break;
    }

    /*-- 3. Manual STOP: SCL high, SDA low → high ---------------------------*/
    /* Switch SDA to output temporarily to drive the STOP */
    GPIO_setDirectionMode(I2C_GPIO_SDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(I2C_GPIO_SDA,
                      GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_OD);

    GPIO_writePin(I2C_GPIO_SDA, 0U);          /* SDA low (setup)             */
    DEVICE_DELAY_US(BB_HALF_PERIOD_US);
    GPIO_writePin(I2C_GPIO_SCL, 1U);          /* SCL high                    */
    DEVICE_DELAY_US(BB_HALF_PERIOD_US);
    GPIO_writePin(I2C_GPIO_SDA, 1U);          /* SDA high while SCL high = STOP */
    DEVICE_DELAY_US(BB_HALF_PERIOD_US);

    /*-- 4. Remux SCL and SDA back to I2C peripheral ------------------------*/
    GPIO_setPinConfig(GPIO_28_I2CA_SDA);
    GPIO_setDirectionMode(I2C_GPIO_SDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(I2C_GPIO_SDA,
                      GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_OD);
    GPIO_setQualificationMode(I2C_GPIO_SDA, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_29_I2CA_SCL);
    GPIO_setDirectionMode(I2C_GPIO_SCL, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(I2C_GPIO_SCL,
                      GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_OD);
    GPIO_setQualificationMode(I2C_GPIO_SCL, GPIO_QUAL_ASYNC);

    DEVICE_DELAY_US(100U);

    /*-- 5. Soft reset the I2C module ----------------------------------------*/
    softReset();
}
