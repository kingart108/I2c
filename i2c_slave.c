/******************************************************************************
 * FILE   : i2c_slave.c
 * DEVICE : TMS320F2800157
 * ROLE   : I2C Slave — interrupt-driven health responder
 *
 * REGISTER-LEVEL RULES FOLLOWED:
 *
 *  1. AAS (addressed-as-slave) fires for BOTH write and read transactions.
 *     SDIR bit in I2CSTR distinguishes: 0=master-write, 1=master-read.
 *
 *  2. For WRITE (master → slave):
 *     - Each incoming byte triggers RRDY interrupt.
 *     - SCD (stop-condition-detected) fires at end of write transaction.
 *     - Read I2CDRR on every RRDY or the next byte is lost (no FIFO here).
 *
 *  3. For READ (master ← slave):
 *     - Load I2CDXR on AAS interrupt (byte 0) immediately.
 *     - Hardware copies to shift register instantly, fires XRDY again.
 *     - Load I2CDXR again on that XRDY (byte 1 — double buffer pre-load).
 *     - Master sends NACK after byte 1 and generates STOP.
 *     - SCD fires.  We do a soft reset (IRS=0 then IRS=1) to flush the
 *       TX shift register; otherwise the stale byte stays for next read.
 *
 *  4. Interrupt source is read from I2CISRC (I2C interrupt source register).
 *     Only process the source indicated; do not poll all flags manually.
 *
 *  5. I2CSTR NACK bit is NOT an error on the slave side during a read
 *     transaction — the master's NACK is the normal end-of-read signal.
 *     DO NOT enable I2C_INT_NO_ACK on the slave.
 *
 *  6. After every ISR, acknowledge PIE group 8 (I2CA is PIE 8.1 on F280015x).
 ******************************************************************************/

#include "i2c_slave.h"

/*===========================================================================
 * Private types
 *===========================================================================*/
typedef enum
{
    SLV_IDLE      = 0,   /* waiting for AAS                                  */
    SLV_RX_CMD    = 1,   /* receiving cmd byte  (byte 0 of write)            */
    SLV_RX_SUBCMD = 2,   /* receiving subcmd    (byte 1 of write)            */
    SLV_TX        = 3    /* transmitting health reply to master               */
} SlaveState_t;

/*===========================================================================
 * Private variables
 *===========================================================================*/
static volatile SlaveState_t s_state   = SLV_IDLE;
static volatile uint16_t     s_cmd     = 0U;
static volatile uint16_t     s_subcmd  = 0U;
static volatile uint16_t     s_txIdx   = 0U;
static          uint16_t     s_txBuf[2];   /* [0]=ACK byte  [1]=STATUS byte  */

/*===========================================================================
 * Public: health status updated by application each task cycle
 *===========================================================================*/
volatile uint16_t g_healthStatus = HEALTH_STATUS_OK;

/*===========================================================================
 * Slave_I2C_UpdateHealth
 * Call from your main task loop to update the health byte the slave
 * will send to the master on the next read request.
 *===========================================================================*/
void Slave_I2C_UpdateHealth(uint16_t cpuOK,
                            uint16_t periphOK,
                            uint16_t wdgOK)
{
    uint16_t status = 0U;
    if(cpuOK)    status |= STATUS_BIT_CPU;
    if(periphOK) status |= STATUS_BIT_PERIPH;
    if(wdgOK)    status |= STATUS_BIT_WDG;
    g_healthStatus = status;
}

/*===========================================================================
 * Slave_I2C_Init
 * Configure I2CA as a slave (target) device.
 * Uses interrupt-driven operation: no polling anywhere.
 *===========================================================================*/
void Slave_I2C_Init(void)
{
    /*-----------------------------------------------------------------------
     * GPIO — same pins as master, open-drain + pull-up + async qualification
     *-----------------------------------------------------------------------*/
    GPIO_setPinConfig(GPIO_28_I2CA_SDA);
    GPIO_setDirectionMode(28U, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28U, GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_OD);
    GPIO_setQualificationMode(28U, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_29_I2CA_SCL);
    GPIO_setDirectionMode(29U, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(29U, GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_OD);
    GPIO_setQualificationMode(29U, GPIO_QUAL_ASYNC);

    /*-----------------------------------------------------------------------
     * I2C module — slave (target) mode
     *-----------------------------------------------------------------------*/
    I2C_disableModule(I2CA_BASE);

    /* Clock dividers are required even in slave mode on F280015x */
    I2C_initController(I2CA_BASE,
                       DEVICE_SYSCLK_FREQ,
                       100000UL,
                       I2C_DUTYCYCLE_50);

    I2C_setConfig(I2CA_BASE, I2C_TARGET_RECEIVE_MODE);
    I2C_setOwnAddress(I2CA_BASE, MY_SLAVE_ADDR);
    I2C_setAddressMode(I2CA_BASE, I2C_ADDR_MODE_7BITS);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);
    I2C_disableLoopback(I2CA_BASE);
    I2C_disableFIFO(I2CA_BASE);    /* plain DR register mode — simplest      */
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_FREE_RUN);

    /*-----------------------------------------------------------------------
     * Clear any stale flags before enabling interrupts
     *-----------------------------------------------------------------------*/
    I2C_clearStatus(I2CA_BASE,
                    I2C_STS_ARB_LOST       |
                    I2C_STS_NO_ACK         |
                    I2C_STS_REG_ACCESS_RDY |
                    I2C_STS_STOP_CONDITION);

    /*-----------------------------------------------------------------------
     * Enable exactly the interrupts the slave needs:
     *   AAS  — addressed as slave (start of every transaction)
     *   RRDY — byte received from master
     *   XRDY — TX register empty (master reading from us)
     *   SCD  — stop condition detected (transaction ended)
     * NOT enabling NACK — master NACK at end of read is normal.
     *-----------------------------------------------------------------------*/
    I2C_enableInterrupt(I2CA_BASE,
                        I2C_INT_ADDR_TARGET    |
                        I2C_INT_RX_DATA_RDY    |
                        I2C_INT_TX_DATA_RDY    |
                        I2C_INT_STOP_CONDITION);

    /*-----------------------------------------------------------------------
     * Wire ISR into PIE vector table and enable PIE group 8, INT1
     * I2CA is PIE interrupt 8.1 on F280015x
     *-----------------------------------------------------------------------*/
    Interrupt_register(INT_I2CA, &i2cA_ISR);
    Interrupt_enable(INT_I2CA);

    I2C_enableModule(I2CA_BASE);
}

/*===========================================================================
 * i2cA_ISR — Slave interrupt service routine
 *
 * Reads I2CISRC to get the interrupt source code and dispatches to the
 * correct handler.  Acknowledges PIE at the end.
 *
 * I2CISRC codes on C2000 (I2C_getInterruptSource return values):
 *   I2C_INTSRC_NONE       = 0  (spurious — clear and exit)
 *   I2C_INTSRC_ARB_LOST   = 1
 *   I2C_INTSRC_NO_ACK     = 2
 *   I2C_INTSRC_REG_ACCESS = 3  (ARDY)
 *   I2C_INTSRC_RX_DATA    = 4  (RRDY)
 *   I2C_INTSRC_TX_DATA    = 5  (XRDY)
 *   I2C_INTSRC_SCD        = 6  (stop condition)
 *   I2C_INTSRC_ADDR_TARGET = 7 (AAS)
 *===========================================================================*/
__interrupt void i2cA_ISR(void)
{
    uint16_t intSrc = I2C_getInterruptSource(I2CA_BASE);
    uint16_t sts    = I2C_getStatus(I2CA_BASE);

    switch(intSrc)
    {
        /*-------------------------------------------------------------------
         * AAS — Addressed As Slave: start of a new transaction
         *-------------------------------------------------------------------*/
        case I2C_INTSRC_ADDR_TARGET:
        {
            if(sts & I2C_STS_TARGET_DIR)
            {
                /*------------------------------------------------------------
                 * SDIR=1 → master wants to READ from us.
                 * Load first TX byte immediately — hardware will copy it to
                 * the shift register before the first SCL edge.
                 *------------------------------------------------------------*/
                s_txBuf[0] = HEALTH_ACK;
                s_txBuf[1] = (uint16_t)g_healthStatus;
                s_txIdx    = 0U;
                s_state    = SLV_TX;

                I2C_putData(I2CA_BASE, s_txBuf[s_txIdx++]);
            }
            else
            {
                /*------------------------------------------------------------
                 * SDIR=0 → master is WRITING to us.
                 *------------------------------------------------------------*/
                s_state = SLV_RX_CMD;
            }
            break;
        }

        /*-------------------------------------------------------------------
         * RRDY — byte received from master
         *-------------------------------------------------------------------*/
        case I2C_INTSRC_RX_DATA:
        {
            uint16_t rxByte = I2C_getData(I2CA_BASE);   /* MUST read to clear */

            if(s_state == SLV_RX_CMD)
            {
                s_cmd   = rxByte;
                s_state = SLV_RX_SUBCMD;
            }
            else if(s_state == SLV_RX_SUBCMD)
            {
                s_subcmd = rxByte;
                /* Command is fully received — nothing to do here;
                 * the reply is prepared fresh each time master reads */
                s_state  = SLV_IDLE;
            }
            else
            {
                /* Unexpected byte — read and discard */
                (void)rxByte;
            }
            break;
        }

        /*-------------------------------------------------------------------
         * XRDY — TX data register empty; master is clocking out our bytes
         *-------------------------------------------------------------------*/
        case I2C_INTSRC_TX_DATA:
        {
            if(s_state == SLV_TX && s_txIdx < 2U)
            {
                /* Load next byte (due to double-buffering this is byte 1) */
                I2C_putData(I2CA_BASE, s_txBuf[s_txIdx++]);
            }
            else
            {
                /* All bytes sent.  Write 0xFF as a dummy so shift register
                 * is not left empty — master will NACK this anyway.       */
                I2C_putData(I2CA_BASE, 0xFFU);
            }
            break;
        }

        /*-------------------------------------------------------------------
         * SCD — Stop Condition Detected: transaction ended
         *
         * CRITICAL: Do a soft reset (IRS=0 → IRS=1) after every read
         * transaction to flush any pre-loaded byte from the TX shift
         * register.  Without this, the stale byte is sent as byte 0 of
         * the next read transaction (double-buffer artifact).
         *-------------------------------------------------------------------*/
        case I2C_INTSRC_SCD:
        {
            I2C_clearStatus(I2CA_BASE, I2C_STS_STOP_CONDITION);

            if(s_state == SLV_TX)
            {
                /* Flush double-buffer — IRS=0 then IRS=1 */
                I2C_disableModule(I2CA_BASE);
                I2C_enableModule(I2CA_BASE);

                /* Re-enable interrupts (cleared by module reset) */
                I2C_enableInterrupt(I2CA_BASE,
                                    I2C_INT_ADDR_TARGET    |
                                    I2C_INT_RX_DATA_RDY    |
                                    I2C_INT_TX_DATA_RDY    |
                                    I2C_INT_STOP_CONDITION);
            }

            s_state  = SLV_IDLE;
            s_txIdx  = 0U;
            break;
        }

        /*-------------------------------------------------------------------
         * ARB LOST — should not happen in single-master topology.
         * Reset module to recover.
         *-------------------------------------------------------------------*/
        case I2C_INTSRC_ARB_LOST:
        {
            I2C_clearStatus(I2CA_BASE, I2C_STS_ARB_LOST);
            I2C_disableModule(I2CA_BASE);
            I2C_enableModule(I2CA_BASE);
            I2C_enableInterrupt(I2CA_BASE,
                                I2C_INT_ADDR_TARGET    |
                                I2C_INT_RX_DATA_RDY    |
                                I2C_INT_TX_DATA_RDY    |
                                I2C_INT_STOP_CONDITION);
            s_state = SLV_IDLE;
            break;
        }

        default:
        {
            /* Spurious — clear all flags and continue */
            I2C_clearStatus(I2CA_BASE,
                            I2C_STS_ARB_LOST       |
                            I2C_STS_NO_ACK         |
                            I2C_STS_REG_ACCESS_RDY |
                            I2C_STS_STOP_CONDITION);
            break;
        }
    }

    /* Acknowledge PIE group 8 so further interrupts can be received */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}
