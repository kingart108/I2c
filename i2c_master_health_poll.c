//#############################################################################
//
// FILE:   i2c_master_health_poll.c
//
// TITLE:  I2C Master – Health Poll for 15 Slave TMS320F280015x Controllers
//
// DESCRIPTION:
//   Master sends a 2-byte health request to each of 15 slave controllers
//   in turn, then reads back a 2-byte health response.
//
//   Health request  (master → slave): [0xA5] [0x01]
//   Health response (slave  → master): [0x5A] [STATUS_BYTE]
//
//   STATUS_BYTE:
//     Bit 0 – CPU OK
//     Bit 1 – Peripherals OK
//     Bit 2 – Watchdog OK
//     0x07  = fully healthy
//
//   Slave addresses: 0x10 … 0x1E  (15 slaves, adjust as needed)
//
// CONNECTIONS:
//   SCL → DEVICE_GPIO_PIN_SCLA  (GPIO28 on F280015x launchpad)
//   SDA → DEVICE_GPIO_PIN_SDAA  (GPIO29 on F280015x launchpad)
//   External pull-ups: 4.7 kΩ to 3.3 V on both lines
//   All devices share common GND
//
//#############################################################################

#include "driverlib.h"
#include "device.h"
#include "i2cLib_FIFO_polling.h"

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------
uint16_t TX_MsgBuffer[MAX_BUFFER_SIZE];
uint16_t RX_MsgBuffer[MAX_BUFFER_SIZE];
uint16_t status;

// Health status table – one entry per slave
SlaveHealthStatus g_slaveTable[NUM_SLAVES];

// Bus scan result (optional – useful for debug at startup)
uint16_t g_availableTargets[NUM_SLAVES + 4U];

//-----------------------------------------------------------------------------
// Forward declarations
//-----------------------------------------------------------------------------
void I2C_GPIO_init(void);
void I2Cinit(void);
void printSlaveHealthSummary(void);   // stub – implement via SCI/CAN/etc.

//=============================================================================
// main
//=============================================================================
void main(void)
{
    //-------------------------------------------------------------------------
    // 1. Device / clock / GPIO init
    //-------------------------------------------------------------------------
    Device_init();
    Device_initGPIO();

    I2C_GPIO_init();

    Interrupt_initModule();
    Interrupt_initVectorTable();

    //-------------------------------------------------------------------------
    // 2. I2C peripheral init
    //-------------------------------------------------------------------------
    I2Cinit();

    //-------------------------------------------------------------------------
    // 3. One-time slave table setup
    //-------------------------------------------------------------------------
    I2C_InitSlaveTable(g_slaveTable);

    //-------------------------------------------------------------------------
    // 4. Optional: scan bus at startup to confirm slaves are present
    //    Addresses found are stored in g_availableTargets[]
    //-------------------------------------------------------------------------
    status = I2CBusScan(I2CA_BASE, g_availableTargets);
    if(status != SUCCESS)
    {
        // Bus scan failed – check pull-ups, wiring, slave power
        ESTOP0;
    }

    //-------------------------------------------------------------------------
    // 5. Main polling loop
    //    Polls all 15 slaves continuously.
    //    In a real system replace the bare loop with a timer-based trigger
    //    (e.g. every 10 ms via CPU timer ISR flag).
    //-------------------------------------------------------------------------
    for(;;)
    {
        // Poll every slave: send request + read response, update g_slaveTable
        I2C_PollAllSlaves(I2CA_BASE, g_slaveTable);

        //---------------------------------------------------------------------
        // 6. Act on health results
        //    Example: flag a fault if any slave is unhealthy
        //---------------------------------------------------------------------
        uint16_t s;
        for(s = 0U; s < NUM_SLAVES; s++)
        {
            if(!g_slaveTable[s].healthOK)
            {
                // Slave s is unhealthy or not responding.
                // g_slaveTable[s].commError  – error code
                // g_slaveTable[s].lastStatus – raw status byte
                // g_slaveTable[s].failCount  – total failures so far
                //
                // Insert your fault handling here (CAN message, LED, etc.)
                //
                // Example: set a GPIO fault pin
                // GPIO_writePin(FAULT_GPIO_PIN, 1U);
            }
        }

        // Inter-cycle delay (adjust to your required polling interval)
        DEVICE_DELAY_US(10000U);   // 10 ms between full cycles
    }
}

//=============================================================================
// I2C_GPIO_init
// Configure SDAA / SCLA pins as open-drain I2C with pull-ups enabled.
//=============================================================================
void I2C_GPIO_init(void)
{
    // SDA
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SDAA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SDAA, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SDAA, GPIO_QUAL_ASYNC);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SDAA);

    // SCL
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCLA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCLA, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCLA, GPIO_QUAL_ASYNC);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCLA);
}

//=============================================================================
// I2Cinit
// Initialise I2CA as controller (master), 100 kHz, 7-bit addressing.
// OwnAddress set to 0x60 – arbitrary; master rarely needs one unless
// multi-master is used.
//=============================================================================
void I2Cinit(void)
{
    I2C_disableModule(I2CA_BASE);

    // 100 kHz, 50% duty cycle
    I2C_initController(I2CA_BASE, DEVICE_SYSCLK_FREQ, 100000U,
                       I2C_DUTYCYCLE_50);

    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_setOwnAddress(I2CA_BASE, 0x60U);
    I2C_disableLoopback(I2CA_BASE);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);
    I2C_setDataCount(I2CA_BASE, 2U);
    I2C_setAddressMode(I2CA_BASE, I2C_ADDR_MODE_7BITS);

    I2C_enableFIFO(I2CA_BASE);
    I2C_setFIFOInterruptLevel(I2CA_BASE, I2C_FIFO_TXEMPTY, I2C_FIFO_RX2);

    // Clear any stale flags before enabling
    I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_ARB_LOST | I2C_INT_NO_ACK);

    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_FREE_RUN);
    I2C_enableModule(I2CA_BASE);
}
