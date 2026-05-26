/******************************************************************************
 * FILE   : main.c
 * DEVICE : TMS320F2800157
 *
 * Flash to master boards  → define ROLE_MASTER  (Project Properties → Predefined Symbols)
 * Flash to slave boards   → define ROLE_SLAVE   and set MY_SLAVE_ADDR per board
 *
 * MASTER: CPU Timer 0 fires every 100 ms and sets a flag.
 *         Main loop sees the flag and polls all 15 slaves.
 *
 * SLAVE:  Main loop updates g_healthStatus each cycle.
 *         i2cA_ISR handles all I2C traffic automatically.
 ******************************************************************************/

#include "driverlib.h"
#include "device.h"

#if defined(ROLE_MASTER)
  #include "i2c_master.h"
  static SlaveRecord_t g_slaves[NUM_SLAVES];
  static volatile uint16_t g_pollFlag = 0U;

#elif defined(ROLE_SLAVE)
  #include "i2c_slave.h"

#else
  #error "Define ROLE_MASTER or ROLE_SLAVE in project symbols"
#endif

/*===========================================================================
 * CPU Timer 0 ISR  (master only — 100 ms tick)
 *===========================================================================*/
#if defined(ROLE_MASTER)
__interrupt void cpuTimer0ISR(void)
{
    g_pollFlag = 1U;
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
#endif

/*===========================================================================
 * main
 *===========================================================================*/
void main(void)
{
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    /*-----------------------------------------------------------------------
     * MASTER
     *-----------------------------------------------------------------------*/
#if defined(ROLE_MASTER)

    Master_I2C_Init();
    Master_I2C_InitSlaveTable(g_slaves);

    /* CPU Timer 0: 100 ms interrupt */
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_setPeriod(CPUTIMER0_BASE,
                       (uint32_t)(DEVICE_SYSCLK_FREQ / 10U) - 1U); /* 100 ms */
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);

    EINT;   /* enable global interrupts */
    ERTM;   /* enable real-time debug    */

    for(;;)
    {
        if(g_pollFlag)
        {
            g_pollFlag = 0U;
            Master_I2C_PollAllSlaves(g_slaves);

            /*----------------------------------------------------------------
             * ACT ON RESULTS HERE
             * g_slaves[i].healthy   — 1=OK, 0=fault
             * g_slaves[i].statusByte— raw status bits
             * g_slaves[i].lastError — I2CM_OK / TIMEOUT / NACK / BAD_RESP
             * g_slaves[i].failCount — cumulative fail count
             *
             * Example: trigger a fault output if any slave is unhealthy
             *----------------------------------------------------------------*/
            uint16_t i;
            for(i = 0U; i < NUM_SLAVES; i++)
            {
                if(!g_slaves[i].healthy)
                {
                    /*
                     * INSERT YOUR FAULT HANDLING:
                     *   GPIO_writePin(FAULT_PIN, 1U);
                     *   CAN_sendMessage(...)
                     *   Log to flash, etc.
                     */
                }
            }
        }
    }

    /*-----------------------------------------------------------------------
     * SLAVE
     *-----------------------------------------------------------------------*/
#elif defined(ROLE_SLAVE)

    Slave_I2C_Init();

    EINT;
    ERTM;

    for(;;)
    {
        /*--------------------------------------------------------------------
         * Update health status once per main-loop cycle.
         * Replace the 1U literals with your actual diagnostic checks:
         *   cpuOK    — watchdog kick OK, no CPU trap
         *   periphOK — ADC/PWM/SPI self-check result
         *   wdgOK    — independent watchdog status register
         *--------------------------------------------------------------------*/
        uint16_t cpuOK    = 1U;   /* replace with real check */
        uint16_t periphOK = 1U;   /* replace with real check */
        uint16_t wdgOK    = 1U;   /* replace with real check */

        Slave_I2C_UpdateHealth(cpuOK, periphOK, wdgOK);

        /* Yield / sleep — adjust to your RTOS tick or bare-metal delay */
        DEVICE_DELAY_US(1000U);
    }

#endif
}
