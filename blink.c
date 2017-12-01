/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"

/* Standart includes */
#include <stdint.h>
#include <stdlib.h>

/* Texas includes */
#include <msp430.h>
#include "driverlib.h"
#include "queue.h"

const uint8_t i2c_linear_acc_read = 0x33;
const uint8_t i2c_linear_acc_write = 0x32;
const uint8_t i2c_magnetic_sen_read = 0x3d;
const uint8_t i2c_magnetic_sen_write = 0x3c;

#define RDMODE  0
#define WRMODE  1

const uint16_t ACCEL = (0x32 >> 1);  // 0011001b;
const uint16_t MAG = (0x3C >> 1);  // 0011110x


struct I2CMsg
{
    uint8_t mode;      // RDMODE, WRMODE
    uint8_t addr;
    uint8_t reg_addr;
    uint8_t count;
};

void vApplicationSetupTimerInterrupt()
{
    const uint16_t ACLK_Freq = 32768;

    // Stop the timer.
    TA0CTL = 0;

    // Set the compare value
    TA0CCR0 = ACLK_Freq / configTICK_RATE_HZ;

    // Timer interrupt enabled
    TA0CCTL0 = CCIE;

    // Start with up mode and ACLK as clock source
    TA0CTL |= TACLR + MC_1 + TASSEL_1;
}

void vBlinkLED(void * pvParameters)
{
    // this section will execute once
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    QueueHandle_t *taskQueue = (QueueHandle_t*) pvParameters;

//    P1DIR |= 0x01;
    P4DIR |= (1 << 6);
//    volatile int data;
    PM5CTL0 &= ~LOCKLPM5;

    while (1)
    {
//        P1OUT ^= BIT0;
//        P1OUT ^= 0x01;
        P4OUT ^= (1 << 6);
//        data = P1OUT;
        struct I2CMsg accel_setup_reg = {.mode = WRMODE, .addr = ACCEL, .reg_addr = 0, .count = 1};
        xQueueSendToBack(*taskQueue, &accel_setup_reg, portMAX_DELAY);

        vTaskDelay(xDelay);
    }
}

void vHardwareSetup()
{
    // Stop watchdog timer (classic at msp430)
    WDTCTL = WDTPW + WDTHOLD;

    // P1.0 as output
    P1DIR |= BIT0;

    // The tick timer setup
    vApplicationSetupTimerInterrupt();

    // Enable the interrupts
    taskENABLE_INTERRUPTS()
    ;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    /* If the buffers to be provided to the Idle task are declared inside this
     function then they must be declared static - otherwise they will be allocated on
     the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
     state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     Note that, as the array is necessarily of type StackType_t,
     configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void USART0_Init(void)
{   // Controller -> PC
    P2SEL1 |= BIT0 | BIT1;                    // Configure UART pins
    P2SEL0 &= ~(BIT0 | BIT1);

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Startup clock system with max DCO setting ~8MHz
    CSCTL0_H = CSKEY >> 8;                    // Unlock clock registers
    CSCTL1 = DCOFSEL_3 | DCORSEL;             // Set DCO to 8MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers
    CSCTL0_H = 0;                           // Lock CS registers

    // Configure USCI_A1 for UART mode
    UCA0CTLW0 = UCSWRST;                      // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
    UCA0BR0 = 4;     //4;                             // 8000000/16/115200
    UCA0BR1 = 0x00;
    UCA0MCTLW |= 0x55;                         //UCOS16 | UCBRF_1;
    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
    // UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

}
void USART1_Init(void)
{   // Controller <-> PAN1026

    P2SEL1 |= BIT5 | BIT6;                    // Configure UART pins
    P2SEL0 &= ~(BIT5 | BIT6);

    // Configure USCI_A1 for UART mode
    UCA1CTLW0 = UCSWRST;                      // Put eUSCI in reset
    UCA1CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
    UCA1BR0 = 4;                             // 8000000/16/115200
    UCA1BR1 = 0x00;
    UCA1MCTLW |= 0x55; //UCOS16 | UCBRF_1;
    UCA1CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
    // UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}
void USART0_SendByte(unsigned char data)
{
    while (!(UCA0IFG & UCTXIFG))
        taskYIELD();

    UCA0TXBUF = data;
}

void USART1_SendByte(unsigned char data)
{
    while (!(UCA1IFG & UCTXIFG))
        ;
    UCA1TXBUF = data;
}

void USART0_SendData(unsigned char data[], unsigned char length)
{
    unsigned char i;

    for (i = 0; i < length; i++)
    {
        USART0_SendByte(data[i]);
    }
}

void USART1_SendData(unsigned char data[], unsigned char length)
{

    unsigned char i;

    for (i = 0; i < length; i++)
    {
        USART1_SendByte(data[i]);
    }
}

// Only for +ve numbers
int my_itoa(char *buf, int n)
{
    char tmp[8];
    int i = 0;
    while (n > 0)
    {
        tmp[i] = '0' + n % 10;
        n = n / 10;
        i++;
    }

    // Reverse the string
    int j = 0;
    --i;
    for (; i >= 0; --i, ++j)
    {
        buf[j] = tmp[i];
    }

    return j;
}

void vtestUART(void * pvParameters)
{
    const TickType_t ms500 = 500 / portTICK_PERIOD_MS;
    int count = 0;
    char strcount[8];

    while (1)
    {
        int k = my_itoa(strcount, count++);
        USART0_SendData((unsigned char*) strcount, k);
        USART0_SendData(" HELLO World\r\n", 14);
        //    USART1_SendData(TCU_HCI_RESET_REQ,7);
        vTaskDelay(ms500);
    }
}

int main2(void)
{
    // each hardware needs its own setup
    vHardwareSetup();

    USART0_Init();

//Each task should have a handler
    static xTaskHandle BlinkLED_Handler;
    static xTaskHandle Uart_Handler;

//task creation
    xTaskCreate(vBlinkLED, "Blink P1.0", configMINIMAL_STACK_SIZE, NULL, 1,
                &BlinkLED_Handler);
    xTaskCreate(vtestUART, "UART Test", configMINIMAL_STACK_SIZE, NULL, 1,
                &Uart_Handler);

//scheduler initialization
    vTaskStartScheduler();

// should never reach here
    return 0;
}

/*const uint8_t i2c_linear_acc_read = 0x33;
 const uint8_t i2c_linear_acc_write = 0x32;
 const uint8_t i2c_magnetic_sen_read = 0x3d;
 const uint8_t i2c_magnetic_sen_write = 0x3c;
 */
void i2c_init()
{
//    PM5CTL0 &= ~LOCKLPM5;

    UCB0CTL1 |= UCSWRST;                    // put eUSCI_B in reset state
    //UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC;
    UCB0CTLW0 |= UCMODE_3 | UCMST;
    //    UCB0BRW = 0xf00;
    UCB0BRW = 0x000A;                       // baud rate = SMCLK / 10 = 100khz

    //P1DIR &= ~(BIT6 | BIT7);
    //P1SEL0 &= ~(BIT6 | BIT7);
    UCB0I2CSA = 0x33;
    UCB0TBCNT = 1;
    P1SEL1 |= BIT6 | BIT7;

    UCB0CTLW1 |= UCASTP_2;    //?               // Automatic stop generated

    UCB0IE |= UCRXIE | UCNACKIE | UCBCNTIE;  // ?

//    P1OUT |= (BIT6 | BIT7);
//    P1REN |= (BIT6 | BIT7);

    //    UCB0CTL1 &= ~UCSWRST;
    UCB0CTL1 &= ~UCSWRST;                   // eUSCI_B in operational state
    //    UCB0I2CSA = 0;

    __delay_cycles(1600);

    while (UCB0STAT & UCBBUSY)
        ;

    P1SEL1 |= BIT6 | BIT7;                  // configure I2C pins
    P1SEL0 &= ~(BIT6 | BIT7);               // configure I2C pins
    // baud rate = SMCLK / 10 = 100khz
    UCB0CTL1 &= ~UCSWRST;                   // eUSCI_B in operational state
}

void i2c_write(uint8_t slv_addr, uint8_t reg_addr, uint8_t data)
{

    while (UCB0STAT & UCBBUSY)
        ;

    UCB0I2CSA = slv_addr;                   // set slave address
    UCB0CTLW0 |= UCTR | UCTXSTT;        // transmitter mode and START condition.

    while (UCB0CTLW0 & UCTXSTT)
        ;
    UCB0TXBUF = reg_addr;
    while (!(UCB0IFG & UCTXIFG0))
        ;
    UCB0TXBUF = data;
    while (!(UCB0IFG & UCTXIFG0))
        ;

    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP)
        ;             // wait for stop
}

uint8_t i2c_read(uint8_t slv_addr, uint8_t reg_addr)
{
    __disable_interrupt();
    uint8_t data = 0;

    while (UCB0STAT & UCBBUSY)
        ;
    UCB0I2CSA = slv_addr;                   // set slave address
    UCB0CTLW0 |= UCTR | UCTXSTT; // transmitter mode and START condition. UCB0CTLW0

    while (UCB0CTLW0 & UCTXSTT)
        ;
    UCB0TXBUF = reg_addr;
    while (!(UCB0IFG & UCTXIFG0))
        ;

    UCB0CTLW0 &= ~UCTR;                     // receiver mode
    UCB0CTLW0 |= UCTXSTT;                   // START condition

    while (UCB0CTLW0 & UCTXSTT)
        ;             // make sure start has been cleared
    UCB0CTLW0 |= UCTXSTP;                   // STOP condition

    while (!(UCB0IFG & UCRXIFG0))
        ;
    data = UCB0RXBUF;

    while (UCB0CTLW0 & UCTXSTP)
        ;

    return data;
}

void i2c_read_multi(uint8_t slv_addr, uint8_t reg_addr, uint8_t l, uint8_t *arr)
{

    uint8_t i;

    while (UCB0STAT & UCBBUSY)
        ;

    UCB0I2CSA = slv_addr;                   // set slave address

    UCB0CTLW0 |= UCTR | UCTXSTT;        // transmitter mode and START condition.

    while (UCB0CTLW0 & UCTXSTT)
        ;

    UCB0TXBUF = reg_addr;

    while (!(UCB0IFG & UCTXIFG0))
        ; //UCTXIFG0 - eUSCI_B receive interrupt flag
    ;

    UCB0CTLW0 &= ~UCTR;                     // receiver mode

    UCB0CTLW0 |= UCTXSTT;                   // START condition

    while (UCB0CTLW0 & UCTXSTT)
        ;             // make sure start has been cleared

    for (i = 0; i < l; i++)
    {

        while (!(UCB0IFG & UCRXIFG0))
            ;

        if (i == l - 1)
        {

            UCB0CTLW0 |= UCTXSTP;           // STOP condition

        }

        arr[i] = UCB0RXBUF;

    }

    while (UCB0CTLW0 & UCTXSTP)
        ;

}

int mainx()
{

//    __bis_SR_register(LPM0_bits); // Enter LPM3

    //__bic_SR_register(LPM3_bits); // exits LPM3
    // Stop watchdog timer (classic at msp430)
    WDTCTL = WDTPW + WDTHOLD;
    i2c_init();

    for (;;)
    {
        //        data = i2c_read(i2c_linear_acc_read, 0x27);

        i2c_write(i2c_linear_acc_read, 0X27, 0X40);
        //i2c_read_multi(i2c_linear_acc_read, 0x28, 1, data);
        __delay_cycles(2000);
    }
}

// {Num_bytes, reg_addr, data..}
uint8_t buf[16];
uint8_t gCount;
uint8_t gCountDone;
const uint8_t ctrl_reg1 = 0x97;     // 10010111b
int writeI2C(uint8_t addr, int count)
{
    UCB0I2CSA = addr;
    UCB0TBCNT = count;
    gCount = 0;
    gCountDone = count;
    UCB0CTL1 |= UCTR + UCTXSTT;
    return 0;
}

void myInit(void);


void vI2CTask(void *pvParameters)
{
    volatile unsigned int i;            // volatile to prevent optimization
    QueueHandle_t *taskQueue = (QueueHandle_t*) pvParameters;
    struct I2CMsg msg;
    buf[0] = ctrl_reg1;
    writeI2C(ACCEL, 1);
    while (1)
    {
        //       vTaskDelay(500);
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
        xQueueReceive(*taskQueue, &msg, portMAX_DELAY);
        if (msg.mode == WRMODE) {
            writeI2C(msg.addr, msg.count);
        }
        //      vTaskDelay(500);

    }
}

// QueueHandle_t xQueueCreate( UBaseType_t uxQueueLength, UBaseType_t uxItemSize );
//BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer,TickType_t xTicksToWait);
// BaseType_t xQueueSendToBack(QueueHandle_t xQueue,const void * pvItemToQueue,TickType_t xTicksToWait);

static xTaskHandle I2CTask_Handler;
volatile unsigned char RXData;
int main(void)
{
    myInit();

    //Each task should have a handler
    static xTaskHandle BlinkLED_Handler;
//    static xTaskHandle Uart_Handler;

    QueueHandle_t xI2CTaskQueue = xQueueCreate(16, sizeof(struct I2CMsg));

    struct I2CMsg accel_setup_reg = {.mode = WRMODE, .addr = ACCEL, .reg_addr = 0, .count = 1};
    xQueueSendToBack(xI2CTaskQueue, &accel_setup_reg, portMAX_DELAY);

    //task creation
    xTaskCreate(vBlinkLED, "Blink P1.0", configMINIMAL_STACK_SIZE, &xI2CTaskQueue, 1,
                &BlinkLED_Handler);
//    xTaskCreate(vtestUART, "UART Test", configMINIMAL_STACK_SIZE, NULL, 1,
//                &Uart_Handler);

    xTaskCreate(vI2CTask, "I2C Task", configMINIMAL_STACK_SIZE, &xI2CTaskQueue, 1,
                &I2CTask_Handler);

    //scheduler initialization
    vTaskStartScheduler();
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    volatile int i = 0;
    BaseType_t xHigherPriorityTaskWoken;

    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
    case USCI_NONE:
        ++i;
        break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:
        ++i;
        break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
        UCB0CTL1 |= UCTXSTT;                  // I2C start condition
        break;
    case USCI_I2C_UCSTTIFG:
        ++i;
        break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:
        ++i;
        break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:
        ++i;
        break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:
        UCB0TXBUF = 0x77;         // fill TX buffer
        break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:
        ++i;
        break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:
        ++i;
        break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:
        ++i;
        break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:
        ++i;
        break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        RXData = UCB0RXBUF;                   // Get RX data
        __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
        break;
    case USCI_I2C_UCTXIFG0:         // Byte Sent. Send more...
        if (gCountDone > 0)
        {
            UCB0TXBUF = buf[gCount]; // fill TX buffer
            --gCountDone;
            ++gCount;
        }
        else
        {
            UCB0CTL1 |= UCTR + UCTXSTP;
        }
        break;         // Vector 24: TXIFG0
    case USCI_I2C_UCBCNTIFG: // Byte counter zero;                // Vector 26: BCNTIFG
        //      prvClearInterruptSource();
        xHigherPriorityTaskWoken = pdFALSE;
//        vTaskNotifyGiveFromISR(vI2CTask, &xHigherPriorityTaskWoken);
        vTaskNotifyGiveFromISR(I2CTask_Handler, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        P1OUT ^= BIT0;                        // Toggle LED on P1.0
        break;
    case USCI_I2C_UCCLTOIFG:
        ++i;
        break;         // Vector 28: clock low timeout
    case USCI_I2C_UCBIT9IFG:
        ++i;
        break;         // Vector 30: 9th bit
    default:
        break;
    }
}

void myInit(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // Disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings
    P1DIR |= 0x01;

    UCB0CTL1 |= UCSWRST; // put eUSCI_B in reset state
    UCB0CTLW0 |= UCMODE_3 + UCMST; // I2C master mode
    UCB0BRW = 12; // baud rate = SMCLK / 8
    UCB0CTLW1 = UCASTP_2; // automatic STOP assertion
    UCB0TBCNT = 0x00; // TX 7 bytes of data
    UCB0I2CSA = 0x1f; // address slave is 12hex
//    UCB0I2CSA = MAG;
//    P2SEL |= 0x03; // configure I2C pins (device specific)
    P1SEL1 |= BIT6 | BIT7;
    UCB0IE |= UCTXIE; // enable TX-interrupt
    UCB0CTL1 &= ~UCSWRST; // eUSCI_B in operational state

//    UCB0CTLW0 |= UCTR | UCTXSTT;        // transmitter mode and START condition.
    UCB0IE |= UCSTPIE | UCSTTIE | UCNACKIE;             //START STOP
    UCB0IE = 0xFF;

//    __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupt

//    __bis_SR_register(GIE);     // Enter LPM0 w/ interrupt
    __enable_interrupt(); // Enable Global Interrupts by GIE = 1
    //   __bis_SR_register(LPM0_bits);
//    buf[0] = 0x97;
//    writeI2C(ACCEL, 1);

//    UCB0CTL1 |= UCTR + UCTXSTT;

}
