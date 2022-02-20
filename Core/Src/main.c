#include <ssd1306_conf.h>
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdio.h"
#include "ssd1306_conf.h"
#include "stdint.h"
#include "thermometer_icon_bitmap.h"
#include "humidity_icon_bitmap.h"

I2C_HandleTypeDef hi2c2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);

void CANbusInit(void);
void GPIODCANbusInit(void);
void CANbusSetupRemoteFrame(void);
void CANbusFilterInit(void);
void CAN1_RX0_IRQHandler(void);
void Tim2Init(void);
void TIM2_IRQHandler(void);
void CAN1_TX_IRQHandler(void);

void Pb0_Config(void);
void Pb7_Config(void);
void Pb14_Config(void);
void Set_Green_Diode(void);
void Set_Blue_Diode(void);
void Set_Red_Diode(void);
void Reset_Green_Diode(void);
void Reset_Blue_Diode(void);
void Reset_Red_Diode(void);
void CAN1_RX0_IRQHandler(void);
void CAN1_SCE_IRQhandler(void);
void CAN1_RX1_IRQHandler(void);

int RxBuffer[8];
char charTemperature[5];
char charHumidity[5];

int idMsg;

int main(void)
{
    uint16_t temperature = 0;
    uint16_t humidity = 0;
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C2_Init();
    ssd1306_Init();

    // Enable clock for green, blue and red diodes
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    Pb0_Config();
    Pb7_Config();
    Pb14_Config();

    GPIODCANbusInit();
    CANbusInit();
    CANbusFilterInit();
    //CANbusSetupRemoteFrame();
    //Tim2Init();
    // Enable FIFO1 message pending interrupt
    charTemperature[2] = '.';
    charHumidity[2] = '.';
    char charsign[1] = {'C'};
    /* Infinite loop */
    while (1)
    {
        temperature = RxBuffer[0] & 0x0000FFFF;
        humidity = (RxBuffer[0] >> 16) & 0x0000FFFF;

        charTemperature[0] = temperature / 100 + '0';
        charTemperature[1] = temperature % 100 / 10 + '0';
        charTemperature[3] = temperature % 10 + '0';

        if((humidity / 1000) == 1){
            charHumidity[0] = 1;
        }
        else{
            charHumidity[0] = humidity / 100 + '0';
        }
        charHumidity[1] = humidity % 100 / 10 + '0';
        charHumidity[3] = humidity % 10 + '0';

        ssd1306_DrawBitmap(0, 0, thermometr_bmp, 16, 16, White);

        ssd1306_SetCursor(20, 6);
        ssd1306_WriteString(charTemperature, Font_7x10, White);

        ssd1306_DrawCircle(49, 6, 1, White);

        ssd1306_SetCursor(51, 6);
        ssd1306_WriteChar('C', Font_7x10, White);

        ssd1306_DrawBitmap(0, 20, humidity_bmp, 16, 16, White);

        ssd1306_SetCursor(20, 26);
        ssd1306_WriteString(charHumidity, Font_7x10, White);

        ssd1306_SetCursor(51, 26);
        ssd1306_WriteChar('%', Font_7x10, White);

        ssd1306_UpdateScreen();
    }

}
// Timer 2 interrupt request handler
void TIM2_IRQHandler(){
    CANbusSetupRemoteFrame();
}
// Timer 2 initialization
void Tim2Init(){
    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;                       // Enable clock for TIM2
    TIM2 -> CR1 |= TIM_CR1_CEN;                                 // Enable Timer 2
    TIM2 -> DIER |= TIM_DIER_UIE;                               // Enable update interrupt
    TIM2 -> PSC = 8000-1;                                       // 8MHz/8000 = 1MHz -> 1ms
    TIM2 -> ARR = 1000-1;                                       // Write 1000 to auto reload register 1ms*1000=1sek
    // Generate interrupt every one second
    __NVIC_EnableIRQ(TIM2_IRQn);								// Enable TIM2 interrupt request
}
// CAN
void CAN1_SCE_IRQhandler(void){
    Set_Green_Diode();
}
// CAN transmit interrupt request handler
void CAN1_TX_IRQHandler(void){

}
// CAN received message interrupt request handler
void CAN1_RX0_IRQHandler(void){
    Set_Blue_Diode();
    idMsg = (CAN1 -> sFIFOMailBox[0].RIR) >> 21;                // Copy ID message to idMsg varaible
    RxBuffer[0] = CAN1 -> sFIFOMailBox[0].RDLR;                 // Copy contents of FIFOMailBox0 low register to RxBuffer
    CAN1 -> RF0R |= CAN_RF0R_RFOM0;                             // Release FIFOMailBox0
}
// CAN filter initialization
void CANbusFilterInit(){
    CAN1 -> FA1R &= ~CAN_FA1R_FACT0;							// Clear filter activation register in order to enter in initialization mode
    CAN1 -> FMR = CAN_FMR_FINIT;                                // Enable filter initialization mode
    while((CAN1 -> FMR & CAN_FMR_FINIT) == 0);				    // Waiting for filter initialization mode
    // Configuration filter 0 in identifier mask mode
    CAN1 -> FM1R &= ~CAN_FM1R_FBM0;							    // Filter bank 0 in identifier mask mode
    CAN1 -> FS1R |= CAN_FS1R_FSC0;							    // Single 32-bit scale configuration
    CAN1 -> FFA1R &= ~CAN_FFA1R_FFA0;							// Filter bank 0 assigned to FIFO 0
    CAN1 -> sFilterRegister[0].FR1 = (0xA << 21);               // Filter 0 ID message
    CAN1 -> sFilterRegister[0].FR2 = 0;						    // Filter 0 mask value
    // Configuration filter 0 in identifier list mode
    //CAN1 -> FM1R |= CAN_FM1R_FBM0;                            // Filter bank 0 in identifier list mode
    //CAN1 -> FS1R |= CAN_FS1R_FSC0;                            // Single 32 bit scale configuration
    //CAN1 -> FFA1R &= ~CAN_FFA1R_FFA0;
    //CAN1 -> sFilterRegister[0].FR1 = 10 << 21;                // Identifier for filter 0
    //CAN1 -> sFilterRegister[0].FR2 = 10 << 21;
    //CAN1 -> FFA1R |= CAN_FFA1R_FFA0;
    // Filter 0 activation
    CAN1 -> FA1R = CAN_FA1R_FACT0;							    // Filter 0 activation
    CAN1 -> FMR &= ~CAN_FMR_FINIT;                              // Software request to enter in normal mode
    while((CAN1 -> FMR & CAN_FMR_FINIT) == 1);				    // Waiting to enter in normal mode
}
// Setup CANbus frame
void CANbusSetupRemoteFrame(){
    if(CAN1 -> TSR & CAN_TSR_TME0){
        // Transmit mailbox 1
        CAN1 -> sTxMailBox[0].TIR = (0xA << 21);                // Identifier (standard format)
        CAN1 -> sTxMailBox[0].TDTR = 4;                         // Data frame, data length code
        CAN1 -> sTxMailBox[0].TDLR = 0xABCDEF10;                // Data low register
        //CAN1 -> sTxMailBox[0].TDHR = 0x0;                     // Data high register
        CAN1 -> sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;             // Software request to transmitting data frame
        //while((CAN1 -> TSR & CAN_TSR_RQCP0) == 0){
        //Set_Blue_Diode();
        //}
        //Reset_Blue_Diode();
    }
}
// Initialization bxCAN1
void CANbusInit(){
    RCC -> APB1ENR |= RCC_APB1ENR_CAN1EN;                       // CAN1 clock enable
    CAN1 -> MCR |= CAN_MCR_INRQ;                                // Software request to enter into initialization mode
    CAN1 -> MCR &= ~CAN_MCR_SLEEP;                              // Software request to quit sleep mode
    while((CAN1 -> MSR & CAN_MSR_INAK) == 0);                   // Waiting to confirm initialization mode by hardware
    if (CAN1 -> MSR & 0x1){                                     // Confirmed initialization mode by hardware
        // Baud rate 500kb/s, Clock frequency 16MHz
        CAN1 -> BTR = 0x001c0001;                               // Prescaler = 2
        // Time segment 1 = 13
        // Time segment 2 = 2
        // Synchronization jump with = 1
        // CAN1 interrupts enable
        CAN1 -> IER = CAN_IER_TMEIE | CAN_IER_FMPIE0			// Transmit message empty, FIFO message pending,
                | CAN_IER_ERRIE;							    // Error interrupt interrupt enable

        // Nested vector interrupt controller
        __NVIC_EnableIRQ(CAN1_SCE_IRQn);
        __NVIC_EnableIRQ(CAN1_RX0_IRQn);
        __NVIC_EnableIRQ(CAN1_TX_IRQn);
        CAN1 -> MCR &= ~CAN_MCR_INRQ;                           // Software request to enter in normal mode
        while(CAN1 -> MSR & 0x1);                               // Waiting to confirm normal mode by hardware
    }
}
// Initialization PD0, PD1 as TX, RX signal of CANbus
void GPIODCANbusInit(){
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIODEN;                      // GPIOD clock enable
    GPIOD -> MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;// Set up mode PD0 as alternate function
    GPIOD -> OSPEEDR |= 0xF;                                    // PD0, PD1 in high speed mode
    GPIOD -> AFR[0] |= GPIO_AF9_CAN1 | (GPIO_AF9_CAN1 << 4);    // Setup appropriate alternate function CAN1 on PD0, PD1
}
// Light up green diode on stm32f722ze board
void Set_Green_Diode() {
    GPIOB->BSRR |= GPIO_BSRR_BS0;
}
// Light up blue diode on stm32f722ze board
void Set_Blue_Diode() {
    GPIOB->BSRR |= GPIO_BSRR_BS7;
}
// Light up red diode on stm32f722ze board
void Set_Red_Diode() {
    GPIOB->BSRR |= GPIO_BSRR_BS14;
}
// Turn off green diode on stm32f722ze board
void Reset_Green_Diode(void) {
    GPIOB -> BSRR = 1<<16;
}
// Turn off blue diode on stm32f722ze board
void Reset_Blue_Diode(void) {
    GPIOB -> BSRR = 1<<23;
}
// Turn off red diode on stm32f722ze board
void Reset_Red_Diode(void) {
    GPIOB -> BSRR = 1<<30;
}

void Pb0_Config(void) {
    GPIOB->MODER |= GPIO_MODER_MODER0_0;
}

void Pb7_Config(void) {
    GPIOB->MODER |= GPIO_MODER_MODER7_0;
}

void Pb14_Config(void) {
    GPIOB->MODER |= GPIO_MODER_MODER14_0;
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x00303D5B;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
