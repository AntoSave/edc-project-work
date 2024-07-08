/* 
    GPL - 
    Gianluca Canzolino      -       Master Student of Computer Engineering - UNISA
    Giuseppe Gambardella    -       Master Student of Computer Engineering - UNISA 
    Alberto Provenza        -       Master Student of Computer Engineering - UNISA
/*********************** Header Include Start *************************/
#include "soc_stm_encoder_arma.h"
#include "mw_mbed_interface.h"

#ifdef TARGET_NUCLEO_F401RE
#include "stm32f401xe.h"
#include "stm32f4xx_hal_tim.h"
#else
#error "Encode supported with STM32 Nucleo-F401RE"
#endif

/*********************** Header Include End ***************************/
/*********************** Encoder defines Start ************************/
//Timers for ONLY F401RE                            
//ENCODER1 on TIM1
#define ENCODER1_TIMER                        (TIM1)                                              // Encoder timer
#define ENCODER1_TIMER_CLK_EN                 0x00000001                                          // Enable Encoder Clock 
#define ENCODER1_TIMER_CLK_REG                APB2ENR                                             // Encoder timer peripheral clock register
#define ENCODER1_GPIO_PORT                    (GPIOA)                                             // GPIO Port for Encoder Channels
#define ENCODER1_GPIO_PORT_CLK_EN             0x00000001                                          // Enable clock to GPIO A port
#define ENCODER1_GPIO_PORT_CLK_REG            AHB1ENR                                             // GPIO A peripheral clock register
#define ENCODER1_GPIO_PIN_CH1                 8                                                   // Encoder timer channel 1 (PA8)
#define ENCODER1_GPIO_PIN_CH2                 9                                                   // Encoder timer channel 1 (PA9)
#define ENCODER1_GPIO_PIN_CH_MODER            (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1)         // GPIO configurations to map PA8 and PA9 to use for Encoder timer channels
#define ENCODER1_GPIO_PIN_CH_OTYPER           (GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9)
#define ENCODER1_GPIO_PIN_CH_OSPEEDR          (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9)   // Low speed
#define ENCODER1_GPIO_PIN_CH_PUPDR            (GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0)
#define ENCODER1_GPIO_PIN_CH_AFR0             (0x00000000)
#define ENCODER1_GPIO_PIN_CH_AFR1             (0x00000011)
//ENCODER4 on TIM4
#define ENCODER4_TIMER                        (TIM4)                                              // Encoder timer
#define ENCODER4_TIMER_CLK_EN                 0x00000004                                          // Enable Encoder Clock 
#define ENCODER4_TIMER_CLK_REG                APB1ENR                                             // Encoder timer peripheral clock register
#define ENCODER4_GPIO_PORT                    (GPIOB)                                             // GPIO Port for Encoder Channels
#define ENCODER4_GPIO_PORT_CLK_EN             0x00000002                                          // Enable clock to GPIO B port
#define ENCODER4_GPIO_PORT_CLK_REG            AHB1ENR                                             // GPIO B peripheral clock register
#define ENCODER4_GPIO_PIN_CH1                 6                                                   // Encoder timer channel 1 (PB6)
#define ENCODER4_GPIO_PIN_CH2                 7                                                   // Encoder timer channel 1 (PB7)
#define ENCODER4_GPIO_PIN_CH_MODER            (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1)         // GPIO configurations to map PB6 and PB7 to use for Encoder timer channels
#define ENCODER4_GPIO_PIN_CH_OTYPER           (GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7)
#define ENCODER4_GPIO_PIN_CH_OSPEEDR          (GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7)   // Low speed
#define ENCODER4_GPIO_PIN_CH_PUPDR            (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0)
#define ENCODER4_GPIO_PIN_CH_AFR0             (0x2 << (6 * 4)) | (0x2 << (7 * 4))
#define ENCODER4_GPIO_PIN_CH_AFR1             (0x00000000)//(0x00000011)
//ENCODER CONSTANT
#define GEAR_RATIO                            (8400)
#define FORWARD                               (1)
#define BACKWARD                              (17) 
// TIM internal clock on TIM9
#define CLOCK_TIMER                           (TIM9)                                              // Internal timer
#define TIMCLOCK                              (84000000)
#define PRESCALER                             (84)
#define REFCLOCK                              (TIMCLOCK/PRESCALER)   
#define DELTA_THRESHOLD                       (1)

// ARMA FILTER
#define PI                                    (3.14159265358979)
#define CUT_FREQUENCY                         (50)
#define CUT_PULSE                             (2*PI*CUT_FREQUENCY)

/*********************** Encoder defines End **************************/
/*********************** Static Variable Start ************************/
static volatile unsigned short int start_1 = 1;
static volatile unsigned short int start_4 = 1;
static volatile unsigned long int last_tick_speed_enc1 = 0;
static volatile unsigned long int last_tick_speed_enc4 = 0;

typedef struct{
	uint32_t ticks[2];
	uint8_t index;
	uint32_t delta;
	double frequency;
	double speed;
} EncoderDataStruct;

static volatile EncoderDataStruct encoder1, encoder4;
/*********************** Static Variable End ***************************/

/*********************** Global Variable Start ************************/
/*********************** Global Variable End ***************************/

/*********************** ISR function Start ****************************/
/*********************** ISR TIM1 Encoder Start ****************************/
void TIM_Callback_TIM1_CC1(){
    encoder1.ticks[encoder1.index] = CLOCK_TIMER->CNT;
    
    // Evaluate interval
    uint32_t ticks1 = encoder1.ticks[encoder1.index];
    uint32_t ticks2 = encoder1.ticks[(encoder1.index+1)%2];
    if (ticks1 > ticks2)
    {
        encoder1.delta = ticks1-ticks2;
    }
    else if (ticks2 > ticks1)
    {
        encoder1.delta = ((CLOCK_TIMER->ARR+1) - ticks2) + ticks1;
    }

    //The minimum frequency that the Timer can read is equal to (TIMx CLOCK/ARR).
    //In our case it will be (1MHz/0xffff)Hz -> 15.2585Hz
    encoder1.frequency = REFCLOCK/(double)encoder1.delta;
    double speed = ((60*4*encoder1.frequency)/(GEAR_RATIO));

    // Check for direction
    if(ENCODER1_TIMER->CR1 == BACKWARD){
        speed *= -1;
    }
    
    // Apply the ARMA filter
    double alpha = exp(-CUT_PULSE/encoder1.frequency);
    encoder1.speed = alpha*encoder1.speed + (1-alpha)*speed;

    // Next cell
    encoder1.index = (encoder1.index+1)%2;
}

void TIM1_CC_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag
        TIM1->SR &= ~TIM_SR_UIF;
        // Handle timer overflow/underflow interrupt
    }
    if (TIM1->SR & TIM_SR_CC1IF) {
        // Clear the capture/compare interrupt flag for channel 1
        TIM1->SR &= ~TIM_SR_CC1IF;
        // Handle capture/compare channel 1 interrupt
        TIM_Callback_TIM1_CC1();
    }
    
    if (TIM1->SR & TIM_SR_CC2IF) {
        // Clear the capture/compare interrupt flag for channel 2
        TIM1->SR &= ~TIM_SR_CC2IF;
        // Handle capture/compare channel 2 interrupt
    }
}
/*********************** ISR TIM1 Encoder End ****************************/

/*********************** ISR TIM4 Encoder Start ****************************/
void TIM_Callback_TIM4_CC1(){
    encoder4.ticks[encoder4.index] = CLOCK_TIMER->CNT;
    
    // Evaluate interval
    uint32_t ticks1 = encoder4.ticks[encoder4.index];
    uint32_t ticks2 = encoder4.ticks[(encoder4.index+1)%2];
    if (ticks1 > ticks2)
    {
        encoder4.delta = ticks1-ticks2;
    }
    else if (ticks2 > ticks1)
    {
        encoder4.delta = ((CLOCK_TIMER->ARR+1) - ticks2) + ticks1;
    }

    //The minimum frequency that the Timer can read is equal to (TIMx CLOCK/ARR).
    //In our case it will be (1MHz/0xffff)Hz -> 15.2585Hz
    encoder4.frequency = REFCLOCK/(double)encoder4.delta;
    double speed = ((60*4*encoder4.frequency)/(GEAR_RATIO));

    // Check for direction
    if(ENCODER4_TIMER->CR1 == BACKWARD){
        speed *= -1;
    }

    // Apply the ARMA filter
    double alpha = exp(-CUT_PULSE/encoder4.frequency);
    encoder4.speed = alpha*encoder4.speed + (1-alpha)*speed;

    // Next cell
    encoder4.index = (encoder4.index+1)%2;
}

void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag
        TIM4->SR &= ~TIM_SR_UIF;
        // Handle timer overflow/underflow interrupt
    }
    if (TIM4->SR & TIM_SR_CC1IF) {
        // Clear the capture/compare interrupt flag for channel 1
        TIM4->SR &= ~TIM_SR_CC1IF;
        // Handle capture/compare channel 1 interrupt
        TIM_Callback_TIM4_CC1();
    }
    
    if (TIM4->SR & TIM_SR_CC2IF) {
        // Clear the capture/compare interrupt flag for channel 2
        TIM4->SR &= ~TIM_SR_CC2IF;
        // Handle capture/compare channel 2 interrupt
    }
}
/*********************** ISR TIM1 Encoder End ****************************/

/*********************** ISR TIM9 Interal Clock Start ****************************/
void TIM9_PeriodElapsedCallback(void){
    // Encoder 4
    uint32_t actual_tick = encoder1.ticks[encoder1.index];

    int32_t delta = (int32_t)actual_tick - (int32_t)last_tick_speed_enc1;

    if(delta < 0){
        delta += (CLOCK_TIMER->ARR+1);
    }

    if(delta < DELTA_THRESHOLD && delta >= 0){
        encoder1.frequency = 0;
        encoder1.speed = 0;
    }

    last_tick_speed_enc1 = actual_tick;

    // Encoder 4
    actual_tick = encoder4.ticks[encoder4.index];

    delta = (int32_t)actual_tick - (int32_t)last_tick_speed_enc4;

    if(delta < 0){
        delta += (CLOCK_TIMER->ARR+1);
    }

    if(delta < DELTA_THRESHOLD && delta >= 0){
        encoder4.frequency = 0;
        encoder4.speed = 0;
    }

    last_tick_speed_enc4 = actual_tick;
}

void TIM9_IRQHandler(void)
{
    if (TIM9->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag
        TIM9->SR &= ~TIM_SR_UIF;
        
        // Handle timer overflow/underflow interrupt
        TIM9_PeriodElapsedCallback(); 
    }
}
/*********************** ISR TIM9 Interal Clock End ****************************/
/*********************** ISR function End ******************************/

/************************ Init Function Start **************************/
void EncoderDataStructs_Init(){
	encoder1.ticks[0] = 0;
	encoder1.ticks[1] = 0;
	encoder1.index = 0;
    encoder1.speed = 0;

    encoder4.ticks[0] = 0;
	encoder4.ticks[1] = 0;
	encoder4.index = 0;
    encoder4.speed = 0;
}

void configTIM4(){
    // configure GPIO PB6 & PB7 as inputs for Encoder4
    RCC->ENCODER4_GPIO_PORT_CLK_REG |= ENCODER4_GPIO_PORT_CLK_EN;                            // Enable clock for GPIO
    ENCODER4_GPIO_PORT->MODER       |= ENCODER4_GPIO_PIN_CH_MODER;                           // Alternate Function
    ENCODER4_GPIO_PORT->OTYPER      |= ENCODER4_GPIO_PIN_CH_OTYPER;                          // 
    ENCODER4_GPIO_PORT->OSPEEDR     |= ENCODER4_GPIO_PIN_CH_OSPEEDR;                         // 
    ENCODER4_GPIO_PORT->PUPDR       |= ENCODER4_GPIO_PIN_CH_PUPDR;                           // 
    ENCODER4_GPIO_PORT->AFR[0]      |= ENCODER4_GPIO_PIN_CH_AFR0;                            // 
    ENCODER4_GPIO_PORT->AFR[1]      |= ENCODER4_GPIO_PIN_CH_AFR1;                            //

    // configure ENCODER4_TIMER as Encoder4 input
    RCC->ENCODER4_TIMER_CLK_REG |= ENCODER4_TIMER_CLK_EN;                                    // Enable clock for ENCODER4_TIMER
    ENCODER4_TIMER->SMCR  = 0x0003;                                                          // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    ENCODER4_TIMER->CCMR1 = 0x0101;                                                          // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    ENCODER4_TIMER->CCMR2 = 0x0000;                                                          //                             < TIM capture/compare mode register 2
    ENCODER4_TIMER->CCER  = 0x0011;                                                          // CC1P CC2P                   < TIM capture/compare enable register
    ENCODER4_TIMER->PSC   = 0x0000;                                                          // Prescaler = (0+1)           < TIM prescaler
    ENCODER4_TIMER->ARR   = 0xffff;                                                          // reload at 0xffff            < TIM auto-reload register

    ENCODER4_TIMER->CNT = 0x0000;                                                            // reset the counter before we use it
    ENCODER4_TIMER->CR1 = 0x0001; 
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // Enable capture/compare channels 1 and 2
    TIM4->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE; // Enable capture/compare 1 and 2 interrupts
    NVIC_SetPriority(TIM4_IRQn, 0);  // Set interrupt priority (adjust as needed)
    NVIC_SetVector(TIM4_IRQn, (uint32_t)TIM4_IRQHandler); // Attach the TIM_CC_IRQHandler
    NVIC_EnableIRQ(TIM4_IRQn); // Enable TIM4 interrupt in NVIC
}

void configTIM1(){
    // configure GPIO PA8 & PA9 as inputs for Encoder1
    RCC->ENCODER1_GPIO_PORT_CLK_REG |= ENCODER1_GPIO_PORT_CLK_EN;                            // Enable clock for GPIO
    ENCODER1_GPIO_PORT->MODER       |= ENCODER1_GPIO_PIN_CH_MODER;                           // Alternate Function
    ENCODER1_GPIO_PORT->OTYPER      |= ENCODER1_GPIO_PIN_CH_OTYPER;                          // 
    ENCODER1_GPIO_PORT->OSPEEDR     |= ENCODER1_GPIO_PIN_CH_OSPEEDR;                         // 
    ENCODER1_GPIO_PORT->PUPDR       |= ENCODER1_GPIO_PIN_CH_PUPDR;                           // 
    ENCODER1_GPIO_PORT->AFR[0]      |= ENCODER1_GPIO_PIN_CH_AFR0;                            // 
    ENCODER1_GPIO_PORT->AFR[1]      |= ENCODER1_GPIO_PIN_CH_AFR1;                            //

    // configure ENCODER1_TIMER as Encoder1 input
    RCC->ENCODER1_TIMER_CLK_REG |= ENCODER1_TIMER_CLK_EN;                                    // Enable clock for ENCODER1_TIMER
    ENCODER1_TIMER->SMCR  = 0x0003;                                                          // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    ENCODER1_TIMER->CCMR1 = 0x0101;                                                          // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    ENCODER1_TIMER->CCMR2 = 0x0000;                                                          //                             < TIM capture/compare mode register 2
    ENCODER1_TIMER->CCER  = 0x0011;                                                          // CC1P CC2P                   < TIM capture/compare enable register
    ENCODER1_TIMER->PSC   = 0x0000;                                                          // Prescaler = (0+1)           < TIM prescaler
    ENCODER1_TIMER->ARR   = 0xffff;                                                          // reload at 0xffff            < TIM auto-reload register

    ENCODER1_TIMER->CNT = 0x0000;                                                            // reset the counter before we use it
    ENCODER1_TIMER->CR1 = 0x0001;               
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // Enable capture/compare channels 1 and 2
    TIM1->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE; // Enable capture/compare 1 and 2 interrupts
    NVIC_SetPriority(TIM1_CC_IRQn, 0);  // Set interrupt priority (adjust as needed)
    NVIC_SetVector(TIM1_CC_IRQn, (uint32_t)TIM1_CC_IRQHandler); // Attach the TIM_CC_IRQHandler
    NVIC_EnableIRQ(TIM1_CC_IRQn); // Enable TIM1 interrupt in NVIC
}

void initEncoder(unsigned short selection)
{
    #ifndef ENCODER1
        #ifndef ENCODER4
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;                                                         // Enable TIM2 clock
        TIM9->PSC = 84 - 1;                                                                         // Set prescaler value (no prescaling)
        TIM9->ARR = 65000 - 1;                                                                      // Set auto-reload value to achieve desired frequency (84kHz)
        TIM9->CR1 |= TIM_CR1_URS;                                                                   // Only overflow/underflow generates an update interrupt
        TIM9->DIER |= TIM_DIER_UIE;                                                                 // Enable update interrupt
        NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0);                                                    // Set interrupt priority (adjust as needed)
        NVIC_SetVector(TIM1_BRK_TIM9_IRQn, (uint32_t)TIM9_IRQHandler);                              // Attach the TIM2_IRQHandler
        NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);                                                         // Enable the TIM2 global interrupt
        TIM9->CR1 |= TIM_CR1_CEN;   
        #endif
    #endif

    switch (selection)
    {
    case 1:
        #ifndef ENCODER1
            #define ENCODER1
            configTIM1();
        #endif
        break;

    case 4:
        #ifndef ENCODER4
            #define ENCODER4
            configTIM4();
        #endif
        break;
    
    default:
        break;
    }
    EncoderDataStructs_Init();                                                                  // Init EncoderDataStructures
}

/************************ Init Function End **************************/
/************************ Step Function Start ************************/
unsigned short int getEncoderCount(unsigned short selection)
{
    if(selection==1){
        //encoder1 count
        if(start_1==1){
            ENCODER1_TIMER->CNT = 0x0000;
            start_1=0;
        }
        return ((TIM_TypeDef *)ENCODER1_TIMER)->CNT;
    }
    if(selection==4){
        //encoder4 count
        if(start_4==1){
            ENCODER4_TIMER->CNT = 0x0000;
            start_4=0;
        }
        return ((TIM_TypeDef *)ENCODER4_TIMER)->CNT;
    }

    return 0;
}

double getSpeed(unsigned short selection)
{
    if(selection==1)
    {
        //encoder1 speed
        return encoder1.speed;
    }
    if(selection==4)
    {
        //encoder4 speed
        return encoder4.speed;
    }
    
    return 0;
}

/*************************Step Function End************************/
/********************* Release Function Start *********************/
void releaseEncoder(unsigned short selection)
{
    //disable the timers
    if(selection==1)
    {
        ENCODER1_TIMER->CR1 = 0x0000;
    }
    if(selection==4)
    {
        ENCODER4_TIMER->CR1 = 0x0000;
    }
    // Disable TIM9 counter
    TIM9->CR1 &= ~TIM_CR1_CEN;
}
/********************** Release Function End************************/
