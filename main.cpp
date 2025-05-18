////////////////////////////////////////////////////////////////////////////////
//                  System Identification for DC Motors                       //
//                    A is for left B is for right                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*
  - Modify the Hardware setting about PWM. Setting register that related to PWM output
  directly, instead of using Mbed's API about PWM.
  - It may provide your motor better performance.
*/

#include "mbed.h"
#include <math.h>

#define pi 3.14159f  //(is defined for) changing unit from rpm to rad/s

/////////////////////////////////////////////////////// you need to modify first
#define VELOCITY_CMD        12.0f    // unit:voltage
///////////////////////////////////////////////////////
#define INPUT_VOLTAGE       12.0f 
#define PWM_FREQUENCY       10.0f // (unit : kHz) you may need to change. Recommand don't exceed 100.0 (100 kHz)
#define PWM_STOP            0.5f    //the range of pwm dutycycle value is from 0~1 and  the value that can let motor stop is 0.5.

#define HALL_RESOLUTION     52.0f   //Encoder number per one circle HALL SENSOR 
      
#define FRICTION_VOLTAGE    0.0f 
#define GEAR_RATIO          18.8f

#define CONTROLLER          0       // 0 for transfer function , 1 for control

RawSerial pc(USBTX,USBRX);
InterruptIn mybutton(USER_BUTTON);
Ticker main_function;               //interrupt function
PwmOut pwm1A(D7);
PwmOut pwm1B(D8);
PwmOut pwm2A(D11);
PwmOut pwm2B(A3);
DigitalOut led1(LED1);

float dt = 0.01; // unit:second
float command = 0;
float velocityA = 0;
float velocityB = 0;
bool  button_state = false;
float dutycycle = PWM_STOP;
float you_need_to_do = 0.5;
float degree_command;

void step_command();
void position_control();
void ReadVelocity();
void motor_drive(float voltA, float voltB);
void InitMotor(float pwm_frequency);
void InitEncoder(void);
void error_function();

uint16_t CCR_value(float duty);


int main() {
    pc.baud(115200);            //default baud rate
    InitEncoder();              //don't change it
    
    InitMotor(PWM_FREQUENCY);   // Set pwm period. Don't change the contents in this function unless you know what you do
    
    mybutton.fall(&step_command);   // if you push blue botton, step_command will work

    main_function.attach_us(&position_control, dt*1000000); //dt*1000000 changing unit to second.
    
}


void InitMotor(float pwm_frequency) {
    uint16_t reload = 90000000 / int(pwm_frequency * 1000) - 1;
    uint16_t stop = 90000000 / int(pwm_frequency * 1000) / 2 - 1;

    TIM1->CR1   &= (~0x0001);   // CEN(Counter ENable) = '0'        < TIM control register 1              ;Disable counter at initial  
    TIM1->PSC    = 0x0001;      // Prescaler = ('1' + 1)            < TIM prescaler                       ;Prescaler timer for Timer 1
    TIM1->ARR    = reload;      // reload at 180MHz/PSC/PWM freq -1 < TIM auto-reload register            ;Set auto-reload, the pwm freq is (timer_clk /PSC /ARR)
    TIM1->CCMR1 |= 0x0808;      // set PWM mode, preload            < TIM capture/compare mode register 1 ;Not necessary
    TIM1->CCER  |= 0x0055;      // CC2NE CC2E CC1NE CC1E            < TIM capture/compare enable register ;Enable complementary PWM for channel 1, channel 2
    TIM1->BDTR  |= 0x0C00;      // OSSI OSSR                        < TIM break and dead-time register    ;Set off-state selection
    TIM1->EGR    = 0x0001;      // UG                               < TIM event generation register       ;Update generation
    TIM1->CR1   |= 0x0001;      // CEN(Counter ENable) = '1'        < TIM control register 1              ;Enable counter
/*
    // used for debug
    pc.printf("CR1 : %d\r",uint16_t(TIM1->CR1));
    pc.printf("PSC : %d\r",uint16_t(TIM1->PSC));
    pc.printf("ARR : %d\r",uint16_t(TIM1->ARR));
    pc.printf("CCMR1 : %x\r",TIM1->CCMR1);
    pc.printf("CCER : %x\r",TIM1->CCER);
    pc.printf("BDTR : %x\r",TIM1->BDTR);
    pc.printf("EGR : %x\r",TIM1->EGR);
    pc.printf("stop : %d\r",stop);
*/
    TIM1->CCR1 = stop;
    TIM1->CCR2 = stop;
}


void step_command() {
    led1 = !led1;
    button_state = !button_state;
    /////////////////////////////////////////////////////// maybe you need to do something here
    
    ///////////////////////////////////////////////////////
}


void position_control() {
#if CONTROLLER == 0
    if(button_state == true){
        ReadVelocity();
        degree_command = 90.0;
        float error = degree_command - velocityA;
        if(error > 0.1 || error < -0.1){
            ReadVelocity();
            error_function();
            pc.printf("%.3f, %.3f\r\n",degree_command, velocityA);
            motor_drive(error,0);
        }else{
           dutycycle = PWM_STOP;
           TIM1->CCR1 = CCR_value(dutycycle);
           TIM1->CCR2 = CCR_value(dutycycle);
           pc.printf("%.3f, %.3f\r\n",command, velocityA);
        }
    }else{
        ReadVelocity();
        command = 0;
        
        while(velocityA>command){
            ReadVelocity();
            pc.printf("%.3f, %.3f\r\n",command, velocityA);
            
            motor_drive(command,0);
        }
           dutycycle = PWM_STOP;
        TIM1->CCR1 = CCR_value(dutycycle);
        TIM1->CCR2 = CCR_value(dutycycle);
        pc.printf("%.3f, %.3f\r\n",command, velocityA);
        
    }
#endif

#if CONTROLLER == 1
    // for lab3
#endif
}

void error_function()
{
    float error = degree_command - velocityA;
    if(error >= 12 || error <= -12)
    {
        error = 12.0;
    }else
    {
        error = error;
    }
}


void ReadVelocity() {
    short EncoderPositionA;
    short EncoderPositionB;
    
    EncoderPositionA = TIM2->CNT ;
    EncoderPositionB = TIM3->CNT ;
    
    // if you want count to zero you can do it
//    TIM2->CNT = 0;
//    TIM3->CNT = 0;

    /////////////////////////////////////////////////////// maybe you need to do something here
    
    ///////////////////////////////////////////////////////
    
    velocityA = EncoderPositionA/HALL_RESOLUTION/GEAR_RATIO*360;  
    // please refer to PPT. (then you will find out you may need to add some variables)
    velocityB = EncoderPositionB/you_need_to_do;
    
//    pc.printf("%d\r\n",EncoderPositionA);
}


void motor_drive(float voltA, float voltB){
    /////////////////////////////////////////////////////// maybe you need to put some constraints here
    
    ///////////////////////////////////////////////////////
    
    // Convert volt to pwm
    
    float dutycycleA = (voltA+VELOCITY_CMD)/(2*VELOCITY_CMD);
    float dutycycleB = you_need_to_do;
    TIM1->CCR1 = CCR_value(dutycycleA);  // this command will drive motorA
    TIM1->CCR2 = CCR_value(dutycycleB);  // this command will drive motorB
}


uint16_t CCR_value(float duty) {
    // Convert PWM duty cycle to CCR (capture/compare register) value
    return duty * uint16_t(TIM1->ARR);
}


void InitEncoder(void) {
    // Hardware Quadrature Encoder AB for Nucleo F446RE
    // Output on debug port to host PC @ 9600 baud

    /* Connections
    PA_0 = Encoder1 A
    PA_1 = Encoder1 B
    PB_5 = Encoder2 A
    PB_4 = Encoder2 B
    */
    
    // configure GPIO PA0, PA1, PB5 & PB4 as inputs for Encoder
    RCC->AHB1ENR |= 0x00000003;  // Enable clock for GPIOA & GPIOB
 
    GPIOA->MODER   |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ;           // PA0 & PA1 as Alternate Function  /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 ;           // Pull Down                        /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOA->AFR[0]  |= 0x00000011 ;                                          // AF1 for PA0 & PA1                /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOA->AFR[1]  |= 0x00000000 ;                                          //                                  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
   
 
    GPIOB->MODER   |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 ;           // PB5 & PB4 as Alternate Function  /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOB->PUPDR   |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0 ;           // Pull Down                        /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOB->AFR[0]  |= 0x00220000 ;                                          // AF2 for PB5 & PB4                /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOB->AFR[1]  |= 0x00000000 ;                                          //                                  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
   
    // configure TIM2 & TIM3 as Encoder input
    RCC->APB1ENR |= 0x00000003;  // Enable clock for TIM2 & TIM3

    TIM2->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1  
    TIM2->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM2->CCMR1 = 0xF1F1;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    TIM2->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    TIM2->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    TIM2->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    TIM2->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register
  
    TIM2->CNT = 0x0000;  //reset the counter before we use it
 
    TIM3->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1    
    TIM3->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM3->CCMR1 = 0xF1F1;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    TIM3->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    TIM3->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    TIM3->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    TIM3->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register
  
    TIM3->CNT = 0x0000;  //reset the counter before we use it
}