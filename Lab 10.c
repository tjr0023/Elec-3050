/*=========================================================*/
/* Victor P. Nelson                                        */
/* Tyler Robins and Jeremy Doll                            */
/* ELEC 3040/3050 - Lab 6, Program 1                       */ 
/* Keypad Interface with Stopwatch                         */
/*=========================================================*/ 

 
#include "STM32L1xx.h"      /* Microcontroller information */ 
 
/* Define global variables */ 
 
 
uint16_t duty_cycle[11] = {0, 210, 419, 629, 839, 1049, 
1258, 1468, 1678, 1887, 2097};

float signal_period;
float motor_v;
float v_acc;
float v_avg;
float v_counter;
 
/*---------------------------------------------------*/ 
/* Initialize GPIO pins used in the program */ 
/*       PA1 = interrupt                    */ 
/*       PB3-PB0 = Row buttons              */ 
/*       PB7-PB4 = Column buttons           */ 
/*       PC3-PC0 = Count LEDs               */ 
/*---------------------------------------------------*/ 
void  PinSetup () {     

    /* Configure PA1 as input pin to read push button */
    RCC->AHBENR |= 0x01;              /* Enable GPIOA clock (bit 0) */
    GPIOA->MODER &= ~(0x0000F0FF);    /* Set PA1 as input mode & Clear PA6 */
	GPIOA->MODER |= 0x0000A0C0;       /* Set PA6 to AF mode, PA3 to analog, PA1 to input */
	GPIOA->AFR[0] &= ~(0xFF000000);    /* Clear AFRL6 */
	GPIOA->AFR[0] |= 0x33000000;       /* PA6 = AF3 */
	GPIOA->PUPDR &= ~(0x0000F000);    //clear pull up/down status on PA7
	GPIOA->PUPDR |= 0x00004000;       //set pull up resistor for PA7
	
    /* Configure PB7-PB0 as row and column pins       */	
	RCC->AHBENR |= 0x02;              /* Enable GPIOB clock (bit 0) */
    GPIOB->MODER &=  ~(0x0000FFFF);   /* Clear PB7-PB0 mode bits*/
	GPIOB->MODER |=  (0x00005500);    /* Set PB7-4 as output bits*/
   
    /* Configure PC3-0 as output pins to drive LEDs */
    RCC->AHBENR |=  0x04;            /* Enable GPIOC clock (bit 2) */
    GPIOC->MODER &= ~(0x0000FFFF);   /* Clear PC7-PC0 mode bits    */
    GPIOC->MODER |=  (0x00005555);   /* General purpose output mode*/

    SYSCFG->EXTICR[0] &= 0xFF0F;     /* Clear EXTI1 fields*/ 
    SYSCFG->EXTICR[0] |= 0x0010;	 /* Set EXTI1 to PA1  */
    EXTI->FTSR |= 0x0002;			 /* Set EXTI1 to falling-edge triggered*/
	EXTI->RTSR &= 0x0000;			 /* Set EXTI1 to falling-edge triggered*/
    EXTI->IMR |= 0x0002;			 /* Enable EXTI1      */
    EXTI->PR |= 0x0002;				 /* Clear EXTI1 pending status*/
	
	NVIC_EnableIRQ(EXTI1_IRQn);
	
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	
	GPIOB->PUPDR &= ~(0x0000FFFF);  /*Clear resitor bits for PB3-PB0*/
	GPIOB->PUPDR |= (0x0000AA55);   /*Pull-up resistors for PB3-PB0*/
	
	GPIOB->ODR &= 0x0000FF00;           /*Set col pins to 0 (PB7-PB4)*/
	
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; //Enable the clock for tim10
	TIM10->PSC = 0;                   //Set the pre-scale to milliseconds
	TIM10->ARR = 2096;                   //Set for 1 KHz PWM
	TIM10->CCMR1 = 0x60;                 //Set CCMR1 bits
	TIM10->CCER = 0x0001;                //Set CCER bits
	TIM10->CCR1 = 0;                     //Set duty cycle to 0%
	TIM10->DIER |= TIM_DIER_UIE;         //Enable tim10 interrupt
	NVIC_EnableIRQ(TIM10_IRQn);          //Enable NVIC tim10
	TIM10->CR1 |= 0x01;                  //Enable timer

					
	//RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; //Enable the clock for tim11
	//TIM11->PSC = 31;                   //Set the pre-scale to milliseconds
	//TIM11->ARR = 65535;                   //Set for 1 KHz PWM
	//TIM11->CCMR1 |= 0x01;                 //Set CCMR1 bits
	//TIM11->CCER |= 0x0003;                //Set CCER bits
	//TIM11->CR1 |= 0x0001;                     //Set duty cycle to 0%
	//TIM11->DIER |= 0x0003;         //Enable tim10 interrupt
	//NVIC_EnableIRQ(TIM11_IRQn);          //Enable NVIC tim10
	
	//Set up the ADC converter
	RCC->CR |= RCC_CR_HSION;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //enable ADC
	ADC1->CR2 |= 1;						//turn on ADC
	//ADC1->CR1 |= 0x03000000;			//change resolution to 6 bits
	ADC1->SMPR1 |= 0;					//set to 4 cycles
	ADC1->SQR5 &= ~ADC_SQR5_SQ1;		//clear SQ1 bits
	ADC1->SQR5 |= 0X00000002;           //choose channel 3
	while((ADC1->SR & 0x040) == 0) {};  //wait for ADC to power on
	
	
   } 

void TIM10_IRQHandler() {   
	TIM10->SR &= ~TIM_SR_UIF;
}

void TIM11_IRQHandler() {   
	signal_period = (double)((TIM11->CCR1)*31)/(2097000);
	TIM11->CNT=0;
	TIM11->SR &= ~TIM_SR_UIF;
}

void convert() {
	ADC1->CR2 |= ADC_CR2_SWSTART; //starts conversion
	while ((ADC1->SR & 0x0040) == 0){}; //wait 
	motor_v = ADC1->DR;
	motor_v = motor_v * ((float) 3 / 4096);
	//ADC1->CR2 = 0; //turn off ADC
	v_acc += motor_v;
	v_counter++;
	if (v_counter == 1000) {
		v_avg = v_acc/v_counter;
		v_acc = 0;
		v_counter = 0;
	}
}
   
void EXTI1_IRQHandler() {
	
	
	uint8_t row_val;
	uint8_t col_val;
	uint8_t key_val;
	uint8_t col_set;
	uint8_t temp_row;
	
	int i, n;
	for (i=0; i<20000; i++) {         /*T-bounce delay*/
		n = i;
	}
	
	row_val = GPIOB->IDR;/*Read and set row value*/
	row_val &= 0x000F;
	
    for (col_val=1; col_val<9; col_val=col_val*2) {
		col_set = ~col_val;
		col_set &= 0x0F;
		GPIOB->ODR &= 0xFF0F;
		GPIOB->ODR |= col_set << 4;
		
		int k;                     
		for (k=0; k<4; k++) {		/*Short propagation delay*/
			n=k;
		}
		
		temp_row = GPIOB->IDR & 0x000F;
		
		/*Check if column value sets appropriate row value*/
		if (row_val == temp_row) { 
			break;                 /*Break if column value is correct*/
		}
		
	}	
	
	//Testing easier method
            /*Switch row_val from one-cold to one-hot*/
	row_val &= 0x0F;
	
	switch (row_val) 
	{
		case 0b1110: //if row1
			switch (col_val)
			{
				case 0b0001:  //if col1
					key_val = 0x1;
					break;   
				case 0b0010:  //if col2
					key_val = 0x2;
					break;
				case 0b0100:  //if col3
					key_val = 0x3;
					break;
				case 0b1000:  //if col4
					key_val = 0xA;
					break;
			}
			break;
		case 0b1101: //if row2
			switch (col_val)
			{
				case 0b0001:
					key_val = 0x4;
					break;
				case 0b0010:
					key_val = 0x5;
					break;
				case 0b0100:
					key_val = 0x6;
					break;
				case 0b1000:
					key_val = 0xB;
					break;
			}
			break;
		case 0b1011: //if row3
			switch (col_val)
			{
				case 0b0001:
					key_val = 0x7;
					break;
				case 0b0010:
					key_val = 0x8;
					break;
				case 0b0100:
					key_val = 0x9;
					break;
				case 0b1000:
					key_val = 0xC;
					break;
			}
			break;
		case 0b0111: //if row4
			switch (col_val)
			{
				case 0b0001:
					key_val = 0xE;
					break;
				case 0b0010:
					key_val = 0x0;
					break;
				case 0b0100:
					key_val = 0xF;
					break;
				case 0b1000:
					key_val = 0xD;
					break;
			}
			break;
	}
	
	
	
	GPIOB->ODR &= 0x0000;
	
	
	if (key_val < 0x0B) {      
		
		TIM10->CCR1 = duty_cycle[key_val];
		
	}
	
	
    for (i=0; i<50000; i++) {           /*T-bounce delay*/
		n=i;
	}
	EXTI->PR |= 0x0002;
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
}
  
  
/*------------------------------------------------*/
/* Main program                                        */
 /*------------------------------------------------*/
 int main(void) {

  PinSetup();     //Configure GPIO pins

  __enable_irq(); //Enable interrupts
 
  /* Endless loop */
  while (1) {        
	//wait for keypad
	convert();
  } /* repeat forever */ 
}
