/*=========================================================*/
/* Victor P. Nelson                                        */
/* Tyler Robins and Jeremy Doll                            */
/* ELEC 3040/3050 - Lab 6, Program 1                       */ 
/* Keypad Interface with Stopwatch                         */
/*=========================================================*/ 

 
#include "STM32L1xx.h"      /* Microcontroller information */ 
 
/* Define global variables */ 
 
uint8_t count_small;
uint8_t count_big;
uint8_t enable;
uint8_t not_set;
 
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
    GPIOA->MODER &= ~(0x0000000C);/* Set PA1 as input mode      */
	
    /* Configure PB7-PB0 as row and column pins       */	
	RCC->AHBENR |= 0x02;    /* Enable GPIOB clock (bit 0) */
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
	
	
   } 

void countDo() {
	
	if (enable) {
	count_small++;
	}
	
	
 	if (count_small == 10) {  //counted up past 10 - reset
		count_small = 0; 
		count_big++;
	}
	
	if (count_big == 10) {
		count_big = 0;
	}

	
	/*Display count*/
	GPIOC->ODR &= 0xFF00;           /*Clear LEDs */
	GPIOC->ODR |= count_small;			/*Store count*/
	GPIOC->ODR |= count_big << 4;
	
	
}  

void TIM10_IRQHandler() {   
	countDo();
	TIM10->SR &= ~TIM_SR_UIF;
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
	
	
	if (key_val == 0) {      //Do if button 0 pressed
		
		if (not_set) {       //Check if timer has been set up
			not_set = 0;     //Set not_set to false
			RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; //Enable the clock for tim10
			TIM10->PSC = 2096;                   //Set the pre-scale to milliseconds
			TIM10->ARR = 99;                     //Set Tout = 0.1 s (100 ms)
			TIM10->DIER |= TIM_DIER_UIE;         //Enable tim10 interrupt
			NVIC_EnableIRQ(TIM10_IRQn);          //Enable NVIC tim10
			TIM10->CR1 |= 0x01;                  //Enable timer
		}
		
		if (enable)  {
			enable = 0;          //Set enable to false
			TIM10->CR1 &= ~0x01; //disable timer
			
		}
		
		else {  
			enable = 1;         //Set enable to true
			TIM10->CR1 |= 0x01; //enable timer
		}
		
	}
	  
	if (key_val == 1) {    //Do if button 1 pressed
		if (enable == 0) { //Do if already paused
			count_small = 0;
			count_big = 0;
			countDo();     //Update LEDs
		}
	}
	
	
    for (i=0; i<100000; i++) {           /*T-bounce delay*/
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
  count_small = 0;      
  count_big = 0;       
  enable = 0;
  not_set = 1;
  __enable_irq(); //Enable interrupts
 
  /* Endless loop */
  while (1) {        
	//wait for keypad
  
  } /* repeat forever */ 
} 
