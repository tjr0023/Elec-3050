/*=========================================================*/
/* Victor P. Nelson                                        */
/* Tyler Robins and Jeremy Doll                            */
/* ELEC 3040/3050 - Lab 5, Program 1                       */ 
/* Keypad Interface                                        */
/*=========================================================*/ 

 
#include "STM32L1xx.h"      /* Microcontroller information */ 
 
/* Define global variables */ 
 
uint8_t count;
uint8_t hold;
 
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
    GPIOA->MODER    &=  ~(0x0000000C);/* Set PA1 as input mode      */
	
    /* Configure PB7-PB0 as row and column pins       */	
	RCC->AHBENR |= 0x02;    /* Enable GPIOB clock (bit 0) */
  GPIOB->MODER &=  ~(0x0000FFFF);   /* Clear PB7-PB0 mode bits*/
	GPIOB->MODER |=  (0x00005500);    /* Set PB7-4 as output bits*/
   
    /* Configure PC3-0 as output pins to drive LEDs */
    RCC->AHBENR |=  0x04;            /* Enable GPIOC clock (bit 2) */
    GPIOC->MODER &= ~(0x000000FF);   /* Clear PC3-PC0 mode bits    */
    GPIOC->MODER |=  (0x00000055);   /* General purpose output mode*/

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
	
	
 	if (row_val == 0b1110) {
		key_val = 0;
	}
	
	
	else if (row_val == 0b1101) {
		key_val = 4;
	}
	
	else if (row_val == 0b1011) {
		key_val = 8;
	}
	
	else {
		key_val = 12;
	}
	
	 if (col_val == 0b0001) {
		key_val += 1;
	}
	
	
	else if (col_val == 0b0010) {
		key_val += 2;
	}
	
	else if (col_val == 0b0100) {
		key_val += 3;
	}
	
	else {
		key_val += 4;
	}
	
	GPIOB->ODR &= 0x0000;
	
	
	GPIOC->ODR &= 0xFFF0;
	GPIOC->ODR |= key_val;
	
	hold = 5;
	
    for (i=0; i<100000; i++) {           /*T-bounce delay*/
		n=i;
	}
	EXTI->PR |= 0x0002;
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
}
 
/*----------------------------------------------------------*/ 
/* Delay function - do nothing for about 1 seconds */ 
/*----------------------------------------------------------*/ 
void delay () {
    int i,j,n;
    for (i=0; i<20; i++) {         //outer loop
		for (j=0; j<20000; j++) {    //inner loop
		n = j;    //dummy operation for single-step test
		}                             //do nothing
	} 
} 

void countDo() {
	
	count++;
	
 	if (count == 10)   //counted up past 16 - reset
			count = 0; 
			
	if(hold) {                      /*Executes when key is pressed*/
		hold--;
	}

	else {							/*Display count if no key pressed*/
	GPIOC->ODR &= 0xFFF0;           /*Clear LEDs */
	GPIOC->ODR |= count;			/*Store count*/
	}
}  

 
  
/*------------------------------------------------*/
/* Main program                                        */
 /*------------------------------------------------*/
 int main(void) {
  uint8_t alt_count;
 
  PinSetup();     //Configure GPIO pins
  count = 0;      //Initial value of count
  hold = 0;       //Initial value of hold
  
  __enable_irq(); //Enable interrupts
 
  /* Endless loop */
  while (1) {        

  delay();
  
  countDo();
   

  } /* repeat forever */ 
  }   
  
  
  
  
  
  
  
  
  
