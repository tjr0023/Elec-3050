/*=========================================================*/
/* Victor P. Nelson                                        */
/* Tyler Robins and Jeremy Doll                            */
/* ELEC 3040/3050 - Lab 4, Program 1                       */ 
/* Interrupt Example with Counters                         */
/*=========================================================*/ 

 
#include "STM32L1xx.h"      /* Microcontroller information */ 
 
/* Define global variables */ 
 
uint8_t countUp;
uint8_t count;
uint8_t var_count;
 
/*---------------------------------------------------*/ 
/* Initialize GPIO pins used in the program  */ 
/*       PA0 = push button                               */ 
/*       PC8 = blue LED, PC9 = green LED    */ 
/*---------------------------------------------------*/ 
void  PinSetup () {     
    /* Configure PA0 as input pin to read push button */
    RCC->AHBENR |= 0x01;    /* Enable GPIOA clock (bit 0) */
    GPIOA->MODER    &=  ~(0x00000003);       /* General purpose input mode */
   
    /* Configure PC8,PC9 as output pins to drive LEDs */
    RCC->AHBENR |=  0x04;            /* Enable GPIOC clock (bit 2) */
    GPIOC->MODER &= ~(0x000FFFFF);   /* Clear PC9-PC8 mode bits    */
    GPIOC->MODER |=  (0x00055555);   /* General purpose output mode*/

    SYSCFG->EXTICR[0] &= 0xFF00;     /* Clear EXTI0 and EXTI1 fields*/ 
    SYSCFG->EXTICR[0] |= 0x0011;	 /* Set EXTI0 to PA0 and EXTI1 to PA1*/
    EXTI->RSTR |= 0x0003;			 /* Set EXTI0 and EXTI1 to rising-edge triggered*/
    EXTI->IMR |= 0x0003;			 /* Enable EXTI0 and EXTI1*/
    EXTI->PR |= 0x0003;				 /* Clear EXTI0 and EXTI1 pending status*/
	
	NVIC_EnableIRQ(IRQ0);			 /* Enable NVIC EXTI0 and EXTI1*/
	NVIC_EnableIRQ(IRQ1);
	
	NVIC_ClearPendingIRQ(IRQ0);		 /* Clear EXTI0 and EXTI1 NVIC pending status*/
	NVIC_ClearPendingIRQ(IRQ1);
	
   } 
   
void EXTI0_IRQHandler() {
	XTI->PR |= 0x0001;				/* Clear EXTI0 pending status*/
	countUp = 0;
	GPIOC->ODR ^= 0x0100;
	NVIC_ClearPendingIRQ(IRQ0);
}
   
void EXTI1_IRQHandler() {
	XTI->PR |= 0x0002;				/* Clear EXTI1 pending status*/
	countUp = 1;
	GPIOC->ODR ^= 0x0200;
	NVIC_ClearPendingIRQ(IRQ1);
}
 
/*----------------------------------------------------------*/ 
/* Delay function - do nothing for about 0.5 seconds */ 
/*----------------------------------------------------------*/ 
void delay () {
    int i,j,n;
    for (i=0; i<10; i++) {         //outer loop
		for (j=0; j<20000; j++) {    //inner loop
		n = j;    //dummy operation for single-step test
		}                             //do nothing
	} 
} 

void countDo() {
	
	count++
	
	if (count == 10) //counted up past 9 - reset
			count = 0;
	//else if (count > 10) //overflow from subtracting 1
	//	count = 9;
		
	
	GPIOC->ODR &= 0xFFF0;
	GPIOC->ODR |= count;
}  

void count_var() {
	uint16_t store;
	
	if (countUp)
		var_count++;
	else
		var_count--;
	
	if (var_count == 10)
			var_count = 0;
	else if (var_count > 10)
		var_count = 9;
	
	
	GPIOC->ODR &= 0xFF0F;
	GPIOC->ODR |= var_count << 4;
}  
 
  
/*------------------------------------------------*/
/* Main program                                        */
 /*------------------------------------------------*/
 int main(void) {
  uint8_t alt_count;
 
  PinSetup();     //Configure GPIO pins
  
  count = 0;      //Initial value of count
  var_count = 0;  //initial value of var_count
  countUp = 1;
  alt_count = 0;
  
  __enable_irq();
 
  /* Endless loop */
  while (1) {        

  delay();
  
  countDo();
  
  //execute count_var() 
  if (alt_count) {
	  count_var();
	  alt_count = 0;
  }
  
  else {
	  alt_count = 1;
  }
  

  } /* repeat forever */ 
  }   
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  