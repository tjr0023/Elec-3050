/*====================================================*/
/* Victor P. Nelson                                        */
/* Tyler Robins and Jeremy                                 */
/* ELEC 3040/3050 - Lab 3, Program 1                       */ 
/* Count up or down from 0-9 and display on LEDs in binary */
/* Added opposite counter to original counter              */
/*====================================================*/ 

 
#include "STM32L1xx.h"      /* Microcontroller information */ 
 
/* Define global variables */ 
 
uint8_t countUp;
uint16_t count;
uint16_t anti_count;
 
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
   GPIOC->MODER |=  (0x00055555);   /* General purpose output mode*/ } 
 
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
	uint16_t store;
	
	if (countUp)
		count++;
		anti_count--;
	else 
		count--;
		anti_count++;
	
	
	if (count == 10) //counted up past 9 - reset
			count = 0;
	else if (count > 10) //overflow from subtracting 1
		count = 9;
	
	if (anti_count == 10)
			anti_count = 0;
	else if (anti_count > 10)
		count = 9;
	
	
	GPIOC->ODR &= 0xFF00;
	GPIOC->ODR |= count;
	GPIOC->ODR |= anti_count << 4;
}  
 
  
/*------------------------------------------------*/
/* Main program                                        */
 /*------------------------------------------------*/
 int main(void) {
  unsigned char sw1;          //state of SW1
  unsigned char sw2;          //state of SW2
 
  PinSetup();     //Configure GPIO pins
  
  sw2 = 0;		  //Initial counting state
  count = 0;      //Initial value of count
  anti_count = 0; //initial value of anti_count
 
  /* Endless loop */
  while (1) {        

  delay();
  
  //read sw2
  sw2 = GPIOA->IDR & 0x0004;
  
  if (sw2 == 0)     //Direction is down
  countUp = 1;
  else      		//Direction is up  
  countUp = 0;

  // read SW2
  sw1 = GPIOA->IDR & 0x0002;

  if (sw1 > 0) {    //Wait for SW1 = 1 on PE0
		countDo();
  }
  

  } /* repeat forever */ 
  }   
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  