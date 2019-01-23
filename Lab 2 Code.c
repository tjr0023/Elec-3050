/*====================================================*/
/* Victor P. Nelson                                                  */
/* ELEC 3040/3050 - Lab 1, Program 1                                */ 
/* Toggle LED1 while button pressed, with short delay inserted     */ 
/*====================================================*/ 

 
#include "STM32L1xx.h"      /* Microcontroller information */ 
 
/* Define global variables */ 
 
uint8_t countUp;
uint16_t count;
 
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
   GPIOC->MODER &= ~(0x000F0000);   /* Clear PC9-PC8 mode bits    */
   GPIOC->MODER |=  (0x00050000);   /* General purpose output mode*/ } 
 
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

void count() {
	if (countUp)
		count++;
	else 
		count--;
	
	if (count > 9)
			count = 0;
	else if (count < 0)
		count = 9;
	
	GPIOC->BSRR = count;
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
 
  /* Endless loop */
  while (1) {        

  delay();
  
  //read sw1
  sw1 = GPIOA->IDR & 0x0001;
  
  if (sw1 == 0)     //Direction is down
  countUp = 0;
  else      		//Direction is up  
  countUp = 1;

  // read SW2 
  sw2 = GPIOA->IDR & 0x0004;

  if (sw2) {    //Wait for SW1 = 1 on PE0
		count();
  }
  

  } /* repeat forever */ 
  }   
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  