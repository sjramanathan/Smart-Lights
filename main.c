#include <stdint.h>
#include <stdio.h>
#include "pinsActivation.h"
#include "tm4c123gh6pm.h"

/** UART Module 1 Initialization
 *
 *  Configures necessary UART Module 1 registers
 */
void UART_Init();

/** GPIO Initialization
 *
 *  Configures necessary GPIO registers for UART 1 Module
 */
void GPIO_Init();

/** Transmit Data via UART
 *
 *  Takes in the data to be transferred
 */
void TransmitData(char* data);

/** Delay the system
 */
void delay();


// reads one character form the UART connection
char readChar(void) {
    char c;
    while ((UART_FR & (1<<4)) != 0); // waits for buffer to free
    c = UART_DR; // reads character
    return c; // returns character
}

// This code is used to initialize the LED ports
void LED_Init(void) { volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x1;  // activate port A
    delay = SYSCTL_RCGC2_R; // delay for system clock

    GPIO_PORTA_PCTL_R &= ~0xFFF00000;  // PA 5,6,7
    GPIO_PORTA_AMSEL_R &= ~0xE0;       // dissable analog
    GPIO_PORTA_DIR_R |= 0xE0;         // set them to outputs(Pa5,6,7)
    GPIO_PORTA_AFSEL_R |= ~0xE0;      // GPIO functionality
    GPIO_PORTA_DEN_R |= 0xE0;         // Enable the digital pins
}

// Turns on the green LED
void LED_GREEN_On(void) {
    GPIO_PORTA_DATA_R |= 0x80;
}

// Turns off the green LED
void LED_GREEN_Off(void) {
    GPIO_PORTA_DATA_R &= (~0x80);
}

// Turns the Buzzer ON
void Buzzer_On() {
    GPIO_PORTA_DATA_R |= 0x40;
}

// Turns the Buzzer OFF
void Buzzer_Off() {
    GPIO_PORTA_DATA_R &= (~0x40);
}

int main() {
  UART_Init();  // initialize UARt
  GPIO_Init();  // initialize GPIO
  LED_Init();   // initialize LED

  delay();
  char* data = "EE 474 Final Project\r\n";  // Title of Project
  TransmitData(data); // Transmit to PuTTy for display

  while (1) {
    char c = readChar(); // read character from the UART connection that will come through our android app
    switch(c) {    // use input to make decisions
      case '1': TransmitData("Switch Pressed\r\n");   // switch was pressed
                // Turn on the LED
                LED_GREEN_On();
                // Make the buzzer play a small ring
                Buzzer_On();
                delay();
                Buzzer_Off();
                // End of case
                break;
      default: // Turn off the LED and Buzzer
               LED_GREEN_Off();
               Buzzer_Off();
               // End of case
               break;
    }
  }

  return 1;
}

void GPIO_Init() {
  // Congiguring the GPIO pins to enable UART function
  RCGCGPIO = 0x2;                   // enable clock to PortB
  GPIO_AFSEL_PORTB = 0x3;           // allow alternative function for PB 0&1
  GPIO_DEN_PORTB = 0x3;             // enable digital pins
  GPIO_PCTL_PORTB |= 0x11;            // set alternative function to UART
}

void UART_Init() {
  // Configuring the UART Module
  RCGCUART |= 0x2;                   // enable clock to UART Module 1
  int32_t delay = RCGCUART;   // delay to wait for clock setup

  UART_CTL &= ~(0x1);                   // disable module before configuration
  //UART_LCRH &= ~0x10;                 //Flushes Fifo

  // set the baud rate (integer & fraction parts)
  UART_IBRD = 0x8;
  UART_FBRD = 0x2C;

  UART_LCRH = (0x3 << 5);                  // set the transmission line control
  UART_CC = 0x0;                    // use system clock

  UART_CTL = 0x301;         // enable UART now
}

void delay() {
  // runs a set amount of delay
  for (int i = 0; i < 1000000 / 3; i++) {}
}

void TransmitData(char* data) {
  char* temp = data; // gets refernce to data given

  // Transmit data until we reach end of the given data
  while(*temp != '\0') {
    // Here you check the flag to see
    // if the UART is available
    while(UART_FR & 0x20) {}
    UART_DR = *temp; // send data to UART when available
    temp++; // increment to get next character
  }
}
