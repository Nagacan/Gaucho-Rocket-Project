// Where is the LED connected? (Use Nucleo-UserManual "LED")
// USER LED : PA5 (pin 21) or PB13 (pin 34) : High = ON, Low = OFF
// Each port has a number of pins (Example: PA1: Port A Pin 1)
// Port: A , Data Sheet "Memory Map"shows Peripheral Memory Addresses
// Pin: 5
// An unsigned long (UL) is a data type used to store non-negative whole numbers (integers).
// Busses can be found on data sheet "block diagram" (internal connections of the controller)
// APB = Advanced Peripheral Bus (slower clock cycles than AHB), AHB = Advanced High Performance Bus
// Peripherals have addresses, inside these peripherals they have registers which have sub-addresses
// Within this range of addresses 0x4002 0000 - 0x4002 03FF are all GPIOA registers
// AHB1 range is 0x4002 0000 - 0x4007 FFFF
// 0x4000 0000 + 0x	0002 0000= 0x4002 0000
// 0x0002 0000 is the offset from  the peripheral base to arrive at AHB1
// 32 bit values means we need 8 digits when we write
// Each hexadecimal gives 4 binary digits (4x8=32)
// Deleting a 0 after the number in hexadecimal it becomes a different number 0x00020000UL
// Position of the active bits would change when we expand to binary form

// IMPORTANT //
// Every peripheral needs access to clock, busses carry clock to various parts of the busses/micro-controller
// Need to access (turn on) RCC (Reset Clock Control) Module which enables a bus to transport a clock
// Need Pin5 in GPIOA to turn on LED
// RCC has registers for enabling clock for AHB1, clock for APB2 ETC
// GPIOA has registers for configuring pins, pin speeds, set input/output/analog, reading/storing data
// Need to get to register for enabling clock for AHB1 (Register known as "AHB1EN")
// RCC Address Offset = Add this value to RCC Base Address we get AHB1ENR
// Bit 0 GPIOAEN: IO port A clock enable when Bit = 0 : IO port A clock disabled | 1 : IO port A clock enabled
// (1U<<4) means 0b 0000 0000 0000 0000 0000 0000 0000 0000 becomes 0b 0000 0000 0000 0000 0000 0000 0004 0000
// GPIOs have at least Mode Register("Moder" as output; set in/output pins) and data registers (store data)
// MODER3 is for configuring all Pin 3 (Interested in Pin5 -> MODER5, which is bits 10/11)
// "01: General purpose output mode" Pin5 becomes output when bit 10 is set to "1", and bit 11 is "0"
// Only have to set Bit 10 to "1", reset value has other bits to 0 already
// (1U<<10UL) = Set bit 10 to 1, "&=~(1U<<11)"= Set bit 11 to "0"
// Alternate function mode: can be set to RX/TX line for UART Peripheral, or SPI/I2C Pin
// After configured output pin need to write on/off to a output data register (ODR5 = PIN5) to turn LED on/off
// To use/access registers need to typecast as "(*(volatile unsigned int *) XXXX )"

#define PERIPH_BASE			(0x40000000UL)
// Start of address
#define AHB1PERIPH_OFFSET 	(0x00020000UL) // Value same as 0x0020000UL = 0x020000UL = 0x20000UL
#define AHB1PERIPH_BASE 	(PERIPH_BASE + AHB1PERIPH_OFFSET)
// Bus Address
#define GPIOA_OFFSET		(0x0000U)	// Same as 0x0000 0000 and 0x0
#define GPIOA_BASE 			(AHB1PERIPH_BASE + GPIOA_OFFSET) // AKA Just AHB1PERIPH_BASE
// Peripheral Address
#define RCC_OFFSET 			(0x00003800UL)
#define RCC_BASE			(AHB1PERIPH_BASE + RCC_OFFSET)
// Reset Clock Control Address
#define AHB1EN_R_OFFSET		(0x30UL)
#define AHB1EN_R_RCC		(* (volatile unsigned int *) (RCC_BASE + AHB1EN_R_OFFSET))
// Clock for Bus Address
#define MODE_R_OFFSET		(0x00UL)
#define GPIOA_MODE_R		(* (volatile unsigned int *) (GPIOA_BASE + MODE_R_OFFSET))
// GPIO Mode Register
#define OD_R_OFFSET			(0x14UL)
#define GPIOA_OD_R			(* (volatile unsigned int *) (GPIOA_BASE + OD_R_OFFSET))
// GPIO Output Data Register
#define GPIOAEN				(1U<<0) // Shift 1 to position 0 (Bit 0 is set to 1 : Enabled)
// Bit 0 of Address Line is set to 1 (Enabled)
#define PIN5				(1U<<5)
#define LED_PIN				PIN5

// If RCC AHB1 Enable Register = 0b 0000 0000 0000 0000 1000 1111 0000 0000 and we set it = GPIOAEN;
// Then RCC AHB1 Enable Register = 0b 0000 0000 0000 0000 0000 0000 0000 0001, which overwrites default
// This is a problem because it writes the other bits to 0, where other default registers may need them to be high
// To fix Use "|=" , "or" operator, which means no other bits will be changed except for bit #1.
// So RCC AHB1 Enable Register |= GPIOAEN |= 0b 0000 0000 0000 0000 1000 1111 0000 0001
// "(1U<<10)" means set bit 10 to 1 while "&=~ (1U<<11)" means set bit 11 to 0

int main (void)
{
	/*1. Enable Clock Access to GPIOA*/ // Set bit 0 of the address line (AHB1EN_R_RCC) to 1
	AHB1EN_R_RCC |= GPIOAEN ;
	/*2. Set PA5 as Output Pin*/ // Pin5 becomes output when bit 10 is set to "1", and bit 11 is "0" from ModeR
	GPIOA_MODE_R |= (1U<<10);
	GPIOA_MODE_R &=~ (1U<<11);

	while(1)
	{
		/*3. Set PA5 high (turn on)*/ //Write 1 in position 5 in Output Data Register (GPIOA_OD_R |= (1U<<5))
		GPIOA_OD_R |= LED_PIN;
	}
}
