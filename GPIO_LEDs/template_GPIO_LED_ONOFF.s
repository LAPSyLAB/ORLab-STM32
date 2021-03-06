// STM32F4 Discovery - Assembly template for CO LAB course
//
// Based on: https://github.com/fcayci/stm32f4-assembly
//
// Turns on an LED attached to GPIOD Pin 12
// We need to enable the clock for GPIOD and set up pin 12 as output.

// Start with enabling thumb 32 mode since Cortex-M4 do not work with arm mode
// Unified syntax is used to enable good of the both words...

// Make sure to run arm-none-eabi-objdump.exe -d prj1.elf to check if
// the assembler used proper instructions. (Like ADDS)

.thumb
.syntax unified
.cpu cortex-m4

//.arch armv7e-m

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////
// Definitions section. Define all the registers and
// constants here for code readability.

// Constants
.equ     LEDDELAY,      4000
// By default 16MHz internal HSI clock is enabled
// Internal loop takes 4 cycles

// Register Addresses
// You can find the base addresses for all peripherals from Memory Map section
// RM0090 on page 64. Then the offsets can be found on their relevant sections.

// RCC   base address is 0x40023800
//   AHB1ENR register offset is 0x30
.equ     RCC_AHB1ENR,   0x40023830 // RCC AHB1 peripheral clock reg (page 180)

// GPIOD base address is 0x40020C00
.equ     GPIOD_BASE,   0x40020C00 // GPIOD base address)
//   MODER register offset is 0x00
.equ     GPIOD_MODER,   0x00 // GPIOD port mode register (page 281)
//   ODR   register offset is 0x14
.equ     GPIOD_ODR,     0x14 // GPIOD output data register (page 283)
//   BSSR   register offset is 0x18
.equ     GPIOD_BSSR,     0x18 // GPIOD port set/reset register (page 284)

//Absolute addresses
// .equ     GPIOD_MODER,   0x40020C00 // GPIOD port mode register (page 281)
// .equ     GPIOD_ODR,     0x40020C14 // GPIOD output data register (page 283)
// .equ     GPIOD_BSSR,    0x40020C18 // GPIOD port set/reset register (page 284)

// Values for BSSR register - pin 12
.equ     LEDs_ON,       0x0000F000
.equ     LEDs_OFF,   	0xF0000000



// Start of text section
.section .text
///////////////////////////////////////////////////////////////////////////////
// Vectors
///////////////////////////////////////////////////////////////////////////////
// Vector table start
// Add all other processor specific exceptions/interrupts in order here
	.long    __StackTop                 // Top of the stack. from linker script
	.long    _start +1                  // reset location, +1 for thumb mode

///////////////////////////////////////////////////////////////////////////////
// Main code starts from here
///////////////////////////////////////////////////////////////////////////////

_start:
    bl INIT_IO

loop:
	bl LED_ON
	mov r0, 500
	bl DELAY
	bl LED_OFF
	mov r0, 500
	bl DELAY
	b loop                      // Jump to loop

INIT_IO:
  push {r5, r6, lr}
	// Enable GPIOD Peripheral Clock (bit 3 in AHB1ENR register)
	ldr r6, = RCC_AHB1ENR       // Load peripheral clock reg address to r6
	ldr r5, [r6]                // Read its content to r5
	orr r5, 0x00000008          // Set bit 3 to enable GPIOD clock
	str r5, [r6]                // Store result in peripheral clock register

	// Make GPIOD Pin12 as output pin (bits 25:24 in MODER register)
	ldr r6, =GPIOD_BASE       // Load GPIOD BASE address to r6
	ldr r5, [r6,#GPIOD_MODER]  // Read GPIOD_MODER content to r5
	and r5, 0x00FFFFFF          // Clear bits 31-24 for P12-15
	orr r5, 0x55000000          // Write 01 to bits 31-24 for P12-15
	str r5, [r6]                // Store result in GPIOD MODER register
  pop {r5, r6, pc}



LED_ON:
    push {r5, r6, lr}
	// Set GPIOD Pins to 1 (through BSSR register)
	ldr    r6, =GPIOD_BASE       // Load GPIOD BASE address to r6
	mov    r5, #LEDs_ON
	str    r5, [r6,#GPIOD_BSSR] // Write to BSRR register
  	pop {r5, r6, pc}

LED_OFF:
    push {r5, r6, lr}
	// Set GPIOD Pins to 0 (through BSSR register)
	ldr    r6, =GPIOD_BASE       // Load GPIOD BASE address to r6
	mov    r5, #LEDs_OFF
	str    r5, [r6,#GPIOD_BSSR] // Write to BSRR register
  	pop {r5, r6, pc}


DELAY:
    push {r1, lr}

MSEC: ldr r1,=LEDDELAY
LOOP:    subs r1, r1, 1
         bne LOOP
      subs r0, r0, 1
      bne MSEC
    pop {r1, pc}
