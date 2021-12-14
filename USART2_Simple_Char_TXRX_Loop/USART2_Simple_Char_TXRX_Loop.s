// STM32F4 Discovery - Assembly template for CO LAB course
//
// Based on: https://github.com/fcayci/stm32f4-assembly
//
// Initializes USART2 device for single char transmit/receive
// Single char is sent on TX and, if loopback wire is installed (PD5-PD6), char is received on RX channel
// LED blinks on 0.5 secs, when TX and RX are working ok.

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
.equ     RCC_BASE,   0x40023800 // RCC AHB1 peripheral clock reg (page 180)
//   AHB1ENR register offset is 0x30
.equ     RCC_AHB1ENR,   0x30 // RCC AHB1 peripheral clock reg (page 180)
//   APB1ENR register offset is 0x30
.equ     RCC_APB1ENR,   0x40 // RCC APB1 peripheral clock reg (page 183)

// GPIOD base address is 0x40020C00
.equ     GPIOD_BASE,   0x40020C00 // GPIOD base address)
//   MODER register offset is 0x00
.equ     GPIOD_MODER,   0x00 // GPIOD port mode register (page 281)
//   ODR   register offset is 0x14
.equ     GPIOD_ODR,     0x14 // GPIOD output data register (page 283)
//   BSSR   register offset is 0x18
.equ     GPIOD_BSSR,     0x18 // GPIOD port set/reset register (page 284)
//   AFRL  register offset is 0x20
.equ     GPIOD_AFRL,     0x20 		// GPIOD output data register (page 285)
.equ     GPIOD_AFRL_VALUE,     0x07700000 		// AF7 on PD5,6

//Absolute addresses
// .equ     GPIOD_MODER,   0x40020C00 // GPIOD port mode register (page 281)
// .equ     GPIOD_ODR,     0x40020C14 // GPIOD output data register (page 283)
// .equ     GPIOD_BSSR,    0x40020C18 // GPIOD port set/reset register (page 284)

// USART2 base address is 0x40004400
.equ     USART2_BASE,   0x40004400 // USART2 base address)
//   SR register
.equ     USART2_SR,    0x00 // SR register (page 1007)
//   DR register
.equ     USART2_DR,    0x04 // SR register (page 1007)
//   BRR register
.equ     USART2_BRR,   0x08 // BRR register (page 1010)
.equ     USART2_BRR_VAL,   (52<<4)+0b1010 // BRR register
//   CR1 register
.equ     USART2_CR1,   0x0C // CR1 register
.equ     USART2_CR1_VAL,   0x200C // CR1 register

// SysTick Timer definitions
.equ     SCS, 			0xe000e000
.equ     SCS_SYST_CSR,	0x10	// Control/Status register
.equ     SCS_SYST_RVR,	0x14	// Value to countdown from
.equ     SCS_SYST_CVR,	0x18	// Current value

.equ	 SYSTICK_RELOAD_1MS,	15999  //1 msec at 16MHz ...  16 000 000 / 500 - 1


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
    bl 	INIT_IO
    bl 	INIT_UART
    bl	INIT_TC

loop:
    mov r0,#'a'
    bl	SEND_UART

    bl LED_OFF

	mov r0, 500
	bl 	DELAYTC

    bl	RECV_UART

	bl LED_ON

	mov r0, 500
	bl 	DELAYTC

	b loop                      // Jump to loop

INIT_UART:
  push {r5, r6, lr}
	// Enable USART2 Peripheral Clock (bit 17 in APB1ENR register)
	ldr r6, =RCC_BASE          	// Load peripheral clock reg base address to r6
	ldr r5, [r6,#RCC_APB1ENR]   // Read its content to r5
	orr r5, #(1<<17)          	// Set bit 17 to enable USART2 clock
	str r5, [r6,#RCC_APB1ENR]   // Store result in peripheral clock register

	ldr r6, =GPIOD_BASE       // Load GPIOD BASE address to r6
	// Make GPIOD Pina 5,6 as AF7 (bits 20:27 in AFRL register)
	ldr r5, [r6,#GPIOD_AFRL]  // Read GPIOD_AFRL content to r5
	and r5, 0x0FF00000
	orr r5, GPIOD_AFRL_VALUE
	str r5, [r6,#GPIOD_AFRL]  // Store result in GPIOD AFRL register

	// Make GPIOD Pins 5,6 as AF (bits 10:13 in MODER register)
	ldr r5, [r6,#GPIOD_MODER]  // Read GPIOD_MODER content to r5
	and r5, 0xFFFFC3FF          // Clear bits
	orr r5, 0x00002800          // Write 10 to bits
	str r5, [r6,#GPIOD_MODER]                // Store result in GPIOD MODER register

	ldr r6, =USART2_BASE       // Load USART2 BASE address to r6
	// Set USART2 BaudRate
	ldr r5, =USART2_BRR_VAL
	str r5, [r6,#USART2_BRR]                // Store result in GPIOD MODER register
	// Start USART2
	ldr r5, =USART2_CR1_VAL
	str r5, [r6,#USART2_CR1]                // Store result in GPIOD MODER register

  pop {r5, r6, pc}


RECV_UART:
  push {r1, r2, lr}
      ldr r1, =USART2_BASE
RECV_LP:
      ldr r2, [r1, #USART2_SR]
      tst r2, #(1 << 5)
      beq RECV_LP
      ldr r0, [r1, #USART2_DR]
  pop {r1, r2, pc}


SEND_UART:
  push {r1, r2, lr}
      ldr r1, =USART2_BASE
SEND_LP:
      ldr r2, [r1, #USART2_SR]
      tst r2, #(1 << 7)
      beq SEND_LP
      str r0, [r1, #USART2_DR]
  pop {r1, r2, pc}


INIT_TC:
  push {r0, r1, lr}
	ldr r1, =SCS

	ldr r0, =SYSTICK_RELOAD_1MS
	str r0, [r1, #SCS_SYST_RVR]

	ldr r0, =0
	str r0, [r1, #SCS_SYST_CVR]

	ldr r0, =5
	str r0, [r1, #SCS_SYST_CSR]

  pop {r0, r1, pc}


INIT_IO:
  push {r5, r6, lr}
	// Enable GPIOD Peripheral Clock (bit 3 in AHB1ENR register)
	ldr r6, = RCC_BASE          // Load peripheral clock reg base address to r6
	ldr r5, [r6,#RCC_AHB1ENR]   // Read its content to r5
	orr r5, 0x00000008          // Set bit 3 to enable GPIOD clock
	str r5, [r6,#RCC_AHB1ENR]   // Store result in peripheral clock register

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


DELAYTC:
    push {r1, r2, lr}
    ldr r1, =SCS


LOOPTC:	ldr r2, [r1, #SCS_SYST_CSR]
		tst r2, #0x10000
		beq LOOPTC

      subs r0, r0, 1
      bne LOOPTC
    pop {r1, r2, pc}

DELAY:
    push {r1, lr}

MSEC: ldr r1,=LEDDELAY
LOOP:    subs r1, r1, 1
         bne LOOP
      subs r0, r0, 1
      bne MSEC
    pop {r1, pc}
