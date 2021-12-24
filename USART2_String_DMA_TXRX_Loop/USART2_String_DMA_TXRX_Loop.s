// STM32F4 Discovery - Assembly template for CO LAB course
//
// Based on: https://github.com/fcayci/stm32f4-assembly
//
// Initializes USART2 device for DMA string transmit/receive
// String is sent on TX and, if loopback wire is installed (PD5-PD6), string is received on RX channel
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
.equ     STRING_LENGTH,  12

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
//   CR3 register
.equ     USART2_CR3,   0x14 // CR3 register


// SysTick Timer definitions
.equ     SCS, 			0xe000e000
.equ     SCS_SYST_CSR,	0x10	// Control/Status register
.equ     SCS_SYST_RVR,	0x14	// Value to countdown from
.equ     SCS_SYST_CVR,	0x18	// Current value

.equ	 SYSTICK_RELOAD_1MS,	15999  //1 msec at 16MHz ...  16 000 000 / 500 - 1


// DMA Registers definitions
.equ     DMA1_BASE, 	0x40026000  //RM, page 65
.equ     DMA_LISR, 		0x00
.equ     DMA_HISR, 		0x04
.equ     DMA_LIFCR, 	0x08
.equ     DMA_HIFCR, 	0x0C

.equ     DMA_USART2_RX_STREAM, 5    //Channel 4 on DMA1
.equ     DMA_USART2_TX_STREAM, 6    //Channel 4 on DMA1

.equ     DMA_SxCR_RX,		0x10 + 0x18 * DMA_USART2_RX_STREAM
.equ	 DMA_SxSNDTR_RX,	0x14 + 0x18 * DMA_USART2_RX_STREAM
.equ     DMA_SxPAR_RX,		0x18 + 0x18 * DMA_USART2_RX_STREAM
.equ	 DMA_SxM0AR_RX,		0x1C + 0x18 * DMA_USART2_RX_STREAM

.equ     DMA_SxCR_TX,		0x10 + 0x18 * DMA_USART2_TX_STREAM
.equ	 DMA_SxSNDTR_TX,	0x14 + 0x18 * DMA_USART2_TX_STREAM
.equ     DMA_SxPAR_TX,		0x18 + 0x18 * DMA_USART2_TX_STREAM
.equ	 DMA_SxM0AR_TX,		0x1C + 0x18 * DMA_USART2_TX_STREAM



// Values for BSSR register - pin 12
.equ     LEDs_ON,       0x0000F000
.equ     LEDs_OFF,   	0xF0000000

// Start of data section
.section .rodata
NIZ1:	.ascii "Testni Niz!!"

@ -- Zero initialized data ---
.section .bss
NIZ2:	.space STRING_LENGTH

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
    bl	INIT_DMA

zanka:
//    mov r0,#'a'
//    bl	SEND_UART

// Erase NIZ2
    ldr r0, =NIZ2
    mov r1,#STRING_LENGTH
    mov r2,#0
brisi: strb	r2,[r0]
	add r0,r0,#1
	subs r1,r1,#1
	bne brisi


/* sproži sprejemanje preko DMA */
    ldr r0, =NIZ2
    ldr r1, =STRING_LENGTH
    bl RCV_DMA

/* sproži oddajanje preko DMA */
    ldr r0, =NIZ1
    ldr r1, =STRING_LENGTH
    bl SND_DMA

// Wait for the end of transmission
/*    ldr r6, =USART2_BASE
WAIT_TC:
    ldr r5, [r6, #USART2_SR]
    tst r5, #(1 << 6)
    beq WAIT_TC*/

    ldr r6, =DMA1_BASE
WAIT_TCIF6:
    ldr r5, [r6, #DMA_HISR]
    tst r5, #(1 << 21)
    beq WAIT_TCIF6
// clear reception flag
	mov r5, #(1 << 21)
	str	r5,[r6, #DMA_HIFCR]
// clear DMAT flag - seems not necessary
/*    ldr r6, =USART2_BASE
    ldr r5, [r6, #USART2_CR3]
	bic r5, #(1<<7)          	// Set bit 7 to disable DMAT bit
	str r5, [r6,#USART2_CR3]    // Store result
*/

    bl LED_OFF

	mov r0, 500
	bl 	DELAYTC

//    bl	RECV_UART
/* pocakaj na zastavico ENDRX*/

// Wait for the end of reception
    ldr r6, =DMA1_BASE
WAIT_TCIF5:
    ldr r5, [r6, #DMA_HISR]
    tst r5, #(1 << 11)
    beq WAIT_TCIF5
// clear reception flag
	mov r5, #(1 << 11)
	str	r5,[r6, #DMA_HIFCR]
// clear DMAR flag - seems not necessary
/*    ldr r6, =USART2_BASE
    ldr r5, [r6, #USART2_CR3]
	bic r5, #(1<<6)          	// Set bit 6 to disable DMAR bit
	str r5, [r6,#USART2_CR3]    // Store result
*/

	bl LED_ON

	mov r0, 500
	bl 	DELAYTC

	b zanka                      // Jump to loop

INIT_DMA:
  push {r5, r6, lr}
	// Enable DMA1 Peripheral Clock (bit 21 in AHB1ENR register)
	ldr r6, =RCC_BASE          	// Load peripheral clock reg base address to r6
	ldr r5, [r6,#RCC_AHB1ENR]   // Read its content to r5
	orr r5, #(1<<21)          	// Set bit 21 to enable DMA1 clock
	str r5, [r6,#RCC_AHB1ENR]   // Store result in peripheral clock register

	// Enable DMA Reception for USART2
    ldr r6, =USART2_BASE
    ldr r5, [r6, #USART2_CR3]
	orr r5, #(1<<6)          	// Set bit 6 to enable DMAR bit
	str r5, [r6,#USART2_CR3]    // Store result

	// Enable DMA Transmission for USART2
    ldr r6, =USART2_BASE
    ldr r5, [r6, #USART2_CR3]
	orr r5, #(1<<7)          	// Set bit 7 to enable DMAT bit
	str r5, [r6,#USART2_CR3]    // Store result



  pop {r5, r6, pc}

RCV_DMA:
  push {r5, r6, lr}

//wait for previous DMA transfer to finish
/*	while((usart->SR& USART_SR_TC) == 0);
	usart->SR&= ~USART_SR_TC;
      ldr r6, =USART2_BASE
WAIT_TC:
      ldr r5, [r6, #USART2_SR]
      tst r5, #(1 << 6)
      beq WAIT_TC */

	ldr r6, =DMA1_BASE          	// Load reg base address to r6

WAIT_EN:							// Wait EN bit to become zero
      ldr r5, [r6, #DMA_SxCR_RX]
      tst r5, #1
      bne WAIT_EN

	ldr r6, =DMA1_BASE          	// Load reg base address to r6
//  Receive (RX) DMA Init
	ldr r5, =USART2_BASE+USART2_DR	// RX peripheral address to r5
	str r5, [r6,#DMA_SxPAR_RX]   	// Store result in peripheral DMA pointerclock register
	str r0, [r6,#DMA_SxM0AR_RX]   	// Store address pointerr
	str r1, [r6,#DMA_SxSNDTR_RX]   	// Store result in peripheral DMA pointerclock register

	ldr r5,=(0b0100 << 25) + (1 << 10) + (0x00 << 6) + 1 // CHSEL = 4, MINC=1, DIR=Periph2Memory, EN=1
    str r5, [r6, #DMA_SxCR_RX]		// Enable channel

// Wait for the end of transmission
/*    ldr r6, =USART2_BASE
WAIT_TC:
    ldr r5, [r6, #USART2_SR]
    tst r5, #(1 << 6)
    beq WAIT_TC */

  pop {r5, r6, pc}


SND_DMA:
  push {r5, r6, lr}

//wait for previous DMA transfer to finish
/*	while((usart->SR& USART_SR_TC) == 0);
	usart->SR&= ~USART_SR_TC;
      ldr r6, =USART2_BASE
WAIT_TC:
      ldr r5, [r6, #USART2_SR]
      tst r5, #(1 << 6)
      beq WAIT_TC */

	ldr r6, =DMA1_BASE          	// Load reg base address to r6

WAIT_EN1:							// Wait EN bit to become zero
      ldr r5, [r6, #DMA_SxCR_TX]
      tst r5, #1
      bne WAIT_EN1

	ldr r6, =DMA1_BASE          	// Load reg base address to r6
//  Transmit (TX) DMA Init
	ldr r5, =USART2_BASE+USART2_DR	// RX peripheral address to r5
	str r5, [r6,#DMA_SxPAR_TX]   	// Store result in peripheral DMA pointerclock register
	str r0, [r6,#DMA_SxM0AR_TX]   	// Store address pointerr
	str r1, [r6,#DMA_SxSNDTR_TX]   	// Store result in peripheral DMA pointerclock register

// Clear TC bit in USART Status
    ldr r6, =USART2_BASE
    ldr r5, [r6, #USART2_SR]
	bic r5, #(1<<6)          	// Clear bit 7 TC bit
	str r5, [r6, #USART2_SR]    // Store result

// Activate DMA Channel
	ldr r6, =DMA1_BASE          	// Load reg base address to r6
	ldr r5,=(0b0100 << 25) + (1 << 10) + (0x01 << 6) + 1 // CHSEL = 4, MINC=1, DIR=Memory2Periph , EN=1
    str r5, [r6, #DMA_SxCR_TX]		// Enable channel

  pop {r5, r6, pc}


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


