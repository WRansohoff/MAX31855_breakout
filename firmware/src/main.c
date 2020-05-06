#include "main.h"

// Default system clock frequency.
uint32_t core_clock_hz = 4000000;
// SysTick delay counter.
volatile uint32_t systick = 0;

// Reset handler: set the stack pointer and branch to main().
__attribute__( ( naked ) ) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}

// Delay for a specified number of milliseconds.
// TODO: Prevent rollover bug on the 'systick' value.
void delay_ms( uint32_t ms ) {
  // Calculate the 'end of delay' tick value, then wait for it.
  uint32_t next = systick + ms;
  while ( systick < next ) { __asm__( "WFI" ); }
}

// Override the 'write' clib method to implement 'printf' over UART.
int _write( int handle, char* data, int size ) {
  int count = size;
  while( count-- ) {
    while( !( USART2->ISR & USART_ISR_TXE ) ) {};
    USART2->TDR = *data++;
  }
  return size;
}

// Receive two bytes of data over SPI (blocking)
uint16_t spi_r16( SPI_TypeDef *SPIx ) {
  // Transmit two dummy bytes.
  while ( !( SPIx->SR & SPI_SR_TXE ) ) {};
  SPIx->DR = 0xFFFF;
  // Wait to receive data, then return it.
  while( !( SPIx->SR & SPI_SR_RXNE ) ) {};
  return SPIx->DR;
}

/**
 * Main program.
 */
int main(void) {
  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit.
  SCB->CPACR    |=  ( 0xF << 20 );

  // Use the 16MHz HSI oscillator for the system core clock.
  RCC->CR |=  ( RCC_CR_HSION );
  while ( !( RCC->CR & RCC_CR_HSIRDY ) ) {};
  RCC->CFGR &= ~( RCC_CFGR_SW );
  RCC->CFGR |=  ( RCC_CFGR_SW_HSI );
  while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSI ) {};
  core_clock_hz = 16000000;

  // Setup the SysTick peripheral to generate 1ms ticks.
  SysTick_Config( core_clock_hz / 1000 );

  // Enable peripheral clocks: GPIOA, GPIOB, SPI1, USART2.
  RCC->APB1ENR1 |= ( RCC_APB1ENR1_USART2EN );
  RCC->APB2ENR  |= ( RCC_APB2ENR_SPI1EN );
  RCC->AHB2ENR  |= ( RCC_AHB2ENR_GPIOAEN |
                     RCC_AHB2ENR_GPIOBEN );

  // UART virtual COM port TX pin setup (AF7).
  GPIOA->MODER    &= ~( 0x3 << ( 2 * 2 ) );
  GPIOA->MODER    |=  ( 0x2 << ( 2 * 2 ) );
  GPIOA->OTYPER   &= ~( 0x1 << 2 );
  GPIOA->OSPEEDR  &= ~( 0x3 << ( 2 * 2 ) );
  GPIOA->OSPEEDR  |=  ( 0x2 << ( 2 * 2 ) );
  GPIOA->AFR[ 0 ] &= ~( 0xF << ( 2 * 4 ) );
  GPIOA->AFR[ 0 ] |=  ( 0x7 << ( 2 * 4 ) );

  // SPI pins setup.
  // PA11: software-controlled CS pin.
  GPIOA->MODER    &= ~( 0x3 << ( 11 * 2 ) );
  GPIOA->MODER    |=  ( 0x1 << ( 11 * 2 ) );
  GPIOA->OTYPER   &= ~( 0x1 << 11 );
  GPIOA->OSPEEDR  &= ~( 0x3 << ( 11 * 2 ) );
  GPIOA->OSPEEDR  |=  ( 0x1 << ( 11 * 2 ) );
  GPIOA->ODR      |=  ( 0x1 << 11 );
  // PB3, PB4: hardware-controlled SPI SCK/MISO pins (AF5).
  GPIOB->MODER    &= ~( ( 0x3 << ( 3 * 2 ) ) |
                        ( 0x3 << ( 4 * 2 ) ) );
  GPIOB->MODER    |=  ( ( 0x2 << ( 3 * 2 ) ) |
                        ( 0x2 << ( 4 * 2 ) ) );
  GPIOB->OSPEEDR  &= ~( ( 0x3 << ( 3 * 2 ) ) |
                        ( 0x3 << ( 4 * 2 ) ) );
  GPIOB->OSPEEDR  |=  ( ( 0x2 << ( 3 * 2 ) ) |
                        ( 0x2 << ( 4 * 2 ) ) );
  GPIOB->AFR[ 0 ] &= ~( ( 0xF << ( 3 * 4 ) ) |
                        ( 0xF << ( 4 * 4 ) ) );
  GPIOB->AFR[ 0 ] |=  ( ( 0x5 << ( 3 * 4 ) ) |
                        ( 0x5 << ( 4 * 4 ) ) );

  // USART2 setup: 115200 baud, transmit only.
  USART2->BRR  = ( core_clock_hz / 115200 );
  USART2->CR1 |= ( USART_CR1_TE | USART_CR1_UE );

  // SPI setup: standard host mode.
  SPI1->CR1 |= ( SPI_CR1_SSM |
                 SPI_CR1_SSI |
                 SPI_CR1_MSTR );
  SPI1->CR1 |= ( SPI_CR1_SPE );

  // Main loop: Read data every 5 seconds.
  uint16_t raw_data = 0x0000;
  float temp_c = 0.0;
  while( 1 ) {
    // Wait 5 seconds.
    delay_ms( 5000 );
    // Read data.
    GPIOA->ODR &= ~( 0x1 << 11 );
    raw_data = spi_r16( SPI1 );
    GPIOA->ODR |=  ( 0x1 << 11 );
    // Convert the raw output to big-endian format and print it.
    raw_data = ( raw_data << 8 ) | ( raw_data >> 8 );
    printf( "Raw temperature data: 0x%04X\r\n", raw_data );
    // If a fault was detected, report that.
    if ( raw_data & 0x0001 ) {
      printf( "FAULT\r\n" );
    }
    // Otherwise, calculate the temperature and print that.
    else {
      temp_c = ( ( raw_data & 0x7FFF ) >> 2 ) * 0.25;
      if ( raw_data & 0x8000 ) {
        temp_c = -temp_c;
      }
      printf( "Temperature: %.2fC, %.2fF\r\n",
              temp_c, ( temp_c * 1.8 ) + 32 );
    }
  }
}

// SysTick interrupt handler: increment the global 'systick' value.
void SysTick_IRQn_handler( void ) {
  ++systick;
}
