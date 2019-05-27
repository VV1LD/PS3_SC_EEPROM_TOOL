/************************************************************************
sc_eeprom_tool.c (v0.01) - Teensy++ 2.0 Compatible PS3 Syscon EEPROM utility
Modified from SPIway from judges <judges@eEcho.com>
Hacked together for Original Model Syscons on Fat PS3 consoles by
WildCard https://twitter.com/VVildCard777 2019

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "delay_x.h"
#include "usb_serial.h"
#include "SPI_AVR8.h"

#define VERSION_MAJOR			0
#define VERSION_MINOR			1

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

// Define commands
enum {
	CMD_PING1 = 0,
	CMD_PING2,
	CMD_GET_STATUS,
	CMD_DUMP_EEPROM,
	CMD_UNLOCK_EEPROM,
	CMD_WRITE_EEPROM
} cmd_t;

#define PS3_SC_EEPROM_UNLOCK_CMD	 0xa3 // NOT IMPLEMENTED
#define PS3_SC_EEPROM_WRITE_CMD 	 0xa4 // NOT IMPLEMENTED
#define PS3_SC_EEPROM_READ_CMD 	 	 0xa8
#define PS3_SC_EEPROM_GET_STATUS_CMD 0xa9

//SPI commands (69)
#define SPI_COMMAND_3B_READ				0x03	//
#define SPI_COMMAND_3B_FASTREAD			0x0B	//
#define SPI_COMMAND_3B_2READ			0xBB	//
#define SPI_COMMAND_3B_DREAD			0x3B	//
#define SPI_COMMAND_3B_4READ_BOTTOM		0xEB	//
#define SPI_COMMAND_3B_4READ_TOP		0xEA	//
#define SPI_COMMAND_3B_QREAD			0x6B	//
#define SPI_COMMAND_3B_PAGEPROG			0x02	//
#define SPI_COMMAND_3B_4PAGEPROG		0x38	//
#define SPI_COMMAND_3B_SECTOR_ERASE		0x20	//
#define SPI_COMMAND_3B_BLOCK_ERASE_32K	0x52	//
#define SPI_COMMAND_3B_BLOCK_ERASE_64K	0xD8	//

#define SPI_COMMAND_4B_READ				0x13	//
#define SPI_COMMAND_4B_FASTREAD			0x0C	//
#define SPI_COMMAND_4B_2READ			0xBC	//
#define SPI_COMMAND_4B_DREAD			0x3C	//
#define SPI_COMMAND_4B_4READ			0xEC	//
#define SPI_COMMAND_4B_QREAD			0x6C	//
#define SPI_COMMAND_4B_PAGEPROG			0x12	//
#define SPI_COMMAND_4B_4PAGEPROG		0x3E	//
#define SPI_COMMAND_4B_SECTOR_ERASE		0x21	//
#define SPI_COMMAND_4B_BLOCK_ERASE_32K	0x5C	//
#define SPI_COMMAND_4B_BLOCK_ERASE_64K	0xDC	//

#define SPI_COMMAND_CHIP_ERASE			0x60	//
#define SPI_COMMAND_CHIP_ERASE2			0xC7	//
#define SPI_COMMAND_WREN				0x06	//
#define SPI_COMMAND_WRDI				0x04	//
#define SPI_COMMAND_RDSR				0x05	//
#define SPI_COMMAND_RDCR				0x15	//
#define SPI_COMMAND_WRSR				0x01	//
#define SPI_COMMAND_RDEAR				0xC8	//
#define SPI_COMMAND_WREAR				0xC5	//
#define SPI_COMMAND_WPSEL				0x68	//
#define SPI_COMMAND_EQIO				0x35	//
#define SPI_COMMAND_RSTQIO				0xF5	//
#define SPI_COMMAND_EN4B				0xB7	//
#define SPI_COMMAND_EX4B				0xE9	//
#define SPI_COMMAND_PGM_ERS_SUSPEND		0xB0	//
#define SPI_COMMAND_PGM_ERS_RESUME		0x30	//
#define SPI_COMMAND_DP					0xB9	//
#define SPI_COMMAND_RDP					0xAB	//
#define SPI_COMMAND_SBL					0xC0	//
#define SPI_COMMAND_RDFBR				0x16	//
#define SPI_COMMAND_WRFBR				0x17	//
#define SPI_COMMAND_ESFBR				0x18	//
#define SPI_COMMAND_RDID				0x9F	//
#define SPI_COMMAND_RES					0xAB	//
#define SPI_COMMAND_REMS				0x90	//
#define SPI_COMMAND_QPIID				0xAF	//
#define SPI_COMMAND_RDSFDP				0x5A	//
#define SPI_COMMAND_ENSO				0xB1	//
#define SPI_COMMAND_EXSO				0xC1	//
#define SPI_COMMAND_RDSCUR				0x2B	//
#define SPI_COMMAND_WRSCUR				0x2F	//
#define SPI_COMMAND_GBLK				0x7E	//
#define SPI_COMMAND_GBULK				0x98	//
#define SPI_COMMAND_WRLR				0x2C	//
#define SPI_COMMAND_RDLR				0x2D	//
#define SPI_COMMAND_WRPASS				0x28	//
#define SPI_COMMAND_RDPASS				0x27	//
#define SPI_COMMAND_PASSULK				0x29	//
#define SPI_COMMAND_WRSPB				0xE3	//
#define SPI_COMMAND_ESSPB				0xE4	//
#define SPI_COMMAND_RDSPB				0xE2	//
#define SPI_COMMAND_SPBLK				0xA6	//
#define SPI_COMMAND_RDSPBLK				0xA7	//
#define SPI_COMMAND_WRDPB				0xE1	//
#define SPI_COMMAND_RDDPB				0xE0	//
#define SPI_COMMAND_NOP					0x00	//
#define SPI_COMMAND_RSTEN				0x66	//
#define SPI_COMMAND_RST					0x99	//


#define SPI_IO_PORT		PORTF
#define SPI_IO_PIN		PINF
#define SPI_IO_DDR		DDRF
#define SPI_IO_0		0b00000001		// 0: SIO0
#define SPI_IO_1		0b00000010		// 1: SIO1
#define SPI_IO_2		0b00000100		// 2: SIO2
#define SPI_IO_3		0b00001000		// 3: SIO3
#define SPI_IO_MASK		0b00001111		//

#define SPI_IO_SI		0b00000001		// 0: SI
#define SPI_IO_SO		0b00000010		// 1: SO
#define SPI_IO_WP		0b00000100		// 2: WP
#define SPI_IO_SO_WP_IO3	0b00001110		// 2: WP

#define SPI_CONT_PORT	PORTC
#define SPI_CONT_PIN	PINC
#define SPI_CONT_DDR	DDRC
#define SPI_CONT_CS		0b00000001		// 0: Chip Select
#define SPI_CONT_SCLK	0b00000010		// 1: Clock Input
#define SPI_CONT_RESET	0b00000100		// 2: Reset

#define HWSPI_PORT		PORTB
#define HWSPI_PIN		PINB
#define HWSPI_DDR		DDRB
#define HWSPI_IO_CS		0b00000001		// 0: CS# (Chip select)
#define HWSPI_IO_SCLK	0b00000010		// 1: SCLK (Clock)
#define HWSPI_IO_MOSI	0b00000100		// 2: SI (Master out / Slave in)
#define HWSPI_IO_MISO	0b00001000		// 3: SO (Master in / Slave out)
#define HWSPI_IO_WP		0b00010000		// 4: WP#/SIO2
#define HWSPI_IO_SIO3	0b00100000		// 5: SIO3
#define HWSPI_IO_RESET	0b01000000		// 6: RESET#

#define HWSPI_CS_HIGH		HWSPI_PORT |= HWSPI_IO_CS
#define HWSPI_CS_LOW		HWSPI_PORT &= ~(HWSPI_IO_CS)
#define HWSPI_RESET_HIGH	HWSPI_PORT |= HWSPI_IO_RESET
#define HWSPI_RESET_LOW		HWSPI_PORT &= ~(HWSPI_IO_RESET)
#define HWSPI_WP_HIGH		HWSPI_PORT |= HWSPI_IO_WP
#define HWSPI_WP_LOW		HWSPI_PORT &= ~(HWSPI_IO_WP)
#define HWSPI_SIO3_HIGH		HWSPI_PORT |= HWSPI_IO_SIO3
#define HWSPI_SIO3_LOW		HWSPI_PORT &= ~(HWSPI_IO_SIO3)



#define SPI_CS_HIGH			SPI_CONT_PORT |= SPI_CONT_CS
#define SPI_CS_LOW			SPI_CONT_PORT &= ~(SPI_CONT_CS)
#define SPI_TOGGLE_SCLK		SPI_CONT_PORT |= SPI_CONT_SCLK; SPI_CONT_PORT &= ~(SPI_CONT_SCLK)

#define SPI_IO_READ(_data_)	SPI_TOGGLE_SCLK; (_data_) = ((SPI_IO_PIN & SPI_IO_SO)<<6); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<5); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<4); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<3); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<2); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<1); SPI_TOGGLE_SCLK; (_data_) |= (SPI_IO_PIN & SPI_IO_SO); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)>>1)
//#define SPI_IO_SET(_data_)	SPI_IO_PORT = (((_data_)>>7) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>6) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>5) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>4) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>3) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>2) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>1) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>0) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK
#define SPI_IO_SET(_data_)	SPI_IO_PORT = (((_data_)>>7) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>6) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>5) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>4) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>3) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>2) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>1) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>0) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK
#define SPI_COMMAND(_cmd_)	SPI_IO_SET(_cmd_)
#define QPI_IO_READ(_data_)	(_data_) = 0; SPI_TOGGLE_SCLK; (_data_) = (SPI_IO_PIN & SPI_IO_MASK)<<4; SPI_TOGGLE_SCLK; (_data_) |= (SPI_IO_PIN & SPI_IO_MASK)
#define QPI_IO_SET(_data_)	SPI_IO_PORT = (_data_)>>4; SPI_TOGGLE_SCLK; SPI_IO_PORT = (_data_); SPI_TOGGLE_SCLK
#define QPI_COMMAND(_cmd_)	QPI_IO_SET(_cmd_)

#define SPI_IO_INPUT		SPI_IO_PORT |= SPI_IO_SO //; SPI_IO_DDR = 0x00; SPI_IO_PORT = IO_PULLUPS //0=disable, 0xFF=enable
#define SPI_IO_OUTPUT		SPI_IO_DDR = 0xFF
//#define SPI_BUSY_WAIT		while (1) { if (!(spi_status() & SPI_STATUS_WIP)) { break; } }
#define SPI_BUSY_WAIT		HWSPI_CS_LOW; SPI_SendByte(SPI_COMMAND_RDSR); while (SPI_ReceiveByte() & SPI_STATUS_WIP); HWSPI_CS_HIGH
#define SPI_WREN			while ((spi_status() & SPI_STATUS_WEL) == 0) { cli(); SPI_CS_LOW; SPI_COMMAND(SPI_COMMAND_WREN); SPI_CS_HIGH; sei(); }
//#define HWSPI_WREN			while ((hwspi_status() & SPI_STATUS_WEL) == 0) { cli(); HWSPI_CS_LOW; SPI_SendByte(SPI_COMMAND_WREN); HWSPI_CS_HIGH; sei(); }
#define HWSPI_WREN			do { HWSPI_CS_LOW; SPI_SendByte(SPI_COMMAND_WREN); HWSPI_CS_HIGH; } while (!(hwspi_status() & SPI_STATUS_WEL))

void hwspi_init()
{
	HWSPI_DDR |= (HWSPI_IO_SIO3 | HWSPI_IO_RESET | HWSPI_IO_WP);
	
	HWSPI_RESET_HIGH;
	HWSPI_WP_LOW;
	HWSPI_SIO3_HIGH;
}

void spi_enable()
{
	
	SPI_CONT_DDR = 0xFF; 			// all control ports - output
	//SPI_CONT_DDR &= ~SPI_CONT_SCLK;
	SPI_CONT_PORT = 0; //low
	SPI_CS_HIGH;
	SPI_CONT_PORT |= SPI_CONT_RESET; //high
	
	SPI_IO_DDR = 0xFF;
	SPI_IO_DDR &= ~(SPI_IO_SO) ;
	SPI_IO_PORT = 0;
	//SPI_IO_PORT |= (SPI_IO_3 | SPI_IO_WP);
	//SPI_IO_PORT |= (SPI_IO_3);
	SPI_IO_PORT |= SPI_IO_SO_WP_IO3;
}

void releaseports()
{
	DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
	PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
}

uint8_t wait_ryby()
{
	/* Should be done within 3 milliseconds for all commands. */
	volatile uint32_t timeout = 0x200000; //approx. 3secs

	while (timeout > 0) {
		if ( SPI_CONT_PIN & SPI_CONT_CS ) {
			return 1;
		}
		--timeout;
	}

	return 0;
}

void spi_reset()
{
	spi_enable();
	SPI_COMMAND(SPI_COMMAND_RSTEN);
	SPI_COMMAND(SPI_COMMAND_RST);
	wait_ryby();
}

void bootloader() {
	cli();
	// disable watchdog, if enabled
	// disable all peripherals
	UDCON = 1;
	USBCON = (1<<FRZCLK);  // disable USB
	UCSR1B = 0;
	_delay_ms(50);

	EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
	TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
	DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
	PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;

	__asm volatile("jmp 0x1FC00");
}

int freeRam() {
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

int main(void) {
	int16_t command = -1;
	uint16_t freemem;
	uint32_t cur_status;
	uint8_t offset_low;
	uint8_t offset_high;
	uint8_t write_length_low;
	uint8_t write_length_high;
	uint16_t write_length;
	uint8_t write_buffer[32];
	
	// set for 8 MHz clock because of 3.3v regulator
	CPU_PRESCALE(1);
	
	// set for 16 MHz clock
	//CPU_PRESCALE(0);

	//disable JTAG
	MCUCR = (1<<JTD) | (1<<IVCE) | (0<<PUD);
	MCUCR = (1<<JTD) | (0<<IVSEL) | (0<<IVCE) | (0<<PUD);

	// set all i/o lines to input
	releaseports();

	//Init SPI
	SPI_Init(SPI_SPEED_FCPU_DIV_32 | SPI_ORDER_MSB_FIRST | SPI_SCK_LEAD_RISING | SPI_SAMPLE_LEADING | SPI_MODE_MASTER);
	hwspi_init();
	
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();

	while (!usb_configured()) /* wait */ ;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	while (1) {
		// discard anything that was received prior.  Sometimes the
		// operating system or other software will send a modem
		// "AT command", which can still be buffered.
		usb_serial_flush_input();

		while (usb_configured()) { // is user still connected?
			command = usb_serial_getchar();
			if (command == -1) continue;

			switch (command) {
			case CMD_PING1:
				usb_serial_putchar(VERSION_MAJOR);
				break;
				
			case CMD_PING2:
				freemem = freeRam();
				usb_serial_putchar(VERSION_MINOR);
				usb_serial_putchar((freemem >> 8) & 0xFF);
				usb_serial_putchar(freemem & 0xFF);
				break;
			
			case CMD_GET_STATUS:
				HWSPI_CS_LOW;
				SPI_SendByte(PS3_SC_EEPROM_GET_STATUS_CMD);
				SPI_SendByte(0);
				SPI_SendByte(0);
				SPI_SendByte(0);
				for(uint8_t i = 0; i < 4; i++)
				{
					usb_serial_putchar(SPI_ReceiveByte());
				}
				HWSPI_CS_HIGH;
				break;
				
			case CMD_DUMP_EEPROM:
				HWSPI_CS_LOW;

				SPI_SendByte(PS3_SC_EEPROM_READ_CMD);
				SPI_SendByte(0);
				SPI_SendByte(0);
			
				for (uint16_t k = 0; k < 0x8000; ++k) 
				{
					usb_serial_putchar(SPI_ReceiveByte());
				}

				HWSPI_CS_HIGH;
				break;
				
			case CMD_UNLOCK_EEPROM:
				HWSPI_CS_LOW;

				SPI_SendByte(PS3_SC_EEPROM_UNLOCK_CMD);
				SPI_SendByte(0);
				SPI_SendByte(0);

				HWSPI_CS_HIGH;
				break;		
				
			case CMD_WRITE_EEPROM:
				
				offset_high = usb_serial_getchar();
				offset_low = usb_serial_getchar();
				write_length = usb_serial_getchar();
				
				for(uint16_t i = 0; i < write_length; i++)
				{
					write_buffer[i] = usb_serial_getchar();
				}
				
			
				HWSPI_CS_LOW;

				SPI_SendByte(PS3_SC_EEPROM_WRITE_CMD);
				SPI_SendByte(offset_high);
				SPI_SendByte(offset_low);
				
				for(uint16_t i = 0; i < write_length; i++)
				{
					SPI_SendByte(write_buffer[i]);
				}
				

				HWSPI_CS_HIGH;
				break;	

			default:
				break;
			}
		}		
	}
}
