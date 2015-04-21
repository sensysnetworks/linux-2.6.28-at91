#ifndef _LCD447_H_
#define _LCD447_H_

#include <asm/types.h>

#define LCDMOD_VERSION "1.0.1-E"

/**
 * Setup the keymap
 */
#define CGRAM_DEFAULT
//#define CGRAM_SWEDISH

#define DFLT_DISP_ROWS	4	// Default number of rows the display has
#define DFLT_DISP_COLS	20	// Default number of columns the display has
#define TABSTOP		3	// Length of tabs

/**
 * Low level write/initialization interface
 * this should be expanded with ifdef's for different architectures/boards.
 ******************************************************************************/
#ifdef CONFIG_RDAC_CARRIER
#define PLDBASE		0x10000000
#else
#define PLDBASE 	0x50000000
#endif
#define LCDDREG		1
#define LCDCREG		0
#define RWDELAY		500
#define WWDELAY		5

//#define PLD_WRITE(index,data) *(volatile u8 *)(virt_addr+index)=(u8)data
//#define PLD_READ(index) ENDREV(index,*(volatile u16 *)(virt_addr+index))
// LCD Control Register bitmaps
#define RW_READ		1
#define RW_WRITE	0
#define RS_DATA		2
#define RS_INST		0
#define BL_ON		4
#define BL_OFF		0

// Stores the virtual address of the PLD (MMU friendly)
u8 *virt_addr;

static inline void PLD_WRITE(int index, u8 data)
{
	iowrite8(data, virt_addr + index);
}

static inline u8 PLD_READ(int index)
{
	return ioread8(virt_addr + index);
}

static inline void LCD_Command4(u8 command, int blight)
{
	PLD_WRITE(LCDCREG, (RW_WRITE | RS_INST | blight));
	udelay(RWDELAY);
	PLD_WRITE(LCDDREG, command);
}

/**
 *  Send a command to the LCD
 */
static inline void LCD_Command(u8 command, int blight)
{
	PLD_WRITE(LCDCREG, (RW_WRITE | RS_INST | blight));
	udelay(RWDELAY);
	PLD_WRITE(LCDDREG, (command >> 4));
	udelay(WWDELAY);
	PLD_WRITE(LCDDREG, (command & 0xf));
}

/**
 *  Send data to the LCD
 */
static inline void LCD_Data(u8 data, int blight)
{
	PLD_WRITE(LCDCREG, RW_WRITE | RS_DATA | blight);
	udelay(RWDELAY);
	PLD_WRITE(LCDDREG, (data >> 4));
	udelay(WWDELAY);
	PLD_WRITE(LCDDREG, (data & 0xf));
}

/**
 * Generic LCD bus initialization
 * Initializes the bus interface to the LCD
 * 
 */
static inline void LCD_BUSINIT(void)
{
	//do nothing, chipset parameters set at write time
	virt_addr = ioremap_nocache(PLDBASE, 0x2);
}

int lcd_major;
#define LCD_MINOR    	0

 /*****************************************************************************/

/*
 * Character mapping for HD44780 devices by Mark Haemmerling <mail@markh.de>.
 *
 * Translates ISO 8859-1 to HD44780 charset.
 * HD44780 charset reference: http://markh.de/hd44780-charset.png
 *
 * Initial table taken from lcd.o Linux kernel driver by
 * Nils Faerber <nilsf@users.sourceforge.net>. Thanks!
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 *
 * Following translations are being performed:
 * - maps umlaut accent characters to the corresponding umlaut characters
 * - maps other accent characters to the characters without accents
 * - maps beta (=ringel-S), micro and Yen
 *
 * Alternative mappings:
 * - #112 ("p") -> #240 (large "p"), orig. mapped -> #112
 * - #113 ("q") -> #241 (large "q"), orig. mapped -> #113
 *
 * HD44780 misses backslash
 *
 */
char charmap[] = {
/* #0 */
	0, 1, 2, 3,
	4, 5, 6, 7,
	8, 9, 10, 11,
	12, 13, 14, 15,
	16, 17, 18, 19,
	20, 21, 22, 23,
	24, 25, 26, 27,
	28, 29, 30, 31,
/* #32 */
	32, 33, 34, 35,
	36, 37, 38, 39,
	40, 41, 42, 43,
	44, 45, 46, 47,
	48, 49, 50, 51,
	52, 53, 54, 55,
	56, 57, 58, 59,
	60, 61, 62, 63,
/* #64 */
	64, 65, 66, 67,
	68, 69, 70, 71,
	72, 73, 74, 75,
	76, 77, 78, 79,
	80, 81, 82, 83,
	84, 85, 86, 87,
	88, 89, 90, 91,
/* #92 */
	47, 93, 94, 95,
	96, 97, 98, 99,
	100, 101, 102, 103,
	104, 105, 106, 107,
	108, 109, 110, 111,
	112, 113, 114, 115,
	116, 117, 118, 119,
	120, 121, 122, 123,
	124, 125, 126, 127,
/* #128 */
	128, 129, 130, 131,
	132, 133, 134, 135,
	136, 137, 138, 139,
	140, 141, 142, 143,
	144, 145, 146, 147,
	148, 149, 150, 151,
	152, 153, 154, 155,
	156, 157, 158, 159,
/* #160 */
	160, 33, 236, 237,
	164, 92, 124, 167,
	34, 169, 170, 171,
	172, 173, 174, 175,
	223, 177, 178, 179,
	39, 249, 247, 165,
	44, 185, 186, 187,
	188, 189, 190, 63,
/* #192 */
	65, 65, 65, 65,
	225, 65, 65, 67,
	69, 69, 69, 69,
	73, 73, 73, 73,
	68, 78, 79, 79,
	79, 79, 239, 120,
	48, 85, 85, 85,
	245, 89, 240, 226,
/* #224 */
	97, 97, 97, 97,
	225, 97, 97, 99,
	101, 101, 101, 101,
	105, 105, 105, 105,
	111, 110, 111, 111,
	111, 111, 239, 253,
	48, 117, 117, 117,
	245, 121, 240, 255
};

#ifdef CGRAM_DEFAULT
/* Default characters for lcdmod
 * 
 * Description: characters usefull for drawing graphs, and a funny 's'
 *              The character aren't mapped to any others.
 */

extern char charmap[];

static inline void init_charmap(void)
{
}

unsigned char cg0[] = { 0x1f, 0x1f, 0x11, 0x0f, 0x11, 0x1e, 0x01, 0x1f };
unsigned char cg1[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f };
unsigned char cg2[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f };
unsigned char cg3[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f };
unsigned char cg4[] = { 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f };
unsigned char cg5[] = { 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f };
unsigned char cg6[] = { 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f };
unsigned char cg7[] = { 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f };
#endif

#ifdef CGRAM_SWEDISH
/* Swedish characters for lcdmod
 *
 * Thanks to Erik Zetterberg <mr.z@linux.se>
 * 
 * Description: Adds support for the last three last letters in the
 * swedish alphabet (a/A with ring above, a/A with diaeresis and o/O
 * with diaeresis). And maps the location of where they should be 
 * according to the ISO-8859-1 character set to their location in CGRAM.
 *
 */

extern char charmap[];

static inline void init_charmap(void)
{
	charmap[0xe5] = 0;
	charmap[0xe4] = 1;
	charmap[0xf6] = 2;
	charmap[0xc5] = 3;
	charmap[0xc4] = 4;
	charmap[0xd6] = 5;
}

unsigned char cg0[] = { 0x04, 0x00, 0x0e, 0x01, 0x0f, 0x11, 0x0f, 0x00 };
unsigned char cg1[] = { 0x0a, 0x00, 0x0e, 0x01, 0x0f, 0x11, 0x0f, 0x00 };
unsigned char cg2[] = { 0x0a, 0x00, 0x0e, 0x11, 0x11, 0x11, 0x0e, 0x00 };
unsigned char cg3[] = { 0x04, 0x00, 0x0e, 0x11, 0x1f, 0x11, 0x11, 0x00 };
unsigned char cg4[] = { 0x0a, 0x00, 0x0e, 0x11, 0x1f, 0x11, 0x11, 0x00 };
unsigned char cg5[] = { 0x0a, 0x00, 0x0e, 0x11, 0x11, 0x11, 0x0e, 0x00 };
unsigned char cg6[] = { 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f };
unsigned char cg7[] = { 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f };
#endif

#endif //_LCD447_H_
