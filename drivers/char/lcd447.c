/****************************************************************************
 * lcd447.c 
 * Generic Character LCD driver for HD44780 
 * EMAC.Inc->NZG
 * January 8, 2007
 * 
 * based on the lcdmod project:
 * Copyright (C) by Michael McLellan (mikey@mclellan.org.nz)
 * 
 * Released under the terms of the GNU GPL, see file COPYING for more details.
 * 
 * NZG modifications:
 * 
 * Renamed everything lcd447 so as not to conflict with the strange "Cobalt" driver
 * that has somehow made it's way into the 2.6 kernel and named itself lcd.c
 * 
 * Added BSD licensing to the license (ala Rubini's drivers) 
 * not really sure what this does for us though.
 * 
 * Stripped down to provide LCD support using a macro for data write.control write.
 * bus interface code provided by in-lines in the header.
 * 
 * Cleaned to support only the 2.6 kernel, to eventually integrate into main tree
 * 
 * Currently supports only 1 controller.
 *  This may need to be put back in, but should probably be done with in-lines
 * as heavy ifdefs are not generally mergeable into the main tree.
 * 
 * Merged all .h's into a singe .h, elcd.h which stands for EMAC LCD, as it provides
 * the bus interface functions to the LCD.
 * 
 * inlined write_command since it's pretty small.
 * 
 * fixed bug preventing first word written from being readable by proc
 * 
 * Added LCDDEBUG optional debug statements
 * 
 * Moved MODULE_PARM to MODULE_PARM_DESC to support newer kernels
 * 
 ****************************************************************************/

#include <linux/version.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include "lcd447.h"

MODULE_LICENSE("Dual BSD/GPL");

#define MAX_DISP_ROWS	4	// The HD44780 supports up to 4 rows
#define MAX_DISP_COLS	40	// The HD44780 supports up to 40 columns

/* input_state states */
#define NORMAL		0
#define ESC		1	// Escape sequence start
#define DCA_Y		2	// Direct cursor access, the next input will be the row
#define DCA_X		3	// Direct cursor access, the next input will be the column
#define CGRAM_SELECT	4	// Selecting which slot to enter new character
#define CGRAM_ENTRY	5	// Adding a new character to the CGRAM
#define CGRAM_GENERATE	6	// Waiting fot the 8 bytes which define a character
#define CHAR_MAP_OLD	7	// Waiting for the original char to map to another
#define CHAR_MAP_NEW	8	// Waiting for the new char to replace the old one with
#define ESC_		10	// Waiting for the [ in escape sequence

/**
 * HD44780 Commands and bitflags within them
 */
#define FUNCTION_SET 	0x20
#define DATA_8BIT  	0x10
#define DISPLAY_2LINES 	0x08
//#define DISPLAY_ON    0x0C
#define DISPLAY_ON 	0x0D
#define DISPLAY_OFF	0x08
#define CLEAR_DISPLAY	0x01
#define ENTRY_MODE_SET  0x04
#define INC_RAM		0x02
#define RETURN_HOME 	0x02

static int disp_rows = DFLT_DISP_ROWS;
static int disp_cols = DFLT_DISP_COLS;
static unsigned char state[MAX_DISP_ROWS][MAX_DISP_COLS];	// The current state of the display
static int disp_row = 0, disp_column = 0;	// Current actual cursor position
static int row = 0, column = 0;	// Current virtual cursor position
static int wrap = 0;		// Linewrap on/off
static char backlight = BL_ON;	// Backlight on-off

static struct cdev *my_cdev;	// character device

//#define LCD_DEBUG
#ifdef LCD_DEBUG
#define LCDDEBUG(fmt, args...) printk(fmt, ## args)
#else
#define LCDDEBUG(fmt, args...)
#endif

//backwards compatibility 
#if defined(_LINUX_BLKDEV_H) && LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
#define MODULE_PARM_DESC(var, name) MODULE_PARM( var, name )
#endif

MODULE_DESCRIPTION("General LCD driver for HD44780 compatable controllers");
MODULE_AUTHOR
("Michael McLellan <mikey@mclellan.org.nz>,NZG <ngustavson@emacinc.com>");
MODULE_PARM_DESC(disp_rows, "i");
MODULE_PARM_DESC(disp_cols, "i");

MODULE_PARM_DESC(disp_rows,
		"LCD rows (default: DFLT_DISP_ROWS, max: MAX_DISP_ROWS)");
MODULE_PARM_DESC(disp_cols,
		"LCD columns (default: DFLT_DISP_COLS, max: MAX_DISP_COLS)");
MODULE_PARM_DESC(lowvoltage, "LCD lowvoltage display bootean true/false - 1/0");

/**
 *  Send an instruction to the display, e.g. move cursor
 */
static void writeCommand(u8 command)
{
	LCDDEBUG("command = %x\n", command);
	LCD_Command(command, backlight);
	/* Minimum time to wait after most commands is 39ns except Clear Display
	 * and Return Home which is 1.53ms, we never use Return Home.
	 */
	if (command <= 0x03)
		mdelay(1.6);
	else
		udelay(40);
}

static void writeCommand4(u8 command)
{
	LCDDEBUG("command = %x\n", command);
	LCD_Command4(command, backlight);
	/* Minimum time to wait after most commands is 39ns except Clear Display
	 * and Return Home which is 1.53ms, we never use Return Home.
	 */
	/* FIXME: these delays (and others) should be defines... can we really
	 * have mdelay(float)? */
	if (command <= 0x03)
		mdelay(1.6);
	else
		udelay(40);
}

/* Send character data to the display, i.e. writing to DDRAM */
static void writeData(u8 data)
{
	LCDDEBUG("row=%x, column = %x\n", row, column);

	/* check and see if we really need to write anything */
	if (state[row][column] != data) {
		state[row][column] = data;
		/* set the cursor position if need be.
		 * Special case for 16x1 displays, They are treated as two
		 * 8 charcter lines side by side, and dont scroll along to
		 * the second line automaticly.
		 */
		if ((disp_row != row) || (disp_column != column) ||
				((disp_rows == 1) && (disp_cols == 16) && (column == 8))) {
			LCDDEBUG("disp!=reg , resetting cursor\n");
			/* Some transation done here so 4 line displays work */
			/* FIXME: Ugly nested tertiary statements --
			 *        impossible to read */
			writeCommand(((row >= 2) ? ((row - 2) * 0x40) : (row * 0x40)) |
					((row >= 2) ? (column + disp_cols) : column) | 0x80);

			disp_row = row;
			disp_column = column;
		}

		LCD_Data(data, backlight);

		udelay(250);
		/* Time to wait after write to RAM is 43ns */
		udelay(43);
		disp_column++;
	}
	if (column < (disp_cols - 1))
		column++;
	else if (wrap && (column == (disp_cols - 1)) && (row < (disp_rows - 1))) {
		column = 0;
		row++;
	}
}

/* Write an entire (5x8) character to the CGRAM,
 * takes the CGRAM index, and a char[ 8 ] binary bitmap.
 */
static void writeCGRAM(int index, u8 pixels[])
{
	int i;

	/* Move address pointer to index in CGRAM */
	/* FIXME: these command values (and others) should be defines */
	writeCommand(0x40 + (8 * index));

	for (i = 0; i < 8; i++)
		LCD_Data(pixels[i], backlight);

	udelay(250);
	/* Time to wait after write to RAM is 43 ns */
	udelay(45);

	/*set disp position as being different than row, to be reset by writeData */
	disp_row = disp_column = -1;
}

static void initDisplay(void)
{

	/* initialize state array */
	memset(state, ' ', sizeof(state));
	/* initialize controller 1 */
	//writeCommand(FUNCTION_SET|DATA_8BIT);
	writeCommand4(0x02);
	mdelay(5);
	//writeCommand(FUNCTION_SET|DATA_8BIT);
	writeCommand(FUNCTION_SET);
	udelay(100);
	//writeCommand(FUNCTION_SET|DATA_8BIT|DISPLAY_2LINES);
	writeCommand(FUNCTION_SET | DISPLAY_2LINES);

	writeCommand(DISPLAY_OFF);
	writeCommand(CLEAR_DISPLAY);
	writeCommand(ENTRY_MODE_SET | INC_RAM);
	writeCommand(RETURN_HOME);
	writeCommand(DISPLAY_ON);

	//      /* Set the CGRAM to default values */
	//      writeCGRAM( 0, cg0 );
	//      writeCGRAM( 1, cg1 );
	//      writeCGRAM( 2, cg2 );
	//      writeCGRAM( 3, cg3 );
	//      writeCGRAM( 4, cg4 );
	//      writeCGRAM( 5, cg5 );
	//      writeCGRAM( 6, cg6 );
	//      writeCGRAM( 7, cg7 );
	//      init_charmap();
}

static void handleInput(unsigned char input)
{
	static int cgram_index = 0;
	static int cgram_row_count;
	static unsigned char cgram_pixels[8];
	static unsigned char char_map_old;
	static int input_state = NORMAL;	// the current state of the input handler
	int i;
	int j;
	int temp;

	LCDDEBUG("handling input 0x%x\n", input);

	if (input_state == NORMAL) {
		switch (input) {
		case 0x08:	// Backspace
			if (column > 0) {
				column--;
				writeData(' ');
				column--;
			}
			break;
		case 0x09:	// Tabstop
			column =
				(((column + 1) / TABSTOP) * TABSTOP) + TABSTOP - 1;
			break;
		case 0x0a:	// Newline
			if (row < (disp_rows - 1))
				row++;
			else {
				/* scroll up */
				LCDDEBUG("scroll\n");
				temp = column;
				for (i = 0; i < (disp_rows - 1); i++) {
					row = i;
					for (j = 0; j < disp_cols; j++) {
						column = j;
						writeData(state[i + 1][j]);
					}
				}
				row = disp_rows - 1;
				column = 0;
				for (i = 0; i < disp_cols; i++) {
					writeData(' ');
				}
				column = temp;
			}
			/* Since many have trouble grasping the \r\n concept... */
			column = 0;
			break;
		case 0x0d:	// Carrage return
			column = 0;
			break;
		case 0x1b:	// esc ie. start of escape sequence
			input_state = ESC_;
			break;
		default:
			/* The character is looked up in the */
			writeData(charmap[input]);
		}
	} else if (input_state == ESC_) {
		input_state = ESC;
	} else if (input_state == ESC) {
		if ((input <= '7') && (input >= '0')) {
			/* Chararacter from CGRAM */
			writeData(input - 0x30);
		} else {
			switch (input) {
			case 'A':	// Cursor up
				if (row > 0)
					row--;
				break;
			case 'B':	// Cursor down
				if (row < (disp_rows - 1))
					row++;
				break;
			case 'C':	// Cursor Right
				if (column < (disp_cols - 1))
					column++;
				break;
			case 'D':	// Cursor Left
				if (column > 0)
					column--;
				break;
			case 'H':	// Cursor home
				row = 0;
				column = 0;
				break;
			case 'J':	// Clear screen, cursor doesn't move
				memset(state, ' ', sizeof(state));
				writeCommand(0x01);
				break;
			case 'K':	// Erase to end of line, cursor doesn't move
				temp = column;
				for (i = column; i < disp_cols; i++)
					writeData(' ');
				column = temp;
				break;
			case 'M':	// Charater mapping
				input_state = CHAR_MAP_OLD;
				break;
			case 'Y':	// Direct cursor access
				input_state = DCA_Y;
				break;
			case 'R':	// CGRAM select
				input_state = CGRAM_SELECT;
				break;
			case 'V':	// Linewrap on
				wrap = 1;
				break;
			case 'W':	// Linewrap off
				wrap = 0;
				break;
			case 'b':	// Toggle backlight
				backlight =
					(backlight == BL_OFF ? BL_ON : BL_OFF);
				break;
			default:
				printk
					("LCD: unrecognized escape sequence: %#x ('%c')\n",
					 input, input);
			}
		}
		if ((input_state != DCA_Y) &&
				(input_state != CGRAM_SELECT) &&
				(input_state != CHAR_MAP_OLD)) 
		{
			input_state = NORMAL;
		}
	} else if (input_state == DCA_Y) {
		if ((input - 0x1f) < disp_rows)
			row = input - 0x1f;
		else {
			printk
				("LCD: tried to set cursor to off screen location\n");
			row = disp_rows - 1;
		}
		input_state = DCA_X;
	} else if (input_state == DCA_X) {
		if ((input - 0x1f) < disp_cols)
			column = input - 0x1f;
		else {
			printk
				("LCD: tried to set cursor to off screen location\n");
			column = disp_cols - 1;
		}
		input_state = NORMAL;
	} else if (input_state == CGRAM_SELECT) {
		if ((input > '7') || (input < '0')) {
			printk("LCD: Bad CGRAM index %c\n", input);
			input_state = NORMAL;
		} else {
			cgram_index = input - 0x30;
			cgram_row_count = 0;
			input_state = CGRAM_GENERATE;
		}
	} else if (input_state == CGRAM_GENERATE) {
		cgram_pixels[cgram_row_count++] = input;
		if (cgram_row_count == 8) {
			writeCGRAM(cgram_index, cgram_pixels);
			input_state = NORMAL;
		}
	} else if (input_state == CHAR_MAP_OLD) {
		char_map_old = input;
		input_state = CHAR_MAP_NEW;
	} else if (input_state == CHAR_MAP_NEW) {
		charmap[char_map_old] = input;
		input_state = NORMAL;
	}
}

int lcd_open(struct inode *minode, struct file *mfile)
{
	return (0);
}

/* Handle device file close */
int lcd_release(struct inode *minode, struct file *mfile)
{
	return 0;
}

/* Handle write to device file */
ssize_t lcd_write_byte(struct file * inode, const char *gdata, size_t length,
		loff_t * off_what)
{
	int i;
	for (i = 0; i < length; i++)
		handleInput(gdata[i]);
	return (length);
}

/* Handle read from device file */
ssize_t lcd_read_byte(struct file * inode, char *udata, size_t length,
		loff_t * loff_what)
{
	return (EACCES);
}

/* Handle read from proc file */
int readProcFile(char *buffer, char **start, off_t offset, int size, int *eof,
		void *data)
{
	char *temp = buffer;
	int i, j;

	/* Print module configuration */
	temp += sprintf(temp, "Display rows:    %d\n"
			"Display columns: %d\n"
			"Linewrap:        %s\n",
			disp_rows, disp_cols, wrap ? "On" : "Off");

	/* Print display state */
	temp += sprintf(temp, "+");
	for (i = 0; i < disp_cols; i++)
		temp += sprintf(temp, "-");
	temp += sprintf(temp, "+\n");

	for (i = 0; i < disp_rows; i++) {
		temp += sprintf(temp, "|");
		for (j = 0; j < disp_cols; j++)
			temp +=
				sprintf(temp, "%c",
						(state[i][j] < 10) ? '?' : state[i][j]);
		temp += sprintf(temp, "|\n");
	}

	temp += sprintf(temp, "+");
	for (i = 0; i < disp_cols; i++)
		temp += sprintf(temp, "-");
	temp += sprintf(temp, "+\n");
	return temp - buffer;
}

/* Module cleanup */
static void __exit exit_module(void)
{
	dev_t dev = MKDEV(lcd_major, LCD_MINOR);
	writeCommand(0x01);
	remove_proc_entry("lcd", 0);
	printk("LCD: deleting char device %d\n", dev);
	unregister_chrdev_region(dev, 1);
	cdev_del(my_cdev);
}

/* Module initilisation */
static int __init start_module(void)
{
	dev_t dev = MKDEV(0, 0);
	int result;

	static struct file_operations lcd_fops = {
		.read = lcd_read_byte,
		.write = lcd_write_byte,
		.open = lcd_open,
		.release = lcd_release
	};

	/* Dynamically create device node */
	result = alloc_chrdev_region(&dev, LCD_MINOR, 1, "lcd");
	lcd_major = MAJOR(dev);

	/* Make sure user didn't pass silly numbers, MAX_DISP_???? are just
	 * arbitary numbers and can be increased if need be.
	 */
	disp_rows = (disp_rows <= MAX_DISP_ROWS) ? disp_rows : MAX_DISP_ROWS;
	disp_cols = (disp_cols <= MAX_DISP_COLS) ? disp_cols : MAX_DISP_COLS;

	if (result < 0) {
		printk("LCD: char device register failed\n");
		return -1;
	} else {
		printk("LCD: char device registered\n");
	}

	my_cdev = cdev_alloc();
	my_cdev->owner = THIS_MODULE;
	my_cdev->ops = &lcd_fops;
	kobject_set_name(&my_cdev->kobj, "lcd");

	if (cdev_add(my_cdev, dev, 1)) {
		printk("LCD char device add failed\n");
		return -1;
	} else
		printk("LCD: char device added\n");

	/* creates sysfs interface and allows device node to be created dynamically */
	device_create(class_create(THIS_MODULE, "lcd"), NULL, dev, NULL, "lcd");

	/* setup the LCD bus interface */
	LCD_BUSINIT();
	/* initialize the display controller */
	initDisplay();

	if (!create_proc_read_entry("lcd", 0, 0, readProcFile, NULL)) {
		printk(KERN_ERR "LCD: Error! Can't create /proc/lcd\n");
		return -ENOMEM;
	}

	printk("LCD: init OK,  rows: %d, columns: %d\n", disp_rows, disp_cols);

	return 0;
}

module_init(start_module);
module_exit(exit_module);

