#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#define DRIVER_NAME	"keypad"

#define KEYROW		0x03
#define KEYCOLUMN	0x70
#define KEYGATE		0x04
#define ROWCOLMASK	(KEYROW|KEYCOLUMN)

#define SLEEP_FRACTION 	50
#define REPEAT_DELAY	SLEEP_FRACTION	//in units of HZ/SLEEP_FRACTION

#define FIRSTKEYMINOR	0
#define KEYMINORNUM	3

#if (HZ/SLEEP_FRACTION)
#define TIMEOUT  (HZ/SLEEP_FRACTION)
#else
#define TIMEOUT 1
#endif

#define ROW(pos)		((~pos)&KEYROW)
#define COLUMN(pos) 		(pos>>4)
#define KEYPAD_ROWS		4
#define KEYPAD_COLUMNS  	6
#define DRIVERCODE 		'K'
#define SETKEYARRAY		_IOR(DRIVERCODE,0,char)
#define GETKEYARRAY 		_IOW(DRIVERCODE,1,char)
#define KEYPAD_UNDEFINED_CHAR	'x'

#define KEYPAD_BUFSIZE		8

/**
 * struct defining a keypad device
 * @buf: buffer storing the keypad data
 * @bufhead: location of data queue head
 * @bufcount: number of data bytes in buf
 * @isopen: flag, true if device is currently open
 * @blocking: flag, true if open in blocking mode
 * @countdown: countdown timer used for timeout purposes
 * @intimer: flag, true if the timer is currently active
 * @data: keypad_data_s passed through platform_device data
 * @cdev: the character device associated with the keypad
 * @dev: pointer to the associated platform device
 * @wait_queue: queue head used to wait for keypad events
 * @poll_timer: timer struct for poll function
 * @matrix: key matrix associated with this device
 * @lock: synchronizes device access in critical sections
 */
struct keypad_dev {
	u8 buf[KEYPAD_BUFSIZE];
	int bufhead;
	int bufcount;
	int isopen;
	int blocking;
	int countdown;
	int intimer;
	struct keypad_data_s *data;
	struct cdev cdev;
	struct device *dev;
	wait_queue_head_t wait_queue;
	struct timer_list poll_timer;
	char matrix[KEYPAD_ROWS * KEYPAD_COLUMNS];
	struct mutex lock;
};

static inline u8 _keypad_read(struct keypad_dev *k)
{
	return (ioread8(k->data->controller_base));
}

static inline u8 getkeypos(struct keypad_dev *k)
{
	return (_keypad_read(k) & ROWCOLMASK);
}

static inline void restart_timer(struct keypad_dev *k)
{
	k->poll_timer.expires = jiffies + TIMEOUT;
	add_timer(&k->poll_timer);
	k->intimer = 1;
}

static inline void timer_exit(struct keypad_dev *k)
{
	if (k->data->polling)
		restart_timer(k);
	else
		k->data->unmask_int(k->data);
	k->intimer = 0;
}

#define inode_to_keypad(i) container_of(i->i_cdev, struct keypad_dev, cdev)

#endif /* _KEYPAD_H_ */
