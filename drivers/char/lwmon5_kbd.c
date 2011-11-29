/***********************************************************************
 *
M* Modul:         lwmon_kbd.c
M*
M* Content:       Linux kernel driver for the LWMON keyboard.
 *
 * (C) Copyright 2001, 2002
 * DENX Software Engineering
 * Wolfgang Denk, wd@denx.de
 * All rights reserved.
 *
D* Design:        wd@denx.de
C* Coding:        wd@denx.de
V* Verification:  dzu@denx.de
 *
 ***********************************************************************/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>	/* For (un)lock_kernel */
#include <linux/input.h>	/* interface to input subsystem */
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/fs.h>		/* character device definitions */

#include <asm/uaccess.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <asm/semaphore.h>

#define DRV_NAME "lwmon5_kbd"
#define DRV_VERSION "1.2"

/*--------------------- Local constants and macros --------------------*/

#undef	DEBUG

#ifdef	DEBUG
# define debugk(fmt,args...)	printk(fmt ,##args)
#else
# define debugk(fmt,args...)
#endif

#ifndef KBD_MAJOR
#define KBD_MAJOR	11	/* Raw Keyboard Device	*/
#endif	/* KBD_MAJOR */

/*
 * We implement 3 minor devices:
 *
 * min 0 provides "normal" keyboard data which can be read by any
 *       tool like "cat" or "xd" etc. For each key press exactly one
 *       character can be read from the device. Because of the
 *       special requirements of the keybrd control application no
 *       autorepeat is implemented here.
 *
 * min 1 provides a special interface for the contol application. It
 *       is optimized to provide exact timing information for the
 *       duration of each key press. For each "key event" _two_ bytes
 *       of data can be read from the device: the keycode, and a byte
 *       of "modifiers" which includes information about SHIFT and
 *       CONTROL status and if this was a key PRESS or a key RELEASE
 *       event. For each key press two events are generated: the
 *       first one when the key is pressed, and the second one when
 *       it is released.
 *
 * min 2 provides access to the digital i/o pins sent together
 *       with the modifier keys.  The relevant bits are copied without
 *       modification, i.e. the bit-mask for the three pins is 0xD0.
 */

#define MINOR_COOKED 0
#define MINOR_RAW    1
#define MINOR_DIN    2

#define KBD_DIN_MASK 0xF0	/* Mask for digital in pins (incl. key) */

#define KBD_MINOR	3	/* allow for 3 minor devices */
int	kbd_dev_flags[KBD_MINOR];

#define KBD_DEV_OPEN	1

/*----------------------- Interface prototypes ------------------------*/
static int	kbd_open     (struct inode *, struct file *);
static int	kbd_release  (struct inode *, struct file *);
static ssize_t	kbd_read     (struct file *, char *, size_t, loff_t *);
static unsigned int kbd_poll (struct file *, poll_table *);

/*------------------------- Driver interface --------------------------*/
static struct file_operations kbd_fops = {
	owner:		THIS_MODULE,
	open:		kbd_open,
	release:	kbd_release,
	read:		kbd_read,
	poll:		kbd_poll,
};

/*--------------------- Local macros and static vars ------------------*/
/*
 * Keyboard Controller
 *
 * command codes
 */
#define KEYBD_CMD_READ_KEYS	0x01
#define KEYBD_CMD_READ_VERSION	0x02
#define KEYBD_CMD_READ_STATUS	0x03
#define KEYBD_CMD_RESET_ERRORS	0x10

/* Number of bytes returned from Keyboard Controller */
#define KEYBD_VERSIONLEN	2	/* version information */
#define KEYBD_DATALEN		6	/* normal key scan data */

#define LWMON_KBD_MAX_MINOR	3

/*
 * Input ring buffer
 * Holds two bytes of data per key event (modifiers + keycode).
 * first == last ==> RB empty.
 * first == (last+1) mod size ==> RB full.
 */
#define RBUF_SIZE 256			/* Ring buffer size */
static unsigned char	rbuf_data[LWMON_KBD_MAX_MINOR][2*RBUF_SIZE];
static volatile unsigned short rbuf_first[LWMON_KBD_MAX_MINOR];
static volatile unsigned short rbuf_last[LWMON_KBD_MAX_MINOR];
static int curr_din;		        /* Current DIN data */
static spinlock_t rbuf_lock;
static DECLARE_WAIT_QUEUE_HEAD(rbuf_wait);
static void rbuf_add (int minor, unsigned char keycd, unsigned char mod);
static int  rbuf_get (int minor, unsigned int n, unsigned char *buf);


typedef struct {
	struct task_struct *thread;	/* Linux task structure of thread */
	struct semaphore    sem;	/* semaphore for thread control */
	wait_queue_head_t   queue;	/* queue thread is waiting on */
	volatile unsigned int pend;	/* key event pending */
} kbd_dev_t;

kbd_dev_t kbd_dev;

#define KBD_INTERRUPT		65	/* EXT_IRQ6 on PPC */

#define LWMON5_KBD_I2C_ADDR	0xac

static unsigned short normal_i2c[] = {
	LWMON5_KBD_I2C_ADDR >> 1,
	I2C_CLIENT_END
};

I2C_CLIENT_INSMOD;			/* defines addr_data */

static struct i2c_client *my_client;

static int lwmon5_kbd_attach_adapter(struct i2c_adapter *adapter);
static int lwmon5_kbd_detach_client(struct i2c_client *client);
static int lwmon5_kbd_probe(struct i2c_adapter *adapter, int addr, int kind);

static void lwmon_kbd_cleanup(void);

/* This is the driver that will be inserted */
static struct i2c_driver lwmon5_kbd_driver = {
	.driver = {
		.name	= "lwmon5_kbd",
	},
	.id		= I2C_DRIVERID_LWMON5_KBD,
	.attach_adapter	= lwmon5_kbd_attach_adapter,
	.detach_client	= lwmon5_kbd_detach_client,
};

static int lwmon5_kbd_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, lwmon5_kbd_probe);
}

static int lwmon5_kbd_detach_client(struct i2c_client *client)
{
	lwmon_kbd_cleanup();
	return 0;
}

/*
 * The name for our device, as it will appear in /proc/devices
 */
#define DEVICE_NAME	DRV_NAME

/*
 * /proc filesystem definitions
 */
struct proc_dir_entry *proc_lwmon_kbd;	/* root for lwmon_kbd files	*/
struct proc_dir_entry *kbd_din_f;        /* digital in			*/
static int kbd_din_f_read_proc (char *, char **, off_t, int, int *, void *);


/*
 * We implement 2 minor devices for the keyboard data:
 *
 * minor 0: This device will be used by	 the  LICCON  software	which
 *	    needs  exact  timing  information  since keys are used to
 *	    control hardware;  each  key  event	 (key  press  or  key
 *	    release)  will  be reported to the application as 2 bytes
 *	    of data: the first byte contains key status	 data  (press
 *	    or	release	 event, shift, control and error status), and
 *	    the second byte the decoded key code.
 *
 * minor 1: This will be used mainly for testing and in case  a	 more
 *	    conventional  application  needs  to access the keyboard:
 *	    only the decoded keycodes are sent to the application.
 *
 * The keyboard driver buffers key events if  needed,  but  then  all
 * timing information gets lost; also, no key repeat is performed.
 *
 * Implemented Keyboard Map:
 *
 *     Key   Unshifted	Shift	Ctrl  Shift+Ctrl      HW-Code	Algo.
 *     --------------------------------------------------------------
 *	F1	0x80	0xA0	0x90	0xB0		0x3A	0
 *	F2	0x81	0xA1	0x91	0xB1		0x3B	0
 *	F3	0x82	0xA2	0x92	0xB2		0x3C	0
 *	F4	0x83	0xA3	0x93	0xB3		0x3D	0
 *	F5	0x84	0xA4	0x94	0xB4		0x3E	0
 *	F6	0x85	0xA5	0x95	0xB5		0x3F	0
 *	F7	0x86	0xA6	0x96	0xB6		0x40	0
 *	F8	0x87	0xA7	0x97	0xB7		0x41	0
 *
 *	<	0x71	0x51	0x11	0xF1		0x50	1
 *	>	0x72	0x52	0x12	0xF2		0x4F	1
 *
 *	ENTER	0x0D	0xAD	0x9D	0xBD		0x28	4
 *
 *	P0	0x6D	0x4D	0x0D	0xED		0x10	1
 *	P1	0x61	0x41	0x01	0xE1		0x04	1
 *	P2	0x62	0x42	0x02	0xE2		0x05	1
 *	P3	0x6C	0x4C	0x0C	0xEC		0x0F	1
 *	P4	0x64	0x44	0x04	0xE4		0x08	1
 *	P5	0x65	0x45	0x05	0xE5		0x09	1
 *	P6	0x66	0x46	0x06	0xE6		0x07	1
 *	P7	0x73	0x53	0x13	0xF3		0x16	1
 *	P8	0x74	0x54	0x14	0xF4		0x17	1
 *
 *	P	0x70	0x50	0x10	0xF0		0x13	1
 *	.	0x2E	0x3E	0x1E	0xFE		0x37	2
 *
 *	0	0x30	0x20	0x00	0xD0		0x27	3
 *	1	0x31	0x21	0x01	0xD1		0x1E	3
 *	2	0x32	0x22	0x02	0xD2		0x1F	3
 *	3	0x33	0x23	0x03	0xD3		0x20	3
 *	4	0x34	0x24	0x04	0xD4		0x21	3
 *	5	0x35	0x25	0x05	0xD5		0x22	3
 *	6	0x36	0x26	0x06	0xD6		0x23	3
 *	7	0x37	0x27	0x07	0xD7		0x24	3
 *	8	0x38	0x28	0x08	0xD8		0x25	3
 *	9	0x39	0x29	0x09	0xD9		0x26	3
 *
 * Unfortunately, this keyboard map is VERY  irregular;	 to  minimize
 * memory footprint (for code tables) and CPU (for decoding) we store
 * a  key map that maps the HW code (as read from the PIC controller)
 * into the _Shift+Ctrl_ key  code,  plus  a  "algorithm  code".  The
 * following algorithms are used:
 *
 * Algo 0:	Unshifted = Shift+Ctrl - 0x30
 *		Shift	  = Shift+Ctrl - 0x10
 *		Ctrl	  = Shift+Ctrl - 0x20
 *
 *		Used for: F1 F2 F3 F4 F5 F6 F7 F8
 *
 * Algo 1:	Unshifted = Shift+Ctrl - 0x80
 *		Shift	  = Shift+Ctrl - 0xA0
 *		Ctrl	  = Shift+Ctrl - 0xE0
 *
 *		Used for: < > P0 P1 P2 P3 P4 P5 P6 P7 P8 P
 *
 * Algo 2:	Unshifted = Shift+Ctrl - 0xD0
 *		Shift	  = Shift+Ctrl - 0xC0
 *		Ctrl	  = Shift+Ctrl - 0xE0
 *
 *		Used for: .
 *
 * Algo 3:	Unshifted = Shift+Ctrl - 0xA0
 *		Shift	  = Shift+Ctrl - 0xB0
 *		Ctrl	  = Shift+Ctrl - 0xD0
 *
 *		Used for: 0 1 2 3 4 5 6 7 8 9
 *
 * Algo 4:	Unshifted = Shift+Ctrl - 0xB0
 *		Shift	  = Shift+Ctrl - 0x10
 *		Ctrl	  = Shift+Ctrl - 0x20
 *
 *		Used for: ENTER
 *
 */


static unsigned char kbd_map [] = {
/*	   HW Code  Shift+Ctrl Algo        Key	*/
	[ 2 * 0x04 ] = 0xE1,	1,	/* P1	*/
	[ 2 * 0x05 ] = 0xE2,	1,	/* P2	*/
	[ 2 * 0x07 ] = 0xE6,	1,	/* P6	*/
	[ 2 * 0x08 ] = 0xE4,	1,	/* P4	*/
	[ 2 * 0x09 ] = 0xE5,	1,	/* P5	*/
	[ 2 * 0x0F ] = 0xEC,	1,	/* P3	*/
	[ 2 * 0x10 ] = 0xED,	1,	/* P0	*/
	[ 2 * 0x13 ] = 0xF0,	1,	/* P	*/
	[ 2 * 0x16 ] = 0xF3,	1,	/* P7	*/
	[ 2 * 0x17 ] = 0xF4,	1,	/* P8	*/
	[ 2 * 0x1E ] = 0xD1,	3,	/* 1	*/
	[ 2 * 0x1F ] = 0xD2,	3,	/* 2	*/
	[ 2 * 0x20 ] = 0xD3,	3,	/* 3	*/
	[ 2 * 0x21 ] = 0xD4,	3,	/* 4	*/
	[ 2 * 0x22 ] = 0xD5,	3,	/* 5	*/
	[ 2 * 0x23 ] = 0xD6,	3,	/* 6	*/
	[ 2 * 0x24 ] = 0xD7,	3,	/* 7	*/
	[ 2 * 0x25 ] = 0xD8,	3,	/* 8	*/
	[ 2 * 0x26 ] = 0xD9,	3,	/* 9	*/
	[ 2 * 0x27 ] = 0xD0,	3,	/* 0	*/
	[ 2 * 0x28 ] = 0xBD,	4,	/* ENTER*/
	[ 2 * 0x37 ] = 0xFE,	2,	/* .	*/
	[ 2 * 0x3A ] = 0xB0,	0,	/* F1	*/
	[ 2 * 0x3B ] = 0xB1,	0,	/* F2	*/
	[ 2 * 0x3C ] = 0xB2,	0,	/* F3	*/
	[ 2 * 0x3D ] = 0xB3,	0,	/* F4	*/
	[ 2 * 0x3E ] = 0xB4,	0,	/* F5	*/
	[ 2 * 0x3F ] = 0xB5,	0,	/* F6	*/
	[ 2 * 0x40 ] = 0xB6,	0,	/* F7	*/
	[ 2 * 0x41 ] = 0xB7,	0,	/* F8	*/
	[ 2 * 0x4F ] = 0xF2,	1,	/* >	*/
	[ 2 * 0x50 ] = 0xF1,	1,	/* <	*/
};

/* Keycodes for the modifier keys */
#define KBD_KEYCODE_SHIFT       0x7E
#define KBD_KEYCODE_CONTROL     0x7F

#define KBD_MOD_CONTROL		0x01		/* Control */
#define KBD_MOD_SHIFT		0x02		/* Shift */
#define KBD_MOD_PRESS		0x10		/* Key press(1)/release(0) */
#define KBD_MOD_KEY             0x20            /* Physical key */
#define KBD_MOD_ERROR		0x80		/* Error condition */

#define KBD_ERR_ROLLOVR		0x01		/* Error RollOver */
#define KBD_ERR_POST		0x02		/* POST Fail */
#define KBD_ERR_UNDEF		0x03		/* Undefined Error */
#define KBD_ERR_MAX	(KBD_ERR_UNDEF + 1)

static unsigned char kbd_mod_state;
static unsigned char kbd_key_state [sizeof(kbd_map)/2];
static unsigned int nkey = sizeof(kbd_map) / 2;

/*------------------------ Local prototypes ---------------------------*/
static	void	lwmon_readkeys	(void);
static unsigned char kbd_decode (unsigned char, unsigned char);
static	int	keyboard_thread (void *dev_arg);
static	void	setup_thread	(kbd_dev_t *dev);
static	void	leave_thread	(kbd_dev_t *dev);

/*---------------------- Published prototypes -------------------------*/
int	init_module(void);
void	cleanup_module(void);

MODULE_AUTHOR("Wolfgang Denk, wd@denx.de");
MODULE_DESCRIPTION("Keyboard Driver for LWMON Project");

/***********************************************************************
F* Function:     static int kbd_open (struct inode *inode, struct file *file) P*A*Z*
 *
P* Parameters:   struct inode *inode
P*                - Inode of the device file being opened
P*               struct file *file
P*                - Passed by the kernel, but not used
P*
P* Returnvalue:  int - 0 success
P*                    <0 Errorcondition, which can be
P*                     -ENXIO  Illegal minor device number
P*                     -EBUSY  The device was already opened by another
P*                             process
 *
Z* Intention:    This function is called by the kernel when a device file
Z*               for the driver is opened by open(2).
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static int kbd_open (struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);

	debugk ("kbd_open: minor %d\n", minor);

	if (minor >= KBD_MINOR)
		return (-ENXIO);

	if (kbd_dev_flags[minor] & KBD_DEV_OPEN)
		return (-EBUSY);

	kbd_dev_flags[minor] |= KBD_DEV_OPEN;

	rbuf_first[minor]=rbuf_last[minor]=0;

	return (0);
}

/***********************************************************************
F* Function:     static int kbd_release (struct inode *inode, struct file *file) P*A*Z*
 *
P* Parameters:   struct inode *inode
P*                - Inode of the device file being closed
P*               struct file *file
P*                - Passed by the kernel, but not used
P*
P* Returnvalue:  int - 0 => success
P*                    <0 Errorcondition, which can be
P*                     -ENXIO  Illegal minor device number
 *
Z* Intention:    This function is called by the kernel when a device file
Z*               of the driver is closed with close(2).
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static int kbd_release (struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);

	debugk ("kbd_release: minor %d\n", minor);

	if (minor >= KBD_MINOR)
		return (-ENXIO);

	kbd_dev_flags[minor] &= ~KBD_DEV_OPEN;

	return (0);
}

/***********************************************************************
F* Function:     static int kbd_read (struct file *file, char *buf,
F*                                    size_t count, loff_t *ppos) P*A*Z*
 *
P* Parameters:   struct file *file
P*                - Passed by the kernel, pointer to the file structure
P*                  for the device file
P*               char *buf
P*                - Pointer to buffer in userspace
P*               size_t count
P*                - Number of bytes to read
P*               loff_t *ppos
P*                - Offset for the read - ignored.
P*
P* Returnvalue:  int - 0 => success
P*                    <0 Errorcondition, which can be
P*                     -EINVAL  When reading from MINOR_RAW count is not
P*                              a multiple of two
P*                     -EFAULT  A user-provided pointer is invalid
P*                     -EAGAIN  When reading in non-blocking mode, this
P*                              signals no data available for reading
P*                     -ERESTARTSYS  When interrupted by a signal, this is
P*                              signaled to the VFS layer with this return
P*                              code
 *
Z* Intention:    Read count bytes from the associated device.
Z*               Both blocking and non-blocking reads are supported.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static ssize_t kbd_read (struct file *file,
			 char *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned char io_buf[2*RBUF_SIZE];
	int rc = 0;
	int len = 0;
	int minor;
	unsigned int n;

	minor = MINOR(file->f_dentry->d_inode->i_rdev);


	if (minor == MINOR_RAW) {
		/* Two bytes per key event */
		if ((count % 2) != 0) /* must always read a multiple of 2 */
			return (-EINVAL);
	}

	n = (count > 2*RBUF_SIZE) ? 2*RBUF_SIZE : count;

	add_wait_queue(&rbuf_wait, &wait);

	for (;;) {

		set_current_state (TASK_INTERRUPTIBLE);

		if ((len = rbuf_get (minor, n, io_buf)) != 0) {
			break;
		}

		if (file->f_flags & O_NONBLOCK) {
			rc = -EAGAIN;
			goto OUT;
		}

		if (signal_pending(current)) {
			rc = -ERESTARTSYS;
			goto OUT;
		}

		schedule ();
	}

	/* Copy out */
	if ((rc = access_ok(VERIFY_WRITE, buf, len)) == 0) {
		goto OUT;
	}

	copy_to_user((void *)buf, (void*)(&io_buf[0]), len);
	rc = len;
OUT:
	set_current_state (TASK_RUNNING);
	remove_wait_queue(&rbuf_wait, &wait);

	debugk ("kbd_read: rc=%d (buf[0]=%02x)\n", rc, io_buf[0]);

	return rc;
}

/***********************************************************************
F* Function:     static unsigned int kbd_poll (struct file *file,
F*                                             poll_table *wait) P*A*Z*
 *
P* Parameters:   struct file *file
P*                - Passed by the kernel, pointer to file being polled
P*               poll_table *wait
P*                - Passed by the kernel, accumulates waitqueue entries
P*
P* Returnvalue:  int - 0 => No data available
P*                     POLLIN | POLLRDNORM => Data available
 *
Z* Intention:    This provides the functioanlity needed by the kernel to
Z*               implement the select(2) and poll(2) calls.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static unsigned int kbd_poll (struct file *file, poll_table *wait)
{
	int minor;

	debugk ("kbd_poll enter\n");

	minor = MINOR(file->f_dentry->d_inode->i_rdev);
	poll_wait(file, &rbuf_wait, wait);

	debugk ("kbd_poll rc=%d\n",
		(rbuf_first[minor] != rbuf_last[minor]) ? 1 : 0);

	return (rbuf_first[minor] != rbuf_last[minor]) ? POLLIN | POLLRDNORM : 0;
}

/*
 * LWMON Keyboard controller interrupt
 */

/***********************************************************************
F* Function:     static void lwmon_kbd_interrupt (int irq, void *dev_arg,
F*                                                struct pt_regs * regs) P*A*Z*
 *
P* Parameters:   int irq
P*                - The number of the interrupt being handled - ignored
P*               void *dev_arg
P*                - Pointer to our own parameter - setup during interrupt
P*                  registration
P*               struct pt_regs *regs
P*                - Copy of registers when interrupt occurred - ignored
P*
P* Returnvalue:  none
 *
Z* Intention:    This function is called when an interrupt is signalled
Z*               by the keyboard controller.  The sleeping processes are
Z*               woken up.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static irqreturn_t lwmon_kbd_interrupt (int irq, void *dev_arg)
{
	kbd_dev_t *dev = (kbd_dev_t *)dev_arg;

	debugk ("KBD INT\n");
	dev->pend = 1;
	wake_up_interruptible(&dev->queue);

	return IRQ_HANDLED;
}


/***********************************************************************
F* Function:     static int __init lwmon_kbd_init (void) P*A*Z*
 *
P* Parameters:   none
P*
P* Returnvalue:  int - 0 => success
P*                    <0 Errorcondition, with the following special cases
P*                     -EIO   The keyboard controller could not be
P*                            initialized properly so the driver aborts
P*                     -ENXIO Registering with the /proc file system was
P*                            unsuccessful
P*                     -ENOMEM  Registering the interrupt handler failed
P*                            because of insufficient memory
P*                     -EBUSY Registering the interrupt handler failed
P*                            because the interrupt is used in non-shared
P*                            mode already
 *
Z* Intention:    Initialize the driver, register the interrupt handler
Z*               and register the device with the kernel.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static int lwmon5_kbd_validate_client(struct i2c_client *client)
{
	int rc;
	u8 i2c_data[16];

	debugk ("%s () called\n", __func__);

	/*
	 * Verify correct operation of keyboard controller
	 */
	i2c_data[0] = KEYBD_CMD_READ_STATUS;
	rc = i2c_master_send(client, i2c_data, 1);

	rc = i2c_master_recv(client, i2c_data, 1);
	debugk ("Keybd status code: 0x%02X\n", i2c_data[0]);
	if (i2c_data[0]) {
		printk (" - got error code %02X resetting\n", i2c_data[0]);
		/* Reset error condition */
		i2c_data[0] = KEYBD_CMD_RESET_ERRORS;
		rc = i2c_master_send(client, i2c_data, 1);

		udelay(1000);

		i2c_data[0] = KEYBD_CMD_READ_STATUS;
		rc = i2c_master_send(client, i2c_data, 1);

		rc = i2c_master_recv(client, i2c_data, 1);

		if (i2c_data[0]) {
			printk (" - aborted: error code %02X\n", i2c_data[0]);
			return (-EIO);
		}
	}

	/* read kbd controller version */
	i2c_data[0] = KEYBD_CMD_READ_VERSION;
	rc = i2c_master_send(client, i2c_data, 1);

	rc = i2c_master_recv(client, i2c_data, KEYBD_VERSIONLEN);
	printk (KERN_INFO "Kbd Controller Version %d.%d\n",
		i2c_data[0], i2c_data[1]);

	init_MUTEX_LOCKED (&kbd_dev.sem);

	kernel_thread (keyboard_thread, (void*)&kbd_dev, 0);

	/* wait till it has reached the setup_thread routine */
	down (&kbd_dev.sem);

	/* Install Interrupt handler, edge triggered */
	rc = request_irq(KBD_INTERRUPT, lwmon_kbd_interrupt, IRQF_DISABLED, "kbd",
			 (void *)&kbd_dev);

	if (rc != 0) {
		printk ("Can't install KBD IRQ %d (rc=%d)\n",
			KBD_INTERRUPT, rc);
		return (rc);
	}

	debugk ("Installed KBD IRQ %d\n", KBD_INTERRUPT);

	rc = register_chrdev(KBD_MAJOR, DEVICE_NAME, &kbd_fops);

	if (rc != 0) {
		free_irq (KBD_INTERRUPT, (void *)&kbd_dev);
		return (rc);
	}

	if ( ((proc_lwmon_kbd = proc_mkdir (DEVICE_NAME, NULL)) == NULL) ||
	     ((kbd_din_f      = create_proc_entry("din",
						  S_IRUGO | S_IWUSR,
						  proc_lwmon_kbd)) == NULL) ) {
		printk (DEVICE_NAME ": proc_register failed\n");
		return (-ENXIO);
	}
	kbd_din_f->read_proc  = kbd_din_f_read_proc;

	return (0);
}

static int lwmon5_kbd_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	int rc = 0;
	struct i2c_client *client = NULL;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		rc = -ENODEV;
		goto failout;
	}

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (client == NULL) {
		rc = -ENOMEM;
		goto failout;
	}

	client->addr = addr;
	client->adapter = adapter;
	client->driver = &lwmon5_kbd_driver;
	strlcpy(client->name, DEVICE_NAME, I2C_NAME_SIZE);
	my_client = client;

	if (kind < 0) {
		rc = lwmon5_kbd_validate_client(client);
		if (rc < 0)
			goto failout;
	}

	rc = i2c_attach_client(client);
	if (rc < 0)
		goto failout;

	dev_info(&client->dev,
		 "chip found, driver version " DRV_VERSION "\n");

	return 0;

failout:
	kfree(client);
	return rc;
}

/***********************************************************************
F* Function:     static void lwmon_kbd_cleanup (void) P*A*Z*
 *
P* Parameters:   none
P*
P* Returnvalue:  none
 *
Z* Intention:    Cleanup and unregister the driver.
Z*               The keyboard thread is terminated, the interrupt handler
Z*               unregistered and the character device is also unregistered.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static void lwmon_kbd_cleanup (void)
{
	debugk ("%s () called\n", __func__);

	if (kbd_dev.thread == 0) {
		printk ("Can't terminate non-existant LWMON keyboard thread\n");
	} else {
		init_MUTEX_LOCKED (&kbd_dev.sem);

		kill_proc (kbd_dev.thread->pid, SIGINT, 1);

		/* block till thread terminated */
		down (&kbd_dev.sem);
	}

	free_irq (KBD_INTERRUPT, (void *)&kbd_dev);
	debugk ("Freed KBD IRQ %d\n", KBD_INTERRUPT);

	unregister_chrdev(KBD_MAJOR, DEVICE_NAME);
}

/***********************************************************************
F* Function:     static void lwmon_readkeys (void) P*A*Z*
 *
P* Parameters:   none
P*
P* Returnvalue:  none
 *
Z* Intention:    This function is called from the keyboard thread and
Z*               actually reads the data and puts it into the appropriate
Z*               ring buffer.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static void lwmon_readkeys (void)
{
	ssize_t n;
	int i, j;
	u8 i2c_data[16];
	unsigned char c, mod;
	int rc;
	struct i2c_client *client = my_client;

	debugk ("%s () called\n", __func__);

 again:
	/* read key codes */
	i2c_data[0] = KEYBD_CMD_READ_KEYS;
	rc = i2c_master_send(client, i2c_data, 1);
	n = i2c_master_recv(client, i2c_data, KEYBD_DATALEN);

	if (n == 1) {
		/* An Error occurred */
#ifdef DEBUG
		printk ("KBD-Controller signaled error %02X\n",
			i2c_data[0]);
#endif
		/* Reset error condition */
		i2c_data[0] = KEYBD_CMD_RESET_ERRORS;
		rc = i2c_master_send(client, i2c_data, 1);
		goto again;
	}

#ifdef DEBUG
	printk ("KEYS:");
	for (i=0; i<n; ++i) {
		printk (" %02X", i2c_data[i]);
	}
	printk ("\n");
#endif

	/*
	 * Handle error conditions: bytes 1 ... N will contain error
	 * codes (0 < error code < KBD_ERR_MAX), so it should be
	 * sufficient to test the first byte.
	 */
	if ((c = i2c_data[1]) != 0) {
		if (c < KBD_ERR_MAX) {
			printk (KERN_INFO "KBD: error code %d\n", c);
			return;
		}
		if (c >= nkey) {
			printk (KERN_INFO "KBD: invalid key code 0x%02x\n", c);
			return;
		}
	}

	/*
	 * Handle modifiers
	 * These are encoded by bits in the first data byte
	 */
	mod = i2c_data[0];

	if ((c = mod & KBD_MOD_CONTROL) ^ (kbd_mod_state & KBD_MOD_CONTROL)) {
		kbd_mod_state = (kbd_mod_state & ~KBD_MOD_CONTROL) | c;
		debugk ("KEY: CNTL %s\n", (!!c) ? "pressed" : "released");
		rbuf_add(MINOR_RAW, KBD_KEYCODE_CONTROL,
		       (c) ? kbd_mod_state | KBD_MOD_PRESS : kbd_mod_state);
		if (c)
			rbuf_add(MINOR_COOKED, KBD_KEYCODE_CONTROL, 0);
	}
	if ((c = mod & KBD_MOD_SHIFT) ^ (kbd_mod_state & KBD_MOD_SHIFT)) {
		kbd_mod_state = (kbd_mod_state & ~KBD_MOD_SHIFT) | c;
		debugk ("KEY: SHIFT %s\n", (!!c) ? "pressed" : "released");
		rbuf_add(MINOR_RAW, KBD_KEYCODE_SHIFT,
		       (c) ? kbd_mod_state | KBD_MOD_PRESS : kbd_mod_state);
		if (c)
			rbuf_add(MINOR_COOKED, KBD_KEYCODE_SHIFT, 0);
	}

	if ((mod & KBD_DIN_MASK) != curr_din) {
		rbuf_add(MINOR_DIN, mod & KBD_DIN_MASK, 0);
		curr_din = mod & KBD_DIN_MASK;
	}

	/*
	 * Scan for released and pressed keys
	 *
	 * Optimization: the i2c_rq.data[] array is filled in "from
	 * left", i. e. we can stop searching after the first empty
	 * key entry
	 *
	 * Optimization: the i2c_rq.data[] array will not contain
	 * duplicate entries for the same key code thus it's OK
	 * to perform a single linear scan though all key codes
	 */
	for (i=0; i<nkey; ++i) {
		unsigned char found = 0;

		if (kbd_map[i+i] == 0) {	/* impossible key code */
			continue;
		}

		c = kbd_key_state[i];		/* previous state of this key */

		for (j=1; j<n && i2c_data[j]; j++) {
			if (i == i2c_data[j]) {	/* key "i" pressed */
				unsigned char keycd;

				found = 1;
				if (c) {
					continue;	/* no change */
				}
				kbd_key_state[i] = 1;

				keycd = kbd_decode (i, kbd_mod_state);

				debugk ("KEY: %02x (%c) pressed\n",
					keycd,
					(keycd>=' ' && keycd<0x80) ? keycd : ' ');

				rbuf_add(MINOR_RAW, keycd, kbd_mod_state | KBD_MOD_PRESS);
				rbuf_add(MINOR_COOKED, keycd, 0);
				break;		/* this key won't be there twice */
			}
		}
		if (c && !found) {		/* key "i" released */
			unsigned char keycd;
			kbd_key_state[i] = 0;

			keycd = kbd_decode (i, kbd_mod_state);

			debugk ("KEY: %02x (%c) released\n",
				keycd,
				(keycd>=' ' && keycd<0x80) ? keycd : ' ');

			rbuf_add (MINOR_RAW, keycd, kbd_mod_state);
		}
	}
}

/***********************************************************************
F* Function:     static unsigned char kbd_decode (unsigned char hw_code,
F*                                                unsigned char mod) P*A*Z*
 *
P* Parameters:   unsigned char hw_code
P*                - Hardware code of the key
P*               unsigned char mod
P*                - State of the modifier keys
P*
P* Returnvalue:  unsigned int
P*                 >0 Decoded keycode
P*                  0 Illegal hardware code was passed
 *
Z* Intention:    This function decodes the hardware keycodes together with
Z*               the state of the modifier keys to yield the defined
Z*               keycode.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static
unsigned char kbd_decode (unsigned char hw_code, unsigned char mod)
{
	unsigned char code = kbd_map [ 2 * hw_code ];
	unsigned char algo = kbd_map [ 2 * hw_code +1 ];

	if (hw_code >= nkey) {
		debugk ("decode: illegal hw_code %02X [max. %02X]\n",
			hw_code, nkey);
		return 0;
	}

	code = kbd_map [ 2 * hw_code ];
	algo = kbd_map [ 2 * hw_code +1 ];

	if (code == 0) {
		debugk ("decode: illegal keycode\n");
		return 0;	/* illegal keycode - can't happen */
	}

#if 0
	debugk ("decode: %02x:%02x => algo %d code %02X\n",
		mod, hw_code, algo, code);
#endif

	switch (mod & (KBD_MOD_SHIFT | KBD_MOD_CONTROL)) {
	case 0:				/* neither shift nor control	*/
		switch (algo) {
		case 0:	return (code - 0x30);
		case 1: return (code - 0x80);
		case 2: return (code - 0xD0);
		case 3: return (code - 0xA0);
		case 4: return (code - 0xB0);
		}
		return 0;
	case KBD_MOD_SHIFT:				/* shift only	*/
		switch (algo) {
		case 0:	/* fall through */
		case 4: return (code - 0x10);
		case 1: return (code - 0xA0);
		case 2: return (code - 0xC0);
		case 3: return (code - 0xB0);
		}
		return 0;
	case KBD_MOD_CONTROL:				/* control only	*/
		switch (algo) {
		case 0:	/* fall through */
		case 4: return (code - 0x20);
		case 1: /* fall through */
		case 2: return (code - 0xE0);
		case 3: return (code - 0xD0);
		}
		return 0;
	case (KBD_MOD_SHIFT | KBD_MOD_CONTROL):		/* both		*/
		return (code);
	}
	return 0;
}

/***********************************************************************
F* Function:     static void rbuf_add (int minor, unsigned char keycd,
F*                                   unsigned char mod) P*A*Z*
 *
P* Parameters:   int minor
P*                - Minor number of the device owning the ring buffer
P*               unsigned char keycd
P*                - First byte of data to store
P*               unsigned char mod
P*                - Second byte of data to store.
P*                  Only used for MINOR_RAW
P*
P* Returnvalue:  none
 *
Z* Intention:    This function enters data into the ring buffers belonging
Z*               to the device with minor number minor.
Z* Exceptions:   When no space is left in the buffer, a message is printed
Z*               with printk() and the data is lost.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static void rbuf_add (int minor, unsigned char keycd, unsigned char mod)
{
	unsigned short rbuf_curr, rbuf_next;

	/* Don't log events for closed devices */
	if (!(kbd_dev_flags[minor] & KBD_DEV_OPEN))
		return;

	spin_lock (&rbuf_lock);			/* grab lock */

	if (minor == MINOR_RAW)
		rbuf_next = (rbuf_last[minor] + 2) % (2*RBUF_SIZE);
	else
		rbuf_next = (rbuf_last[minor] + 1) % (2*RBUF_SIZE);

	if (rbuf_next == rbuf_first[minor]) {
		printk ("## minor %d lwmon_kbd RINGBUFFER FULL\n", minor);
		goto DONE;
	}

	rbuf_curr = rbuf_last[minor];
	rbuf_data[minor][rbuf_curr  ] = keycd;
	if (minor == MINOR_RAW)
		rbuf_data[minor][rbuf_curr+1] = mod;

	rbuf_last[minor] = rbuf_next;

	wake_up_interruptible(&rbuf_wait);
DONE:
	spin_unlock (&rbuf_lock);			/* release lock */

	debugk("RBUF_ADD: minor %d - %d / %d [%02X / %02X]\n",
		minor, rbuf_first[minor], rbuf_last[minor], keycd, mod);
}

/***********************************************************************
F* Function:     static int rbuf_get (int minor, unsigned int n,
F*                                  unsigned char *buf) P*A*Z*
 *
P* Parameters:   int minor
P*                - Minor
P*               unsigned int n
P*                - Number of data bytes to read
P*               unsigned char *buf
P*                - Pointer to the buffer where the data is copied to
P*
P* Returnvalue:  int
P*                - Number of bytes actually copied
 *
Z* Intention:    Get data from the ring buffer of the device minor and
Z*               copy it to buf.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static int rbuf_get (int minor, unsigned int n, unsigned char *buf)
{
	int cnt = 0;

	spin_lock (&rbuf_lock);			/* grab lock */

	while ((cnt < n) && (rbuf_first[minor] != rbuf_last[minor])) {
		unsigned char keycd, mod;

		if (minor == MINOR_RAW) {
			/* copy keycode + mode */
			keycd = rbuf_data[minor][rbuf_first[minor]++];
			mod   = rbuf_data[minor][rbuf_first[minor]++];

			if (rbuf_first[minor] >= (2*RBUF_SIZE))
				rbuf_first[minor] = 0;

			*buf++ = keycd;
			*buf++ = mod;
			cnt += 2;
		} else {
			keycd = rbuf_data[minor][rbuf_first[minor]++];

			if (rbuf_first[minor] >= 2*RBUF_SIZE)
				rbuf_first[minor] = 0;

			*buf++ = keycd;
			cnt++;
		}
	}

	spin_unlock (&rbuf_lock);			/* release lock */

	return (cnt);
}

/***********************************************************************
F* Function:     static int kbd_din_f_read_proc (char *page, char **start,
F*                                               off_t off, int count,
F*                                               int *eof, void *data) P*A*Z*
 *
P* Parameters:   char *page
P*                - Passed by the kernel - pointer to buffer page that
P*                  we write to
P*               char **start
P*                - Passed by the kernel, ignored
P*               off_t off
P*                - Passed by the kernel, ignored
P*               int count
P*                - Passed by the kernel, ignored
P*               int *eof
P*                - Passed by the kernel, ignored
P*               void *data
P*                - Passed by the kernel, ignored
P*
P* Returnvalue:  int
P*                - Number of bytes written (excluding the \0)
 *
Z* Intention:    This function is called when the proc entry for the
Z*               digital input is read.  A ascii representation of the
Z*               current state is copied to the buffer.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static int kbd_din_f_read_proc (char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	debugk ("%s din=%02x\n", __func__, curr_din);
	return (sprintf (page, "0x%02x\n", curr_din));
}

/******************************
 ****	Keyboard Thread	  *****
 **************************** */

/***********************************************************************
F* Function:     static int keyboard_thread (void *dev_arg) P*A*Z*
 *
P* Parameters:   void *dev_arg
P*                - Pointer to our data - setup in lwmon_kbd_init()
P*
P* Returnvalue:  int
P*                - Always returns zero when the thread was killed
 *
Z* Intention:    This function implements the separate keyboard thread
Z*               waiting for notification from the interrupt handler.
Z*               When notified, the data is read and copied to the ring
Z*               buffers.
Z*               The thread is terminated by lwmon_kbd_cleanup() by
Z*               sending the signal SIGKILL.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static int keyboard_thread (void *dev_arg)
{
	kbd_dev_t *dev = (kbd_dev_t *)dev_arg;

	setup_thread (dev);		/* setup the thread environment */

	debugk ("KBD Thread started\n");

	lwmon_readkeys ();		/* read any stuck keys, and enable interrupts */

	for (;;) {
		/*
		 * wait for interrupt
		 */
		if (wait_event_interruptible(dev->queue, dev->pend)) {
			break;
		}

		/*
		 * We cannot allow new interrupts from the keyboard
		 * controller before we have not completed to read
		 * (and thus ACK) the keypress over the I2C bus...
		 */
		disable_irq(KBD_INTERRUPT);
		lwmon_readkeys ();
		dev->pend = 0;
		enable_irq(KBD_INTERRUPT);
	}

	leave_thread (dev);

	return (0);
}

/***********************************************************************
F* Function:     static void setup_thread (kbd_dev_t *dev) P*A*Z*
 *
P* Parameters:   kbd_dev_t *dev
P*                - Our device description
P*
P* Returnvalue:  none
 *
Z* Intention:    Setup the environment for the separate keyboard thread.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static void setup_thread (kbd_dev_t *dev)
{
	/* lock the kernel */
	lock_kernel();

	/* set name of this process (max 15 chars + 0 !) */
	strcpy (current->comm, "LWMON keyboard");

	/* disconnect from the fs usage */
	exit_files (current);
	exit_fs	   (current);

	/* fill in thread structure */
	dev->thread = current;

	/* set signal mask to what we want to respond */
	siginitsetinv (&current->blocked, sigmask(SIGINT));

	/* initialise wait queue head */
	init_waitqueue_head (&dev->queue);

	/* let others run */
	unlock_kernel();

	/* tell the creator that we are ready and let him run */
	up (&dev->sem);
}

/***********************************************************************
F* Function:     static void leave_thread (kbd_dev_t *dev) P*A*Z*
 *
P* Parameters:   kbd_dev_t *dev
P*                - Our device description
P*
P* Returnvalue:  none
 *
Z* Intention:    This function is called when the keyboard thread
Z*               received the signal to shutdown.  The exit is signaled
Z*               through the semaphore in our device data.
 *
D* Design:       wd@denx.de
C* Coding:       wd@denx.de
V* Verification: dzu@denx.de
 ***********************************************************************/
static void leave_thread (kbd_dev_t *dev)
{
	/* lock the kernel, the exit will unlock it */
	dev->thread = NULL;

	/* notify the lwmon_kbd_cleanup() routine that we are terminating. */
	up (&dev->sem);
}

static int __init lwmon5_kbd_init(void)
{
	return i2c_add_driver(&lwmon5_kbd_driver);
}

static void __exit lwmon5_kbd_exit(void)
{
	i2c_del_driver(&lwmon5_kbd_driver);
}

module_init(lwmon5_kbd_init);
module_exit(lwmon5_kbd_exit);
