/*
cc16.c : Kernel Module to support a PC16 ISA crate controller for talking
         to a CAMAC data acquisition system (CC16 crate controller).

    (c) Christian Vogel <vogelchr@vogel.cx>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/slab.h>

#include <asm/atomic.h>
#include <asm/io.h>

#include "cc16.h"

/* maxumum number of minors (=cards) supported, default major devno */
#define NR_DEVS 8
/* io size of card */
#define IO_SIZE 0x20

/* offset from base address */
#define OFFS_GEO    0x00	/* 16  crate address and cable delay */
#define OFFS_FNA    0x02	/* 16  function, subaddress, station number */
#define OFFS_HB_W   0x04	/*  8 write high byte (D16..D23) */
#define OFFS_LW_W   0x06	/* 16 write low word (D0..D15) */
#define OFFS_HB_R   0x08	/*  8 read high byte */
#define OFFS_LW_R   0x0A	/* 16 read low word */
#define OFFS_BR_R   0x0C	/* ?? block read with NAF repeat */
#define OFFS_SM_W   0x0E	/*  8 set CAMAC I,C,Z,LAM enable */
#define OFFS_SM_R   0x10	/*  8 read CAMAC I,LAM, on/off */
#define OFFS_RS     0x12	/*  8 read CAMAC X,Q,LAM */
#define OFFS_C      0x14	/* ?? release CAMAC C cycle */
#define OFFS_Z      0x16	/* ?? release CAMAC Z cycle */
#define OFFS_ON     0x18	/* ?? set broadcast bus call on */
#define OFFS_OFF    0x1A	/* ?? set broadcast but call off */

/* commands to write to the SM_W register: */
#define SM_W_SET_I   0x0a	/* set inhibit */
#define SM_W_CLR_I   0x0b	/* clear inhibit */
#define SM_W_ENA_LAM 0x0c	/* enable lam detect */
#define SM_W_DIS_LAM 0x0d	/* disable lam detect */

/*
    --- offs geo ---
  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 |  |  |  |  |D3|D2|D1|D0|  |  |  |  |A3|A2|A1|A0|
 +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
              Delay                   Crate Number

  Crate Address: 0..14 (front panel) 15: broadcast
  Delay: set to 0 for 0..10 or 0..30m, 500ns unit

    --- naf ---
  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 |S |  |A3|A2|A1|A0|F4|F3|F2|F1|F0|N4|N3|N2|N1|N0|
 +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  A: Subaddress, F: Function, N: Station
  S: immediate cycle, set to 0 for write, then write
     data to registers

    --- offs SM_W --- Special Modes
   7  6  5  4  3  2  1  0
 +--+--+--+--+--+--+--+--+
 | see table below...... |
 +--+--+--+--+--+--+--+--+
   you have to write into this register...
      0x0A -> enable inhibit
      0x0B -> disable inhibit
      0x0C -> enable LAM detect
      0x0D -> disable LAM detect

    --- offs SM_R --- Special Modes
   7  6  5  4  3  2  1  0
 +--+--+--+--+--+--+--+--+
 |  |  |  |  |  | B| L| I|
 +--+--+--+--+--+--+--+--+
   I: read camac inhibit, L: read lam enable/disable
   B: bus on/off (broadcast calls)

    --- offs RS --- (Read Controller Status)
   7  6  5  4  3  2  1  0
 +--+--+--+--+--+--+--+--+
 |  |  |  |  | L|  | X| Q|
 +--+--+--+--+--+--+--+--+
   Q: read Q response, X: read X response, L: lam pending
*/

/* private (per minor) data */
struct cc16_data
{
	int iobase;		/* io port */
	int irq;		/* irq */
	int minor;		/* minor device number */
	wait_queue_head_t waitqueue;	/* tasks sleep here to wait for IRQ */

	atomic_t irqctr;	/* irq counter */

	spinlock_t lock;	/* spinlock for hardware and stuff below */
	int crate;		/* currently accessed crate, locked! */
	int inhmask;		/* if irqinhibitmask & (1<<crate) set inhibit
					on that crate on interrupt... */
};

/* cc16_file is created by fops->open and freed by fops->release,
   contains per-opened-device data */
struct cc16_file
{
	struct cc16_data *cc16;	/* cc16_data for this minor device */
	unsigned int crate;	/* crate opened for that process */
	int lastirq;		/* last irq seen */
	struct cc16_naf_data naf_data;	/* temporary space */
};

static dev_t firstdev;		/* first dev in chain (alloc region...) */
static struct cdev cc16_cdev;	/* char dev structure (cdev_add...) */
static struct class *cc16_class; /* driver-model class for cc16 devices */
static struct cc16_data *cc16[NR_DEVS];	/* per minor data, also file->priv... */

static signed int irq[NR_DEVS];	/* module parameters, 0=no irq */
static unsigned int io[NR_DEVS];	/* module parameters, 0=unused */
static int debug; /* enable/disable debug output */

module_param_array (irq, int, NULL, 0);
module_param_array (io, uint, NULL, 0);
module_param(debug,int,0);

MODULE_AUTHOR ("Christian Vogel <vogelchr@vogel.cx>");
MODULE_DESCRIPTION ("Wiener CC16/PC16 Driver");
MODULE_VERSION ("0.0.0");
MODULE_LICENSE ("GPL");

#define cc16_debug(format,arg...) \
    do { \
        if(debug) \
	    	printk(KERN_DEBUG format,## arg); \
    } while(0)

/* ----------------------- HARDWARE ACCESS --------------------- */
/* select crate, has to be called with p->lock held! */
static inline void
cc16_sel_crate (struct cc16_data *p, unsigned int crate)
{
	u_int16_t reg;

	crate &= 0xf;
	p->crate = crate;
	/* delay is 0x01: GEO is delay << 8 | crate */
	reg = 0x0100 | (crate & 0xf);
	outw(reg, p->iobase + OFFS_GEO);
	udelay(10);
	cc16_debug("cc16-%d: selecting crate %d\n",
		p->minor,crate & 0xf);
}

/* Since Kernel 2.6.19 (or so...) the third argument of a interrupt
 * service routine has been removed. If you compile this module
 * on a older kernel, please add this to the argument-list
 * of cc16_isr: struct pt_regs *regs
 */

/* Interrupt service routine. Increases the nirq atomic variable
 * and wakes up sleeping processes. If card->inhmask is set, the
 * inhibit line in matching crates is activated. */
static irqreturn_t
cc16_isr (int theirq, void *devid)
{
	register struct cc16_data *p = devid;
	int i;

	if(p->inhmask){ /* set I on crates whose bits are set */
		spin_lock(&p->lock);
		for(i=0;i<15;i++){ /* crate numbers 0..14 are valid */
			if(p->inhmask & (1<<i)){
				cc16_sel_crate(p,i);
				/* set camac I */
				outb_p(SM_W_SET_I, p->iobase + OFFS_SM_W);
			}
		}
		spin_unlock(&p->lock);
	}
	
	atomic_add (1, &(p->irqctr));
	wake_up (&p->waitqueue);

	cc16_debug("cc16-%d: IRQ irqctr=%d\n",
		p->minor,
		atomic_read(&p->irqctr));

	return IRQ_HANDLED;
};

/* do Z or C cycle: is_z==1 means Z, else C */
static void
cc16_cz (struct cc16_data *p, int crate, int is_z)
{
	unsigned long flags;

	spin_lock_irqsave (&p->lock, flags);

	if(p->crate != crate)
		cc16_sel_crate (p, crate);

	if (is_z)
		outb_p(0x0F, p->iobase + OFFS_Z);
	else
		outb_p(0x0F, p->iobase + OFFS_C);

	cc16_debug("cc16-%d: doing %c on crate %d\n",
		p->minor,is_z?'Z':'C',crate);

	spin_unlock_irqrestore (&p->lock, flags);
}

/* enable(i=1)/disable(i=0) LAM detect on crate */
static void
cc16_lam (struct cc16_data *p, int crate, int i)
{
	unsigned long flags;

	spin_lock_irqsave (&p->lock, flags);

	if (p->crate != crate)
		cc16_sel_crate (p, crate);

	if (i) /* enable LAM servicing */
		outb_p (SM_W_ENA_LAM, p->iobase + OFFS_SM_W);
	else   /* disable LAM servicing */
		outb_p (SM_W_DIS_LAM, p->iobase + OFFS_SM_W);

	cc16_debug("cc16-%d: %sabling LAM servicing on crate %d\n",
		p->minor,i?"en":"dis",crate);

	spin_unlock_irqrestore (&p->lock, flags);
}

/* enable(i=1)/disable(i=0) inhibit in crate */
static void
cc16_i (struct cc16_data *p, int crate, int i)
{
	unsigned long flags;

	spin_lock_irqsave (&p->lock, flags);

	if (p->crate != crate)
		cc16_sel_crate (p, crate);

	if (i)  /* set camac I */
		outb_p (SM_W_SET_I, p->iobase + OFFS_SM_W);
	else    /* clear camac I */
		outb_p (SM_W_CLR_I, p->iobase + OFFS_SM_W);

	cc16_debug("cc16-%d: %sabling INHIBIT on crate %d\n",
		p->minor,i?"en":"dis",crate);

	spin_unlock_irqrestore (&p->lock, flags);
}

/* turn on/off crate broadcast, I have no idea what it is... */
static void
cc16_on_off (struct cc16_data *p, int crate, int i)
{
	unsigned long flags;

	spin_lock_irqsave (&p->lock, flags);

	if (p->crate != crate)
		cc16_sel_crate (p, crate);

	if(i)
		outb_p(0x0F,OFFS_ON); /* turn on */
	else
		outb_p(0x0F,OFFS_OFF); /* turn off */
		
	cc16_debug("cc16-%d: turning crate %d %s\n",
		p->minor,crate,i?"on":"off");

	spin_unlock_irqrestore (&p->lock, flags);
}

static void
cc16_status (struct cc16_data *p, int crate, u_int32_t * i)
{
	unsigned long flags;

	spin_lock_irqsave (&p->lock, flags);

	if (p->crate != crate)
		cc16_sel_crate (p, crate);

	*i = inb_p(p->iobase + OFFS_SM_R); /* read status from hardware */

	spin_unlock_irqrestore (&p->lock, flags);
}

static void
cc16_xqlam(struct cc16_data *p, int crate, u_int32_t * i)
{
	unsigned long flags;

	spin_lock_irqsave (&p->lock, flags);

	if (p->crate != crate)
		cc16_sel_crate (p, crate);

	*i = inb_p(p->iobase + OFFS_RS); /* */

	spin_unlock_irqrestore (&p->lock, flags);
}

/* do a NAF in crate */
static void
cc16_naf (struct cc16_data *p, int crate, struct cc16_naf_data *nafp)
{
	unsigned long flags;
	u_int16_t reg;
	u_int32_t naf = nafp->naf;

	spin_lock_irqsave (&p->lock, flags);

	if (p->crate != crate)
		cc16_sel_crate (p, crate);

	reg = naf & CC16_NAF_NAFMASK;

	/* only if it is NOT a write cycle we set the bit S=1 which
	   means to start the cycle immediately. That is because we
	   have to wait to the point where we put the actual data in
	   to start the cycle when writing. */
	if (!CC16_NAF_WRITE (naf))
		reg |= CC16_NAF_S;
	outw_p (reg, p->iobase + OFFS_FNA);

	/* write cycle: write data, high bits first, but spare the
	   cycles if we are only interested in 16 bits anyway... */
	if (CC16_NAF_WRITE (naf)) {
		if (!(naf & CC16_NAF_16BIT)){ /* high byte... */
			outb_p (0xff & (nafp->data >> 16),
			      p->iobase + OFFS_HB_W);
			/* udelay(10); already in outb_p */
		}
		outw_p (0xffff & nafp->data, p->iobase + OFFS_LW_W); /* low word */
	}

	/* read cycle: get data... see 16bit remark above */
	if (CC16_NAF_READ (naf)) {
		u_int32_t val = 0;

		if (!(naf & CC16_NAF_16BIT)){ /* high byte */
			val = inb_p (p->iobase + OFFS_HB_R) << 16;
			/* here we really some delay, inb_p is not enough */
			/* udelay(10) is enough, outb(dummy,0x80) probably too */
			outb(0,0x80); /* delay by dummy write to port 0x80 */
		}
		val |= inw_p(p->iobase + OFFS_LW_R); /* low word */
		nafp->data = val;
	}

	/* always mask out the X and Q bits */
	nafp->data &= ~ (CC16_DATA_X | CC16_DATA_Q);

	/* if we are interested in X and Q bits, get them... */
	if (naf & CC16_NAF_XQ) {
		u_int32_t val;
		val = inb_p (p->iobase + OFFS_RS);
		/* here the layout of the RS register has to match the
		   CC16_DATA_X, CC16_DTA_Q macros */
		nafp->data |= (val & 0x3) << 30;
	}

	spin_unlock_irqrestore (&p->lock, flags);
}

/* -------------------- FILE OPERATTIONS ---------------------- */
static long
cc16_unlocked_ioctl (struct file *filp,
	    unsigned int cmd, unsigned long arg)
{
	struct cc16_file *f = (struct cc16_file *) filp->private_data;
	struct cc16_data *p = f->cc16;
	u_int32_t u;
	int ret = 0;

	/* ioctls which do not require f->crate to be valid */
	switch(cmd){
	case CC16_IOCTL_CRATE:	/* set crate */
		if (get_user (u, (u_int32_t *) arg))
			return -EFAULT;
		if(u < 0 || u > 15) /* valid crate numbers: 0..14 */
			return -EINVAL;
		f->crate = u;
		return 0;
	case CC16_IOCTL_GET_CRATE: /* get current crate number */
		if (put_user (f->crate, (u_int32_t *) arg))
			ret = -EFAULT;
		return 0;
	case CC16_IOCTL_GETINH:
		if(put_user(p->inhmask,(int*)arg))
			ret=-EFAULT;
		return 0;	
	case CC16_IOCTL_SETINH:
		if(get_user(p->inhmask,(int*)arg))
			ret=-EFAULT;
		return 0;
	}

	/* --- the following IOCTLS need a valid crate selected */
	if(f->crate < 0 || f->crate>14)
		return -EINVAL; /* bad crate address */

	switch (cmd) {
	case CC16_IOCTL_ON_OFF:	/* turn crate broadcast (???) on/off */
		if (get_user (u, (u_int32_t *) arg))
			ret = -EFAULT;
		cc16_on_off (p, f->crate, u);
		break;
	case CC16_IOCTL_LAM: /* enable/disable LAM servicing on crate */
		if (get_user (u, (u_int32_t *) arg))
			ret = -EFAULT;
		cc16_lam(p, f->crate, u);
		break;
	case CC16_IOCTL_STATUS:	/* get crate status */
		cc16_status (p, f->crate, &u);
		if (put_user (u, (u_int32_t *) arg))
			ret = -EFAULT;
		break;
	case CC16_IOCTL_XQLAM: /* get X,Q,LAM - register */
		cc16_xqlam (p, f->crate, &u);
		if (put_user (u, (u_int32_t *) arg))
			ret = -EFAULT;
		break;
	case CC16_IOCTL_NAF:
		if (copy_from_user (&f->naf_data,
				    (void *) arg, sizeof (struct cc16_naf_data))) {
			ret = -EFAULT;
			break;
		}
		cc16_naf (p, f->crate, &(f->naf_data));
		if (copy_to_user ((u_int32_t *) arg,
				  &f->naf_data, sizeof (struct cc16_naf_data))) {
			ret = -EFAULT;
		}
		break;
	case CC16_IOCTL_I:
		if (get_user (u, (u_int32_t *) arg)) {
			ret = -EFAULT;
			break;
		}
		cc16_i (p, f->crate, u);
		break;
	case CC16_IOCTL_Z:
		cc16_cz (p, f->crate, 1/* Z */);
		break;
	case CC16_IOCTL_C:
		cc16_cz (p, f->crate, 0/* Q */);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
};

/* read has special semantics: We store one interrupt counter
 * per hardware device and one per opened device. read blocks as long
 * as those two counters are equal. If they differ, the opened-device-counter
 * is updated to the hardware-device-counter and the current value
 * is returned by the read().
 * If the file is opened non-blocking it will return EAGAIN if the two
 * counters are equal
 */
static ssize_t
cc16_read (struct file *filp, char __user * buf, size_t count, loff_t * ppos)
{
	struct cc16_file *f = (struct cc16_file *) filp->private_data;
	struct cc16_data *p = f->cc16;

	unsigned int l;
	int n;			/* number of interrupts */

	if(0 == p->irq) /* no irq, read() does not make sense! */
		return -EAGAIN;

	if (filp->f_flags & O_NONBLOCK) {	/* non-blocking */
		if(f->lastirq == (n = atomic_read (&(p->irqctr))))
			return -EAGAIN;
	} else {		/* blocking */
		cc16_debug("cc16-%d: blocking read (lastirq=%d)\n",
				p->minor,atomic_read(&p->irqctr));
				
		if(wait_event_interruptible (p->waitqueue,
					      f->lastirq != (n =
							     atomic_read
							     (&p->irqctr))))
			return -ERESTARTSYS;
	}

	l = min (sizeof (n), count);	/* number of bytes to actually copy */
	if (copy_to_user (buf, &n, l))
		return -EFAULT;

	f->lastirq = n; /* store our lastirq ... */
	return l;
}

/* poll for the two counters described above to differ */
static unsigned int
cc16_poll (struct file *filp, poll_table * wait)
{
	struct cc16_file *f = (struct cc16_file *) filp->private_data;
	struct cc16_data *p = f->cc16;

	cc16_debug("cc16-%d: in cc16_poll, irqctr=%d, lastirq=%d\n",
		p->minor,f->lastirq,atomic_read(&p->irqctr));

	poll_wait(filp, &(p->waitqueue), wait);
	if (f->lastirq != atomic_read (&(p->irqctr)))
		return POLLIN | POLLRDNORM;
	return 0;
}

/*
 * opening the device, private_data holds the pointer to cc16[minor]
 */
static int
cc16_open (struct inode *inode, struct file *filp)
{
	unsigned int m = iminor (inode); /* <-- add udev magic here! */
	struct cc16_file *p;

	if (m < 0 || m >= NR_DEVS || !cc16[m])
		return -ENODEV;

	if (NULL == (p = kmalloc (sizeof (struct cc16_file), GFP_KERNEL)))
		return -ENOMEM;

	p->cc16 = cc16[m];
	p->crate = -1;
	p->lastirq = atomic_read (&cc16[m]->irqctr);
	filp->private_data = p;
	
	cc16_debug("cc16-%d: opened\n",p->cc16->minor);
	return 0;
};

/*
 * closing the device
 */
static int
cc16_close (struct inode *inode, struct file *filp)
{
	struct cc16_file *f = (struct cc16_file *) filp->private_data;
	struct cc16_data *p = f->cc16;
	
	kfree (f);		/* free per-opened-device data */
	cc16_debug("cc16-%d: closed\n",p->minor);
	return 0;
};

static struct file_operations cc16_fops = {
	.owner = THIS_MODULE,
	.read = cc16_read,
	.poll = cc16_poll,
	.unlocked_ioctl = cc16_unlocked_ioctl,
	.open = cc16_open,
	.release = cc16_close
};

static void __exit
cc16_free_minor(int i)
{
	device_destroy(cc16_class,firstdev+i);
	if (irq[i])
		free_irq (irq[i], cc16[i]);
	release_region (io[i], IO_SIZE);
	kfree (cc16[i]);
}

static int __init
cc16_init_minor (int i)
{
	struct cc16_data *p;
	struct device *c;

	if (NULL == (p = kmalloc(sizeof (*p), GFP_KERNEL))) {
		printk ("cc16-%d: could not allocate memory\n", i);
		return -1;
	}

	if (NULL == request_region(io[i], IO_SIZE, "cc16")) {
		printk("cc16-%d: cannot get io at 0x%x\n",i,io[i]);
		goto out_free;
	}

	if (irq[i] && request_irq(irq[i], cc16_isr, IRQF_DISABLED, "cc16", p)) {
		printk ("cc16-%d: cannot not get irq %d\n", i, irq[i]);
		goto out_io;
	}

	c=device_create(cc16_class,NULL /*parent*/,
		firstdev+i/*dev*/,NULL /*struct device */,
		"cc16-%d"/*fmt*/,i);
	if(IS_ERR(c)){
		printk(KERN_ERR "cc16-%d: cannot create class_device\n",i);
		goto out_irq;
	}

	printk(KERN_INFO "cc16-%d: io 0x%x irq %d\n", i, io[i], irq[i]);

	atomic_set(&(p->irqctr), 0);		/* interrupt counter */
	init_waitqueue_head(&(p->waitqueue));	/* processes sleep here */
	spin_lock_init(&p->lock);		/* hardware access spinlock */
	p->iobase = io[i];
	p->irq = irq[i];
	p->minor = i;
	p->inhmask = 0;
	p->crate = -1; /* force update of crate number on first access */
	cc16[i] = p;

	return 0;

out_irq:
	if (irq[i])
		free_irq (irq[i], p);
out_io:
	release_region (io[i], IO_SIZE);
out_free:
	kfree(p);
	return -1;
}

/* register major device number, allocate stuff... */
static int __init
cc16_init (void)
{
	int i;
	int n_devs = 0;
	
	if (alloc_chrdev_region(&firstdev, 0, NR_DEVS, "cc16")) {
		printk (KERN_ERR "cc16: alloc_chrdev_region failed\n");
		return -EIO;
	}

	cdev_init(&cc16_cdev, &cc16_fops);
	kobject_set_name(&cc16_cdev.kobj, "cc16%d",0);

	if (cdev_add(&cc16_cdev, firstdev, NR_DEVS)) {
		printk (KERN_ERR "cc16: cdev_add failed\n");
		kobject_put(&cc16_cdev.kobj);
		goto out_region;
	}

	cc16_class = class_create(THIS_MODULE, "cc16");
	if(IS_ERR(cc16_class)){
		printk(KERN_ERR "Error creatinc cc16-class.\n");
		goto out_cdev;
	}

	for (i = 0; i < NR_DEVS; i++) {
		cc16[i]=NULL; // should be the case, but chasing nasty bug...
		if(io[i] && (0 == cc16_init_minor(i)))
			n_devs++;
	}

	if(0 == n_devs){
		printk(KERN_INFO "cc16: no devices, module will refuse to load\n");
		goto out_class;
	}
	
	cc16_debug("cc16: debug is on (%d)\n",debug);

	return 0;
out_class:
	class_destroy(cc16_class);
out_cdev:
	cdev_del (&cc16_cdev);
out_region:
	unregister_chrdev_region (firstdev, NR_DEVS);
	return -EIO;	
};

/* clean up, free resources... */
static void __exit
cc16_exit (void)
{
	int i;

	for (i = 0; i < NR_DEVS; i++){
		if (cc16[i]) 
			cc16_free_minor(i);
	}

	class_destroy(cc16_class);
	cdev_del (&cc16_cdev);
	unregister_chrdev_region (firstdev, NR_DEVS);
	printk(KERN_INFO "cc16: bye, it was a pleasure to serve you.\n");
};

module_init (cc16_init);
module_exit (cc16_exit);
