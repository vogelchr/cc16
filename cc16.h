#include <linux/ioctl.h>

#define CC16_IOCTL_BASE 'c'

/* --- execute a NAF --- */
#define CC16_IOCTL_NAF	  _IOWR('c',0x01, struct cc16_naf_data)

struct cc16_naf_data {
	u_int32_t naf;
	u_int32_t data;
};

/* Encoding of NAF:
 *
 *   31  30        13  12  11  10   9   8   7   6   5   4   3   2   1   0
 * +---+---+ ... +---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 * | 16| XQ| ... | A   A   A   A | F   F   F   F   F   F | N   N   N   N |
 * +---+---+ ... +---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 *  N: module slot		}
 *  F: function number		} use CC16_NAF(n,a,f) macro
 *  A: subaddress		}
 *
 * XQ: if set, read X and Q bits after the NAF (takes an additional IO)
 * 16: restrict to 16bit reads/writes (saves one IO access)
 */

#define CC16_NAF_N_SHIFT	0U
#define CC16_NAF_N_MASK		0x1f

#define CC16_NAF_F_SHIFT	5U
#define CC16_NAF_F_MASK		0x1f

#define CC16_NAF_A_SHIFT	10U
#define CC16_NAF_A_MASK		0x0f

/* NAF-options: */
#define CC16_NAF_16BIT		(1U<<31)  /* only read or write 16bit data */
#define CC16_NAF_XQ		(1U<<30)  /* read X and Q bit afterwards */

#define CC16_NAF_NAFMASK        (0x3fff)  /* mask covering N, A, F */
#define CC16_NAF_S		(1U<<15)  /* S-bit as defined in the CC16 manual */

/*
 * Functions:
 *   0- 7  READ
 *   8-15  ...control...
 *  16-23  WRITE
 *  24-31  ...control...
 */
#define CC16_NAF_READ(naf)   ((CC16_F(naf) & 0x18) == 0x00)  /*  0 ..  7 */
#define CC16_NAF_WRITE(naf)  ((CC16_F(naf) & 0x18) == 0x10)  /* 16 .. 23 */

/* build naf from n,a,f */
#define CC16_NAF(n,a,f) (((n) & CC16_NAF_N_MASK) << CC16_NAF_N_SHIFT | \
			 ((a) & CC16_NAF_A_MASK) << CC16_NAF_A_SHIFT | \
			 ((f) & CC16_NAF_F_MASK) << CC16_NAF_F_SHIFT )

/* extract n,a,f from naf */
#define CC16_N(naf) (((naf) >> CC16_NAF_N_SHIFT) & CC16_NAF_N_MASK)
#define CC16_A(naf) (((naf) >> CC16_NAF_A_SHIFT) & CC16_NAF_A_MASK)
#define CC16_F(naf) (((naf) >> CC16_NAF_F_SHIFT) & CC16_NAF_F_MASK)

/* data in struct naf_data, low 24 bits are from camac */
#define CC16_DATA_X		(1U<<31)  /* X bit read from camac in data word read */
#define CC16_DATA_Q		(1U<<30)  /* Q bit read from camac in data word read */
#define CC16_DATA(data)    (0xffffff & (data))

/* --- select crate --- */
#define CC16_IOCTL_CRATE      _IOW('c',0x02, int)
#define CC16_IOCTL_GET_CRATE  _IOR('c',0x0a, int)

/* --- set/clear inhibit on crate --- */
#define CC16_IOCTL_I  _IOWR(CC16_IOCTL_BASE,0x03, int)

/* --- execute Z cycle on crate --- */
#define CC16_IOCTL_Z    _IO(CC16_IOCTL_BASE,0x04)

/* --- execute C cycle on crate --- */
#define CC16_IOCTL_C    _IO(CC16_IOCTL_BASE,0x05)

/* --- turn crate controller on/off --- */
#define CC16_IOCTL_ON_OFF _IOW('c',0x06,int)

/* --- get status from crate --- */
#define CC16_IOCTL_STATUS _IOR('c',0x07,int)
#define CC16_STATUS_I (1U<<0)  /* inhibit enabled */
#define CC16_STATUS_L (1U<<1)  /* lam enable */
#define CC16_STATUS_B (1U<<2)  /* 0:bus on 1:bus off */

/* --- enable/disable LAM servicing */
#define CC16_IOCTL_LAM    _IOW('c',0x08,int)

/* read XQLAM - register */
#define CC16_IOCTL_XQLAM _IOR('c',0x09,int)
#define CC16_XQLAM_X    (1U<<1)  /* X response */
#define CC16_XQLAM_Q    (1U<<0)  /* Q response */
/* NOTE: documentation says LAM is 1<<3 but observing the real
   card tells us its 1<<2 */
#define CC16_XQLAM_LAM  (1U<<2)  /* LAM pending and enabled */

/* --- get/set inhibit-on-irq  --- */
#define CC16_IOCTL_GETINH _IOR('c',0xb,int)
#define CC16_IOCTL_SETINH _IOW('c',0xc,int)

