#ifndef CC16_LIB_H
#define CC16_LIB_H

/* This file contains inline wrapper functions to the ioctls used
 * by the cc16 driver.
 *
 * License: GPL, Author: vogel@pi2.physik.uni-erlangen.de
 *
 */

#include <cc16.h>
#include <sys/ioctl.h>

/*
 * NAF functions, reading/writing 16/24 bit.
 *
 * 16bit saves one IO operation, compared to 24bit.
 *
 * Specifying a NULL pointer for *q and *x saves one
 * IO operation, because the X/Q-flags are not fetched.
 *
 * The functions return 0 on success, 1 on error.
 *
 * cc16_naf_w24: Write NAF (F=24..31), 24 Bit width
 * cc16_naf_w16: Write NAF (F=24..31), 16 Bit width
 * cc16_naf_r24: Read NAF (F=0..7), 24 Bit width
 * cc16_naf_r16: Read NAF (F=0..7), 16 Bit width
 * cc16_naf    : NAF without read/write (F=8..15 and 24..31)
 *
 */

static inline int 
cc16_naf_w24(int fd, int n,int a,int f,u_int32_t data,int *q, int *x){
	struct cc16_naf_data naf;

	naf.naf = CC16_NAF(n,a,f);

	if(!CC16_NAF_WRITE(naf.naf)){
		fprintf(stderr,"NAF N=%d A=%d F=%d is no write!\n",n,a,f);
		return -1;
	}

	naf.data = data;
	if(q || x) /* user is interested in saving x or q */
		naf.naf |= CC16_NAF_XQ; /* tell driver to query for x and q*/

	if(ioctl(fd,CC16_IOCTL_NAF,&naf)){
		fprintf(stderr,"NAF N=%d A=%d F=%d failed\n",n,a,f);
		return -1;
	}

	if(q)
		*q=!!( CC16_DATA_Q & naf.data );
	if(x)
		*x=!!( CC16_DATA_X & naf.data );
	return 0;
}

/* 16-bit write naf */
static inline int 
cc16_naf_w16(int fd, int n,int a,int f,u_int16_t data,int *q, int *x){
	struct cc16_naf_data naf;

	naf.naf = CC16_NAF(n,a,f)|CC16_NAF_16BIT;

	if(!CC16_NAF_WRITE(naf.naf)){
		fprintf(stderr,"NAF N=%d A=%d F=%d is no write!\n",n,a,f);
		return -1;
	}

	naf.data = data;
	if(q || x) /* user is interested in saving x or q */
		naf.naf |= CC16_NAF_XQ; /* tell driver to query for x and q*/

	if(ioctl(fd,CC16_IOCTL_NAF,&naf)){
		fprintf(stderr,"NAF N=%d A=%d F=%d failed\n",n,a,f);
		return -1;
	}

	if(q)
		*q=!!( CC16_DATA_Q & naf.data);
	if(x)
		*x=!!( CC16_DATA_X & naf.data);
	return 0;
}


/* 24-bit read naf */
static inline int
cc16_naf_r24(int fd, int n,int a,int f,u_int32_t *data,int *q, int *x){
	struct cc16_naf_data naf;

	naf.naf = CC16_NAF(n,a,f);

	if(!CC16_NAF_READ(naf.naf)){
		fprintf(stderr,"NAF N=%d A=%d F=%d is no read!\n",n,a,f);
		return -1;
	}

	if(q || x)
		naf.naf |= CC16_NAF_XQ;

	if(ioctl(fd,CC16_IOCTL_NAF,&naf)){
		fprintf(stderr,"NAF N=%d A=%d F=%d failed\n",n,a,f);
		return -1;
	}

	if(q)
		*q=(CC16_DATA_Q & naf.data) ? 1 : 0;
	if(x)
		*x=(CC16_DATA_X & naf.data) ? 1 : 0;


	if(data)
		*data=CC16_DATA(naf.data);
	return 0;

}

/* 16-bit read naf */
static inline int
cc16_naf_r16(int fd, int n,int a,int f,u_int16_t *data,int *q, int *x){
	struct cc16_naf_data naf;

	naf.naf = CC16_NAF(n,a,f) | CC16_NAF_16BIT;

	if(!CC16_NAF_READ(naf.naf)){
		fprintf(stderr,"NAF N=%d A=%d F=%d is no read!\n",n,a,f);
		return -1;
	}

	if(q || x)
		naf.naf |= CC16_NAF_XQ;

	fflush(stderr);

	if(ioctl(fd,CC16_IOCTL_NAF,&naf)){
		fprintf(stderr,"NAF N=%d A=%d F=%d failed\n",n,a,f);
		return -1;
	}

	if(q)
		*q=(CC16_DATA_Q & naf.data) ? 1 : 0;
	if(x)
		*x=(CC16_DATA_X & naf.data) ? 1 : 0;
	if(data)
		*data=0xffff & CC16_DATA(naf.data);

	return 0;

}

/* naf which is neither read nor write */
static inline int
cc16_naf(int fd, int n,int a,int f,int *q,int *x){
	struct cc16_naf_data naf;

	naf.naf = CC16_NAF(n,a,f);

	if(q || x)
		naf.naf |= CC16_NAF_XQ;

	if(ioctl(fd,CC16_IOCTL_NAF,&naf)){
		fprintf(stderr,"NAF N=%d A=%d F=%d failed\n",n,a,f);
		return -1;
	}

	if(q)
		*q=(CC16_DATA_Q & naf.data) ? 1 : 0;
	if(x)
		*x=(CC16_DATA_X & naf.data) ? 1 : 0;

	return 0;
}

/* select crate */
static inline int
cc16_crate(int fd,int crateno){
	int flags;

	if(ioctl(fd,CC16_IOCTL_CRATE,&crateno)){
		perror("CC16_IOCTL_CRATE");
		return -1;
	}

	if(ioctl(fd,CC16_IOCTL_STATUS,&flags)){
		perror("CC16_IOCTL_STATUS");
		return -1;
	}

	if(flags & CC16_STATUS_B){
		fprintf(stderr,"Crate %d: not turned on? (status=0x%04x)\n",
			crateno,flags);
		return -1;
	}

	return 0;
}


/* enable/disable broadcast calls */
static inline int
cc16_broadcast_on_off(int fd,int onoff){
	int i;
	i=onoff;
	if(ioctl(fd,CC16_IOCTL_ON_OFF,&i)){
		perror("CC16_IOCTL_ON_OFF");
		return -1;
	}
	
	return 0;
}

static inline int
cc16_on_off(int fd,int onoff) __attribute__((deprecated));

static inline int
cc16_on_off(int fd,int onoff){
	return cc16_broadcast_on_off(fd,onoff);
}

/* read status */
static inline int
cc16_status(int fd,int *i){
	if(ioctl(fd,CC16_IOCTL_STATUS,i)){
		perror("CC16_IOCTL_STATUS");
		return -1;
	}
	return 0;
}

/* turn on/off inhibit */
static inline int
cc16_i(int fd,int i){
	int flags;

	if(ioctl(fd,CC16_IOCTL_I,&i)){
		perror("CC16_IOCTL_I");
		return -1;
	}

	if(ioctl(fd,CC16_IOCTL_STATUS,&flags)){
		perror("CC16_IOCTL_STATUS");
		return -1;
	}

	if((!!(flags & CC16_STATUS_I)) != (!!i)){
		fprintf(stderr,"want: I=%d, statusreg: I=%d L=%d B=%d\n",
				!!i,
				!!(flags & CC16_STATUS_I),
				!!(flags & CC16_STATUS_L),
				!!(flags & CC16_STATUS_B));
		return -1;
	}

	return 0;
}

/* turn on/off lam */
static inline int
cc16_lam(int fd,int l){
	int flags;

	if(ioctl(fd,CC16_IOCTL_LAM,&l)){
		perror("CC16_IOCTL_LAM");
		return -1;
	}

	if(ioctl(fd,CC16_IOCTL_STATUS,&flags)){
		perror("CC16_IOCTL_STATUS");
		return -1;
	}

	if((!!(flags & CC16_STATUS_L)) != (!!l)){
		fprintf(stderr,"want: I=%d, statusreg: I=%d L=%d B=%d\n",
				!!l,
				!!(flags & CC16_STATUS_I),
				!!(flags & CC16_STATUS_L),
				!!(flags & CC16_STATUS_B));
		return -1;
	}

	return 0;
}

static inline int
cc16_z(int fd){
	if(ioctl(fd,CC16_IOCTL_Z)){
		perror("CC16_IOCTL_Z");
		return -1;
	}
	return 0;
}

static inline int
cc16_c(int fd){
	if(ioctl(fd,CC16_IOCTL_C)){
		perror("CC16_IOCTL_Z");
		return -1;
	}
	return 0;
}


/* check for LAM */
static inline int
cc16_check_lam(int fd,int *lam){
	int flags;

	if(ioctl(fd,CC16_IOCTL_XQLAM,&flags)){
		perror("CC16_IOCTL_XQLAM");
		return -1;
	}
	*lam = (flags & CC16_XQLAM_LAM);

	return 0;
}

static inline int
cc16_set_inhibit_on_lam(int fd,int onoff){
	int i;
	
	i=onoff;
	if(ioctl(fd,CC16_IOCTL_SETINH,&i)){
		perror("CC16_IOCTL_SETINH");
		return -1;
	}
	
	if(ioctl(fd,CC16_IOCTL_GETINH,&i)){
		perror("CC16_IOCTL_GETINH");
		return -1;
	}
	
	if(i != onoff){
		fprintf(stderr,"%s: could not set flag\n",__FUNCTION__);
		return -1;
	}
	return 0;
}


static inline int
cc16_get_inhibit_on_lam(int fd,int *onoff){

	if(ioctl(fd,CC16_IOCTL_GETINH,&onoff)){
		perror("CC16_IOCTL_GETINH");
		return -1;
	}
	return 0;
}


#endif
