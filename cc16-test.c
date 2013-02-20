#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>

#include "cc16.h"

int main(int argc,char **argv){
	int fd;
	int i,j;

	if(-1 == (fd=open("/cc16",O_RDWR))){
		perror("/cc16");
		return -1;
	}

	j=0; /* crate 0 */
	ioctl(fd,CC16_IOCTL_CRATE,&j);
	j=1; /* turn on crate */
	ioctl(fd,CC16_IOCTL_ON_OFF,&j);
	j=0; /* inhibit off */
	ioctl(fd,CC16_IOCTL_I,&j);

	sleep(1);
	struct cc16_naf_data naf = {
		CC16_NAF(12,j,26)|CC16_NAF_XQ,    /* naf */
		0                    /* data */
	};
	ioctl(fd,CC16_IOCTL_NAF,&naf);

	sleep(1);




	for(j=0;j<12;j++){
		struct cc16_naf_data naf = {
			CC16_NAF(12,j,2)|CC16_NAF_16BIT|CC16_NAF_XQ,    /* naf */
			0                    /* data */
		};
		ioctl(fd,CC16_IOCTL_NAF,&naf);
		printf("naf(12,%d,8) -> 0x%08x\n",j,naf.data);
		sleep(1);
	}

	ioctl(fd,CC16_IOCTL_STATUS,&j);
	printf("status 0x%04x\n",j);
	j=0; /* inhibit on */
	ioctl(fd,CC16_IOCTL_I,&j);
	ioctl(fd,CC16_IOCTL_STATUS,&j);
	printf("status 0x%04x\n",j);

	sleep(1);

	j=0; /* turn off crate */
	ioctl(fd,CC16_IOCTL_ON_OFF,&j);
	


/*
	printf("Trying select loop.\n");
	do {
		fd_set readfds;
		int i,n;

		FD_ZERO(&readfds);
		FD_SET(fd,&readfds);

		select(fd+1,&readfds,NULL,NULL,NULL);

		i=read(fd,&n,sizeof(i));

		if(4 != i){
			perror("read");
			exit(1);
		}
		printf("trigger %d\n",n);



	} while(1);
*/	
	return 0;
}
