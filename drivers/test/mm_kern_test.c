#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>

void main(void){


     printf("This Process pid=%d\n",getpid());
     unsigned char  *buf;
      int i;
     int fd = open("/dev/mmap_driver",O_RDWR);
     buf=mmap(NULL,8192,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
     if(buf==NULL)
        printf("mmap error\n");
     printf("after data is :\n");
     for( i=0;i<60;i++)
           printf("%02x ",buf[i]);
     printf("\n");
     for( i=0;i<30;i++)
           buf[i] ='t';
     for( i=0;i<60;i++)
           printf("%02x ",buf[i]);
     printf("\n");
     close(fd);
     while(1);


}
