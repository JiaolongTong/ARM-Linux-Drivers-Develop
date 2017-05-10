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
     int fd = open("./hello.c",O_RDWR);
     buf=mmap(NULL,66,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
     if(buf==NULL)
        printf("mmap error\n");
     printf("after data is :\n");
     for( i=0;i<66;i++)
           printf("%c",buf[i]);
     printf("\n");
     close(fd);
     while(1);


}
