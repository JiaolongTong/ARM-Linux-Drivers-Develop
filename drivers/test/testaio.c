#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <aio.h>
#include <errno.h>

#define  BUFFERSIZE 20

void main(){

     int fd,ret;
int i;
     struct aiocb my_aiocb;


     fd= open("file.txt",O_RDONLY);

     if(fd<0)
        perror("open()");


     bzero(&my_aiocb,sizeof(struct aiocb));

     my_aiocb.aio_buf =(char *) malloc(BUFFERSIZE+1);

     if(!my_aiocb.aio_buf)
           perror("molloc()");

     my_aiocb.aio_fildes = fd;
     my_aiocb.aio_nbytes = BUFFERSIZE;
     my_aiocb.aio_offset = 0;

     ret=aio_read(&my_aiocb);

     if(ret<0)
        perror("aio_read()");


     while( aio_error(&my_aiocb) == EINPROGRESS ) continue;

      if((ret=aio_return(&my_aiocb))>0){


            printf("file read:");
            i=0;
            while(i<10){

                  printf("%c",my_aiocb.aio_buf[i] );
                  i++;
            }
            printf("\n");
      }
      



}
