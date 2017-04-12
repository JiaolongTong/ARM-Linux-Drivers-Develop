#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <string.h>

#define FIFO_CLEAR 0x1
#define BUFFER_LEN 20

void selectMeans()
{
     int fd,num;
     char red_ch[BUFFER_LEN];
     fd_set rfds,wfds;


     fd=open("/dev/globalfifo",O_RDONLY|O_NONBLOCK);
     if(fd!=-1){

           if(ioctl(fd,FIFO_CLEAR,0)<0)
               printf("ioctl command faild !\n");
           while(1){

                FD_ZERO(&rfds);
                FD_ZERO(&wfds);
                FD_SET(fd,&rfds);
                FD_SET(fd,&wfds);
                select(fd+1,&rfds,&wfds,NULL,NULL);
                if(FD_ISSET(fd,&rfds))
                     printf("Poll mointor:can be read!\n");
                if(FD_ISSET(fd,&wfds))
                     printf("Poll mointor:can be written!\n");   

                sleep(2);             

           }

     }else{
           printf("Device FIFO open failure\n");
     }



}



void epollMeans(){


     int fd;

     fd=open("/dev/globalfifo",O_RDONLY|O_NONBLOCK);
     
     if(fd!=-1){
         struct epoll_event ev_globalfifo;
         int err;
         int epfd; 
         if(ioctl(fd,FIFO_CLEAR,0)<0)
               printf("ioctl command faild !\n");

         epfd = epoll_create(1);
         if(epfd<0){
             perror("epoll_create()");
             return;
         }
         bzero(&ev_globalfifo,sizeof(struct epoll_event));
         ev_globalfifo.events =EPOLLIN | EPOLLPRI;

         err=epoll_ctl(epfd,EPOLL_CTL_ADD,fd,&ev_globalfifo);

         if(err<0){

             perror("epoll_crl()");
             return;

         }

         err = epoll_wait(epfd,&ev_globalfifo,1,15000);

         if(err<0){

              perror("epoll_wait()");
         }else if(err==0){
                printf("No data input in FIFO within 15 seconds\n");

         }else{
              printf("FIFO is no empty\n");
         }
 
         err=epoll_ctl(epfd,EPOLL_CTL_DEL,fd,&ev_globalfifo);
         if(err<0){
             perror("epoll_ctl()");
         }

     }else{
         printf("Device FIFO open failure\n");
     }


}



void main(){

	epollMeans();
}
