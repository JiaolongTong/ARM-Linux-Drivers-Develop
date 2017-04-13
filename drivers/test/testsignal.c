#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#define MAX_LEN 100

static fd = 0;
void input_handle(int num){

    unsigned char buffer[MAX_LEN];
    lseek(fd,0,SEEK_SET);
    read(fd,buffer,13);
    printf("recvice a signal from globalfifo, signalnum is :%d \n buffer:%s \n",num,buffer);

}

void signal_test(){                          // 应用层处理标准输入设备(STDIN_FILENO)的异步信号

     int oflags;



     signal(SIGIO,input_handle);


     fcntl(STDIN_FILENO,F_SETOWN,getpid());

     oflags= fcntl(STDIN_FILENO,F_GETFL);

     fcntl(STDIN_FILENO,F_SETFL,oflags | FASYNC);



      while(1);


}

void signal_glabalfifo_device(){


     int oflags;

     fd = open("/dev/globalfifo",O_RDWR|S_IRUSR|S_IWUSR);

     if(fd!=-1){

             printf("1\n");
	     signal(SIGIO,input_handle);

             printf("2\n");
	     fcntl(fd,F_SETOWN,getpid());
             printf("3\n");
	     oflags= fcntl(fd,F_GETFL);
             printf("4\n");
	     fcntl(fd,F_SETFL,oflags | FASYNC);
             printf("5\n");
	     while(1){
                  //printf("6\n");
		  sleep(100);
             }
     }else{

         printf("device open failure\n");

     }

}

void main(){



    signal_glabalfifo_device();

}
