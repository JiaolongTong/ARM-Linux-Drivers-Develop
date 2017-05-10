#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
void main(void){

     int fd;
     int counter =0;
     int old_counter =0;
     
     fd= open("/dev/second",O_RDONLY);

     if(fd!=-1){

        while(1){

               read(fd,&counter,sizeof(unsigned int));

               if(counter!=old_counter){

                      printf("seconds after open /dev/second :%d s\n",counter);
                      old_counter =counter;
               }
        }

     }else{

               printf("Device open failed\n");
     }


}
