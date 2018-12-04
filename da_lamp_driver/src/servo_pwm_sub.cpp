#include "ros/ros.h"
#include "da_lamp_msgs/JointState.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>


int file;
char filename[11]; // = "/dev/i2c-1";
__u8 addr = 0x40; //  0b01000000;        // The I2C address of the PWM-Hat

enum pwm_regs {
  // Registers/etc.
  __MODE1              = 0x00,
  __MODE2              = 0x01,
  __SUBADR1            = 0x02,
  __SUBADR2            = 0x03,
  __SUBADR3            = 0x04,
  __PRESCALE           = 0xFE,
  __LED0_ON_L          = 0x06,
  __LED0_ON_H          = 0x07,
  __LED0_OFF_L         = 0x08,
  __LED0_OFF_H         = 0x09,
  __ALL_LED_ON_L       = 0xFA,
  __ALL_LED_ON_H       = 0xFB,
  __ALL_LED_OFF_L      = 0xFC,
  __ALL_LED_OFF_H      = 0xFD,
  __RESTART            = 0x80,
  __SLEEP              = 0x10,
  __ALLCALL            = 0x01,
  __INVRT              = 0x10,
  __OUTDRV             = 0x04,
  __PWM_FULL           = 0x01000
};





float pie=3.14159; 

int sinsize=1000; //can not be <400 or min servo resolution default sin wave size (trough to peak or pie in length)
float event=sinsize * 5;  //cycles count limiter not really important as long as speed of servo isn't super slow  


//specific variables 
// servo default positions 
// WARNING:  when a higher or lower number is input than what the servo can handle, 
//           the servo may rapid to some manufacturer safe value 
// WARNING: refer to technical documentation for each servo to stay within its range. 
//  ccw   \->/   cw
//   600- 1200- 2400   =180 sweep servo  type 1800 point resolution
//  1050- 1550 - 1950   =90 sweep servo  type  900 point resolution
//  1500- 1700 - 1900   =1260 sweep winch type 400 point resolution 


//variables for use in all servo functions per servo

float count1 =0; // sine wave x variable counts from 1 to 1700 (servo resolution) only starts counting after wait# reaches its activation value.                 
float speed1=0;  //a value that consists of small increment values that change in magnitude depending on if the wave starts slow and ends fast or vise versa.  
float speedtick1=0; //ticks off how long the hold position for the servo is in very small ms increments giving the illusion of a slower or faster moving servo 

                 // y= a*sin(bx+c)+d 
                 // yvar# = a#(sin (b#*count#+c#)+d#  
float yvar1=0;   // actual ms value thrown at servo ranged, paused, speed shifted etc.  
float a1=0;     //a# amplitude higher value taller wave shorter value shorter wave by magnitude:   a=(highest # - lowest #)/2
float b1=0;     //b# lower value = longer wave or higher value=shorter wave this is phase shift or stretch of function b=2pi/(blabla*2) where blabla are desired wave size 
float c1=0;     //c# is x frequency offset = what part of curve want to look at 
float d1=0;     //d# is y offset  = 0.5*amplitude shifts the curve so it is wholey in 1st quadrant 
int per1=0;     //trigger value either 0 or 1 to declare that that servo has reached its final position and so servo movement sequence of all servos (once all report per#=1)can end.






void setAllPWM(int on, int off){
	int res;
	res = i2c_smbus_write_byte_data(file, __ALL_LED_ON_L, on & 0xFF);
	if(res < 0)
		perror("resultaterror");

	res = i2c_smbus_write_byte_data(file, __ALL_LED_ON_H, on  >> 8);
	if(res < 0)
		perror("err2");

	res = i2c_smbus_write_byte_data(file, __ALL_LED_OFF_L, off & 0xFF);
	if(res < 0)
		perror("err3");

	res = i2c_smbus_write_byte_data(file, __ALL_LED_OFF_H, off >> 8);
	if(res < 0)
		perror("err4");
};

void setPWM(int channel, int value){
	int on = 0; // change it if you need to start PWM cycle with phase delay
	int off = value;
	int res;

	res = i2c_smbus_write_byte_data(file, __LED0_ON_L+4*channel, on & 0xFF);
	if(res < 0)
		perror("ierr1");

	res = i2c_smbus_write_byte_data(file, __LED0_ON_H+4*channel, on >> 8);
	if(res < 0)
		perror("ierr2");

	res = i2c_smbus_write_byte_data(file, __LED0_OFF_L+4*channel, off & 0xFF);
	if(res < 0)
		perror("ierr3");

	res = i2c_smbus_write_byte_data(file, __LED0_OFF_H+4*channel, off >> 8);
	if(res < 0)
		perror("ierr4");


};

void setPWMFreq(int freq){
	int prescale;
	char oldmode, newmode;
	int res;
	float prescaleval = 25000000.0; // 25MHz
	prescaleval /= 4096.0;
	prescaleval /= (float)freq;
	prescaleval -= 1.0;

	ROS_INFO("Setting PWM frequency to %d Hz", freq);
	ROS_INFO("Estimated pre-scale: %f", prescaleval);

	prescale = floor(prescaleval + 0.5);
	ROS_INFO("Final pre-scale: %d", prescale);

	nanosleep((const struct timespec[]){{1, 000000L}}, NULL);


	oldmode = i2c_smbus_read_byte_data(file, __MODE1);
	newmode = (oldmode & 0x7F) | 0x10; // sleep

	res = i2c_smbus_write_byte_data(file, __MODE1, newmode); // go to sleep
	if(res < 0)
		perror("serr0");

	res = i2c_smbus_write_byte_data(file, __PRESCALE, (int)(floor(prescale)));
	if(res < 0)
		perror("serr1");

	res = i2c_smbus_write_byte_data(file, __MODE1, oldmode);
	if(res < 0)
		perror("serr2");

	nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec,

	res = i2c_smbus_write_byte_data(file, __MODE1, oldmode | 0x80);
	if(res < 0)
		perror("serr3");

};


int getPWM(int channel) {
   int regAddress = __LED0_ON_L + (channel << 2);

   int res = i2c_smbus_write_byte_data(file, addr, regAddress);
   if(res < 0)
       perror("gerro1");

   int on = i2c_smbus_read_byte_data(file, __LED0_ON_L+channel*4);
   on |= i2c_smbus_read_byte_data(file, __LED0_ON_H+channel*4) << 8;
   int off = i2c_smbus_read_byte_data(file, __LED0_OFF_L+channel*4);
   off |= i2c_smbus_read_byte_data(file, __LED0_OFF_H+channel*4) << 8;

   int retVal;

   //printf("regAdd: 0x%02X \n", regAddress);
   //printf("on: 0x%02X \n", on);
   //printf("off: 0x%02X \n", off);

   if (off >= __PWM_FULL)
        // Full OFF
        retVal = 0;
   else if (on >= __PWM_FULL)
        // Full ON
        retVal = __PWM_FULL;
   else if (on <= off)
        // start and finish in same cycle
        retVal = off - on;
   else
        // span cycles
        retVal = (off + __PWM_FULL) - on;

   return retVal;
}



//start of primary move function that includes all servos and is called up and activated per each event
void movef(float ecycle,float w81,float spa1,float spb1,float yprev1,float ynext1, int i)
{
         usleep(1);   //master delay 
	 count1 = 1;   //reset of count# 
         speedtick1 = 1;  //reset of speedtick#
         b1=(2*pie/(sinsize*2));  //coefficient of sine math function 
         
          
	    if(ynext1 == yprev1)
	    {
              return; 
	    }
            else if(ynext1 > yprev1)  //sets sine wave coefficients depending on positions
            {
              a1= (ynext1-yprev1)/2;  //coefficient of sine math function 
              c1= (1.5)*pie;           //coefficient of sine math function 
              d1= yprev1+a1;            //coefficient of sine math function 
            }
            else  //(ynext# < yprev#)
            {
              a1= (yprev1-ynext1)/2;
              c1= (0.5)*pie;  
              d1= yprev1-a1; 
            }
            
              //###coefficient establishment for all sine waves end ###
            
            int per1=0; //reset of all trigger values in case there is carry over from a pindexevious run
           
              
       //global loop for all servos cycles by each servo and sees if it has an tiny move.                             
       for (float count = 0; count < ecycle; count +=1)  
       {  
         //traditional speed values start off as spa# and end up as spb# as count# ticks away on the fly as curve is being drawn. 
        // result is a sine curve that is compressed in the x axis on one end (spa#=large number) and stretched on other end (spb#=small number).   
         
         if (spa1 > spb1) {speed1=((count1+1)/sinsize)*(spa1-spb1) + spb1;} //start fast end slow 
         else {speed1= ((count1+1)/sinsize)*(spb1-spa1)+ spa1;} // start slow end fast 
         
         //ROS_INFO("servo # %d, at angle %f", i, yvar1);
        
  // servo #1   3 states or cases 
        if (count < w81) //case 1 servo not ready to move yet      
            {
            setPWM(i, (int)yprev1);  
            
            }
        
         else if (count>w81 && count1 > sinsize) //case 3 motion is done and position is held 
            {
            setPWM(i, (int)ynext1); 
            
            per1=1; //declares this servo is finished with its movement 
            }
         
         else if (count > w81)   //case 2 sin wave function active with optional hold position while big loop asks other servos for their turn  
            {
              
              if (count1 < sinsize && speedtick1 == 1)  //new position of servo is written 
                {   
                   yvar1= a1*sin((count1)*b1+c1)+d1;  //the math function
                   setPWM(i, (int)yvar1);   //throws a command at the servo 
                   speedtick1 += 1; // start of increment to count for possible pauses at this position to simulate slow 
                   count1 += 1; //increments sine wave operator x in y=f(x) 
                   
                }
                else if (speedtick1 > 1 && speedtick1 < speed1)  //sine wave is sustained at old value for 1 to speed# as counted by speedtick# 
                {
                  setPWM(i, (int)yvar1); 
                  speedtick1 += 1;  //increments speedtick1 to delay the servo at one position along its travel to simulate a speed
                  
                }
                else //sine wave is now permitted to continue by having speedtick# reset 
                {
                  count1+=1; //lock out the sine function from going past sinsize by ever increasing count# 
                 speedtick1 = 1; //reset for next fly through of sine function 
                 
                }  
            }  //end if statement for case 2
 

       if(per1 == 1)//breaks FOR loop out of further un necessary cycles as all servos report their movement complete
         {
          break; 
         }
    
   }  //end of for loop statement for all servo steps  
 
} //end of void subroutine function for entire move function 





void initPWMHat(){
	int res;
	char mode1res;

	if ((file = open(filename, O_RDWR)) < 0) {
    		/* ERROR HANDLING: you can check errno to see what went wrong */
    		perror("Failed to open the i2c bus");
    		exit(1);
	}

	if (ioctl(file,I2C_SLAVE,addr) < 0) {
	    	printf("Failed to acquire bus access and/or talk to slave.\n");
    		/* ERROR HANDLING; you can check errno to see what went wrong */
    		exit(1);
	}

	res = i2c_smbus_write_byte_data(file, __MODE2, __OUTDRV);
	if(res < 0)
		perror("erro1");

	res = i2c_smbus_write_byte_data(file, __MODE1, __ALLCALL);
	if(res < 0)
		perror("erro2");

	nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci

	mode1res = i2c_smbus_read_byte_data(file, __MODE1);
	mode1res = mode1res & ~__SLEEP; //                 # wake up (reset sleep)

	res = i2c_smbus_write_byte_data(file, __MODE1, mode1res);
	if(res < 0)
		perror("erro3");

	nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci

}

void chatterCallback(const da_lamp_msgs::JointState::ConstPtr& msg)
{
	short int arr[16];
	int i=0;
	int prePWM=0;

	

	movef(event,200,3,3,(float)getPWM(1),(float)msg->Joint1,1);
	movef(event,200,3,3,(float)getPWM(2),(float)msg->Joint2,2);
	movef(event,200,3,3,(float)getPWM(3),(float)msg->Joint3,3);
	movef(event,200,3,3,(float)getPWM(4),(float)msg->Joint4,4);
	movef(event,200,3,3,(float)getPWM(5),(float)msg->Joint5,5);

	// for(std::vector<short int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	// {
	// 	arr[i] = *it;
	// 	//i++;
	// 	ROS_INFO("val:[%d]", arr[i]);

	// 	if(arr[i] < 4096)
	// 			
	// 		ROS_INFO("get PWM: %d", getPWM(i));
	// 		movef(event,200,3,3,(float)getPWM(i),(float)arr[i],i);
	// 		///setPWM(i,arr[i]);

	// 	//ROS_INFO("Getting PWM %d from channel %d", getPWM(i), i);
	// 	i++;
	// 	
	// }

  //ROS_INFO("I heard: [%d]", msg->data);
}

int main(int argc, char **argv)
{
	snprintf(filename, sizeof(filename), "/dev/i2c-%d", 1);

	initPWMHat();

	setPWMFreq(50);  //50 Hz

	ros::init(argc, argv, "servo_pwm_sub");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("servo_pwm", 1000, chatterCallback);

	ros::spin();

	close(file);

  return 0;
}
