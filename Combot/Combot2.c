#ifdef __cplusplus
  extern "C" {
#endif

//Includes
#include <stdio.h>
#include <stdlib.h>
#include "kbhit.h"
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <ftdi.h>
#include "inc/fmod.h"
#include <sys/stat.h>
#include <fcntl.h>
#include </usr/include/linux/joystick.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <wiringSerial.h>
#include <wiringPi.h> 
#include <softPwm.h>

//Led Functions
int updateColor( int color, int step);
void fadingColor(long double delayvariable);
void randomLED(void);
void angry(void);
void setColor( char color);
void mp3(char music);
void LEDoff(void);
void siren(void);
void fogHorn(void);
void Monster(void);
void Country(void);
void Classical (void);
void Top40(void);
void Rock (void);
void MusicFlagReset(void);

//Prototypes
void *LEDthread(void *not_used);
void *VideoThread(void *not_used);
void *LoopThread(void *not_used);
void *joystickThread(void *not_used);
void *ButtonCallThread(void *not_used);
void *MusicThread(void *not_used);


//defines
#define VideoDelay 3100000
#define bytes 5																		//characters in serial #MTSS=5
#define MaxSpeed 250000  	
#define fname "/dev/input/js0"
#define fname2 "/dev/input/js0"
#define serialname "/dev/ttyACM0"
#define serialname2 "/dev/ttyACM1"
#define bytes 5																		//characters in serial #MTSS=5
#define MaxSpeed 250000  													//250ms delay max speed
#define JOY_DEV "/dev/input/js0"											//default joystick mount point
#define JOY_DEV2 "/dev/input/js1"											//alternate joystick mount point 
#define JS_EVENT_BUTTON         0x01    									/* button pressed/released */
#define JS_EVENT_AXIS           0x02    									/* joystick moved */
#define JS_EVENT_INIT           0x80    									/* initial state of device */
#define pwmScale 1
#define period 225															//period for pwm approx 4Khz
//masks
#define PWMDRRIGHTON 0b00001000												
#define PWMDRRIGHTOFF 0b11110111
#define PWMDRLEFTON 0b00000001
#define PWMDRLEFTOFF 0b11111110
#define ioMask 0b00111111
//LED Defines
////LED Defines
#define RED_PIN 17
#define BLUE_PIN 22
#define GREEN_PIN 24
#define red 1
#define blue 2
#define green 3
#define purple 4
#define yellow 5
#define pink 6
#define iceBlue 7
#define orange 8
#define lightGreen 9
#define white 10
//music define
#define rock 1
#define country 2
#define classical 3
#define top40 4

//globals
unsigned char mode=0;//0=auto 1=Manual
//unsigned char snooze=0;//1 = sleep 0 = Awake
unsigned int 	throttle=63,steering=63,throttleOld=63,steeringOld=63;		//values to be sent by serial speeds					
char inputFlag=0;
char 	bufOut[6];							//serial bufOutfer	
char    bufIn[6];
char ledTrigger=0;
char LEDFlag=0x00;

char MusicTrigger=0;
int MusicFlag=0x00;
char mMode=0;
char mNumber=0;

int CountryFlag=0;
int Top40Flag=0;
int ClassicalFlag=0;
int RockFlag=0;
int soundFlag=0;
char lmode=0x00;
char number=0x00;


unsigned char 	buttonTrigger=0;					//sound call holder
unsigned char   joytest=1;	
unsigned char   joyCheck=0;
unsigned char   parChange=0;
unsigned char   playVideo=0;
unsigned char   VloopFlag=1;
int mFlag=0;	

	
int main(void){


	//Open up the BitbangPort
	
    int fd,i,i2,q=0;
    int retval = 0,serialWaiting=0;
		unsigned char   serialFlag=0;
		float throttleMath=63;
		float steeringMath=63;
  
	
		bufOut[0]='#'; //initialize bufOutfer
		bufOut[1]='S';
		bufOut[2]=throttle;
		bufOut[3]=steering;
		bufOut[4]=0;
		bufOut[5]=0;
   
   while( access( serialname, F_OK ) == -1 ) {
		// file does not exist so wait here until it does
		
		sleep(1);
		} 
if ((fd = serialOpen ("/dev/ttyACM0", 115200)) < 0)
  {
    if ((fd = serialOpen ("/dev/ttyACM1", 115200)) < 0){
		
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    //return 1 ;
		}
  }
	else {
	serialFlag=1;
	}

   

   

	//setup and open threads.
	pthread_t tid2,tid3,tid4,tid5,tid6;							

	pthread_create(&tid2,NULL,LEDthread,NULL);
	printf("\nCreated LEDthread \r\n");
	
	//pthread_create(&tid3,NULL,pwmRightThread,NULL);
	//printf("Created PWM Right thread \r\n");

	pthread_create(&tid3,NULL,joystickThread,NULL);
	printf("Created Main Program thread \r\n");
	
	pthread_create(&tid4,NULL,ButtonCallThread,NULL);
	printf("Created Button Call Program thread \r\n");
	
	pthread_create(&tid5,NULL,LoopThread,NULL);
	printf("Created Video Loop Program thread \r\n");
	
	pthread_create(&tid6,NULL,MusicThread,NULL);
	printf("Created Music thread \r\n");
	
    
	
	
	
	
	
	//system("sudo ./sixpair");
	joytest=0;						//allow bluetooth detection to old joystick
	
	//printf("\nPlaying Sound\n\n");
	//system("aplay -q ./sounds/UFO_Takeoff.wav"); //let user know he is out of time to pair??
	
	
	//start the loop here maybe
	
	
	
	while(1){
		
		//continuously write to serial output if something has changed
		//this is where we send the packets from
		//SEND # A Throttle Steering mode sleep 
		//or # B k values
		if(serialFlag==1){
				if((throttle<67&&throttle>60&&steering<67&&steering>60)&&inputFlag==1){ //if stopped steering then send last byte with zero input commands
					inputFlag=0;	
					
					for(i=0;i<bytes;i++){
						serialPutchar(fd,bufOut[i]);  //output to serial
						
						//printf("%d  %d  \n",bufOut[2],bufOut[3]);
						//printf("%c",bufOut[i]);
						//if(i==bytes)printf("\n");
					}//end for
				}
			
				if(throttle>=67||throttle<=60||steering>=67||steering<=60){ //if steering
					
					if(inputFlag==0)inputFlag=1;	
					
					for(i=0;i<bytes;i++){
						serialPutchar(fd,bufOut[i]);  //output to serial
						
						//printf("%d  %d  \n",bufOut[2],bufOut[3]);
						//printf("%c",bufOut[i]);
						//if(i==bytes)printf("\n");
					}//end for
				}//end reason to send data
				throttleOld=throttle;
				steeringOld=steering;
				if(serialDataAvail(fd)){
					serialWaiting=serialDataAvail(fd);
					usleep(100);
					for(i2=0;i2<serialWaiting;i2++){
						bufIn[i2]=serialGetchar(fd);
						if(bufIn[0]!='#'){
							serialFlush(fd);//if the first character is not the start of a packet dump the buffer
							fflush (stdout);
							i2=99; 	//flag for loop to stop
						
						 }
						
						
					}//end for loop
					
				}//end loop
				if(bufIn[1]=='P'){
						//printf(	"%c ",bufIn[0]);
						//printf(	"%c ",bufIn[1]);
						//printf(	"%c ",bufIn[2]);
						//printf(	"%c ",bufIn[3]);
						//printf(	"%c ",bufIn[4]+18);
						//printf(	"%c \n",bufIn[5]);	
							
						bufIn[1]='@';
						
						if(bufIn[4]==4||bufIn[4]==5||bufIn[4]==6||bufIn[4]==7){ //"country" "Rock" "classical" "top 40" "pop"
							system("sudo pkill mpg123");
							mFlag=0;
						}
						if(bufIn[4]==99){		//"turn off music"
							  
							  MusicFlagReset();
							  system("sudo pkill mpg123");
							  mFlag=0;
							  LEDoff();
							  MusicTrigger=0;					  

						}
						if(bufIn[4]==98){		//"turn off LEDs"
							 
							ledTrigger=0;		
							LEDoff();
						}
						
						if(bufIn[4]==97){		//"Low Battery from arduino"
							MusicFlagReset();
							system("sudo pkill mpg123");
							mFlag=0;
							LEDoff();
							MusicTrigger=0;	
							bufIn[4]=3;
							ledTrigger=0;		
							LEDoff();
							
						}
						
						
						
						
						
						
						buttonTrigger=bufIn[4];
						printf("%d \n",buttonTrigger);
							
											
						}//end else
				usleep(10000);//update 1000 times a second
				
				
		}//endif
		if(serialFlag==0){
			throttleMath=throttle*.0078125;
			steeringMath=steering*.0078125;
			//printf(" %f    %f \n ",throttleMath,steeringMath);
			//printf(" %d    %d \n ",throttle,steering);
		}//endif
		//printf("help");
	}//end while1
	printf("help");
//*.003921
}//end main


/////////////////////////////////////////////////////////////////////////////////////////////////////////
void *MusicThread(void *not_used)
{
	
	//int MusicFlag=0x00;
	//char mMode=0;
	//char mNumber=0;
	
	while(1)
	{	
		if(MusicTrigger==1)
		{
			mMode=((MusicFlag & 0xF0)>>4);  //Sets mode to the first four bits four of LEDFlag
			mNumber=(MusicFlag & 0x0F);     //Sets number to the last four bits of LEDFlag 
			
			switch(mMode)                 //depending on the value of LEDFlag set when a button is pressed, different LED codes are played
			{
			
				case 0x3:
					switch(mNumber)
					{
						case 0x0:  //for country music playing
						Country();
								break;
						case 0x1:  //for rock music
						Rock();
								break;
						case 0x2:  //for classical
						Classical();
								break;
						case 0x3:  //for top40
								
						Top40();
								break;
						case 0x4:
								
								break;
						case 0x5:
								break;
						default:
								break;
					}
				break;
				
				default:
				break;
				
			} //end switch
	
		
		}//if ledTrigger==1
		usleep(100000);
	}//end while
		
	

		
} //end led thread	


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void *LEDthread(void *not_used)  //drives pwm timing
{
	
	wiringPiSetupGpio();      //necessary function call to set up GPIO Pins on pi with wiringPi
	softPwmCreate(17,0,100);  //creates pwm input for LEDs
	softPwmCreate(22,0,100);
	softPwmCreate(24,0,100);
	srand (time(NULL));   //seed for random number generator. Key to being able to generate a number more than once a second is having this only called ONCE in the whole program
	
	while(1)
	{	
		if(ledTrigger==1)
		{
			lmode=((LEDFlag & 0xF0)>>4);  //Sets mode to the first four bits four of LEDFlag
			number=(LEDFlag & 0x0F);     //Sets number to the last four bits of LEDFlag 
			
			switch(lmode)                 //depending on the value of LEDFlag set when a button is pressed, different LED codes are played
			{
				case 0x0:       //mode one is for setting a single LED color
					switch(number)
					{

						case 0x0:
							setColor(red);
						break;
						
						case 0x1:  
							setColor(blue);
						break;
						
						case 0x2:  
							setColor(green);						
						break;
						
						case 0x3:
							setColor(purple);
						break;
						
						case 0x4:
							setColor(yellow);
						break;
						
						case 0x5:
							setColor(pink);
						break;
						
						case 0x6:
							setColor(iceBlue);
						break;
						
						case 0x7:
							setColor(orange);
						break;
						
						case 0x8:
							setColor(lightGreen);
						break;
						
						case 0x9:
							setColor(white);
						break;
						
						
						default:
								break;
					}
				break;
				
				case 0x1:
					switch(number)
					{
						case 0x0:   //Angry LED Effects
								angry();
								break;
						case 0x1:   //Slow Fade
								fadingColor(10);
								break;
						case 0x2:   //Faster Fade
								fadingColor(100);	
								break;
						case 0x3:   //Selects random LED Color
								randomLED();
								break;
						case 0x4:   //PEW PEW LED Effects
								soundFlag=1;
								LEDoff();
								setColor(iceBlue);
								delay(150);
								LEDoff();
								ledTrigger=0;
								soundFlag=0;
								break;
						case 0x5: //siren effect
								
								siren();
								break;
								
						case 0x6: //Fog Horn effect
								
								fogHorn();
								break;
								
						case 0x7: //GunShot
								LEDoff();
								setColor(white);
								delay(400);
								LEDoff();
								ledTrigger=0;
								break;	
						case 0x8:  //monster
								
								Monster();
								
						default:
								break;
					}
				break;
				
				case 0x2:
					switch(number)
					{
						case 0x0:
								break;
						case 0x1:
								break;
						case 0x2:
								break;
						case 0x3:
								break;
						case 0x4:
								break;
						case 0x5:
								break;
						default:
								break;
					}
				break;
				
				default:
				break;
			
				
			} //end switch
	
		
		}//if ledTrigger==1
		usleep(100000);
	}//end while
		
	

		
} //end led thread	


void MusicFlagReset(void)
{
	CountryFlag=0;
	Top40Flag=0;
	ClassicalFlag=0;
	RockFlag=0;
	
	
	
}
void Country(void)
{
		int i=0;
		int r=10;
		int g=1;
		
		
		while(CountryFlag==1)
		{		
			for(i=0;i<=40;i++)
			{
				if(soundFlag==1 || CountryFlag==0)
				{
					break;	
				}
				else
				{
					r = updateColor(r, 1);
					softPwmWrite(RED_PIN,r);
					g = updateColor(g, 1);
					softPwmWrite(GREEN_PIN,g);
				}
			
				delay(100);
			}
			
			for(i=0;i<=40;i++)
			{
				
				if(soundFlag==1 || CountryFlag==0)
				{
					break;	
				}
				else
				{
					r = updateColor(r, -1);
					softPwmWrite(RED_PIN,r);
					g = updateColor(g, -1);
					softPwmWrite(GREEN_PIN,g);
				}
				
				delay(100);
			}
			
			
		}
		while(soundFlag==1){
		delay(100);
		}
		if(soundFlag==0 && CountryFlag==1)Country();
		LEDoff();
		
		MusicTrigger=0;
}	


void Top40 (void)
{
	int i=0;
	
	while(Top40Flag==1)
	{
		if(soundFlag==1 || Top40Flag==0)  	//if dpad up is pressed a second time, it stops playing country music
				{
					break;
				}
		
		fadingColor(10);
		for(i=0;i<=6;i++)
		{
			if(soundFlag==1 || Top40Flag==0)  	//if dpad up is pressed a second time, it stops playing country music
				{
					break;
				}
			
		randomLED();
		delay(1000);
		
		}
		
	}
			while(soundFlag==1){
		delay(100);
	}
	if(soundFlag==0 && Top40Flag==1)Top40();
		LEDoff();
		MusicTrigger=0;
}

void Rock(void)
{
	
	while(RockFlag==1)
	{	
	
		if(soundFlag==1 || RockFlag==0)    //if a sound is triggered, program will wait till its finished to continue country LED loop
		{
			break;
		}
		
		
		randomLED();
		delay(1000);
		
	}
	while(soundFlag==1){
		delay(100);
	}
	if(soundFlag==0 && RockFlag==1)Rock();
	LEDoff();
	MusicTrigger=0;
}

void Classical(void)
{
	
	while(ClassicalFlag==1)
	{	
		
		
		if(soundFlag==1 || ClassicalFlag==0)
		{
			break;
		}
		
			fadingColor(100);
		
	}
		while(soundFlag==1){
		delay(100);
	}
	if(soundFlag==0 && ClassicalFlag==1)Classical();
	LEDoff();
	MusicTrigger=0;
	
	
	
	
}

void Monster (void)
{
	int i=0;
	soundFlag=1;
	delay(500);
	for(i=0;i<=10;i++)
	{
		setColor(i);
		delay(300);
		
	}
	LEDoff();
	ledTrigger=0;
	soundFlag=0;
}



void fogHorn(void)
{
	int i=0;
	int b=0;
	
	LEDoff();
	soundFlag=1;
	for(i=0;i<30;i++)
	{
	b = updateColor(b, 4);
	softPwmWrite(BLUE_PIN,b);
	delay(100);
	}
	
	for(i=0;i<40;i++)
	{
	b = updateColor(b, -4);
	softPwmWrite(BLUE_PIN,b);
	delay(100);
	}
	LEDoff();
	ledTrigger=0;
	soundFlag=0;
}

	
void siren(void)
{
	
	int n =0;
	
	
	LEDoff();
	soundFlag=1;
	for(n=0;n<23;n++)
	{
		setColor(red);
		delay(70);
		setColor(blue);
		delay(70);
	}
		LEDoff();
		ledTrigger=0;
		soundFlag=0;
		
}

void setColor( char color)   //function to set LEDs to a specific color
{
	switch(color)
	{
		case red:
		softPwmWrite(22,0);
		softPwmWrite(24,0);
		softPwmWrite(17,40);
		
		break;
		
		case blue:
		softPwmWrite(17,0);
		softPwmWrite(24,0);
		softPwmWrite(22,40);
		break;
		
		case green:
		softPwmWrite(17,0);
		softPwmWrite(22,0);
		softPwmWrite(24,40);
		break;
		
		case purple:
		softPwmWrite(24,0);
		softPwmWrite(17,40);
		softPwmWrite(22,40);
		break;
		
		case yellow:
		softPwmWrite(22,0);
		softPwmWrite(17,40);
		softPwmWrite(24,10);
		break;
		
		case pink:
		softPwmWrite(24,0);
		softPwmWrite(17,40);
		softPwmWrite(22,10);
		break;
		
		case iceBlue:
		softPwmWrite(17,0);
		softPwmWrite(22,40);
		softPwmWrite(24,40);
		break;
		
		case orange:
		softPwmWrite(22,0);
		softPwmWrite(17,40);
		softPwmWrite(24,2);
		break;
		
		case lightGreen:
		softPwmWrite(17,30);
		softPwmWrite(22,2);
		softPwmWrite(24,40);
		break;
		
		case white:
		softPwmWrite(17,40);
		softPwmWrite(22,40);
		softPwmWrite(24,40);
		
		
		
		//soft Pink
		/*softPwmWrite(17,40);
		softPwmWrite(22,3);
		softPwmWrite(24,10);*/
		
		break;
	
	}
	
}
void angry(void)  //function for anger, leds start out dull, go to max brightness then flash
{
	int i=0;
	softPwmWrite(17,0);  //makes sure all colors are off before starting
	softPwmWrite(22,0);
	softPwmWrite(24,0);
	
	softPwmWrite(17,20);
	delay(250);
	softPwmWrite(17,40);
	delay(500);
	softPwmWrite(17,50);
	delay(500);
	softPwmWrite(17,60);
	delay(500);
	
	for(i=0;i<15;i++)
	{
		softPwmWrite(17,0);
		delay(250);
		softPwmWrite(17,60);
		delay(250);
		
	}
	softPwmWrite(17,0);
	softPwmWrite(22,0);
	softPwmWrite(24,0);
	ledTrigger=0;
	
}


void fadingColor( long double delayvariable)  // this function fades through the colors and accepts speed input
{
	int r = 50;
	int g = 0;
	int b = 0;
	int i =0;
	
	for (i=0;i<(delayvariable*100);i++)
	{
		while(soundFlag==1){
			delay(100);
			
		}
		if(MusicTrigger!=1 && ((ClassicalFlag==0) || (Top40Flag==0)))break;   //this is a fix that allows the user to break out of the fade function when another button is pressed.  
																		//It also allows the fade function to be used a second time in a while loop for example
		
		if( r == 50 && b == 0 && g < 50)
		{
			g = updateColor(g, 1);
			softPwmWrite(GREEN_PIN,g);
			
		}
		
		else if( g == 50 && b == 0 && r > 0)
		{
			r = updateColor(r, -1);
			softPwmWrite(RED_PIN,r);
		
		}
		
		else if( r == 0 && g == 50 && b < 50)
		{
			b = updateColor(b, 1);
			softPwmWrite(BLUE_PIN,b);
			
		}
		
		else if( r == 0 && b == 50 && g > 0)
		{
			g = updateColor(g, -1);
			softPwmWrite(GREEN_PIN,g);
		}
		
		else if( g == 0 && b == 50 && r < 50)
		{
			r = updateColor(r, 1);
			softPwmWrite(RED_PIN,r);
		}
		else if( r == 50 && g == 0 && b > 0)
		{
			b = updateColor(b,-1);
			softPwmWrite(BLUE_PIN,b);
		}
		delay(delayvariable);
	} //end for loop
	
	LEDoff();
	i=0;
	//ledTrigger=0;
	
}

int updateColor(int color, int step)  // this function is called in the fadingColor function to increment color by step thus increasing or decreasing brightness of the LED
{
	color += step;
	
	if (color > 50)  //makes sure that if color goes above 50, its still 50 cause thats the max duty cycle for the voltage were running the LEDs at (16V)
	{
		return 50;
	}
	if (color < 0)
	{
		return 0;
	}
	return (color);
}	

void randomLED(void)   //function that when called generates a random number that corresponds to an LED color
{
	unsigned long long randomLED;
	randomLED = rand() % 10000 ;
	
	
////////////////////////////////////////////////////////////	RED
			if(randomLED>0 && randomLED<=1000)
			{
				softPwmWrite(22,0);
				softPwmWrite(24,0);
				softPwmWrite(17,40);
			}
			
///////////////////////////////////////////////////////////		BLUE

			if(randomLED>1000 && randomLED<=2000)
			{
				softPwmWrite(17,0);
				softPwmWrite(24,0);
				softPwmWrite(22,40);
				
			}
			
////////////////////////////////////////////////////////////	GREEN		
			if(randomLED>2000 && randomLED<=3000)
			{
				softPwmWrite(17,0);
				softPwmWrite(22,0);
				softPwmWrite(24,40);
			}
			
////////////////////////////////////////////////////////////	PURPLE	
		
			if(randomLED>3000 && randomLED<=4000)
			{
				softPwmWrite(24,0);
				softPwmWrite(17,40);
				softPwmWrite(22,40);
			}
////////////////////////////////////////////////////////////  YELLOW				
		
			if(randomLED>4000 && randomLED<=5000)
			{
				softPwmWrite(22,0);
				softPwmWrite(17,40);
				softPwmWrite(24,10);
			}		
////////////////////////////////////////////////////////////	PINK			
		
			if(randomLED>5000 && randomLED<=6000)
			{
				softPwmWrite(24,0);
				softPwmWrite(17,40);
				softPwmWrite(22,10);
			}		
////////////////////////////////////////////////////////////	ICE BLUE			
		
			if(randomLED>6000 && randomLED<=7000)
			{
				softPwmWrite(17,0);
				softPwmWrite(22,40);
				softPwmWrite(24,40);
			}		
////////////////////////////////////////////////////////////	ORANGE			
		
			if(randomLED>7000 && randomLED<=8000)
			{
				softPwmWrite(22,0);
				softPwmWrite(17,40);
				softPwmWrite(24,2);
			}		
////////////////////////////////////////////////////////////	LIGHT GREEN		
		
			if(randomLED>8000 && randomLED<=9000)
			{
				softPwmWrite(17,30);
				softPwmWrite(22,2);
				softPwmWrite(24,40);
			}
////////////////////////////////////////////////////////////	SOFT PINK			
		
			if(randomLED>9000 && randomLED<=10000)
			{
				softPwmWrite(17,40);
				softPwmWrite(22,3);
				softPwmWrite(24,10);
			}			
////////////////////////////////////////////////////////////	


}
 
void LEDoff(void)
{
	softPwmWrite(17,0);
	softPwmWrite(22,0);
	softPwmWrite(24,0);
	
	
}
void mp3(char music)
{
//#define rock 1
//#define country 2
//#define classical 3
//#define top40 4



						  
	switch(music)
	{
		case rock:
		system("sudo mpg123 -Z ./Music/rock/*mp3 -&");
		mFlag=1;
		break;
		
		case country:
		system("sudo mpg123 -Z ./Music/country/*mp3 -&");
		mFlag=1;
		break;
		
		case classical:
		system("sudo mpg123 -Z ./Music/classical/*mp3 -&");
		mFlag=1;
		break;
		
		case top40:
		system("sudo mpg123 -Z ./Music/top40/*mp3 -&");
		mFlag=1;
		break;
	}
	
	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void *joystickThread(void *not_used) //gets inputs sets global values to be used by other threads.
{
		printf("Attempting a Pair request with PS3 Remote \n"); //wait for the ps3 remote to be paired
		while( access( fname, F_OK ) == -1 ) {
		// file does not exist so wait here until it does
		sleep(1);
		} 
		while(joytest==1){}		//wait for joystick usb check to finish																			//This section was adapted from different peices of code found on the web.
		sleep(3);				//wait 3 seconds for pairing of joystick

        int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0, x,i;
        char *button=NULL, name_of_joystick[80];
        struct js_event js;
		
		int axisTemp,axisTemp2;
			
			//while(joyCheck==0){  //keep looking for paired joystick
        if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
        {
                if( ( joy_fd = open( JOY_DEV2 , O_RDONLY)) == -1 ){
						printf( "Couldn't open joystick\n" );
						//return -1;
				}
        }
				//joyCheck=1;
			//}//end joyChecking
        ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
        ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
        ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

        axis = (int *) calloc( num_of_axis, sizeof( int ) );
        button = (char *) calloc( num_of_buttons, sizeof( char ) );

        printf("Joystick detected: %s\n\n"
                , name_of_joystick
                //, num_of_axis
                //, num_of_buttons
				);

        fcntl( joy_fd, F_SETFL, O_NONBLOCK );   /* use non-blocking mode */

        while( 1 )      /* infinite loop */
        {
				while(parChange==1){
					//wait here if parameter changing so we don't mess up the serial
				}
                        /* read the joystick state */
                read(joy_fd, &js, sizeof(struct js_event));
                
                        /* see what to do with the event */
                switch (js.type & ~JS_EVENT_INIT)
                {
				
                        case JS_EVENT_AXIS:										//if a axis changes
								axis[ js.number ] = js.value;

								//THROTTLE SET			
								if(axisTemp!=axis[1]&&axis[1]<=0){ 									//throttle if axis has changed and is negative
									 //throttle=(-axis[1]+32767)/65534; 				//set throttle scaled to 0-.5 for forward
									 throttle=((-axis[1])/512)+64;							//scaled for 0-128
									 bufOut[2]=throttle;
								}
								else{
										if(axisTemp!=axis[1]){
										//throttle=(axis[1]/65534)+.5;  //scaled throttle for .5-1 reverse
											throttle=(-axis[1]+32767)/512;  //scaled throttle for 128-255 reverse
											bufOut[2]=throttle;
										}
								}
								//STEERING SET
								if(axisTemp2!=axis[2]&&axis[2]<=0){ 									//right motor
									//steering=(-axis[1]+32767)/65534; 				//set steering scaled to 0-.5 for left
									steering=(axis[2]+32767)/512;						//scaled for 0-128
									bufOut[3]=steering;
								}
								else{
										if(axisTemp2!=axis[2]){
											//steering=(axis[2]/65534)+.5;  //scaled steering for .5-1 right
											steering=(axis[2]/512)+64;  //scaled throttle for 128-255 right
											bufOut[3]=steering;
										}
								}								

								axisTemp=axis[1];
								axisTemp2=axis[2];						
						//debug
						//printf("%d  %d  %d  %d\r",axis[1],axis[2],throttle,steering);				
								break;
                       
					   case JS_EVENT_BUTTON:									//if a button is pressed set sounds to play
                                button [ js.number ] = js.value;
								for(i=1; i<=16;i++){
									if(button[i]==1){
									buttonTrigger=i;
									}//end if
								}//end for
							   
							   
							   break;
									
                }//end switch
							
							
							//printf("%s\n",bufOut);
								//printf("Throttle %d  Steering %d  \n",throttle,steering);
               //         /* print the results */
               // printf( "X: %6d  Y: %6d  ", axis[0], axis[1] );
               // 
               // if( num_of_axis > 2 )
               //         printf("Z: %6d  ", axis[2] );
               //         
               //if( num_of_axis > 3 )
               //         printf("R: %6d  ", axis[3] );
               //         
               // for( x=0 ; x<num_of_buttons ; ++x )
               //         printf("B%d: %d  ", x, button[x] );
			   //
               // printf("  \r");
               

			   fflush(stdout);
        }//end while 1

        close( joy_fd );        /* too bad we never get here */
        return 0;
}
	

/////////////////////////Sound Thread//////////////////////////////////////////// 
 
void *ButtonCallThread(void *not_used) //gets inputs sets global values to be used by other threads.
{
	int mp3Flag=0;
	while(1){
	pthread_t tid5;
		
		switch (buttonTrigger) {			//use global to choose what button was pushed and play a sound with aplay
							
						case 1: //click left stick
						system("aplay -q ./sounds/MonsterGrowl.wav -&");	  //call aplay -q to prevent text printout
						LEDFlag=0x18;
						ledTrigger=1;
						buttonTrigger=0;
						  break;
						case 2:  //click right stick
							LEDFlag=0x13;
							ledTrigger=1;	
							buttonTrigger=0; 
						  break;
						 case 3: //start button
						   system("aplay -q ./sounds/descending_craft.wav");
						   system("sudo shutdown -h now"); //turn off pi
						   
							buttonTrigger=0; 
						  break;
						case 4: //dpad up  (country)
						 
						 if(mFlag==1)  //mFlag is mp3 flag so if mp3 is playing, mFlag=1 so that when button is pressed again, ,music stop playing
						  {			  
							  MusicFlagReset();
							  system("sudo pkill mpg123");
							  mFlag=0;
							  LEDoff();
							  MusicTrigger=0;						  
						  }
						  else
						  {
							  
							CountryFlag=1;
							mp3(country);
							MusicFlag=0x30;
							MusicTrigger=1;	
						  }
						  
							buttonTrigger=0; 
						
						  break;
						  
						case 5: //dpad right  (top40)
						    if(mFlag==1)
						  {	
						      MusicFlagReset();
							  system("sudo pkill mpg123");
							  mFlag=0;
							  LEDoff();
							  MusicTrigger=0;
						  }
						  else
						  {
							  
							  
							  Top40Flag=1;
							  mp3(top40);
							  MusicFlag=0x33;
							MusicTrigger=1;
						  }
						  
							buttonTrigger=0; 
						  break; 
						  
						case 6: //dpad down  (Classical)
						  
						 if(mFlag==1)
						  {	
							MusicFlagReset();
							  system("sudo pkill mpg123");
							  mFlag=0;
							  LEDoff();
							 MusicTrigger=0;
						  }
						  else
						  {
							 
							  ClassicalFlag=1;
							  mp3(classical);
							  MusicFlag=0x32;
							MusicTrigger=1;
							 
						  }
						  
						buttonTrigger=0; 
						  break;  
						  
						case 7: //dpad left (Rock)
						   
						  if(mFlag==1)
						  {	
							MusicFlagReset();
							  system("sudo pkill mpg123");
							  mFlag=0;
							  LEDoff();
							 MusicTrigger=0;
						  }
						  else
						  {
							   RockFlag=1;
							  mp3(rock);
							 MusicFlag=0x31;
							MusicTrigger=1;
						  }
						  
							buttonTrigger=0; 
						  break;
						  
						case 8: //L2 button
						system("aplay -q ./sounds/GunFire.wav -&");
						LEDFlag=0x17;
						ledTrigger=1;
						buttonTrigger=0;
						  						
						break;
						case 9:  //R2 Button
						 system("aplay -q ./sounds/AirHorn.wav -&");
						   LEDFlag=0x16;
						   ledTrigger=1;
							buttonTrigger=0; 
						 					
						break;
						case 10:  //L1 Button
						 system("aplay -q ./sounds/PoliceSiren.wav -&");	
						LEDFlag=0x15; //change to red blue flashing				
						ledTrigger=1;
						buttonTrigger=0; 
						break;
						case 11:  //R1 button
						   system("aplay -q ./sounds/PewPew.wav -&");
						   LEDFlag=0x14;
						   ledTrigger=1;
							buttonTrigger=0; 
						break;
						case 12:  // triangle button HAPPY
						
							LEDFlag=0x10;
							ledTrigger=1;
							buttonTrigger=0; 
						break;
						case 13:  // Circle button JOKE
						 	
							LEDFlag=0x04;
							ledTrigger=1;						
							buttonTrigger=0; 
						break;
						case 14:  // X button SAD
						 
						  LEDFlag=0x02;
						  ledTrigger=1;
						  buttonTrigger=0;
						break;
						case 15:   //square button RANDOM
						    
							LEDFlag=0x01;
							ledTrigger=1;
							buttonTrigger=0; 					 				
						break;
						case 16:
							system("aplay -q ./sounds/BombSiren.wav -&");	
							buttonTrigger=0; 					
						break;	
									
						default:
							//printf("poop %d \n",buttonTrigger);
							delay(100);
							
						break;
						   
					  }//end switch for keyget
		
	}//end while

}//end of sound thread


void *VideoThread(void *not_used) //gets inputs sets global values to be used by other threads.
{
switch (playVideo) {
	
	case 1:
		system("sudo omxplayer ./Videos/Happy/HuggingChad.mp4 -b --no-osd --amp 32600");
		break;
	case 2:
		printf("play video2");
		break;
	default:
		break;
}//end switcha



return(0);
}
void *LoopThread(void *not_used) //gets inputs sets global values to be used by other threads.
{
while(1){
		if(VloopFlag==1){
			system("sudo omxplayer ./Videos/BlankLoop.mp4 -b --no-osd --loop --vol 0 "); //start video loop if not running and flag set
			VloopFlag=0;
		}
		if(VloopFlag==2){
			system("sudo omxplayer /Videos/BlankLoop.mp4 -b --no-osd --loop "); //start video loop if not running and flag set
			
			VloopFlag=0;
		}
		if(VloopFlag==3){
			system("sudo omxplayer /Videos/BlankLoop.mp4 -b --no-osd --loop "); //start video loop if not running and flag set
			
			VloopFlag=0;
		}
		if(VloopFlag==4){
			system("sudo omxplayer /Videos/BlankLoop.mp4 -b --no-osd --loop "); //start video loop if not running and flag set
			
			VloopFlag=0;
		}
}//end while1
printf("LOST2!!");
return(0);
}





/*
 playVideo=1;
						  
						  ledTrigger=1;
						  pthread_create(&tid5,NULL,VideoThread,NULL);						  
						  usleep(VideoDelay); //delay for video to start
						  VloopFlag=0;
						  system("sudo pkill -f BlankLoop.mp4"); //kill loop mp4 file
						  usleep(VideoDelay);//however many seconds we want to wait for video to play idea is tunable
						  VloopFlag=1; //start loop again
						  usleep(VideoDelay);//delay for loop to start up again
						  system("sudo pkill -f HuggingChad.mp4"); //kill video mp4 (needs tidying up)
						 //LEDFlag=0x00;
*/

