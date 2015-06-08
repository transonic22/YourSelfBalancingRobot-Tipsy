
// *** June 8 2015 V2.8 A lot of stuff has changed Check out the RevisionHistory.txt for data. This will be close to the last version based on the arduino Leonardo. 
//		I have ran out of program space and will be switching over to using the mega2560 due to its low price and lots more memory to add features.
// program space
// *** April 25 V1.1 adding commands to be read from Serial1 bluetooth input and sent to pi our the Serial usb output for voice and emotion commands
// ***April 14 2015 adding remote programming of pid loop via serial input to be sent from pi or bluetooth module

// ***Chad Paget Modification March 2015
// ***Motor Control modified to work with different stepper controller. Chad Paget
// ***Removed Wireless AP and dependency on WITA board. Ported to be controlled by Serial data sent from raspberry pi
// ***Tuning paramaters pulled out to be controlled remotely
// ***Autotuning of PID meant to be implemented remotely
// ***Original Code Ported and Modified by Chad Paget based on:

// https://github.com/JJulio/b-robot
// B-ROBOT  SELF BALANCE ROBOT WITH STEPPER MOTORS
// WITA Board (Arduino Leonardo + Wifi module board)
// MPU6050 using DMP processor
// Author: Jose Julio
// Date: 04/10/2013
// Updated: 18/10/2013
// License: GPL v2

// Angle calculations and control part is running at 200Hz from DMP solution
// DMP is using the gyro_bias_no_motion correction method.
// The board needs at least 10-15 seconds to give good values...

 
 
  
  //Pot Pins

// STEPPER MOTOR PINS
// ENABLE PIN: D4
// STEP Motor1: D6 -> PORTD,7
// DIR  Motor1: D8 -> PORTB,4
// STEP Motor2: D12-> PORTD,6
// DIR  Motor2: D13-> PORTC,7
// 
// To control the stepper motors we use Timer1 interrupt running at 25Khz. We control the speed of the motors
// Note: We could not use WITA led because pin 13 is already used for stepper motor control

// Robot servo arm connected to D5
// Distance sensor (sonar) connected to A0 (analog 0)
// Distance sensor (IR) connected to A0 (analog 1)
// Distance sensor (IR) connected to A0 (analog 2)
// Battery monitor (voltage divider) connected to A5
//
// We use a standard PID control for robot stability
// We have a P control for speed control and a PD control for stability (robot angle)
// 		The output of the control (motor speed) is integrated so itÂ´s really an acceleration

// We control the robot from a WIFI module using OSC standard UDP messages (OSCmini library)
//    ThrottleFadder: Throttle 
//    SteeringFadder: Steering 
//    push1: Move arm (and robot raiseup)
//    push2: push up
//    AutonomousMode: autonomous mode
//	  **toggle2: sleep mode  //Chad
//    PAGE2: PID adjustements

// Robot autonomous mode
//   the robot start walking until reach an obstacle. When find an obstacle, start steering until it find a free way and continue walking


//#include <WITA.h> 						//was from Joses' code for his custom board. We are using a leonardo and HC-06 so we don't need this library
//#include <OSCmini.h> 						//We no longer use WiFi for communication instead we use Bluetooth. 
#include <Wire.h>							//
#include <I2Cdev.h>							//IMU uses I2C so we need this library
#include <JJ_MPU6050_DMP_6Axis.h> 			 // Modified version of the library to work with DMP (see comments inside)
//#include <servo.h>
//owen defines
#define DEBUG 0   //9=buffer output 8=steering and throttle stuff #2 estimated speed anglular speed 3=estimated speed and target angle
#define OBST_DIST_MIN 30  //AUTONOMOUS MODE --ODA
#define OBST_DIST_MED 50
#define OBST_DIST_MAX 70

#define IR_OBST_DIST_MIN 420
#define IR_OBST_DIST_MED 550
#define IR_OBST_DIST_MAX 570

#define WALK_DISTANCE_MIN 76
#define AUTONOMOUS_SPEED 50		// --ODA

#define USCEN 0
#define IRLEFT 1
#define IRRIGHT 2

#define RIGHT 1
#define LEFT -1

#define OFF LOW
#define ON HIGH
#define BAUD 115200 //serial baud rate for arduino communication, used for compatibility with your serial input source BT vs PI

#define SHUTDOWN_WHEN_BATTERY_OFF 1

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define ZERO_SPEED 65535  //65535
//#define MAX_ACCEL 7 //was 7, 5
#define TimerSpeed 80 		//2Mhz devided by this number will give you your timer speed for the motors default was 25Khz was 80
                            //more kd increases response time to error
// Default control terms   //low kp reduces how often updates happen to setpoint
#define KP .77 //.45// .5//.15 //.2 default robot jitters back and forth constantly// .45 proportional gain .1 was about 1 foot of swing and 10 oscillations 0.22 0.20   .22better .26 overdoes it    this value sets how far out the swing will be as it reduces .26 1" .4 let it stop current position
#define KD 23//35 //26  //35 //26 is default which caused it to jitter about an inch // Start 35 too far overshoot// derivitive gain 35 30 28 26 20      10 too slow 20is messed? 35 lots of jitter 25/26 oretty close small jitter 22is closer 19 too slow again  this sets how quick to attempt to return
#define KP_THROTTLE 0.065  //0.08  .065 .03
#define KI_THROTTLE 0.03  //.04 integral .05 .03
#define velocityScaler .9 //.9 default to adjust angler velocity to real units.
#define upright 52	//angle to start trying to balance
#define pickupTimeout 400 //timeout in ms

#define bytes 5				//how many bytes counted from 0 for the serial protocol we designed
#define max_throttle 600 //was 530 before remote worked
#define max_steering 300 //136
#define max_target_angle 12 





//#define I2C_SPEED 100000L
#define I2C_SPEED 400000L
//#define I2C_SPEED 800000L

#define ACCEL_SCALE_G 8192     //8192             // (2G range) G = 8192
#define ACCEL_WEIGHT 0.05 //.01
#define GYRO_BIAS_WEIGHT 0.005

// MPU6000 sensibility   (0.0609 => 1/16.4LSB/deg/s at 2000deg/s, 0.03048 1/32.8LSB/deg/s at 1000deg/s)
#define Gyro_Gain 0.03048
#define Gyro_Scaled(x) x*Gyro_Gain //Return the scaled gyro raw data in degrees per second

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489



// Control gains for raiseup
#define KP_RAISEUP 0.16
#define KD_RAISEUP 40
#define KP_THROTTLE_RAISEUP 0  // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define SERVO_AUX_NEUTRO 1470

#define ITERM_MAX_ERROR 80   // Iterm windup constants 40
#define ITERM_MAX 5000

#define OBSTACLE_DISTANCE_MIN 36
#define WALK_DISTANCE_MIN 76

#define BATTERY_WARNING 695  // (11 volts) aprox
#define BATTERY_SHUTDOWN 670// (10.4 volts)

uint8_t stable=0;
bool Robot_shutdown=false;   // Robot shutdown flag => Out of batteryVoltage
uint8_t mode=0;   // 0: MANUAL MODE   1: autonomous MODE
uint8_t autonomous_mode_status;   // 1: NORMAL STATUS=Walking  2: 
int16_t autonomous_mode_counter;  
int16_t autonomous_mode_distance;
int16_t pushUp_counter;  // for pushUp functionality (experimental)
float swing=0;
float swingOld=0;
float balanceAngle[2];

int16_t batteryVoltage=1000; //set to some value that wont trigger shutdown.
float KpAdjusted=3;
uint8_t balancedFlag=0;
uint8_t CountBalance=0;

// MPU control/status vars
uint8_t motorStateFlag=0;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (for us 18 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;
uint8_t MAX_ACCEL=7;
uint8_t edgeFlag=0;

int16_t estimated_speed;
uint8_t loop_counter;       // To generate a medium loop 40Hz 
uint8_t slow_loop_counter;  // slow loop 2Hz
uint8_t pickupFlag=0;
long motor1Timer;
long motor2Timer;
long pickupTimer;
long timer_old;
long timer_value;
int debug_counter;
float dt;
float balance_offset;
// class default I2C address is 0x68
MPU6050 mpu;

float angle_adjusted;
float angle_adjusted_Old;

float Kp=KP;
float Kd=KD;
float Kp_thr=KP_THROTTLE;
float Ki_thr=KI_THROTTLE;
float Kp_user=KP;
float Kd_user=KD;
float Kp_thr_user=KP_THROTTLE;
float Ki_thr_user=KI_THROTTLE;

float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float steering;
//float max_throttle = max_throttle;
//float max_steering = max_steering;
//float max_target_angle = max_target_angle;
float control_output;
int16_t motor1;
int16_t motor2;
char M0Flag=0;
char M1Flag=0;

float ThrottleFadder=.5;           // Throttle fadder value 
float SteeringFadder=.5;           // Steering fadder value
float fadder3=.5;           // KP Throttle user fadder value
float fadder4=.5;           // KI Throttle user fadder value
uint8_t AutonomousMode=0;             // Autonomous mode 1 for auto
uint8_t sleepMode=0;             // Sleep
uint8_t bufferA[6]={0,0,0,0,0,0};							//buffer for serial
unsigned char count=0,flag=0;


int16_t speed_m[2];           // Actual speed of motors
uint8_t dir_m[2];             // Actual direction of steppers motors
int16_t actual_robot_speed;          // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;          // overall robot speed (measured from steppers speed)
float estimated_speed_filtered;

uint16_t counter_m[2];        // counters for periods
uint16_t period_m[2][8];      // Eight subperiods 
uint8_t period_m_index[2];    // index for subperiods
//owen varialbes
//uint8_t count_dooku=1;
uint8_t count_dist=0; // important
uint16_t distance_sensor; // central ultra sonic sensor--ODA
uint16_t distance_sensor2; //left IR sensor
uint16_t distance_sensor3; //right IR sensor
uint16_t read_sensor[5]={0,0,0,0,0};  //important
uint16_t read_sensor2[5]={0,0,0,0,0}; //important
uint16_t read_sensor3[5]={0,0,0,0,0}; //important
uint8_t i=0; //important
uint8_t j=0; //important
uint16_t a=0; //important
//uint8_t n=0; 
uint8_t RF; //important
uint8_t CF; //important
uint8_t LF; //important
uint8_t DF; //important
uint8_t RandomRF=0; //important
uint8_t RandomLF=0; //important
//uint8_t STOP=0;
//uint8_t urgency_mult;
long old_time;
//long reverse_time;
uint8_t stillturning=0;
//int16_t autonomous_counter;  

// STEPPER MOTOR PINS
// ENABLE PIN: D4
// STEP Motor1: D6 -> PORTD,7
// DIR  Motor1: D8 -> PORTB,4
// STEP Motor2: D12-> PORTD,6
// DIR  Motor2: D13-> PORTC,7


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}

// Quick calculation to obtein Phi angle from quaternion solution
float dmpGetPhi() {
   mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
   mpu.dmpGetQuaternion(&q, fifoBuffer); 
   mpu.resetFIFO();  // We always reset FIFO
    
   //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
   return (atan2(2*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* RAD2GRAD);
}

// PD implementation. DT is in miliseconds

float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint-input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp*error + (Kd*(setPoint - setPointOld) - Kd*(input - PID_errorOld2))/DT;       // + error - PID_error_Old2
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return(output);
}

// PI implementation. DT is in miliseconds
//speedPIControl(dt,estimated_speed_filtered,throttle,Kp_thr,Ki_thr);
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint-input;
  PID_errorSum += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum,-ITERM_MAX,ITERM_MAX);

  output = Kp*error + Ki*PID_errorSum*DT*0.001;
  return(output);
}

// 200ns => 4 instructions at 16Mhz
void delay_200ns()  
{
  __asm__ __volatile__ (
		"nop" "\n\t"
                "nop" "\n\t"
                "nop" "\n\t"
		"nop"); 
}


ISR(TIMER1_COMPA_vect)
{
  counter_m[0]++;
  counter_m[1]++;
  if (counter_m[0] >= period_m[0][period_m_index[0]])
    {
    counter_m[0] = 0;
    if (period_m[0][0]==ZERO_SPEED)
      return;
    if (dir_m[0]){
      SET(PORTB,4);  // DIR Motor 1
		if(edgeFlag==0){
			edgeFlag=1;
			balanceAngle[0]=angle_adjusted;
		}//end edgeflag
	}//end direction
    else{
      CLR(PORTB,4);
	  			if(edgeFlag==1){
				edgeFlag=0;
				balanceAngle[1]=angle_adjusted;
		
				}//end if
	}//end else
    // We need to wait at lest 200ns to generate the Step pulse...
    period_m_index[0] = (period_m_index[0]+1)&0x07; // period_m_index from 0 to 7
    //delay_200ns();
    M0Flag=1;  
    }

  if (counter_m[1] >= period_m[1][period_m_index[1]])
    {
    counter_m[1] = 0;
    if (period_m[1][0]==ZERO_SPEED) return;
    if (dir_m[1]){
		SET(PORTC,7);   // DIR Motor 2
	
	}
    else {
		CLR(PORTC,7);

	}
    
    period_m_index[1] = (period_m_index[1]+1)&0x07;
    //delay_200ns();
    M1Flag=1;
    }
	
	
    if(M0Flag==1&&M1Flag==1){
    //CLR(PORTD,7); // STEP Motor 1
    //CLR(PORTD,6); // STEP Motor 1
    PORTD=PORTD&0b00111111;
	M0Flag=0;
    M1Flag=0;
    delayMicroseconds(3);
    //SET(PORTD,6);
    //SET(PORTD,7);
	PORTD=PORTD|0b11000000;
    
    }//end if both set
    else if(M0Flag==1){
      CLR(PORTD,7); // STEP Motor 1
      M0Flag=0;
      delayMicroseconds(3);
      SET(PORTD,7);
    }
    else if(M1Flag==1){
      CLR(PORTD,6); // STEP Motor 1
      M1Flag=0;
      delayMicroseconds(3);
      SET(PORTD,6);
    }
    
    
}


// Dividimos en 8 subperiodos para aumentar la resolucion a velocidades altas (periodos pequeÃ±os)
//***Divided into 8 sub-periods to increase the resolution at high speeds (short periods)
// subperiod = ((1000 % vel)*8)/vel;
// Examples 4 subperiods:
// 1000/260 = 3.84  subperiod = 3
// 1000/240 = 4.16  subperiod = 0
// 1000/220 = 4.54  subperiod = 2
// 1000/300 = 3.33  subperiod = 1 
void calculateSubperiods(uint8_t motor)
{
  int subperiod;
  int absSpeed;
  uint8_t j;
  
  if (speed_m[motor] == 0)
    {
    for (j=0;j<8;j++)
      period_m[motor][j] = ZERO_SPEED;
    return;
    }
  if (speed_m[motor] > 0 )   // Positive speed
    {
    dir_m[motor] = 1;
    absSpeed = speed_m[motor];
    }
  else                       // Negative speed
    {
    dir_m[motor] = 0;
    absSpeed = -speed_m[motor];
    }
    
  for (j=0;j<8;j++)
    period_m[motor][j] = 1000/absSpeed;
  // Calculate the subperiod. if module <0.25 => subperiod=0, if module < 0.5 => subperiod=1. if module < 0.75 subperiod=2 else subperiod=3
  subperiod = ((1000 % absSpeed)*8)/absSpeed;   // Optimized code to calculate subperiod (integer math)
  if (subperiod>0)
   period_m[motor][1]++;
  if (subperiod>1)
   period_m[motor][5]++;
  if (subperiod>2)
   period_m[motor][3]++;
  if (subperiod>3)
   period_m[motor][7]++;
  if (subperiod>4)
   period_m[motor][0]++;
  if (subperiod>5)
   period_m[motor][4]++;
  if (subperiod>6)
   period_m[motor][2]++;
  
  // DEBUG
  /*
  if ((motor==0)&&((debug_counter%10)==0)){
    Serial.print(1000.0/absSpeed);Serial.print("\t");Serial.print(absSpeed);Serial.print("\t");
    Serial.print(period_m[motor][0]);Serial.print("-");
    Serial.print(period_m[motor][1]);Serial.print("-");
    Serial.print(period_m[motor][2]);Serial.print("-");
    Serial.println(period_m[motor][3]);
    }
  */  
}

void setMotorSpeed(uint8_t motor, int16_t tspeed)
{
  // WE LIMIT MAX ACCELERATION
  if ((speed_m[motor] - tspeed)>MAX_ACCEL)
    speed_m[motor] -= MAX_ACCEL;
  else if ((speed_m[motor] - tspeed)<-MAX_ACCEL)
    speed_m[motor] += MAX_ACCEL;
  else
    speed_m[motor] = tspeed;
  
  calculateSubperiods(motor);  // We use four subperiods to increase resolution
  
  // To save energy when its not running...
   if ((speed_m[0]==0)&&(speed_m[1]==0)){
    digitalWrite(4,OFF);   // Disable motors
   }
    else {
    digitalWrite(4,ON);   // Enable motors
	}
}

void readBattery()
{
  batteryVoltage = analogRead(5);//(batteryVoltage*9 + (analogRead(5)/8))/10; // output : Battery voltage*10 (aprox) and filtered
}
//add owens distance sensor

void ObstacleUrgency()
{

	if (distance_sensor >= OBST_DIST_MAX)
	{
		CF=0; //no objects in sight
	}																									
	else if (distance_sensor < OBST_DIST_MIN)
	{
		CF=3;; //objects are close: evasive manoeuvres required
	}
	else if (distance_sensor < OBST_DIST_MED)
	{
		CF=2; //objects within medium safe distance: caution
	}
	else if (distance_sensor < OBST_DIST_MAX)
	{
		CF=1;	 //objects within maximum safe distance: slight caution	
	}
	
	if (distance_sensor2 >= IR_OBST_DIST_MAX)
	{
		LF=0; //no objects in sight
	}																									
	else if (distance_sensor2 < IR_OBST_DIST_MIN)
	{
		LF=3;; //objects are close: evasive manoeuvres required
	}
	else if (distance_sensor2 < IR_OBST_DIST_MED)
	{
		LF=2; //objects within medium safe distance: caution
	}
	else if (distance_sensor2 < IR_OBST_DIST_MAX)
	{
		LF=1;	 //objects within maximum safe distance: slight caution	
	}	
	if (distance_sensor3 >= IR_OBST_DIST_MAX)
	{
		RF=0; //no objects in sight
	}																									
	else if (distance_sensor3 < IR_OBST_DIST_MIN)
	{
		RF=3;; //objects are close: evasive manoeuvres required
	}
	else if (distance_sensor3 < IR_OBST_DIST_MED)
	{
		RF=2; //objects within medium safe distance: caution
	}
	else if (distance_sensor3 < IR_OBST_DIST_MAX)
	{
		RF=1;	 //objects within maximum safe distance: slight caution			
	}
	/*Serial.print(LF);
	Serial.print("  ,  ");
	Serial.print(CF);
	Serial.print("  ,  ");
	Serial.println(RF);	*/
	/*if ((throttle > 20)&&(estimated_speed_filtered < 10)) //this does not work when the robot is stopped 
	{
		STOP=1;
	}
	else 
	{
		STOP=0;
	}*/
	
	
}
void RobotDirection()
{
	/*if (STOP == 1) //this function does not work
	{
		DF=4;
	}*/
	if (RF==3 || CF==3 || LF==3) //objects are close: evasive manoeuvres required
	{
		DF=3;  
		//urgency_mult=30; //used as a multiple of steering for simplicity, but not important
	}																									
	else if (RF==2 || CF==2 || LF==2) //objects within medium safe distance: caution 
	{
		if (stillturning == 1)DF=3; //this check is so it does not jump out of random direction
		else
		{
			DF=2; 
		}
		//urgency_mult=25; //used as a multiple of steering for simplicity, but not important
	}
	else if (RF==1 || CF==1 || LF==1) //objects within maximum safe distance: slight caution
	{
		if (stillturning ==1)DF=3; 
		else
		{
			DF=1; 
		}
		//urgency_mult=15; //used as a multiple of steering for simplicity, but not important		
	}
	else if (RF==0 || CF==0 || LF==0)
	{
		DF=0;	 //no objects in sight
	}		
}
void RandomDirection()
{
	if (RandomLF==0 && RandomRF==0)
	{
		if ((millis() % 2)==0)RandomLF=1;
		else
		{
			RandomRF=1;
		}	
		old_time=millis();
	}
	else
	{
		throttle = throttle -5; 
		if (throttle < -30)throttle = -30;
		if(RandomLF==1)
		{
			steering = 30*LEFT;
			//if (steering < (urgency_mult*LEFT))steering = urgency_mult*LEFT;			
		}
		if(RandomRF==1)
		{
			steering = 30*RIGHT;
			//if (steering > (urgency_mult*RIGHT))steering = urgency_mult*RIGHT;			
		}
		if ((DF < 2) || (millis() - old_time > 2000))
		{
			steering=0;
			RandomLF=0;
			RandomRF=0;
			stillturning=0;
		}
	}
}
void RobotSteering()
{

	switch (DF)
	{
		case 0: //no objects in sight
		{
			steering = 0;
			if (throttle < AUTONOMOUS_SPEED)
				throttle = throttle + 5;  
			if (throttle > AUTONOMOUS_SPEED) 
				throttle = AUTONOMOUS_SPEED;
			break;
		}
		case 1: //objects within maximum safe distance: slight caution
		{
			throttle = throttle - 2;
			if (throttle < 40)throttle = 40;
			
			if (distance_sensor2 > distance_sensor3)
			{
				steering = 15*LEFT;
				//if (steering > (urgency_mult*LEFT))steering = urgency_mult*LEFT;
			}																									
			else if (distance_sensor2 <= distance_sensor3)
			{
				steering = 15*RIGHT;
				//if (steering > (urgency_mult*RIGHT))steering = urgency_mult*RIGHT;	
			}
			else
			{
				steering = 0;
				throttle = throttle - 2;
				if (throttle < 50)throttle = 50;
			}
			break;
		}
		case 2: //objects within medium safe distance: caution 
		{
			throttle = throttle - 4;
			if (throttle < 25)throttle = 25;
			if (distance_sensor2 > distance_sensor3)
			{
				steering = 25*LEFT;
				//if (steering > (urgency_mult*LEFT))steering = urgency_mult*LEFT;
			}																									
			else if (distance_sensor2 <= distance_sensor3)
			{
				steering = 25*RIGHT;
				//if (steering > (urgency_mult*RIGHT))steering = urgency_mult*RIGHT;
			}
			else
			{
				steering = 0;
				throttle = throttle - 4;
				if (throttle < 25)throttle = 25;
			}	
			break;
		}
		case 3: //objects are close: evasive manoeuvres required
		{
			throttle = throttle - 6;
			if (throttle < 0)throttle = 0;
			if ((distance_sensor2 > (distance_sensor3 + 35)) && stillturning==0) // changed
			{

				steering = 30*LEFT;
			}																									
			else if (((distance_sensor2 + 35) < distance_sensor3) && stillturning==0) // changed
			{
				steering = 30*RIGHT;
			}
			else// if (distance_sensor3 == distance_sensor2)
			{
				stillturning=1;
				RandomDirection();
			}				
			break;
		}	
		/*case 4: for if the robot is stopped
		{
			throttle = 0;
			steering = 0;
			break;
		}*/
		default:
		{
			throttle = 0;
			steering = 0;
			break;
		}
	}
	
}

void readDistanceSensor()
{
  //#define USCEN 0;
  //#define IRLEFT 1;
  //#define IRRIGHT 2;

  double aux; 
  double aux2;
  double aux3;


  aux=2.54*analogRead(USCEN)/2; //Gives the ultra sonic sensor a distance value in centimetres 
  aux2=analogRead(IRLEFT);  //IR left sensor up to 650 bits-ish at 5 centimetres
  aux3=analogRead(IRRIGHT); //IR right sensor up to 650 bits-ish at 5 centimetres
  aux2=625-aux2; //gives a scale of 5 to 635. 5 being the closest and 635 being the farthest away from the sensor
  aux3=625-aux3; //gives a scale of 5 to 625. 5 being the closest and 625 being the farthest away from the sensor
  if (aux2 <= 0 )aux2=0;
  if (aux3 <= 0 )aux3=0;  
  if (count_dist < 5)
  {
 	read_sensor[count_dist]=aux;
	read_sensor2[count_dist]=aux2;
    read_sensor3[count_dist]=aux3;
	count_dist++;
  }
  


    if (count_dist==5)
	{

		count_dist=0;
		for (i = 0; i < 5; i++)
		{
			for (j = (i + 1); j <= 5; j++)
			{
				if (read_sensor[i] > read_sensor[j])
				{
					a =  read_sensor[i];
					read_sensor[i] = read_sensor[j];
					read_sensor[j] = a;
				}
			}
		}
		for (i = 0; i < 5; i++)
		{
			for (j = (i + 1); j <= 5; j++)
			{
				if (read_sensor2[i] > read_sensor2[j])
				{
					a =  read_sensor2[i];
					read_sensor2[i] = read_sensor2[j];
					read_sensor2[j] = a;
				}
			}
		}
		for (i = 0; i < 5; i++)
		{
			for (j = (i + 1); j <= 5; j++)
			{
				if (read_sensor3[i] > read_sensor3[j])
				{
					a =  read_sensor3[i];
					read_sensor3[i] = read_sensor3[j];
					read_sensor3[j] = a;
				}
			}
		}
		
	}


	if (count_dist==0)
	{
		/*distance_sensor = (aux*9 + aux)/10;   // heavy filtering gives the ultra sonic sensor a distance value in centimetres 
		distance_sensor2 = (aux2*9 + aux2)/10;   // heavy filtering IR left sensor up to 650 bits-ish at 5 centimetres
		distance_sensor3 = (aux3*9 + aux3)/10;   // heavy filtering IR right sensor up to 650 bits-ish at 5 centimetres*/
		distance_sensor = (distance_sensor*9 + read_sensor[2])/10;   // heavy filtering gives the ultra sonic sensor a distance value in centimetres 
		distance_sensor2 = (distance_sensor2*9 + read_sensor2[2])/10;   // heavy filtering IR left sensor up to 650 bits-ish at 5 centimetres
		distance_sensor3 = (distance_sensor3*9 + read_sensor3[2])/10;   // heavy filtering IR right sensor up to 650 bits-ish at 5 centimetres&
		/*Serial.print("pre filter  ");		
		Serial.print(read_sensor[2]);
		Serial.print("  ,post filter  ");
		Serial.print(distance_sensor);
		Serial.print("  |  ");		
		Serial.print("pre filter  ");		
		Serial.print(read_sensor2[2]);
		Serial.print("  ,post filter  ");
		Serial.print(distance_sensor2);
		Serial.print("  |  ");
		Serial.print("pre filter  ");		
		Serial.print(read_sensor3[2]);
		Serial.print("  ,post filter  ");
		Serial.println(distance_sensor3);*/

	}
	
	/*else if ((( distance_sensor2 > IR_OBST_DIST_MAX ) | (distance_sensor3 > IR_OBST_DIST_MAX)) && ((distance_sensor2 < IR_OBST_DIST_MED)|( distance_sensor3 < IR_OBST_DIST_MED)))
	{
		autonomous_mode_status=1;	 //objects within maximum safe distance: slight caution	
		//Serial.println("Within Max");		
	}*/

  /*
  Serial.print(distance_sensor2);
  Serial.print("  left,  ");
  Serial.print(distance_sensor);
  Serial.print("  center,  ");
  Serial.print(distance_sensor3);
  Serial.println("  right");
  //delay(100);
  */
}
void autonomousMode()
{
	readDistanceSensor();
	ObstacleUrgency();
	RobotDirection();
	RobotSteering();

}

void getInput(){ //this is where we get serial data from the PI
	if(Serial1.available()>6)Serial1.flush();
	if(Serial.available()>6)Serial.flush();
	while (Serial.available()&&count!=99) {  //if there is serial data in the buffer start filling the buffer array until there is nothing in buffer. must be careful to not send too much info without a break from the pi or this may not be able to empty. which would cause lost data but not a crash. --ODA PS3 Remote input will be skipped if phone used.
   
				bufferA[count] = (char)Serial.read();  	//get the next byte and store in the buffer
				//if(count!=0&&bufferA[count]=='#'){		//if we see a # in the string we assume we are lost and ditch the string and wait for next # to read in again
				//	Serial.flush();
				//	count=0;
				//}
				if(count==0&&bufferA[0]!='#'){ 					//check if the buffer started in right spot. This may wander due to motor interrupt
						Serial.flush();											//if faulty data flush the serial buffer
						count=98;														//flag to exit loop 
					}//end if
				count++;

	}//end while serial available	
	
	while (Serial1.available()&&count!=99&&count!=6) {  //if there is serial data in the buffer start filling the buffer array until there is nothing in buffer. must be careful to not send too much info without a break from the pi or this may not be able to empty. which would cause lost data but not a crash.
   
				bufferA[count] = (char)Serial1.read();  	//get the next byte and store in the buffer
				//if(count!=0&&bufferA[count]=='#'){		//if we see a # in the string we assume we are lost and ditch the string and wait for next # to read in again
				//	Serial1.flush();
				//	count=0;
				//}
				//
				if(count==0&&bufferA[0]!='#'){ 					//check if the buffer started in right spot. This may wander due to motor interrupt
						Serial1.flush();											//if faulty data flush the serial buffer
						count=98;														//flag to exit loop 
					}//end if
				count++;
				
	}//end while serial available
	


if (count>=bytes && count!=99 && bufferA[0]=='#'){ //if buffer filled		
			count=0; //reset count			
			
					//fill in fadder values
				
				if(bufferA[1]=='S'){
					bufferA[1]='@';
				switch(bufferA[5]){ //[5] is tuning mode [4] is data
					case 'A':
						Kp_user=(float)bufferA[4]/100; //update gains for PID functions
						break;
					case 'B':	
						Kd_user=(float)bufferA[4]; //update gains for PID functions		
						break;
					case 'C':	
						Kp_thr=(float)bufferA[4]/1000; //update gains for PID functions
						break;
					case 'D':	
						Ki_thr=(float)bufferA[4]/1000; //update gains for PID functions
						break;
					case 'E':						// Mode Change?   0 : MANUAL  1: AUTONOMOUS (based on AutonomousMode switch)
						AutonomousMode = bufferA[4]; //change mode
						bufferA[5]='@';
					  break;
					case 'F':			//call command from pi based on single number being passed
					{
						if(bufferA[4]==96){ //autonomous mode
							AutonomousMode^=1;
						}
						bufferA[1]='P';//set buffer to send to Pi
						//bufferA[4]+=30;
						int i;
						for (i = 0; i <= 5; i = i + 1) {
							Serial.print((char)bufferA[i]);
						}	
						
						
						break;
					}
					case 'G': //Pro Mode not sure if we will show this off or not yet
					{
						
					}
								
						break;
					default:							//default steering code  
						ThrottleFadder=(float)bufferA[2]/128;	//throttle 0-128
						SteeringFadder=(float)bufferA[3]/128;	//steering 0-128
						
				}//end switch
			 
					}//end if setting mode
					else{//not setting mode take regular throttle input
						ThrottleFadder=(float)bufferA[2]/128;	//throttle 0-128
						SteeringFadder=(float)bufferA[3]/128;	//steering 0-128
					}
						#if (DEBUG==9)  //if debug set to 9 print out buffer
						//Serial.println("flush");
						Serial.print("buffer 0: ");
						Serial.print(bufferA[0]);
						Serial.println();
						Serial.print("buffer 1: ");
						Serial.print(bufferA[1]);
						Serial.println();
						Serial.print("buffer 2: ");
						Serial.print(bufferA[2]);
						Serial.println();
						Serial.print("buffer 3: ");
						Serial.print(bufferA[3]);
						Serial.println();
						Serial.print("buffer 4: ");
						Serial.print(bufferA[4]);
						Serial.println();
						Serial.print("buffer 5: ");
						Serial.print(bufferA[5]);
						Serial.println();
						#endif //end debug if
	}//end filled buffer if
	if(count>=10)count=0;
}//end getInput

void setup() 
{ 
  // STEPPER PINS 
  pinMode(4,OUTPUT);  // ENABLE MOTORS
  pinMode(6,OUTPUT);  // STEP MOTOR 1 PORTD,7
  pinMode(8,OUTPUT);  // DIR MOTOR 1
  pinMode(12,OUTPUT); // STEP MOTOR 2 PORTD,6
  pinMode(13,OUTPUT); // DIR MOTOR 2
  //pinMode(A0,INPUT);	//Distance Sensor doesn't need called as it is default for Arduino Leonardo
  digitalWrite(4,OFF);   // Disbale motors
  
  //Pot Pins
  
 
  
  //pinMode(5,OUTPUT);  // Auxiliar servo 
  
  Serial.begin(BAUD);//to match default HC-06 module 
 //delay(5000);
 //Serial.println("test");  // CONFIGURATION ONLY NEEDED FIRST TIME
 Serial1.begin(BAUD);//serialpins for BlueTooth
  //servoAux.attach();
  
  //WITA.WifiInit();  //no loger needed in our code
  //
  //WITA.WifiFactoryReset();
  //WITA.WifiChangeBaudRateFast();
  //WITA.WifiEnableUDP("2222","2223","192.168.1.11");
  //WITA.WifiAP();       // Soft AP mode SSID:"WITA_AP"
  //WITA.LedBlink(3,true);
  
  // Join I2C bus
  Wire.begin();
  // 4000Khz fast mode
  TWSR = 0;
  TWBR = ((16000000L/I2C_SPEED)-16)/2;
  TWCR = 1<<TWEN;
  
  Serial.println("Initializing I2C devices...");
  //mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);
  
  //delay(2000);
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        delay(15000);   // Time to settle things... the bias_from_no_motion algorithm needs some time to take effect and reset gyro bias.
    
		} else { // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
  // Gyro calibration
  // The robot must be steady during initialization


  Serial.print("Free RAM: ");
  Serial.println(freeRam());
  Serial.print("max_throttle: ");
  Serial.println(max_throttle);
  Serial.print("max_steering: ");
  Serial.println(max_steering);
  Serial.print("max_target_angle: ");
  Serial.println(max_target_angle);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  timer_old = millis();
  
  // Init servos
  //Serial.println("Servo initialization...");
  //WITA.InitServos();   // Servos initialization
  //WITA.Servo(3,SERVO_AUX_NEUTRO);
  
  //We are going to overwrite the Timer1 to use the stepper motors
  
  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 
  
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  //OCR1A = 125;  // 16Khz
  //OCR1A = 100;  // 20Khz
  //OCR1A = 80;   // 25Khz
  OCR1A = TimerSpeed;    // 50Khz
  TCNT1 = 0;

  //delay(2000);
  
  //Adjust sensor fusion gain
  Serial.println("Adjusting DMP sensor fusion gain...");
  dmpSetSensorFusionAccelGain(0x20);
  
  Serial.println("Initializing Stepper motors...");
  //delay(1000);
  TIMSK1 |= (1<<OCIE1A);  // Enable Timer1 interrupt
  digitalWrite(4,ON);    // Enable stepper drivers
  
  // Little motor vibration to indicate that robot is ready
  for (uint8_t k=0;k<3;k++)
	{
	setMotorSpeed(0,3);
	setMotorSpeed(1,-3);
	delay(150);
	setMotorSpeed(0,-3);
	setMotorSpeed(1,3);
	delay(150);
	}

  distance_sensor = 150;
  batteryVoltage = 1000; 
  
  // OSC initialization
 ThrottleFadder = 0.5;
 SteeringFadder = 0.5;
 
  mpu.resetFIFO();
  timer_old = millis();
  Robot_shutdown = false;
}


// Main loop
void loop() 
{ 

  //Chad
  // If we have no batteryVoltage, we do nothing...
  #ifdef SHUTDOWN_WHEN_BATTERY_OFF
    if (Robot_shutdown == true){
		digitalWrite(4,OFF);   // Disable motors
            Serial.println("shutdown");
            Serial.println(batteryVoltage);
      return;
    }
  #endif
  
  debug_counter++;
  //OSC.MsgRead();  // Read UDP OSC messages
  //getPiInput(); //get serial input from PI
  
	getInput();
	//Serial.println(bufferA);
		if ((mode == 0)&&(AutonomousMode==1)) 
			 {
			  mode = 1;
			  distance_sensor = 150;
			  autonomous_mode_status = 0;
			  autonomous_mode_counter = 0;
			  Serial.println("AUTO");
		    }//end if
			if ((mode==1)&&(AutonomousMode==0))
			 {
				  mode = 0;
				  distance_sensor = 150;
				  Serial.println("MANUAL");
			}//end if
	
        if (mode == 0)
          {
          if ((ThrottleFadder > 0.45)&&(ThrottleFadder<0.55))   // Small deadband on throttle
            throttle = 0;
          else
	    throttle = (ThrottleFadder-0.5)*max_throttle;
		// We add some exponential on steering to smooth the center band
		if ((SteeringFadder > 0.47)&&(SteeringFadder<0.53))   // Small deadband on throttle
            steering = 0;
          else{
			steering = SteeringFadder-0.5;
			  if (steering>0)
				steering = (steering*steering+0.5*steering)*max_steering;
			  else
				steering = (-steering*steering+0.5*steering)*max_steering;
			  }
			//modifing_control_parameters=false;
		  }
	
 
 //////////////////////////////////////////
 timer_value = millis();   
  // New DMP Orientation solution?
  fifoCount = mpu.getFIFOCount();
  if (fifoCount>=18) //WORK CODE FOR BALANCING
      {
			  if (fifoCount>18)  // If we have more than one packet we take the easy path: discard the buffer 
				{
				Serial.println("FIFO RESET!!");
				mpu.resetFIFO();
				return;
				}
			loop_counter++;
			slow_loop_counter++;
		   
			dt = (timer_value-timer_old);
			timer_old = timer_value;
			
			
			angle_adjusted_Old = angle_adjusted;
			angle_adjusted = dmpGetPhi();
			
			#if DEBUG==8
			  Serial.print(ThrottleFadder);
			  Serial.print(" ");
				Serial.print(SteeringFadder);
			  Serial.print(" ");
				Serial.print(throttle);
			  Serial.print(" ");
			  Serial.print(steering);
			  Serial.print(" ");
			  Serial.println(mode);
			#endif

			//angle_adjusted_radians = angle_adjusted*GRAD2RAD;
			#if DEBUG==1
			  Serial.println(angle_adjusted);
			#endif
			//Serial.print("\t");
			mpu.resetFIFO();  // We always reset FIFO

			if (mode==1)  autonomousMode();

			// We calculate the estimated robot speed
			// Speed = angular_velocity_of_stepper_motors - angular_velocity_of_robot(angle measured by IMU)
			actual_robot_speed_Old = actual_robot_speed;
			
			actual_robot_speed = (speed_m[1] - speed_m[0])/2;  // Positive: forward

			int16_t angular_velocity = (angle_adjusted-angle_adjusted_Old)*velocityScaler;     // 90 is an empirical extracted factor to adjust for real units  //adjustment
			
			estimated_speed = actual_robot_speed_Old - angular_velocity;     // We use robot_speed(t-1) or (t-2) to compensate the delay
			estimated_speed_filtered = estimated_speed_filtered*0.99 + (float)estimated_speed*0.01;  //.95,.05  trying removing the filter
			#if DEBUG==2
			  Serial.print("\n est speed filtered");
			  Serial.println(estimated_speed_filtered);
			  Serial.print("\n angular_velocity");
			  Serial.println(angular_velocity);
			  
			#endif
			
			target_angle = speedPIControl(dt,estimated_speed_filtered,throttle,Kp_thr,Ki_thr)+balance_offset;
		    target_angle = constrain(target_angle,-max_target_angle,max_target_angle);   // limited output
			
			#if DEBUG==3
			  Serial.print(" ");Serial.print(estimated_speed_filtered);
			  Serial.print(" ");Serial.println(target_angle);
			#endif
			
			// New user control parameters ?
			//readControlParameters();
			
			if (pushUp_counter>0)  // pushUp mode?
			  target_angle = 10;
			  
			// We integrate the output (acceleration)
			control_output += (stabilityPDControl(dt,angle_adjusted,target_angle,Kp,Kd));//*(((float)KpAdjusted/100)+1);;
			
			control_output = constrain(control_output,-500,500);   // Limit max output from control   don't think this is needed
	
			// The steering part of the control is injected directly on the output
			//add deadband

			motor1 = (control_output + steering);
			motor2 = (-control_output + steering);  // Motor 2 is inverted
			
			
			// Limit max speed
			motor1 = constrain(motor1,-500,500);   
			motor2 = constrain(motor2,-500,500);
			
			#if DEBUG==10
				Serial.print(motor1);
				Serial.print(" | ");
				Serial.println(target_angle);
			#endif
			#if DEBUG==11
				Serial.print(Kd);
				Serial.print(" | ");
				Serial.println(Kp);
			#endif
				
			
			// Is robot ready (upright?)
			if ((angle_adjusted<upright)&&(angle_adjusted>-upright)) //if the robot is at an acceptable angle to control
			  {
					
					
					  if ((angle_adjusted<25)&&(angle_adjusted>-25))//adjust here for compensation of overshoot on standing up was 40 //set flag then stand up and cancle flag when past the 5 degreeish
						{
									
									
									
									if (((motor1==500 && motor2==-500) || (motor1==-500 && motor2==500) || (pickupFlag==1))&& (balancedFlag==1||(balancedFlag==0 || balancedFlag==5))){
										if(pickupFlag==0){
											pickupTimer=millis();
											pickupFlag=1;
										}								
										if(pickupFlag==1 && millis()-pickupTimer>=pickupTimeout){ //if the robot has been at max speed for 250ms turn off the motors
	
											MAX_ACCEL=25;
											setMotorSpeed(0,0);
											setMotorSpeed(1,0);
											MAX_ACCEL=7;
											balancedFlag=2; //motors off
											Serial.print("   pickedup  ");
											//digitalWrite(4,OFF); //turn off motors disabled to reduce transients in stepper motor
										}
									 
									}//end if max speed picked up function
									
									if(throttle==0&&steering==0&&(estimated_speed_filtered<50||estimated_speed_filtered>-50)){ //able to start balance mode
										swing=balanceAngle[0]-balanceAngle[1];	//get swing
										if(swing<1)swing*=-1; 					//Convert to Absolute Value
									}//end if

						}  //end if 25 degrees   

				// NORMAL MODE if not in picked up mode
				//Serial.println(target_angle);
				//debug
				if(control_output<1.75&&control_output>-1.75&&balancedFlag==2)control_output=0; //dont allow pid to compensate unless falling 
				
				
				if(steering!=0||throttle!=0){											//regular driving mode if any input from driver
					balancedFlag=3;
				}
				
				else if (((angle_adjusted>25)||(angle_adjusted<-25))&&pickupFlag==0){ 	//standing up mode
					balancedFlag=4;
				}
				
				else if(pickupFlag==1){ //got picked up
					balancedFlag=2;
				}
				else if(control_output!=0&&balancedFlag==2)balancedFlag=5;
					
				
				
				
				switch(balancedFlag){
					case 0:{ //stationary but working toward fuzzy balance
							
							Kp = Kp_user;
							Kd = Kd_user;  			//default/user control gains
							Kp_thr = Kp_thr_user;
							Ki_thr = Ki_thr_user;
							if(swing<10 && pickupFlag==0 && swing!=0 && CountBalance>=10)	{
								balancedFlag=1;
								CountBalance=0;
							}
							else CountBalance++;	
							
							break;	
							}
					
					case 1:{ //trying to fuzzy balance

							if(swing!=swingOld ){ 					//swing changed

								if(swing>swingOld||swing>1){	//getting worse or larger then 
									KpAdjusted+=.25;
									if(KpAdjusted>4.5){
										CountBalance++;
										KpAdjusted=4.5;
										
									}

								}
								if(swing<swingOld&&motor1<5&&motor1>-5){			//getting better by more then number
									KpAdjusted-=.2;
								}
								if(KpAdjusted<.45)KpAdjusted=.45;
							}//end if swing!=swing old
							
							swingOld=swing; //update old swing
							
							if(swing<.6){
								balancedFlag=2;
								control_output=0;
							}
							if(CountBalance>=20){
										Kd--;
										CountBalance=0;
										if(Kd<10)Kd=10;
										
							}

							else {
								Kd = KD;
							}
							if (motor1>200||motor1<-200)balancedFlag=0;
							
							Kp=KpAdjusted;
							
					
							break;
							}
							
							
					
					case 2:{ //motors off balanced
							CountBalance=0;
							Kd=0; //don't effect the pid loop
							Kp=1;
							KpAdjusted=3.45;
							motor1=0;
							motor2=0;
							
							
							
							break;
							}
					case 3:{    //driving
							Kd=35;
							Kp=.45;
							Kp_thr = Kp_thr_user;
							Ki_thr = Ki_thr_user;
							if(throttle==0&&steering==0){
								balancedFlag=5;
								stable=0;
							}
							
							break;	
					}
					case 4:{  //getting up from fallen
							Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
							Kd = KD_RAISEUP;
							Kp_thr = KP_THROTTLE_RAISEUP;
							Ki_thr = KI_THROTTLE_RAISEUP;
							balance_offset=0;
							if((angle_adjusted<35)&&(angle_adjusted>-35)){
								balancedFlag=0;
								//swing=0;
							}
							
							break;	
					}	
					case 5:{ //recover from stand still
						
							Kp = Kp_user;
							Kd = Kd_user;  			//default/user control gains
							Kp_thr = Kp_thr_user;
							Ki_thr = Ki_thr_user;
							if((swingOld!=swing))stable++;
							swingOld=swing;
							if(stable>=2){
								balancedFlag=1;
								stable=0;
								KpAdjusted=3;
							}
							
							break;	
					}
					
					default:{
						//balancedFlag=0;
						break;
					}
					
				}//end switch
				setMotorSpeed(0,motor1);
				setMotorSpeed(1,motor2);

			  }//end if within 52 degrees
			
			
			////////////////////////////////////////////////Robot Fallen///////////////////////////////////////////////////////
			
			else   // Robot not ready, angle > 52Âº which was the angle set to upright when this comment written
			  {
				  balancedFlag=4;
				  swing=50;
				  pickupFlag=0;  //reset picked up flag
				  setMotorSpeed(0,0);
				  setMotorSpeed(1,0);
				  PID_errorSum = 0;  // Reset PID I term
				  
			}//end robot fallen
			#if DEBUG==13	
						
						//Serial.print("estimated_speed ");
						//Serial.print(estimated_speed_filtered);
												
						//Serial.print("   Kd ");
						//Serial.print(Kd);
						
						//Serial.print("   control ");
						//Serial.print(control_output);
						Serial.print("   swing ");
						Serial.print(swing);
						
						//Serial.print("   ThrottleFadder ");
						//Serial.print(ThrottleFadder);
						
						//Serial.print("   kd ");
						//Serial.print(Kd);
						
						Serial.print("   balancedFlag ");
						Serial.println(balancedFlag);
						
						//Serial.print("   Kp ");
						//Serial.println(Kp);
						
						//Serial.print("   pickedup ");
						//Serial.println(pickupFlag);
			
					#endif
    } // New IMU data
	
  // Medium loop 40Hz
  if (loop_counter >= 5) 
    {
			loop_counter = 0;
			
			// if we are in autonomous mode we read the distance sensor
			if (mode==1)
				{
				readDistanceSensor();
				#if DEBUG == 5
				  Serial.print(distance_sensor);
				  Serial.print(" A");
				  Serial.println(autonomous_mode_status);
				#endif
				}
				if(throttle==0 && steering==0 && (estimated_speed > 3 || estimated_speed < -3) && balancedFlag!=4){ //if no throttle and still walking offset the balancing angle.
				if(estimated_speed<0){
					balance_offset+=.1;	
				} 
				else {balance_offset-=.1;				
				}
			}
    } // Medium loop
      
  if (slow_loop_counter>=199)  // two hertz is still excsesive may we will drop this later to every two or more seconds 
    {
			slow_loop_counter = 0;
			//getGains(); //read the pots
			// Read batteryVoltage status
			readBattery();
			
			//sendSerial(); //for autonomous mode

			//Serial.print("hi");
			
			#if DEBUG==6
			  Serial.print("B");
			  Serial.println(batteryVoltage);
			  Serial.print(" ");
			#endif
			#if DEBUG==7
			  Serial.print(distance_sensor);
			  Serial.print(" A");
			  Serial.println(autonomous_mode_status);
			#endif
			if (batteryVoltage < BATTERY_SHUTDOWN)
			  {
			  // Robot shutdown !!!
			  #ifdef SHUTDOWN_WHEN_BATTERY_OFF
				bufferA[1]='P';//set buffer to send to Pi
				bufferA[4]=3;
				//bufferA[4]+=30;
				int i;
				for (i = 0; i <= 5; i = i + 1) {
				Serial.print((char)bufferA[i]);
				}
				Robot_shutdown = true;
				// Disable steppers
				digitalWrite(4,OFF);   // Disable motors
			  #endif	
			  }
			else if (batteryVoltage < BATTERY_WARNING)
			  {
			  // Battery warning
			  // What to do here???
				bufferA[1]='P';//set buffer to send to Pi
				bufferA[4]=97;
				//bufferA[4]+=30;
				int i;
				for (i = 0; i <= 5; i = i + 1) {
				Serial.print((char)bufferA[i]);
				}	
			  
			  
			  
			  //Serial.println("LOW BAT!!");
			  //WITA.Servo(3,SERVO_AUX_NEUTRO+300);  // Move arm?
			  }
    }  // Slow loop
}

