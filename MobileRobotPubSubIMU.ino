#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <Encoder.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#define ENCODER_USE_INTERRUPTS

//If you choose to drive this robot with the android app then make sure the "define RC_android" is uncommented
//If you choose to use line following application, comment both "define RC_ANDROID" and "define NAVI_STACK"
//comment out 'define NAVI_STACK' below if you are not using the android app to remote control the mobile robot
#define NAVI_STACK

//#define RC_ANDROID

#define CURVE_STEER

#ifdef NAVI_STACK
double vmax =180.099;     //m/s message convert to rpm
double rotmax= 26.383513;   //rad/s of robot message convert to rpm of wheel- rad/s to m/s [*radius(m)] then m/s to rpm factor of 180 
#endif

#ifdef RC_ANDROID
double vmax =70;
double rotmax=25;
#endif

#if !defined(RC_ANDROID) && !defined(NAVI_STACK)
double vmax =1;
double rotmax=1;
#endif

#ifdef CURVE_STEER
bool curve_steer = true;
#else
bool curve_steer = false;
#endif


Adafruit_BNO055 bno = Adafruit_BNO055(55);
Encoder motor2(18,19);
Encoder motor1(2,3);


int pwm1 = 7;
int dir1 = 6;
int pwm2 = 5;
int dir2 = 4;
int redLED = 22;

int stallcount1 = 0;
int stallcount2 = 0;
int blockcnt = 0;
bool stallflag = false;

long prevpos2 =0;
long currentpos2= 0;
long prevpos1 =0;
long currentpos1= 0;
unsigned long prevtime = 0;

double rpm2;
double rpm1;

double Vtarget=0;
double Rottarget =0;


//motor 1 speed control/////////////////////
double Vtarget1=0;
double error1 = 0;
double P1 = 0;
double kp1 = 0.4;//= 0.6;
double I1 = 0;
double integral1 = 0;
double ki1 =0.9;// 0.9;
double pwmVal1 = 0;

//motor 2 speed control////////////////////
double Vtarget2=0;
double error2 = 0;
double P2 = 0;
double kp2 = 0.4;//= 0.6;
double I2 = 0;
double integral2 = 0;
double ki2 = 0.9;//0.6;
double pwmVal2 = 0;


ros::NodeHandle  nh;




void messageCb( const geometry_msgs::Twist& msg){
   if(!stallflag){
    if(abs(msg.linear.x)*rotmax < 0.1){
      Vtarget = 0;
    }
    else{
      Vtarget = (msg.linear.x)*vmax;
    }
    if(abs(msg.angular.z)*rotmax < 0.1){
      Rottarget =0;
      Vtarget1 = Vtarget;
      Vtarget2 = Vtarget;
    }
    else{
      Rottarget = (msg.angular.z)*rotmax;
      if(curve_steer){
        if(Vtarget>0){
         Vtarget1 = Vtarget + Rottarget;
         Vtarget2 = Vtarget - Rottarget;            
        }
        else{     //no arc turning backwards
         Vtarget1 = Rottarget;
         Vtarget2 = -Rottarget;
        }
      }
      else{
        Vtarget1 = Rottarget;
        Vtarget2 = -Rottarget;
      }

    }
   }
   else{
    blockcnt++;
    Vtarget= 0;
    Vtarget1 = Vtarget;
    Vtarget2 = Vtarget;
    if(blockcnt == 10){
      stallflag = false;
      blockcnt=0;
      digitalWrite(redLED,LOW);
    }
   }
   
}

std_msgs::Int16MultiArray qecounts;
std_msgs::Float32MultiArray imuRead;

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", messageCb );
ros::Publisher quadenc("quadenc", &qecounts);
ros::Publisher imupub("imupub", &imuRead);
char dim0_label[] = "quad";
char dim0_label2[] = "imu";

void setup()
{
  //imu data
  imuRead.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 10);
  imuRead.layout.dim[0].label = dim0_label;
  imuRead.layout.dim[0].size = 10;
  imuRead.layout.dim[0].stride = 1*10;
  imuRead.layout.data_offset = 0;
  imuRead.data_length = 10;
  imuRead.data = (float *)malloc(sizeof(float)*10);

  //qe data
  qecounts.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  qecounts.layout.dim[0].label = dim0_label;
  qecounts.layout.dim[0].size = 2;
  qecounts.layout.dim[0].stride = 1*2;
  qecounts.layout.data_offset = 0;
  qecounts.data_length = 2;
  qecounts.data = (int *)malloc(sizeof(int)*2);

  //i/o
  pinMode(dir2, OUTPUT);
  pinMode(dir1,OUTPUT);
  pinMode(redLED, OUTPUT);
  digitalWrite(dir2,HIGH);
  digitalWrite(dir1,HIGH);

  //node setup
  nh.initNode();
  nh.advertise(quadenc);
  nh.advertise(imupub);
  nh.subscribe(sub); 

  //imu calib
  bno.begin();
 bno.setMode(bno.OPERATION_MODE_CONFIG);
  int eeAddress = 0;
  long bnoID;
  EEPROM.get(eeAddress, bnoID);
  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  bno.getSensor(&sensor);
  if (bnoID == sensor.sensor_id){
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    bno.setSensorOffsets(calibrationData);
    digitalWrite(redLED,HIGH);
    delay(500);
    digitalWrite(redLED,LOW);
    delay(500);    
    digitalWrite(redLED,HIGH);
    delay(500);
    digitalWrite(redLED,LOW); 
  } 
  bno.setMode(bno.OPERATION_MODE_NDOF);
  delay(500);
  bno.setExtCrystalUse(true);
  
}

void loop()
{  
  Wire.begin();
  nh.spinOnce(); 
  if((millis()-prevtime) >= 50){

   //RPM CALC ///////////////////////////////////////////////////    
   currentpos1 = motor1.read();  
   currentpos2 = motor2.read();
   qecounts.data[0] = currentpos1-prevpos1;
   qecounts.data[1]= currentpos2-prevpos2;
   if(currentpos1-prevpos1 == 0){
     rpm1 = 0;
   }
   else{
     rpm1 = (currentpos1 - prevpos1)*.267857142;
   }
        
   if(currentpos2-prevpos2 == 0){
     rpm2 = 0;
   }
   else{
     rpm2 = (currentpos2 - prevpos2)*.267857142;
   }
   prevtime = millis();
   prevpos1 = currentpos1;
   prevpos2 = currentpos2;
  //END RPM CALC/////////////////////////////////////////////////
   if (Vtarget2 != 0 && Vtarget1 != 0){

        //STALL PROTECTION////////////////////////////////////////////
        if(abs(rpm2) <= 0.1){
          stallcount2++;
          if(stallcount2 == 10){
            digitalWrite(redLED,HIGH);
            stallflag=true;
            stallcount2 = 0;
          }
        }
        else{
          stallcount2 = 0;
        }
        if(abs(rpm1) <= 0.1){
          stallcount1++;
          if(stallcount1 == 10){
            digitalWrite(redLED,HIGH);
            stallflag=true;
            stallcount1 = 0;
          }
        }
        else{
          stallcount1 = 0;
        }
        //END STALL PROTECTION/////////////////////////////////////////

        //PI SPEED CONTROL/////////////////////////////////////////////
        error1 = Vtarget1 - rpm1;
        error2 = Vtarget2 - rpm2; 
        
        P1 = error1 * kp1;   
        P2 = error2 * kp2;

        integral1 += error1;
        integral2 += error2;

        I1 = integral1* ki1;
        I2 = integral2 * ki2;

        pwmVal1 = P1+I1;
        pwmVal2 = P2+I2;
        
        if(pwmVal1 <0){
          digitalWrite(dir1,LOW);
          pwmVal1 = abs(pwmVal1);
        }
        else{
          digitalWrite(dir1,HIGH);
        }
        
        if(pwmVal2 <0){
          digitalWrite(dir2,LOW);
          pwmVal2 = abs(pwmVal2);
        }
        else{
          digitalWrite(dir2,HIGH);
        }

        analogWrite(pwm1,constrain(pwmVal1,0,255));
        analogWrite(pwm2,constrain(pwmVal2,0,255));
        //END PI SPEED CONTROL/////////////////////////////////////////
        
     }
     else{
        analogWrite(pwm1,0);
        analogWrite(pwm2,0);
     }
  imu::Quaternion quat = bno.getQuat();
  imuRead.data[0] = quat.x();
  imuRead.data[1] = quat.y();
  imuRead.data[2] = quat.z();
  imuRead.data[3] = quat.w();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imuRead.data[4] = gyro.x();
  imuRead.data[5] = gyro.y();
  imuRead.data[6] = gyro.z();
  imu::Vector<3> linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imuRead.data[7] = linacc.x();
  imuRead.data[8] = linacc.y();
  imuRead.data[9] = linacc.z();
  
  imupub.publish(&imuRead);
  
  quadenc.publish(&qecounts);
  
    
  }
  
}
