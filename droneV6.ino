//bialo czarne clockwise
//czerwono - niebieskie  anti-clockwise
//--------------00000-------- NORMAL SIDE UP, FROM BATTERY SIDE
//A2 B3 
//B2 A1

#include <MPU6500_WE.h>
#include <Wire.h>
#include <math.h>

#define MOT1 D5
#define MOT2 D7
#define MOT3 D6
#define MOT4 D8

#define KP 0.3
#define KI 0.2
#define KD 0.3
#define MPU6500_ADDR 0x68
   //Change to the address of the IMU
#define NUM_ANG 4
#define UPSIDEDOWN 1 //-1 NORMAL, 1 UPSIDEDOWN
enum Drone_status {STOP,START,FLYING};
enum Angle {ROLL,PITCH,YAW,LEVEL};
enum Dir {X,Y,Z};

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);



int mot[4]={MOT1,MOT2,MOT3,MOT4};
int k=0;
float accel_weight=0.7;
float gyro_weight=0.3;

double dir_accel[3]={0};
double comp_angle[NUM_ANG]={0};
double setpoint[NUM_ANG]={0};
double last_error[NUM_ANG]={0};
double integral_error[NUM_ANG]={0};

long last_time=0;
long deltaT=0;


Drone_status status;

void setup() {
  pinMode(MOT1,OUTPUT);
  pinMode(MOT2,OUTPUT);
  pinMode(MOT3,OUTPUT);
  pinMode(MOT4,OUTPUT);

  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
  }
  
  /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical 
   * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the 
   * MPU6500 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset 
   * values. It assumes your MPU6500 is positioned flat with its x,y-plane. The more you deviate 
   * from this, the less accurate will be your results.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called at the beginning since it can overwrite your settings!
   */
  // Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  // delay(1000);
  // myMPU6500.autoOffsets();
  Serial.println("Done!");
  
  /*  This is a more accurate method for calibration. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  Use either autoOffset or setAccOffsets, not both.
   */
  //myMPU6500.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

  /*  The gyroscope data is not zero, even if you don't move the MPU6500. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffset or setGyrOffsets, not both.
   */
  //myMPU6500.setGyrOffsets(45.0, 145.0, -105.0);

  /*  You can enable or disable the digital low pass filter (DLPF). If you disable the DLPF, you 
   *  need to select the bandwdith, which can be either 8800 or 3600 Hz. 8800 Hz has a shorter delay,
   *  but higher noise level. If DLPF is disabled, the output rate is 32 kHz.
   *  MPU6500_BW_WO_DLPF_3600 
   *  MPU6500_BW_WO_DLPF_8800
   */
  myMPU6500.enableGyrDLPF();
  //myMPU6500.disableGyrDLPF(MPU6500_BW_WO_DLPF_8800); // bandwdith without DLPF
  
  /*  Digital Low Pass Filter for the gyroscope must be enabled to choose the level. 
   *  MPU6500_DPLF_0, MPU6500_DPLF_2, ...... MPU6500_DPLF_7 
   *  
   *  DLPF    Bandwidth [Hz]   Delay [ms]   Output Rate [kHz]
   *    0         250            0.97             8
   *    1         184            2.9              1
   *    2          92            3.9              1
   *    3          41            5.9              1
   *    4          20            9.9              1
   *    5          10           17.85             1
   *    6           5           33.48             1
   *    7        3600            0.17             8
   *    
   *    You achieve lowest noise using level 6  
   */
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);

  /*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
   *  Sample rate = Internal sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
   *  Divider is a number 0...255
   */
  myMPU6500.setSampleRateDivider(5);

  /*  MPU6500_GYRO_RANGE_250       250 degrees per second (default)
   *  MPU6500_GYRO_RANGE_500       500 degrees per second
   *  MPU6500_GYRO_RANGE_1000     1000 degrees per second
   *  MPU6500_GYRO_RANGE_2000     2000 degrees per second
   */
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);

  /*  MPU6500_ACC_RANGE_2G      2 g   (default)
   *  MPU6500_ACC_RANGE_4G      4 g
   *  MPU6500_ACC_RANGE_8G      8 g   
   *  MPU6500_ACC_RANGE_16G    16 g
   */
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);

  /*  Enable/disable the digital low pass filter for the accelerometer 
   *  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
   */
  myMPU6500.enableAccDLPF(true);

  /*  Digital low pass filter (DLPF) for the accelerometer, if enabled 
   *  MPU6500_DPLF_0, MPU6500_DPLF_2, ...... MPU6500_DPLF_7 
   *   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
   *     0           460               1.94           1
   *     1           184               5.80           1
   *     2            92               7.80           1
   *     3            41              11.80           1
   *     4            20              19.80           1
   *     5            10              35.70           1
   *     6             5              66.96           1
   *     7           460               1.94           1
   */
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);

  /* You can enable or disable the axes for gyroscope and/or accelerometer measurements.
   * By default all axes are enabled. Parameters are:  
   * MPU6500_ENABLE_XYZ  //all axes are enabled (default)
   * MPU6500_ENABLE_XY0  // X, Y enabled, Z disabled
   * MPU6500_ENABLE_X0Z   
   * MPU6500_ENABLE_X00
   * MPU6500_ENABLE_0YZ
   * MPU6500_ENABLE_0Y0
   * MPU6500_ENABLE_00Z
   * MPU6500_ENABLE_000  // all axes disabled
   */
  //myMPU6500.enableAccAxes(MPU6500_ENABLE_XYZ);
  //myMPU6500.enableGyrAxes(MPU6500_ENABLE_XYZ);
  delay(200);

  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();

  dir_accel[Z]=UPSIDEDOWN*gValue.z;
  dir_accel[Y]=gValue.y;
  dir_accel[X]=gValue.x;
  comp_angle[ROLL]=atan2(gValue.y,UPSIDEDOWN*gValue.z)*180/PI;
  comp_angle[PITCH]=atan2(gValue.x,UPSIDEDOWN*gValue.z)*180/PI;
  setpoint[LEVEL]=-1;
 }

void loop() {

  deltaT=millis()-last_time;
  last_time=millis();
  status=FLYING;
  if(status==START){
    for(int i=0;i<NUM_ANG;i++){
      integral_error[i]=0;
    }
    status=FLYING;
  }

  if(status==FLYING){
    
    xyzFloat gValue = myMPU6500.getGValues();
    xyzFloat gyr = myMPU6500.getGyrValues();


    //********************************************
    //CALCULATING COMPLEMENTARY ANGLES
    //********************************************
    
    //low pass filter of accel data
    // dir_accel[X]=dir_accel[X]+(gValue.x-dir_accel[X])/32;
    // dir_accel[Y]=dir_accel[Y]+(gValue.y-dir_accel[Y])/32;
    // dir_accel[Z]=dir_accel[Z]+(UPSIDEDOWN*gValue.z-dir_accel[Z])/32;
    dir_accel[X]=gValue.x;
    dir_accel[Y]=gValue.y;
    dir_accel[Z]=UPSIDEDOWN*gValue.z;

    double angle_accel[NUM_ANG]={0};
    angle_accel[ROLL]=atan2(dir_accel[Y],dir_accel[Z])*180/PI;
    angle_accel[PITCH]=atan2(dir_accel[X],dir_accel[Z])*180/PI;
    angle_accel[YAW]=0;

    double angle_gyro[NUM_ANG]={0};
    angle_gyro[ROLL]=-gyr.x*deltaT/1000+comp_angle[ROLL];
    angle_gyro[PITCH]=gyr.y*deltaT/1000+comp_angle[PITCH];

   //complementary angles from measured gyro and accel angle
    for(int i=0;i<NUM_ANG;i++){
      if(i==YAW){
        comp_angle[i]=gyr.z;
        if(comp_angle[YAW]<2)comp_angle[YAW]=0;
      }
      if(i==LEVEL){
        comp_angle[i]=dir_accel[Z];
      }
      else{
       comp_angle[i]=accel_weight*angle_accel[i]+gyro_weight*angle_gyro[i];
      }
    }

    //********************************************
    //PID PART
    //********************************************

      
    double error[NUM_ANG]={0};
    double derivative_error[NUM_ANG]={0};
    double output[NUM_ANG]={0};

    for(int i=0;i<NUM_ANG;i++){
      error[i]=comp_angle[i]-setpoint[i];
      derivative_error[i]=(last_error[i]-error[i]);
      integral_error[i]+=error[i]*deltaT/1000;
      output[i]=(KP*error[i]+KD*derivative_error[i]+KI*integral_error[i]);
      last_error[i]=error[i];      
    }
    
    int motor_pwm[4]={0};
    motor_pwm[0]=(int)((-output[PITCH]+output[ROLL])-output[YAW]+output[LEVEL]);
    motor_pwm[1]=(int)((-output[PITCH]-output[ROLL])+output[YAW]+output[LEVEL]);
    motor_pwm[2]=(int)((output[PITCH]+output[ROLL])-output[YAW]+output[LEVEL]);
    motor_pwm[3]=(int)((output[PITCH]-output[ROLL])+output[YAW]+output[LEVEL]);

    for(int i=0;i<4;i++){
      if(motor_pwm[i]>255)motor_pwm[i]=255;
      if(motor_pwm[i]<0)motor_pwm[i]=0;
      analogWrite(mot[i],motor_pwm[i]);
    }

    if(k>=25){
      for(int i=0;i<NUM_ANG;i++){
       // Serial.printf("%d: accel angle=%f gyro angle=%f \n",i,angle_accel[i],angle_gyro[i]);
        
        //Serial.printf("%d: error=%f integ_error=%f der_error=%f\n",i,error[i],integral_error[i],derivative_error[i]);
        //Serial.printf("%d mot pwm [ %d ] \n",i,motor_pwm[i]);
      }
      for(int i=0;i<4;i++){
             Serial.printf("%d mot pwm [ %d ] \n",i,motor_pwm[i]);
      }
      k=0;
    }
    k++;
  }
    
  delay(1);
}