//Updated September 30, 2022

// Servo is leveling but only when board is flat. Needs to be when board is upright. (Roll Servo)


#include <Arduino.h>
#include <Adafruit_BNO08x.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BNO055.h> // Prototype is using IMU BNO055
 #include <utility/imumaths.h>
#include <Servo.h>
#include <math.h> 

Servo pitchServo;
Servo rollServo; 

uint8_t system1, gyro, accel, mg = 0;


// delta t variables 
float milliold = 0;
float millinew = 0;
float dt = 0;


//// roll variables
float rollservoval = 90; // value feed back into control system 
float pitchservoval = 90;  


/// Quaternion variables 
float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; // estimated orientation quanternion elements with initial conditions
int incomingByte;
bool ok = false;


//set the delay bewtweem fresh samples
uint16_t BNO055_SAMPLERATE_DELAY_MS = 0; //Lower this value for a less jittery servo 


struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;


struct PID {
  float kp = 0.29f;  // .4 INCREASE INTUL 
  float ki = 0.1f; // devide by delta t  10 derivative 
  float kd = 0.010f; // multiplied by delta t RAMP UP UNTIL OSCILLATION STARTS. 

  float tau = 0.23f;

  float servolimMin = 55; //Servo Limits
  float servolimMax = 115;
  
  float limMin = -25.0f; 
  float limMax = 25.0f;

  float limMinServ = 0.0f; 
  float limMaxServ = 180.0f;

  float limMinInt = -5.0f; //was -5.0
  float limMaxInt = 5.0f; // was 5.0

  float integrator = 0.0f;
  float prevError = 0.0f;
  float differentiator = 0.0f;
  float prevMeasurement = 0.0f;

  float out = 0.0f; 
  float outPitch = 0.0f;
 
  float setpoint = -90.0f; // Roll Target Setpoint
  float setpointPitch = 0.0f;
} pid;


// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  pitchServo.attach(36);
  rollServo.attach(37);
  pitchServo.write(pitchservoval); 
    delay(1000);
  rollServo.write(rollservoval); 
    delay(1000);
  Serial.println("Adafruit BNO055 test!");



  if(!bno.begin())
  {
    Serial.println("Oooops, no BNO055 detected...Check your wiring or I2C address!");
    while(1);
  }
  else if (bno.begin())
  {
     Serial.println("BNO055 Found!!");
     delay(1000);
     bno.setExtCrystalUse(true);
     bno.getCalibration(&system1, &gyro, &accel, &mg);
     int8_t temp = bno.getTemp(); //Getting the temp values from the IMU
    

  }
 
}

void quaternionToEuler(float q0, float q1, float q2, float q3, euler_t* ypr, bool degrees = false) {

    float sq0 = sq(q0);
    float sq1 = sq(q1);
    float sq2 = sq(q2);
    float sq3 = sq(q3);

  /*
    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
*/

    ypr->roll = -atan2(2.0 * (q0 * q1 + q2 * q3), 1-2 * (sq1 + sq2));
    ypr->pitch = asin(2.0 * (q0 * q2 - q3 * q1));
    ypr->yaw = -atan2(2.0 * (q0 * q3 + q1 * q2), 1-2 * (sq2 + sq3));
    
    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

 float PIDupdate(float measurement, float dt, float setpoint, PID *pid) 
 {
  
  float error = setpoint - measurement; 
  float proportional = pid->kp * error; 
  pid->integrator = pid->integrator + 0.5f * pid ->ki * dt * (error + pid->prevError);
  
  if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

   }
   pid->differentiator = -(2.0f * pid->kd * (measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - dt) * pid->differentiator)
                        / (2.0f * pid->tau + dt);
   pid->out = proportional + pid->integrator + pid->differentiator;
//   Serial.print(proportional);  Serial.print("\t");
//   Serial.print(pid->integrator); Serial.print("\t");
//   Serial.print(pid->differentiator); Serial.print("\t");

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }
 /* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;


  /* Return controller output */
    return pid->out;
}

void loop() {

  imu::Quaternion quat = bno.getQuat();           // Request quaternion data from BNO055

  Serial.print("Quat W: "); Serial.print(quat.w(), 4);  Serial.print("\t"); // Print quaternion w
  Serial.print("Quat X: ");Serial.print(quat.x(), 4);  Serial.print("\t"); // Print quaternion x
  Serial.print("Quat Y: ");Serial.print(quat.y(), 4);  Serial.print("\t"); // Print quaternion y
  Serial.print("Quat W: ");Serial.print(quat.z(), 4);  Serial.println();   // Print quaternion z

  //Convert to Euler and fill ypr-> roll, ypr->yaw, ypr->pitch
  quaternionToEuler(quat.w(), quat.x(), quat.y(), quat.z(), &ypr, true); 



  PIDupdate(ypr.roll, dt, pid.setpoint, &pid);
  Serial.print("\nPid Out Update: "); Serial.print(pid.out); Serial.print("\t");
  //rollservoval += -pid.out * 6; // MULTIPLY BY SIX. MATLAB AND SIMULINK BETTER IN C++ OR ARDUINO FOR SIMULAITONS. original
  rollservoval += pid.out * 1; // rollservoval += pid.out * 1; // 
  
 
  
  if (rollservoval > pid.servolimMax) {

    rollservoval = pid.servolimMax;

   } else if (rollservoval < pid.servolimMin) {

     rollservoval = pid.servolimMin;

   }

     Serial.print("\nRollServoVal: "); Serial.print(rollservoval); Serial.print("\t\n\n");
     Serial.print("\n ypr.roll: "); Serial.print(ypr.roll); Serial.print("\t\n");
     rollServo.write(rollservoval);
/*
     
//-------------------------------------------------------------------------------------//
      // Pitch
      //Bottom Servo 
     PIDupdate(ypr.pitch, dt, pid.setpoint, &pidw);
     Serial.print(pid.out); Serial.print("\t");
     pitchservoval += -pid.out * 1;
    // pitchservoval += -pid.out * 6;
     Serial.print(pitchservoval); Serial.print("\t");
     if (pitchservoval > pid.servolimMax) 
     {

        pitchservoval = pid.servolimMax;

     } else if (pitchservoval < pid.servolimMin) {

        pitchservoval = pid.servolimMin;

     }

       Serial.print("\nPitchServoVal: "); Serial.print(pitchservoval); Serial.print("\t\n\n");
       Serial.print("\n ypr.pitch: "); Serial.print(ypr.pitch); Serial.print("\t\n");
       pitchServo.write(pitchservoval);
        
//-------------------------------------------------------------------------------------//

*/
  

 

  /*
  if (bno08x.wasReset())  ///------------------------------------
  {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (bno08x.getSensorEvent(&sensorValue))  ///------------------------------------
  {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) { //----------
      case SH2_GYROSCOPE_CALIBRATED: //--------
        milliold = millinew;
        millinew = millis();
        dt = millinew - milliold; 
        // calculate preliminary values 
        float SEqDot[3];
        float halfSEq_1 = 0.5 * SEq_1;
        float halfSEq_2 = 0.5 * SEq_2;
        float halfSEq_3 = 0.5 * SEq_3;
        float halfSEq_4 = 0.5 * SEq_4;
        float twoSEq_1 = 2 * SEq_1;
        float twoSEq_2 = 2 * SEq_2;
        float twoSEq_3 = 2 * SEq_3;
        float twoSEq_4 = 2 * SEq_4;
        float x = sensorValue.un.gyroscope.x;
        float y = sensorValue.un.gyroscope.y;
        float z = sensorValue.un.gyroscope.z;
    
        //compute quaternion derivative from measured gyroscope or rate of change of quaternion from gyroscope. 
        SEqDot[0] = -halfSEq_2 * x - halfSEq_3 * y - halfSEq_4 * z;
        SEqDot[1] = halfSEq_1 * x + halfSEq_3 * z - halfSEq_4 * y;
        SEqDot[2] = halfSEq_1 * y - halfSEq_2 * z + halfSEq_4 * x;
        SEqDot[3] = halfSEq_1 * z + halfSEq_2 * y - halfSEq_3 * x;

        // Compute then integrate the estimated quanternion derrivative 
        SEq_1 += SEqDot[0] * (dt/1000);
        SEq_2 += SEqDot[1] * (dt/1000);
        SEq_3 += SEqDot[2] * (dt/1000);
        SEq_4 += SEqDot[3] * (dt/1000);

        // Normalize quaternions
        float norm = sqrt(SEq_1*SEq_1 + SEq_2*SEq_2 + SEq_3*SEq_3 + SEq_4*SEq_4);
        SEq_1 /= norm;
        SEq_2 /= norm;
        SEq_3 /= norm;
        SEq_4 /= norm;
      
        // Convert quaternion to euler 
        quaternionToEuler(SEq_1, SEq_2, SEq_3, SEq_4, &ypr, true); 

        
        */

        /* Important DO NOT DELETE
         
         
        PIDupdate(ypr.roll, dt, pid.setpoint, &pid);
        Serial.print("\nPID ROLL UpDate: "); Serial.print(pid.out); Serial.print("\t");
        rollservoval += -pid.out * 6; // MULTIPLY BY SIX. MATLAB AND SIMULINK BETTER IN C++ OR ARDUINO FOR SIMULAITONS
        Serial.print("\nRollServoVal: "); Serial.print(rollservoval); Serial.print("\t\n");
        if (rollservoval > pid.servolimMax) {

        rollservoval = pid.servolimMax;

        } else if (rollservoval < pid.servolimMin) {

        rollservoval = pid.servolimMin;

        }
        rollServo.write(rollservoval);
        
        PIDupdate(ypr.yaw, dt, pid.setpoint, &pid);
        Serial.print("\nPID YAW UpDate: ");Serial.print(pid.out); Serial.print("\t");
        pitchservoval += -pid.out * 6;
       Serial.print("\nPitchServoVal: "); Serial.print(pitchservoval); Serial.print("\t\n");
        if (pitchservoval > pid.servolimMax) {

        pitchservoval = pid.servolimMax;

        } else if (pitchservoval < pid.servolimMin) {

        pitchservoval = pid.servolimMin;

        }
        pitchServo.write(pitchservoval);
        
         

          
//        if (Serial.available()) {
//          // read the incoming byte:
//           incomingByte = Serial.read();
//        }
//        if (incomingByte == 'A') { ok = true; }
//        if(ok) {
//          //calculate delta e for roll

//          rollerror = rolltarget - ypr->pitch;
//          rollerrorslope = rollerror - rollerrorold/dt;
//          rollerrorarea += rollerror*dt;
//          rollerrorold = rollerror;

//          pitcherror = pitchtarget - ypr->roll;   //proportional 
//          pitcherrorslope = (pitcherror - pitcherrorold)/dt; // derivative slope
//          pitcherrorarea += pitcherror*dt;  // intergral 
//          pitcherrorold = pitcherror;
          
//          rollservoval += (pid.k1*rollerror)+(pid.k2*rollerrorslope)+(pid.k3*rollerrorarea);     
//          rollServo.write(rollservoval);
//          pitchservoval += (pid.k1*pitcherror)+(pid.k2*rollerrorslope)+(pid.k3*rollerrorarea);
//          pitchServo.write(pitchservoval);  
        //}
//        break;
   // }
    */

    
    static long last = 0;
    long now = micros();
    //Serial.print(now - last);             Serial.print("\t");
    last = now;

    /*
   // Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print("\nYPR Yaw: "); Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print("\nYPR Pitch: ");Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.print("\nYPR Roll: ");Serial.println(ypr.roll);
    */
   
 // }
 
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
