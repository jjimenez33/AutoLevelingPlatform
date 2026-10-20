#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Servo.h>


//// Define Servos ////////////////////////////////////////////////////////////////////////////////////////////////////
Servo pitchServo;
Servo rollServo;

//// Accelerometer ////////////////////////////////////////////////////////////////////////////////////////////////////
float rollM;
float pitchM;
float g      = 9.807;

//// Magnetometer /////////////////////////////////////////////////////////////////////////////////////////////////////
float yawM;

//// Define Quaternion [q0, q1, q2, q3] ///////////////////////////////////////////////////////////////////////////////
float q0;
float q1;
float q2;
float q3;

//// Roll Variables ///////////////////////////////////////////////////////////////////////////////////////////////////
float rollTarget = 0;     // Sets target to be 0 deg (stabilized)
float rollActual;         // Real Roll angle measured by IMU
float rollError = 0;      // Error between Target and Actual Roll
float rollErrorOld;       // Old error value
float rollErrorChange;    // Change in error from previous timestep to current
float rollErrorSlope = 0; // Derivative of roll error 
float rollErrorArea = 0;  // Integral of roll error
float rollServoVal = 90;  // This sets the initial position of servo to 90 deg and is +/- by a # 

//// Pitch Variables //////////////////////////////////////////////////////////////////////////////////////////////////
float pitchTarget = 0;      // Sets target to be 0 deg (stabilized)
float pitchActual;          // Real Roll angle measured by IMU
float pitchError = 0;       // Error between Target and Actual Roll
float pitchErrorOld;        // Old error value
float pitchErrorChange;     // Change in error from previous timestep to current
float pitchErrorSlope = 0;  // Derivative of roll error 
float pitchErrorArea = 0;   // Integral of roll error
float pitchServoVal = 90;   // This sets the initial position of servo to 90 deg and is +/- by a # 

//// Madgwick Filter Filter /////////////////////////////////////////////////////////////////////////////////////////////////////


// For testing results
float target_mag = 0;
float actual_mag;

//// PID Gains /////////////////////////////////////////////////////////////////////////////////////////////////////////
float k1 = 1.2;
float k2 = 0.01; // lowering this reduces huge gitter
float k3 = 0.0001; // if we lower this it resets better

//// Update ////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long previousTime = 0;
const int loopInterval = 20;      // Loops every 20 ms = 50 Hz
unsigned long milliOld;
unsigned long milliNew;
const float dt = 0.02;

float smoothMove(float current, float target, float maxStep) {
    if (target > current + maxStep) return current + maxStep;
    if (target < current - maxStep) return current - maxStep;
    return target;
}


Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);
  myIMU.begin();
  delay(1000);

  myIMU.setExtCrystalUse(true);

  rollServo.attach(2);
  pitchServo.attach(3);

  rollServo.write(rollServoVal);
  
  pitchServo.write(pitchServoVal);

  delay(1000);

}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= loopInterval) {
        previousTime = currentTime;
     
        uint8_t system, gyro, accel, mg = 0;

        myIMU.getCalibration(&system, &gyro, &accel, &mg); 

        imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

        // Get pitch and roll from accelerometer
        pitchM  = atan2(-1*acc.x() / g,  sqrt(pow(acc.y(),2) + pow(acc.z(),2)) / g);
        rollM   = atan2(acc.y() / g, acc.z() / g);

        // Get yaw from magnetometer
        yawM   = -1*atan2(mag.y()*cos(rollM)-mag.z()*sin(rollM), mag.x()*cos(pitchM)+mag.y()*sin(pitchM)*sin(rollM)+mag.z()*sin(pitchM)*cos(rollM));

        // Extract Quaternions from IMU //
        imu::Quaternion quat = myIMU.getQuat();

        q0 = quat.w();
        q1 = quat.x();
        q2 = quat.y();
        q3 = quat.z();

        // Calculate Roll and Pitch from Quaternion //
        rollActual = atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2)) * 180/3.14;
        pitchActual = asin(2 * (q0*q2 - q3*q1)) * 180/3.14;

        // Calculate Error Between Target and Actual Angle //
        // Roll
        rollErrorOld = rollError;                         // Updates old error as error from previous timestep
        rollError = rollTarget - rollActual;              // Calcules new error (Target - Actual)
        rollErrorChange = rollError - rollErrorOld;       // Calculates change in error from current timestep to current
        rollErrorSlope = rollErrorChange / dt;            // Calculates slope of the change
        rollErrorArea = rollErrorArea + rollError*dt;     // Integrates the error
        // Pitch
        pitchErrorOld = pitchError;
        pitchError = pitchTarget - pitchActual;
        pitchErrorChange = pitchError - pitchErrorOld;
        pitchErrorSlope = pitchErrorChange / dt;
        pitchErrorArea = pitchErrorArea + pitchError*dt;

        // Deadband //
        if (abs(rollError) < 0.1) rollError = 0;
        if (abs(pitchError) < 0.1) pitchError = 0;

        // PID Controller //
        //Roll
        rollServoVal = smoothMove(rollServo.read(), 90 - k1 * rollError + k2 * rollErrorSlope + k3 * rollErrorArea, 2);
        rollServo.write(rollServoVal);
        
        // Pitch
        pitchServoVal = smoothMove(pitchServo.read(), 90 + k1 * pitchError + k2 * pitchErrorSlope + k3 * pitchErrorArea, 2);
        pitchServo.write(pitchServoVal);


        // Print the data
        //Serial.print(rollTarget);
        //Serial.print(",");
        //Serial.print(rollActual);
        //Serial.print(",");
        //Serial.print(pitchTarget);
        //Serial.print(",");
        //Serial.print(pitchActual);
        Serial.print(pitchM);
        Serial.print(",");
        Serial.print(rollM);
        Serial.print(",");
        Serial.println(yawM);
        //Serial.print(",");
        //Serial.print(accel);
        //Serial.print(",");
        //Serial.print(gyro);
        //Serial.print(",");
        //Serial.print(mg);
        //Serial.print(",");
        //Serial.println(system);

    }
}
