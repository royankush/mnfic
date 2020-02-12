/* 
  DC brushed Motor Velocity control in Closed Loop PID Control
  Ankush Roy, Indian Institute of Technology Kharagpur

  Motor: N20 6V 30RPM Micro Metal Gear Motor With Encoder
  Motor V_max = 6V
  Encoder: 7 PPR, 2 Channel
  Programming Board: Arduino UNO
  Motor Driver: L293d
  
*/

// Motor encoder output pulse per rotation 
// use 7 for normal, 14 for x2 and 28 for x4 mode
#define PPR 14

// Encoder output to Arduino Interrupt pin
#define encA 2
#define encB 3

// For PWM
#define enable_1 6 
#define enable_2 9

// For Motor A
#define input_1 4
#define input_2 5

/* Pulse counts from encoder */
volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value


//Intervals for measurement (Keep  PID loop longer than the RPM measurement interval)
int measure_interval = 50;
int pid_control_interval = 50;

//counter of milliseconds during interval for Encoder
long prevMillis = 0;
long currMillis = 0;

// Counter of milliseconds for PID controller
long pid_prevMillis = 0;
long pid_currMillis = 0;

// variable related to RPM
// Max RPM measured from the motor at 6V is 7800
int RPM_max = 30;
float measured_rpm = 0;
long setpoint_rpm;
int pwm;

// For PID Control
float prevError = 0;
float integralError = 0;

//float gearRatio = 0.1017;
float gearRatio = 298;

char option; // for switch case
boolean flag = false;

void setup() {
  
  pinMode(enable_1, OUTPUT); // For the PWM
  pinMode(enable_2, OUTPUT); // For the PWM
  pinMode(input_1, OUTPUT);
  pinMode(input_2, OUTPUT);
  pinMode(encB, INPUT_PULLUP); //Set encoder input with internal pullup
  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(encB), updateEncoder, CHANGE);
  Serial.begin(115200);
  Serial.println(" 'l' for CW, 'r' for CCW and 's' for stop");

}

void loop() {
  
//For setting new rpm to the motor
//*******************************************************************
  if(flag == false)
  {
  
    String motor_rpm = "Enter new motor speed in rpm (0-" + (String)RPM_max + "): ";
    Serial.println(motor_rpm);
    
    //Serial.println("Enter new motor speed in rpm (0-833) for bot");
    
    while(!Serial.available()); //Wait until data is read
    
    setpoint_rpm = (int)(Serial.parseInt());
    pwm = map(setpoint_rpm, 0, RPM_max, 0, 255);
    
    analogWrite(enable_1,pwm);
    analogWrite(enable_2,pwm);
          
    prevMillis = millis();
    pid_prevMillis = millis();
    flag = true;  
  }

//Reading the control commands whenever availabale for CW, CCW rotations and stop  
//******************************************************************************* 
  if(Serial.available()>0)
  {
    option = Serial.read();
    Serial.println(option);   
  } 

// RPM measurement values
//**************************************************************
  currMillis = millis();
  
  if(currMillis - prevMillis > measure_interval)
  {
    prevMillis = currMillis;
    measureRPM();    
  }

//PID Controller
//**************************************************************
 pid_currMillis = millis();

  if(pid_currMillis - pid_prevMillis > pid_control_interval)
  {
    pid_prevMillis = pid_currMillis;
    PID_controller();
  }


// Motor Control cases - CW, CCW, Stop and RPM change
//**************************************************************
    if(option == 'l')
    { 
      // First let it slow down a bit
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, LOW);
      delay(1000);
           
      digitalWrite(input_1, HIGH);
      digitalWrite(input_2, LOW);
      delay(1000);
    }
    else if(option == 'r')
    {
      // First let it slow down a bit
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, LOW);
      delay(1000);
      
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, HIGH);
      delay(1000);
    }
    else if(option == 's')
    {
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, LOW);
      delay(1000);
    }
    else if(option == 'c')
    {
      // to change motor rpm
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, LOW);
      flag = false;
    }
  
}
//-----------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------

// PID Controller
//*****************************************************
void PID_controller()
{
  float Kp = 1.0115733295713129; //1.104056;
  float Ki = 2.72969379009806; //0.710442;
  float Kd = 0;
  float u_t;
  int rpm_threshold = 100;
  float intErrorMax = 1000;
  
  float dt = pid_control_interval/1000.0;
  
  float error = setpoint_rpm - measured_rpm;

    // To ensure the input doesnt unncessarily shoot to max
    
    if(measured_rpm == 0)
      integralError = 0.0;
    else
      integralError += error*dt;
     
    float derivativeError = (error - prevError)/dt;
    u_t = Kp*error + Ki*integralError + Kd*derivativeError;
    
    prevError = error; 

    Serial.print("\t Error: ");
    Serial.print(error);
    Serial.print("   u_t: ");
    Serial.print(u_t);
    Serial.println(" ");
  
    if(u_t > RPM_max)
      u_t = RPM_max;
    else if(u_t < 0)
      u_t = 0;
      
  if(error <-rpm_threshold || error>rpm_threshold)
  {  
    pwm = map(u_t, 0, RPM_max, 0, 255);
    analogWrite(enable_1,pwm);
    analogWrite(enable_2,pwm);
  }
  
}

// RPM Measurement
//*****************************************************
void measureRPM()
{
    float factor = (float) 1000.00/measure_interval;
    measured_rpm = (float)(encoderValue * factor * 60 / (PPR*gearRatio));

    Serial.print("Ref RPM: ");
    Serial.print(setpoint_rpm);
    //Serial.print("\t PULSES: ");
    //Serial.print(encoderValue);
    Serial.print("\t RPM: ");
    Serial.print(measured_rpm);
    //Serial.println(" ")
    // Reset the encoder count
    encoderValue = 0;
}

// Interrupt serive routine function
//***************************************************
void updateEncoder()
{
  encoderValue++;
}
