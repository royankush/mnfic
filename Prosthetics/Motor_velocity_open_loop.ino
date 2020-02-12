// DC brushed Motor Velocity control (L293D) in Open Loop
/*
 * try removing print statements of encoder/code
 * check for voltage at encoder input
 * try using (interrupt guards) nointerrupts() for Serial.write & serial.print
 * 
 */
 
// Motor encoder output pulse per rotatio, 
// use 7 for normal, 14 for x2 and 28 for x4 mode
// Gear Ratio 298
#define PPR 28

// Encoder output to Arduino Interrupt pin
#define encA 2
#define encB 3

// For PWM
#define enable_1 6 
#define enable_2 9

// For Motor A
#define input_1 4
#define input_2 5

// Pulse count from encoder
volatile long encoderCount = 0;

//Interval for measurement in millis
int measure_interval = 1000;

//counters for milliseconds during interval 
long prevMillis = 0;
long currMillis = 0;

// variable related to RPM
float measured_rpm = 0;
long setpoint_rpm;
int pwm;


//float gearRatio = 0.1017;
float gearRatio = 298;

// Flags and checks
char option = 'l';
boolean flag = false;
boolean start = false;

void setup() {
  
  pinMode(enable_1, OUTPUT); // For the PWM
  pinMode(enable_2, OUTPUT); // For the PWM
  pinMode(input_1, OUTPUT);
  pinMode(input_2, OUTPUT);
 // pinMode(encA, INPUT_PULLUP);
 // pinMode(encB, INPUT_PULLUP); //Set encoder input with internal pullup
  // Attach interrupt
  attachInterrupt(0, updateEncoder1, CHANGE);
  attachInterrupt(1, updateEncoder2, CHANGE);
  Serial.begin(115200);
  Serial.println(" 'l' for CW, 'r' for CCW and 's' for stop");

}

void loop() {

//For setting new rpm to the motor
//*******************************************************************
  if(flag == false)
  {
    // Max RPM measured from the motor at 6V is 7800
    int RPM_max = 25; // Since I am getting 5V output at max pwm
    String motor_rpm = "Enter new motor speed in rpm (0-" + (String)RPM_max + "): ";
    Serial.println(motor_rpm);
    
    while(!Serial.available()); //Wait until data is read
    
    setpoint_rpm = (int)(Serial.parseInt());
    pwm = map(setpoint_rpm, 0, RPM_max, 0, 255);

    //analogWrite(enable_1,pwm);
    //analogWrite(enable_2,pwm);
          
    prevMillis = millis();
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
//*********************************************************
  currMillis = millis();
  
  if(currMillis - prevMillis > measure_interval)
  {
    prevMillis = currMillis;
    measureRPM();    
  }
  
// Motor Control cases - CW, CCW, Stop and RPM change
//**************************************************************
    if(option == 'l')
    { 
      if(start == false)
      {  startingTorque(option);
         start = true;

         digitalWrite(input_1, HIGH);
         digitalWrite(input_2, LOW);
         
         delay(500);
      }          

      analogWrite(enable_1,pwm);
      analogWrite(enable_2,pwm);
      
    }
    else if(option == 'r')
    {
      if(start == false)
      {  startingTorque(option);
         start = true;

         digitalWrite(input_1, LOW);
         digitalWrite(input_2, HIGH);
         
         delay(500);
      }       

      analogWrite(enable_1,pwm);
      analogWrite(enable_2,pwm);
      
    }
    else if(option == 's')
    {
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, LOW);
      delay(1000);
      start = false;
    }
    else if(option == 'c')
    {
      // to change motor rpm
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, LOW);
      flag = false;
      start = false;
    }
  
}
//-----------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------

// RPM Measurement
//*************************************************
void measureRPM()
{
    float factor = (float) 1000.0/measure_interval;
    measured_rpm = (float)(encoderCount * factor * 60 / PPR);

    Serial.print("Ref RPM: ");
    Serial.print(setpoint_rpm);
    Serial.print("\t PULSES/sec: ");
    Serial.print(encoderCount*factor);
    Serial.print("\t RPM: ");
    Serial.print(measured_rpm/gearRatio);
    Serial.println(" ");

    // Reset the encoder count
    encoderCount = 0;
}

// Interrupt serive routine function
//***************************************************
void updateEncoder1()
{
  /*
  static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_val = 0;

  enc_val = enc_val << 2;
  enc_val = enc_val | ((PIND & 0b1100) >> 2);

  encoderCount = encoderCount + lookup_table[enc_val & 0b1111];
  */
  encoderCount++;
}

void updateEncoder2()
{
  /*
  static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_val = 0;

  enc_val = enc_val << 2;
  enc_val = enc_val | ((PIND & 0b1100) >> 2);

  encoderCount = encoderCount + lookup_table[enc_val & 0b1111];
  */
  encoderCount++;
}


//Starting Torque
//****************************************************
void startingTorque(char c)
{
  int starting_pwm = 250;
  
  analogWrite(enable_1,starting_pwm);
  analogWrite(enable_2,starting_pwm);
    
}
