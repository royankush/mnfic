#define motorPin1 11
#define motorPin2 12
long int currTimeHour = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(motorPin1,OUTPUT);
  pinMode(motorPin2,OUTPUT);

}  

void loop() {
  // put your main code here, to run repeatedly: 
unsigned long runMillis= millis();
unsigned long allSeconds=millis()/1000;
int hours= allSeconds/3600;
int secsRemaining=allSeconds%3600;
int runMinutes=secsRemaining/60;
int runSeconds=secsRemaining%60;

char buf[21];
sprintf(buf,"Runtime%02d:%02d:%02d", hours,runMinutes,runSeconds);


Serial.println(buf);
delay(5000);

currTimeHour = hours;


if(hours%6 == 0) 
  {
    if(runMinutes < 2)
        {
         digitalWrite(motorPin1, HIGH);
         digitalWrite(motorPin2, HIGH);
         Serial.print("     ");
         Serial.println("ON");
         
         }
     else
     {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
     }
  }
else
    {
     digitalWrite(motorPin1, LOW);
     digitalWrite(motorPin2, LOW);
    }
  
}


  
