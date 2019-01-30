#include <LiquidCrystal.h>
#include <QTRSensors.h>

    #define NUM_SENSORS             6  // number of sensors used
    #define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
    #define EMITTER_PIN             2  // emitter is controlled by digital pin 2
    #define Kp 10
    #define Ki 0.0001
    #define Kd 20
    const int rs = 14, en = 15, d4 = 16, d5 = 17, d6 = 18, d7 = 19;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#define trigPinLEFT A8
#define echoPinLEFT A9
#define trigPinFRONT A10
#define echoPinFRONT A11
#define trigPinRIGHT A12
#define echoPinRIGHT A13

long duration, distance, RightSensor,BackSensor,FrontSensor,LeftSensor;
  
    int speedLeft=100;
    int speedRight=100;




    // sensors 0 through 7 are connected to analog inputs 0 through 7, respectively
    QTRSensorsAnalog qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5},
      NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
    unsigned int sensorValues[NUM_SENSORS];


    int MOTOR1_PIN1 = 12;

    int MOTOR1_PIN2 = 13;

    int MOTOR1_PWM = 11;
    
    int MOTOR2_PIN1 = 4;

    int MOTOR2_PIN2 = 3;

    int MOTOR2_PWM = 5;

    unsigned int last_proportional = 0;
    long integral = 0;





    void setup() {
      pinMode(trigPinLEFT, OUTPUT);
pinMode(echoPinLEFT, INPUT);
pinMode(trigPinFRONT, OUTPUT);
pinMode(echoPinFRONT, INPUT);
pinMode(trigPinRIGHT, OUTPUT);
pinMode(echoPinRIGHT, INPUT);


lcd.begin(16,2);

lcd.setCursor(0,0);



      pinMode(MOTOR1_PIN1, OUTPUT);

      pinMode(MOTOR1_PIN2, OUTPUT);

      pinMode(MOTOR1_PWM, OUTPUT);
      
      pinMode(MOTOR2_PIN1, OUTPUT);

      pinMode(MOTOR2_PIN2, OUTPUT);

      pinMode(MOTOR2_PWM, OUTPUT);
     
      int i, c=1, d = 1;
      pinMode(13, OUTPUT);
      
    digitalWrite(13, HIGH);    // turn on LED to indicate we are in calibration mode
    delay(1000);
      for (i = 0; i < 20; i++)  // make the calibration take about 10 seconds
      {
        if ( i %2 == 0) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
        {
          turn_right(c);
          c *= -1;
        }
        else
        {
          turn_left(d);
          d *= -1;
        }
        qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
      }
      digitalWrite(13, LOW);     // turn off LED to indicate we are through with calibration


    }





    void loop() {
      LeftSensor = SonarSensor(trigPinLEFT, echoPinLEFT);
FrontSensor =SonarSensor(trigPinFRONT, echoPinFRONT);
RightSensor = SonarSensor(trigPinRIGHT, echoPinRIGHT);
lcd.begin(16,2);
lcd.setCursor(0,0);
lcd.print(LeftSensor);
lcd.print(' ');
lcd.print(FrontSensor);
lcd.print(' ');
lcd.print(RightSensor);
lcd.print(' ');
lcd.println();

 
if(FrontSensor>40 &&LeftSensor>10 && RightSensor>10)
{
  go();}
  else if(FrontSensor<40 && LeftSensor <40 && RightSensor>40)
  { right(); }
   else if(FrontSensor<40 && LeftSensor >40 && RightSensor<40)
  {left();}
  else if(FrontSensor<40 && LeftSensor >40 && RightSensor>40)
  {right();}
  else if (FrontSensor>40  &&(LeftSensor <20 || RightSensor<20))
  {
    right360();
    delay(200);
    }

  else
  {
    stop1();}




    unsigned int position = qtra.readLine(sensorValues);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = ((int)position) - 3500;
    Serial.begin(9600);
    Serial.print(position);
    Serial.print('-');
    Serial.println();
    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    int power_difference = proportional*Kp + integral*Ki + derivative*Kd;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    const int max = 85;
    if(power_difference > max)
        power_difference = max;
    if(power_difference < -max)
        power_difference = -max;

    if(power_difference < 0)
      go(max+power_difference, max);
    else
        go(max, max-power_difference);
       

     

    }


    void go(int speedLeft, int speedRight) {

      if (speedLeft > 0) {

        digitalWrite(MOTOR1_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
        analogWrite(MOTOR1_PWM, speedLeft);

      }

      else {

        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, HIGH);
        analogWrite(MOTOR1_PWM, speedLeft);;
      }



      if (speedRight > 0) {

        digitalWrite(MOTOR2_PIN1, HIGH);
        digitalWrite(MOTOR2_PIN2, LOW);
        analogWrite(MOTOR2_PWM, speedRight);;

      }

      else {

        digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, HIGH);
        analogWrite(MOTOR2_PWM, speedRight);;

      }

    }
    void turn_right(int c){
        if(c == 1)
        {
          digitalWrite(MOTOR2_PIN1, HIGH);
          digitalWrite(MOTOR2_PIN2, LOW);
        }
        else {
          digitalWrite(MOTOR2_PIN1, LOW);
          digitalWrite(MOTOR2_PIN2, HIGH);
        }
        analogWrite(MOTOR2_PWM, 50);
        digitalWrite(MOTOR1_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
        analogWrite(MOTOR1_PWM, 0);
        delay(500);
    }

    void turn_left(int c){
        if(c == -1)
        {
          digitalWrite(MOTOR1_PIN1, HIGH);
          digitalWrite(MOTOR1_PIN2, LOW);
        }
        else {
          digitalWrite(MOTOR1_PIN1, LOW);
          digitalWrite(MOTOR1_PIN2, HIGH);
        }
        analogWrite(MOTOR1_PWM, 50);
        digitalWrite(MOTOR2_PIN1, HIGH);
        digitalWrite(MOTOR2_PIN2, LOW);
        analogWrite(MOTOR2_PWM, 0);
        delay(500);
    }



int SonarSensor(int trigPin,int echoPin)
{ distance=0;
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = duration*0.034/2;
return distance;


}
void go() {

      

        digitalWrite(MOTOR1_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
        analogWrite(MOTOR1_PWM, speedLeft);
         digitalWrite(MOTOR2_PIN1, HIGH);
        digitalWrite(MOTOR2_PIN2, LOW);
        analogWrite(MOTOR2_PWM, speedRight);

      

      

      }

void right() {


        digitalWrite(MOTOR1_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
        analogWrite(MOTOR1_PWM, speedLeft);
         digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, LOW);
        analogWrite(MOTOR2_PWM, speedRight);

      

     

      }

void left() {

   

        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, LOW);
        analogWrite(MOTOR1_PWM, speedLeft);
         digitalWrite(MOTOR2_PIN1, HIGH);
        digitalWrite(MOTOR2_PIN2, LOW);
        analogWrite(MOTOR2_PWM, speedRight);

  
      }
void stop1()
{
  
  
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, LOW);
        analogWrite(MOTOR1_PWM, speedLeft);
         digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, LOW);
        analogWrite(MOTOR2_PWM, speedRight);
  
  
  
  }
  void right360()
  {
    
     digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, HIGH);
        analogWrite(MOTOR1_PWM, speedLeft);
         digitalWrite(MOTOR2_PIN1, HIGH);
        digitalWrite(MOTOR2_PIN2, LOW);
        analogWrite(MOTOR2_PWM, speedRight);
    
    
    }
     void rev()
  {
    
     digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, HIGH);
        analogWrite(MOTOR1_PWM, speedLeft);
         digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, HIGH);
        analogWrite(MOTOR2_PWM, speedRight);
    
    
    }
