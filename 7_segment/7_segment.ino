int a=7;
int b= 8;
int  c= 9;
int d= 10;
int e= 11;
int f= 12;
int g =13;
void setup() {
  // put your setup code here, to run once:
  pinMode(a,OUTPUT);
    pinMode(b,OUTPUT);
  pinMode(c,OUTPUT);
  pinMode(d,OUTPUT);
  pinMode(e,OUTPUT);
  pinMode(f,OUTPUT);
  pinMode(g,OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
  //1
  digitalWrite(b,HIGH);
  digitalWrite(c,HIGH);
  digitalWrite(a,LOW);
    digitalWrite(d,LOW);
      digitalWrite(e,LOW);
        digitalWrite(f,LOW);
          digitalWrite(g,LOW);

  delay(1000);
  
  //2
     digitalWrite(a,HIGH);
  digitalWrite(g,HIGH);
     digitalWrite(b,HIGH);
    digitalWrite(e,HIGH);
    digitalWrite(d,HIGH);
    digitalWrite(c,LOW);
    digitalWrite(f,LOW);


    delay(1000);
  
  //3
    digitalWrite(a,HIGH);
    digitalWrite(b,HIGH);
    digitalWrite(g,HIGH);
    digitalWrite(c,HIGH);
    digitalWrite(d,HIGH);
    digitalWrite(e,LOW);
        digitalWrite(f,LOW);

    delay(1000);
    
  
  //4
    digitalWrite(f,HIGH);
    digitalWrite(g,HIGH);
    digitalWrite(b,HIGH);
    digitalWrite(c,HIGH);
    digitalWrite(e,LOW);
        digitalWrite(a,LOW);
            digitalWrite(d,LOW);



    delay(1000);
  
  //5
    digitalWrite(a,HIGH);
    digitalWrite(f,HIGH);
    digitalWrite(g,HIGH);
    digitalWrite(c,HIGH);
    digitalWrite(d,HIGH);
       digitalWrite(b,LOW);
           digitalWrite(e,LOW);


    delay(1000);
  
  }
