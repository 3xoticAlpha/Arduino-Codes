#include <Keypad.h>
#include<LiquidCrystal.h>
const int rs=2, en=3, d4=4,d5=5,d6=6,d7=7;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

const byte ROWS = 4; 
const byte COLS = 3; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {A0,A1,A2,A3}; 
byte colPins[COLS] = {A4,A5,13}; 

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

void setup(){
  lcd.begin(16,2);
  lcd.setCursor(0,0);
}
  
void loop(){
  char customKey = customKeypad.getKey();
  
  if (customKey){
    lcd.print(customKey);
  }
}
