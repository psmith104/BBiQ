// MLX90614 IR Thermometer Library here: http://bildr.org/2011/02/mlx90614-arduino/
// Instructions for wiring thermometer here: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1214872633
// RGB LED wiring and simple fading sketch: http://wiring.org.co/learning/basics/rgbled.html

#include <avr/io.h>
#include <avr/interrupt.h>
#include <i2cmaster.h>
#include <SoftwareSerial.h>
#include <math.h>

#define thermistorPinA 0                 // Analog Pin 0
#define thermistorPinB 1                 // Analog Pin 1

SoftwareSerial lcd(2, 3);  // This is required, to start an instance of an LCD
SoftwareSerial bluetooth(4, 5);  // This is required, to start an instance of bluetooth

// set your thresholds for temperatures here
int irTarget = 70; 
int irVariance = 15;
int tmTargetA = 70; 
int tmVarianceA = 15;
int tmTargetB = 70; 
int tmVarianceB = 15;

int redPin = 11;    // RED pin of the LED
int greenPin = 10;  // GREEN pin of the LED
int bluePin = 9;   // BLUE pin of the LED
int brightness = 0; // LED brightness

int buzzerPin = 6; // BUZZER pin 

int noChange = 0; // For second count since last "off" temperature
int audibleAlarm = 1; // Enable Audible Alarm
int alertBeep = 1; // Enable Alert Beep
int Celsius = 0; //Set default temperature scale
float thermA = tmTargetA;  //set temp of first probe to init goal
float thermB = tmTargetB;  //set temp of second probe to init goal
float irTemp = irTarget;  //set temp of ir probe to init goal
char inChar;

float vcc = 4.91;                       // only used for display purposes, if used
                                        // set to the measured Vcc.
float pad = 9850;                       // balance/pad resistor value, set this to
                                        // the measured resistance of your pad resistor
float thermr = 10000;                   // thermistor nominal resistance



void setup(){
  //  i2c_init(); //Initialise the i2c bus (only use if ir probe attached)
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
  //Serial.begin(9600); //Set up debugging
  
  //initialize bluetooth communication
  bluetooth.begin(115200);
  bluetooth.print("$$$");
  delay(100);
  bluetooth.println("U,9600,N");
  bluetooth.begin(9600);
  //Serial.print("BLUETOOTH ENABLED"); //Debug Code
  //initialize Timer1
  cli();                  // disable global interrupts
  TCCR1A = 0;             // set entire TCCR1A register to 0
  TCCR1B = 0;             // same for TCCR1B
  // set compare match register to desired timer count:
  OCR1A = 15624;
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  // enable global interrupts:
  sei();

  //Setup led
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  
  //cycle through startup colors
  setColor(255, 0, 0);  // red
  delay(1000);
  setColor(0, 0, 255);  // blue
  delay(1000);
  setColor(0, 255, 0);  // green
  delay(1000);
  setColor(255, 255, 0);  // yellow
  delay(1000);  
  setColor(255, 0, 0);  // orange
  delay(1000);  
  setColor(255, 0, 255);  // purple
  delay(1000);
 
  //Setup LCD display
  lcd.begin(9600);  // Start the LCD at 9600 baud
  clearDisplay();  // Clear the display
  
  // Flash the backlight:
  for (int i=0; i<3; i++)
  {
    setBacklight(0);
    delay(250);
    setBacklight(255);
    delay(250);
  }
  
}

//holds command being recieved
String command;
//holds parameters for command
String param;
//becomes true when accepting param values for an already entered  command
boolean receivedCommand = false;
//signals command has been executed and should not be repeated
boolean commandComplete = false;
//builds from char recieved
String received = "";

//main logical loop
void loop(){
  //hold values received
  char sent;
  
  //if bluetooth message recieved
  if(bluetooth.available())
  {
    //read incoming char
    sent = (char)bluetooth.read();
    
    //if sent is marker then
    if(sent == ';')
   {
     //if command has already been recieved
     if(receivedCommand)
     {
       //give param completed value
       param = received;
       //set command as completed and ready to be executed
       commandComplete=true;
       //prepare to recieve new command
       receivedCommand = false;
     }
     //if we have not yet recieved a new command
     else
     {
       //give command the completed value
       command = received;
       //prepare to reieve parameters for the command
       receivedCommand = true;
     }
     //empty the buffer
     received = "";
   }
   //if the read char is not the marker
   else
   {
     //add char to the buffer
     received = received + sent;
   }
 }
 
 //If command need to be executed
 if(commandComplete)
 {
   //if command is "sound" we play one of the alarms dependent on the parameter
   if( command == "sound" )
   {
     if(param == "1")
     {
       nextStep();
     }
     else if(param == "2")
     {
       nextStepWarning();
     }
   }
   commandComplete=false;
 }
 //if command is "alarmA" we set the target of probe A to the value taken as the parameter
 else if(command == "alarmA")
 {
   //need char array to hold param
   char carray[param.length() + 1];
   //fill aray with param
   param.toCharArray(carray, sizeof(carray));
   //convert array to int value and assign
   tmTargetA = atoi(carray);
 }
 //if command is "alarmB" we set the target of probe B to the value taken as the parameter
 else if(command == "alarmB")
 {
   //need char array to hold param
   char carray[param.length() + 1];
   //fill aray with param
   param.toCharArray(carray, sizeof(carray));
   //convert array to int value and assign
   tmTargetB = atoi(carray);
 }
 
  
//if temps are below alarm threashold send message of alarm
/*  if (irTemp<=(irTarget-irVariance))
{
    bluetooth.print("AC1;");
    bluetooth.print(irTemp);
        bluetooth.print("~");
    setColor(0, 0, 255);  // blue
    noChange=0;
    if (audibleAlarm) { alarm(200); }
  }
else */if   (thermA<=(tmTargetA-tmVarianceA))
{
    bluetooth.print("AC2;");
    bluetooth.print(thermA);
        bluetooth.print("~");
    setColor(0, 0, 255);  // blue
    noChange=0;
    if (audibleAlarm) { alarm(200); }
}
else if(thermB<=(tmTargetB-tmVarianceB))   
{
    bluetooth.print("AC3;");
    bluetooth.print(thermB);
        bluetooth.print("~");
    setColor(0, 0, 255);  // blue
    noChange=0;
    if (audibleAlarm) { alarm(200); }
}

//if temps are above alarm threashold send message of alarm
/*  if (irTemp>=(irTarget+irVariance))
{
    bluetooth.print("AH1;");
    bluetooth.print(irTemp);
        bluetooth.print("~");
    setColor(255, 0, 0);  // blue
    noChange=0;
    if (audibleAlarm) { alarm(200); }
  }
else */if   (thermA>=(tmTargetA+tmVarianceA))
{
    bluetooth.print("AH2;");
    bluetooth.print(thermA);
        bluetooth.print("~");
    setColor(255, 0, 0);  // blue
    noChange=0;
    if (audibleAlarm) { alarm(200); }
}
else if(thermB>=(tmTargetB+tmVarianceB))   
{
    bluetooth.print("AH3;");
    bluetooth.print(thermB);
        bluetooth.print("~");
    setColor(255, 0, 0);  // blue
    noChange=0;
    if (audibleAlarm) { alarm(200); }
}
}


//different sounds and lights for alarms
void nextStep(){
      setColor(255, 128, 0);  // yellow
      if (alertBeep) beep();
}

//different sounds and lights for alarms
void nextStepWarning(){
      setColor(255, 32, 0);  // orange
      if (alertBeep) {
        beep();
        delay(1000);  
        alarm(200);
      }
}  

//different sounds and lights for alarms
void alarm(unsigned char delayms){
  analogWrite(6, 19);      // Almost any value can be used except 0 and 255
  delay(delayms);          // wait for a delayms ms
  analogWrite(6, 0);       // 0 turns it off 
}  

//different sounds and lights for alarms
void beep(){
  analogWrite(6, 19);      // Almost any value can be used except 0 and 255
  delay(200);          // wait for a delayms ms
  analogWrite(6, 211);     // experiment to get the best tone
  delay(50);          // wait for a delayms ms
  analogWrite(6, 0);       // 0 turns it off
  delay(20);          // wait for a delayms ms 
  analogWrite(6, 211);     // experiment to get the best tone
  delay(50);          // wait for a delayms ms
  analogWrite(6, 0);       // 0 turns it off
  delay(20);          // wait for a delayms ms    
  analogWrite(6, 211);     // experiment to get the best tone
  delay(50);          // wait for a delayms ms
  analogWrite(6, 0);       // 0 turns it off 
}  

//function to set color
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

//set lcd backlight
void setBacklight(byte brightness)
{
  lcd.write(0x80);  // send the backlight command
  lcd.write(brightness);  // send the brightness value
}

//clear lcd
void clearDisplay()
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x01);  // send the clear screen command
}

//move lcd cursor
void setLCDCursor(byte cursor_position)
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x80);  // send the set cursor command
  lcd.write(cursor_position);  // send the cursor position
}

//get temp from probe
float Thermistor(int RawADC) {
  long Resistance;  
  float Temp;  // Dual-Purpose variable to save space.

  Resistance=((1024 * pad / RawADC) - pad); 
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.000339148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius         
  return Temp;                                      // Return the Temperature
}

ISR(TIMER1_COMPA_vect)
{
    int dev = 0x5A<<1;
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    //get temp from IR thermometer
//    i2c_start_wait(dev+I2C_WRITE);
  //  i2c_write(0x07);
    // read
    //i2c_rep_start(dev+I2C_READ);
    //data_low = i2c_readAck(); //Read 1 byte and then send ack
    //data_high = i2c_readAck(); //Read 1 byte and then send ack
    //pec = i2c_readNak();
    //i2c_stop();
    
    thermA=Thermistor(analogRead(thermistorPinA));       // read ADC and  convert it to Celsius
    thermB=Thermistor(analogRead(thermistorPinB));       // read ADC and  convert it to Celsius
    
    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    double tempData = 0x0000; // zero out the data
    int frac; // data past the decimal point
    
    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    //tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    //tempData = (tempData * tempFactor)-0.01;
    //irTemp = tempData - 273.15;
    
    if (!Celsius)  {
      thermA = (thermA * 9.0)/ 5.0 + 32.0;   // converts to  Fahrenheit
      thermB = (thermB * 9.0)/ 5.0 + 32.0;
      //irTemp = (irTemp* 1.8) + 32;
    }

    //if(no alarm has not been sounded for a amount of time clear light
    if (/*irTemp>(irTarget-irVariance) && */thermA>(tmTargetA-tmVarianceA)&& thermB>(tmTargetB-tmVarianceB) && /*irTemp<(irTarget+irVariance) && */thermA<(tmTargetA+tmVarianceA) && thermB<(tmTargetB+tmVarianceB)) {
      
      if (noChange==6) {
        setColor(0, 0, 0);  // LED off
        noChange++;
      }
  
      if (noChange<=5) {
        setColor(0, 255, 0);  // green
        noChange++;
      }
  
    }    

//Send temp info over bluetooth
/*    bluetooth.print("D1;");
    
    bluetooth.print(irTemp);
        bluetooth.print("~");
    //Serial.print("IR Temp: ");
    //Serial.println(irTemp);
*/
    bluetooth.print("D1;");
    bluetooth.print(thermA);
        bluetooth.print("~");
  //  Serial.print("Probe A: ");
    //Serial.println(thermA);
    
    
    bluetooth.print("D2;");
    bluetooth.print(thermB);
        bluetooth.print("~");
    //Serial.print("Probe B: ");
    //Serial.println(thermB);

 //display temp on lcd screen
  clearDisplay();  // Clear the display
  //setLCDCursor(1);  // Set cursor to the 3rd spot, 1st line
 // lcd.print((int)irTemp);
  //lcd.write(0xDF);
  //if (Celsius) {lcd.print("C");}
  //else {lcd.print("F");}
  
  setLCDCursor(17);
  lcd.print((int)thermA);
  lcd.write(0xDF);
  if (Celsius) {lcd.print("C");}
  else {lcd.print("F");}
  
  setLCDCursor(24);
  lcd.print((int)thermB);
  lcd.write(0xDF);
  if (Celsius) {lcd.print("C");}
  else {lcd.print("F");}
}
