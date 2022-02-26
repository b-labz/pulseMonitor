//Library for SSD1306
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   

#define BUZZER_PORT 2
#define BIG  0
#define TINY 1

//  Variables
const int PulseWire = 0;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED13 = 10;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 516;           // Determine which Signal to "count as a beat" and which to ignore.
                               // Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
                               // Otherwise leave the default "550" value. 

PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"
void drawHeart(unsigned char hType, unsigned int rate);


#define ledPin 13
int reload;

long oldTime = 0;
long newTime = 0;
char strBuf[5];
int myBPM = 0;
volatile unsigned char flag = true;

volatile unsigned char timeCntr = 0, dispRef = false, timeOut = false, tOutWriteOnce = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);          // For Serial Monitor
  u8g2.begin();
  u8g2.setContrast(10);
  u8g2.clearBuffer();                      // clear the internal memory
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.drawStr(0,25,"B"); // write text to the internal memory
  u8g2.setFont(u8g2_font_crox2h_tf );      // choose a suitable font
  u8g2.drawStr(10,25,"-LABZ"); // write text to the internal memory
  u8g2.setFont(u8g2_font_open_iconic_human_2x_t);
  u8g2.drawGlyph(60, 25, 66);  
  u8g2.setFont(u8g2_font_crox1hb_tf );      // choose a suitable font
  u8g2.drawStr(80,0,"Pulse"); // write text to the internal memory
  u8g2.setFont(u8g2_font_crox2h_tf );      // choose a suitable font
  u8g2.drawStr(90,25,"MON"); // write text to the internal memory
  u8g2.sendBuffer();                       // transfer internal memory to the display
  delay(1500);

  drawHomeScr();

  pinMode(BUZZER_PORT, OUTPUT);
  digitalWrite(BUZZER_PORT, HIGH);   // turn the LED on (HIGH is the voltage level)
  
  pulseSensor.begin();
  // Configure the PulseSensor object, by assigning our variables to it. 
  pulseSensor.analogInput(PulseWire);   
  pulseSensor.blinkOnPulse(LED13);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);   

  pinMode(ledPin, OUTPUT);

}


int Signal;
void loop() {
  // put your main code here, to run repeatedly: 
  myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".// "myBPM" hold this BPM value now. 

  Signal = analogRead(PulseWire);  // Read the PulseSensor's value. // Assign this value to the "Signal" variable.
  Serial.println(Signal);                    // Send the Signal value to Serial Plotter.

  if (pulseSensor.sawStartOfBeat()) // Constantly test to see if "a beat happened". 
  {
    digitalWrite(BUZZER_PORT, LOW);   // turn the LED on (HIGH is the voltage level)            
    displayBPM();
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
    digitalWrite(BUZZER_PORT, HIGH);   // turn the LED on (HIGH is the voltage level)
    oldTime = millis();
    timeOut = false;
    if(tOutWriteOnce == false)
    {
      drawHomeScr();
      tOutWriteOnce = true;
    }

  }
  else
  {
    newTime = millis();
    if((newTime - oldTime) > 5000)//TIme out no pulses detected
    {
       timeOut = true;
    }
  }
  delay(10);


  if(timeOut == true && tOutWriteOnce == true)
  {
    tOutWriteOnce = false;
    u8g2.clearBuffer();                      // clear the internal memory
    u8g2.setFont(u8g2_font_crox1h_tf );      // choose a suitable font
    u8g2.drawStr(0,25,"NO SIGNAL"); // write text to the internal memory
    u8g2.sendBuffer(); 
  }
}


void displayBPM()
{
  strBuf[0]=strBuf[1]=strBuf[2]=strBuf[3]=strBuf[4]=strBuf[5] = '0';  
  u8g2.clearBuffer();                      // clear the internal memory
  u8g2.sendBuffer();
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.setCursor(50, 25);
  myBPM = 82;
  sprintf(strBuf, "%3d", myBPM);
  u8g2.print(strBuf);

  u8g2.setFont(u8g2_font_open_iconic_human_2x_t);
  u8g2.drawGlyph(0, 25, 66);  
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.setCursor(90, 25);
  u8g2.print("BPM");
  u8g2.sendBuffer();
}

void drawHomeScr()
{
  u8g2.clearDisplay();                                //Clear the display
  u8g2.setFont(u8g2_font_open_iconic_human_2x_t);
  u8g2.drawGlyph(0, 25, 66);  
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.setCursor(90, 25);
  u8g2.print("BPM");
  u8g2.sendBuffer();
}
