// Date and time functions using just software, based on millis() & timer

#include <Arduino.h>
#include <Wire.h>         // this #include still required because the RTClib depends on it
#include "RTClib.h"
#include <Servo.h> 

#if defined(ARDUINO_ARCH_SAMD)  // for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
  #define Serial SerialUSB
#endif

RTC_Millis rtc;
Servo myservo, myservo1;
//////////////////////////////////////////////////  
//PUT YOUR LATITUDE, LONGITUDE, AND TIME ZONE HERE
  float latitude = 6.796;
  float longitude = -79.88;
  float timezone = -5.5;
//////////////////////////////////////////////////  
  
//If you live in the southern hemisphere, it would probably be easier
//for you if you make north as the direction where the azimuth equals
//0 degrees. To do so, switch the 0 below with 180.  
  float northOrSouth = 0;

/////////////////////////////////////////////////////////// 
//MISC. VARIABLES
///////////////////////////////////////////////////////////  
  float pi = 3.14159265;
  float altitude;
  float azimuth;
  float delta;
  float h;
  int ServoAzimuthAngle;
  int ServoZenithAngle;
  int hour2 = 8;
/////////////////////////////////////////////////////////// 
//END MISC. VARIABLES
///////////////////////////////////////////////////////////
  
void setup(){
  #ifdef ESP8266
  Wire.pins(2, 14);   // ESP8266 can use any two pins, such as SDA to #2 and SCL to #14
#endif
    
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  Serial.begin(9600);
  latitude = latitude * pi/180;
  
  myservo.attach(8);
  myservo1.attach(9);
}

void loop(){
DateTime now = rtc.now();
  float month2;
  float day;
  
  float minute2;

  //SET TIME AND DATE HERE//////////////
  month2 = 3;
  day =18;
  //Use 24hr clock (ex: 1:00pm = 13:00) and don't use daylight saving time.
  minute2 = 0;
  //END SET TIME AND DATE /////////////


  //START OF THE CODE THAT CALCULATES THE POSITION OF THE SUN
  float n = daynum(month2) + day;//NUMBER OF DAYS SINCE THE START OF THE YEAR. 
  delta = .409279 * sin(2 * pi * ((284 + n)/365.25));//SUN'S DECLINATION.
  day = dayToArrayNum(day);//TAKES THE CURRENT DAY OF THE MONTH AND CHANGES IT TO A LOOK UP VALUE ON THE HOUR ANGLE TABLE.
  h = (FindH(day,month2)) + longitude + (timezone * -1 * 15);//FINDS THE NOON HOUR ANGLE ON THE TABLE AND MODIFIES IT FOR THE USER'S OWN LOCATION AND TIME ZONE.
  h = ((((hour2 + minute2/60) - 12) * 15) + h)*pi/180;//FURTHER MODIFIES THE NOON HOUR ANGLE OF THE CURRENT DAY AND TURNS IT INTO THE HOUR ANGLE FOR THE CURRENT HOUR AND MINUTE.
  altitude = 90-(asin(sin(latitude) * sin(delta) + cos(latitude) * cos(delta) * cos(h)))*180/pi;//FINDS THE SUN'S ALTITUDE.
  azimuth = ((atan2((sin(h)),((cos(h) * sin(latitude)) - tan(delta) * cos(latitude)))) + (northOrSouth*pi/180)) *180/pi+180;//FINDS THE SUN'S AZIMUTH.
  //END OF THE CODE THAT CALCULATES THE POSITION OF THE SUN
hour2=hour2+1;;
  //Serial.println("Altitude");
 // Serial.println(altitude);  
 ServoAzimuthAngle =270-azimuth;
// ServoAngle=180- ServoAngle;
 //int alti =altitude;
 ServoZenithAngle=90-altitude;
  Serial.println("Azimuth");
 Serial.println(azimuth);
 //Serial.println(day);
  Serial.println();
myservo.write(ServoAzimuthAngle+25);
  Serial.println("altitude");
  Serial.println(altitude);
myservo1.write(ServoZenithAngle);
  delay(5000);
  
}//End Void Loop
