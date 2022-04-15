#include <DFRobot_GDL.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//display
#define TFT_DC  8
#define TFT_CS  10
#define TFT_RST 9

#define arrSize(X) sizeof(X) / sizeof(X[0])

int last = 0;

float LAT;
float LON;

float topSpeed;
float Speed;
float Course;
float Alt;
float aveSpeed;
float aveSpeeds;
int Sat;

//GPS
static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;

int bearings[] =               { 0,    22.5,  45,   67.5,  90, 112.5,  135,  157.5, 180,  202.5, 225,  247.5, 270, 292.5,  315, 337.5, 360 };
const char* bearingStrings[] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE",  "SSE", "S",  "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW", "N" };

DFRobot_ILI9488_320x480_HW_SPI screen(TFT_DC,TFT_CS,TFT_RST);
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

#define screenColor COLOR_RGB565_NAVY
#define backColor COLOR_RGB565_DGRAY

double HOME_LAT;
double HOME_LON;

void setup() {  
  Serial.begin(115200);
  
  ss.begin(GPSBaud);
  
  screen.begin();
  screen.setRotation(1);
  screen.fillScreen(COLOR_RGB565_WHITE);
  screen.setTextColor(COLOR_RGB565_RED);
  
  retry:
  while (ss.available() > 0){
    gps.encode(ss.read());
  }

  while (!gps.location.isUpdated()){
    screen.setCursor(190,150);
    screen.print("No GPS Data");
    goto retry;
  }
  
  HOME_LAT = gps.location.lat();
  HOME_LON = gps.location.lng();

  screen.fillRoundRect(0, 0, 480, 320, 15, backColor);
  screen.fillRoundRect(7, 7, 466, 306, 15, screenColor);

  //title
  screen.setTextColor(COLOR_RGB565_CYAN);
  screen.setTextSize(3);
  screen.setCursor(70, 13);
  screen.print("GPS Speedometer");

  screen.setTextSize(1);

  //title underline
  screen.drawLine(20, 50, 460, 50, 0x0700);

  //speed
  screen.drawRoundRect(30, 60, 205, 65, 15, 0x0700);
  screen.setCursor(200,64);
  screen.print("MPH");

  //course
  screen.drawRoundRect(245, 60, 205, 65, 15, 0x0700);
  screen.setCursor(415,64);
  screen.print("DEG"); 
  
  //altitude
  screen.drawRoundRect(30, 135, 205, 65, 15, 0x0700);
  screen.setCursor(200,140);
  screen.print("FT");
  
  //direction
  screen.drawRoundRect(245, 135, 205, 65, 15, 0x0700);
  screen.setCursor(415,140);
  screen.print("ROS");
  
  //top speed
  screen.drawRoundRect(30, 210, 205, 45, 15, 0x0700);
  
  //direction (left or right)
  screen.drawRoundRect(245, 210, 205, 45, 15, 0x0700);
  
  //course to home
  screen.drawRoundRect(30, 265, 205, 45, 15, 0x0700);
  
  //dist to home
  screen.drawRoundRect(245, 265, 205, 45, 15, 0x0700);

  last = millis();
}

void loop(){
  screen.setTextColor(COLOR_RGB565_GREEN);
  screen.setTextSize(3);

  while (ss.available() > 0){
    gps.encode(ss.read());
  }

  if (gps.location.isUpdated()){
    LAT = gps.location.lat();
    LON = gps.location.lng();
  }

  if (gps.speed.isUpdated()){
    Speed = gps.speed.mph();
    if (gps.speed.mph() < 1){
      Speed = 0;
    }
  }

  if (gps.course.isUpdated()){
    Course = gps.course.deg();
  }

  if (gps.altitude.isUpdated()){
    Alt = gps.altitude.feet();
  }
  
  if (gps.satellites.isUpdated()){
    Sat = gps.satellites.value();
  }

  if (millis() - last > 1000){
    displayData();
    last = millis();
  }
}

int nearestBearing(int x, bool sorted = true) {
  int idx = 0; // by default near first element
  int distance = abs(bearings[idx] - x);
  for (int i = 1; i < arrSize(bearings); i++) {
    int d = abs(bearings[i] - x);
    if (d < distance) {
      idx = i;
      distance = d;
    }
    else if (sorted) return idx;
  }
  return idx;
}

void displayData(){
  screen.setTextColor(COLOR_RGB565_GREEN);
  screen.setTextSize(3);
  
  //speed
  screen.fillRect(110,78,120,25,screenColor);
  screen.setCursor(45, 80);
  screen.print("SPD: ");
  screen.println(Speed);

  //course
  screen.fillRect(325,78,120,25,screenColor);
  screen.setCursor(260, 80);
  if (Course >= 100){
    screen.print("CSE:");
  }
  else {
    screen.print("CSE: ");
  }
  screen.println(Course);

  //altitude
  screen.setCursor(45, 155);
  if (Alt >= 100 && Alt < 1000){
    screen.fillRect(110,153,120,25,screenColor);
    screen.print("ALT:");
  }
  else if (Alt >= 1000){
    screen.fillRect(70,153,160,25,screenColor);
    screen.print("A: ");
  }
  else {
    screen.fillRect(110,153,120,25,screenColor);
    screen.print("ALT: ");
  }
  screen.println(Alt);

  //direction
  screen.fillRect(330,153,80,25,screenColor);
  screen.setCursor(260, 155);
  screen.print("DIR: ");
  screen.println(bearingStrings[nearestBearing(Course)]);
  
  //top speed
  if (Speed > topSpeed){
    topSpeed = Speed;
    screen.fillRect(92,223,135,25,screenColor);
    screen.setCursor(45, 225);
    screen.print("TS: ");
    screen.print(topSpeed);
  }

  double distanceToHome =
        TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          HOME_LAT, 
          HOME_LON) / 1000;
  double courseToHome =
        TinyGPSPlus::courseTo(
          gps.location.lat(),
          gps.location.lng(),
          HOME_LAT, 
          HOME_LON);
  double distanceToHomeFT = distanceToHome*3281;
  
  Serial.println(distanceToHomeFT, 6);
  Serial.println(courseToHome, 6);
  
  screen.fillRect(307,223,135,25,screenColor);
  screen.setCursor(260, 225);
  screen.print("D: ");
  if (turnRight(Course,courseToHome) == true){
    Serial.println("Right");
    screen.print("Right");
  }
  else if (turnRight(Course,courseToHome) == false){
    Serial.println("Left");
    screen.print("Left");
  }
   
  screen.setTextSize(2);
    
  screen.fillRect(90,278,140,18,screenColor);//screenColor
  screen.setCursor(45, 280);
  screen.print("Dist:");
  screen.println(distanceToHomeFT, 2);//in feet
    
  screen.fillRect(305,278,140,18,screenColor);
  screen.setCursor(260, 280);
  screen.print("CSE:");
  screen.println(courseToHome, 1);

  screen.setTextSize(3);

  //satellites
  screen.setTextColor(COLOR_RGB565_WHITE);
  screen.setTextSize(2);
  screen.fillRect(435,10,28,18,screenColor);
  screen.setCursor(390, 12);
  screen.print("SAT:");
  screen.println(Sat);
}

bool turnRight(long CourseNow,long CourseNew){
  bool retval = true;  
  if(CourseNew > CourseNow){
    if((CourseNew-CourseNow) > 180)
      retval = false;
    }
  else {
    if((CourseNow-CourseNew) < 180){
      retval = false;
    }
  }
  return retval;    
}
