#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 3; // pins actually switched on the arduino
static const uint32_t GPSBaud = 9600;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000; 

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

float destinationLng = -71.26430;
float destinationLat = 42.29319;
float distance;
float currentLng;
float currentLat;
float currentX;
float currentY;
float prevX;
float prevY;
float distX;
float distY;
float headingRad;
float heading;
int currentDirection = 1;  // 1 = left; 0 = right;
bool useX;
bool xIsCloser;
bool yIsCloser;
float leftspeed;
float rightspeed;
float defaultspeed = 20;
int waypoint_num = 5;
float waypoint_Xdiff;
float waypoint_Xdist;
float waypoint_Ydist;
int n = 1;

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Serial.setTimeout(1);
  startMillis = millis();
}

float getX(float lat, float lng) {
  float num = 6371 * cos(lat) * lng;
  return fmod(num, pow(10, 9));
}

float getY(float lat) {
  int num = 6371 * lat;
  return fmod(num, pow(10, 9));
}

float destinationX = getX(destinationLat, destinationLng);
float destinationY = getY(destinationLat);
// float getHeading() {
//   distX = destinationX - currentX;
//   distY = destinationY - currentY;
//   headingRad = atan2(distX, distY);
//   heading = degrees(headingRad);
//   return heading;
// }

float getHeading(float flat1, float flat2, float flon1, float flon2) {
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  float diflon = radians((flon2) - (flon1));
  float a = sin(diflon) * cos(flat2);
  float b = cos(flat1) * sin(flat2) - sin(flat1) * cos(flat2) * cos(diflon);
  a = atan2(a, b);
  heading = degrees(a);
  if (heading < 0) { heading = 360 + heading; }
  if (heading <= 45 || heading >= 315 || heading <= 225 && heading >= 135) {
    useX = true;
    Serial.println("using x");
  }
  else {
    useX = false;
    Serial.println("using y");
  }
  return heading;
}


// Constants
float R = 6371000;  // Earth's radius in meters (can be adjusted if needed)

// Convert degrees to radians
float toRadians(float degrees) {
  return degrees * (PI / 180);
}

// Function to calculate the great-circle distance
float getDistance(float lat1, float lon1) {
  // Convert latitude and longitude from degrees to radians
  lat1 = toRadians(lat1);
  lon1 = toRadians(lon1);
  float lat2 = toRadians(destinationLat);
  float lon2 = toRadians(destinationLng);

  // Calculate the distance using the provided formula
  float d = 2 * R * asin(sqrt(sq(sin((lat2 - lat1) / 2)) + cos(lat1) * cos(lat2) * sq(sin((lon2 - lon1) / 2))));

  return d;  // Return distance in meters
}

bool getCloserX(float currentX, float prevX) {
  bool x = getXDifference(currentX) <= getXDifference(prevX);
  //Serial.println(getXDifference(currentX));
  //Serial.println(getXDifference(prevX));
  //Serial.println("x difference:" + x);
  if(x) {
    Serial.println("Closer X");
  } else {
    //Serial.println("Not closer X");
  }
  return x;
}

bool getCloserY(float currentY, float prevY) {
  bool y = getYDifference(currentY) <= getYDifference(prevY);
  //Serial.println(getYDifference(currentY));
  //Serial.println(getYDifference(prevY));
  if(y) {
    Serial.println("Closer Y");
  } else {
    //Serial.println("Not closer Y");
  }
  return y;
}

float getXDifference(float currentX) {
  return waypoint_Xdist - currentX;
}
float getYDifference(float currentY) {
  return waypoint_Ydist - currentY;
}

void goLeft() {
  currentDirection = 1;
  if (useX == true) {
    leftspeed = 10*getXDifference(currentX);
  }
  else {
    leftspeed = 10*getYDifference(currentY);
  }
  rightspeed = defaultspeed;
  Serial.println("go left");
}

void goRight() {
  currentDirection = 0;
  if (useX == true) {
    rightspeed = 10*getXDifference(currentX);
  }
  else {
    rightspeed = 10*getYDifference(currentY);
  }
  leftspeed = defaultspeed;
  Serial.println("go right");
}

void toggleDirection() {
  if (currentDirection == 1) {  // 1 = left
    goRight();
  } else {
    goLeft();
  }
}

float gen_waypoint(int n, float heading) {
    waypoint_Xdiff = getXDifference(currentX)/waypoint_num;
    waypoint_Xdist = n * waypoint_Xdiff; // multiply generated waypoint number by x distance between each generated waypointed
    waypoint_Ydist = waypoint_Xdist * tan(heading);
    return (waypoint_Xdist, waypoint_Ydist);
  }

void loop() {
  // Serial.println("HII");
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude= ");
      Serial.print(gps.location.lat(), 6);
      currentLat = gps.location.lat();
      Serial.print(" Longitude= ");
      Serial.println(gps.location.lng(), 6);
      currentLng = gps.location.lng();
    }
  }
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    heading = getHeading(currentLat, destinationLat, currentLng, destinationLng);
    Serial.println(heading);
    // Serial.println("hii");
    waypoint_Xdist, waypoint_Ydist = gen_waypoint(n,heading);
    currentX = getX(currentLat, currentLng);
    currentY = getY(currentLat);

    // logic
    xIsCloser = getCloserX(currentX, prevX);
    yIsCloser = getCloserY(currentY, prevY);

    if (useX && !xIsCloser) {
      toggleDirection();
    } else if (!useX && !yIsCloser) {
      toggleDirection();
    }

    prevX = currentX;
    prevY = currentY;

    distance = getDistance(currentLat, currentLng)/waypoint_num;
    // Serial.println(distance);
    if(currentDirection == 1) {
      Serial.println("Keep Left");
    } else {
      Serial.println("Keep Right");
    }

    if (distance < (10/waypoint_num)) {
      n++;
      if (n == waypoint_num) {
        Serial.println("WE'RE DONE, DELAYING");
        delay(200000);
      }
      else {
        Serial.println("NEW WAYPOINT");
      }
    }
    Serial.println("");
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current state.
  }
}
