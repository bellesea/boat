#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 3; // pins actually switched on the arduino
static const uint32_t GPSBaud = 9600;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 3000; 

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// long lat coordinates
float destinationLng = -71.2643118;
float destinationLat = 42.2931823;
float startingLng = 0;
float startingLat = 0;
float waypointLng;
float waypointLat;
float currentLng;
float currentLat;

// distance and heading
float distance;
float heading;
float startingHeading = 0;
int currentDirection = 1;  // 1 = left; 0 = right;
bool useX;

// intermediate variables
float currentX;
float currentY;
float prevX;
float prevY;
float distX;
float distY;
float waypointX;
float waypointY;
bool xIsCloser;
bool yIsCloser;

// motor control
float defaultspeed = 20;
float leftspeed;
float rightspeed;

// waypoint generation
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
  float num = 6371 * cos(radians(lat)) * radians(lng);
  return num;
}

float getY(float lat) {
  int num = 6371 * radians(lat);
  return num;
}

float destinationX = getX(destinationLat, destinationLng);
float destinationY = getY(destinationLat);

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
float getDistance(float lat1, float lon1, float lat2, float lon2) {
  // Convert latitude and longitude from degrees to radians
  lat1 = toRadians(lat1);
  lon1 = toRadians(lon1);
  lat2 = toRadians(lat2);
  lon2 = toRadians(lon2);

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
  return waypointX - currentX;
}
float getYDifference(float currentY) {
  return waypointY - currentY;
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

float gen_waypoint(int n, float startingHeading) {
  float waypoint_Xdiff = (destinationLng - startingLng) / waypoint_num;
  float waypoint_Xdist = n * waypoint_Xdiff;  // multiply generated waypoint number by x distance between each generated waypointed
  
  float waypoint_Ydiff = (destinationLat - startingLat) / waypoint_num;
  float waypoint_Ydist = n * waypoint_Ydiff;
  // float waypoint_Ydist = waypoint_Xdist * tan(startingHeading);
  waypointLng = startingLng + waypoint_Xdist;
  waypointLat = startingLat + waypoint_Ydist;

  return (waypointLng, waypointLat);
}

void loop() {
  // Serial.println("HII");
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      // Serial.print("Latitude= ");
      // Serial.print(gps.location.lat(), 6);
      currentLat = gps.location.lat();
      // Serial.print(" Longitude= ");
      // Serial.println(gps.location.lng(), 6);
      currentLng = gps.location.lng();

      if (startingLng == 0) {
        startingLng = currentLng;
      }

      if (startingLat == 0) {
        startingLat = currentLat;
      }
    }
  }
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    heading = getHeading(currentLat, destinationLat, currentLng, destinationLng);

    if (startingHeading == 0) {
      startingHeading = heading;
    }

    Serial.println(heading);
    // Serial.println("hii");

    if (n >= waypoint_num) {
      waypointLat = destinationLat;
      waypointLng = destinationLng;
    }
    else {
      waypointLng, waypointLat = gen_waypoint(n,heading);
    }

    currentX = getX(currentLat, currentLng);
    currentY = getY(currentLat);

    waypointX = getX(waypointLat, waypointLng);
    waypointY = getY(waypointLat);

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

    distance = getDistance(currentLat, currentLng, waypointLat, waypointLng);
    // Serial.println(distance);
    if(currentDirection == 1) {
      Serial.println("Keep Left");
    } else {
      Serial.println("Keep Right");
    }
    
    float finalDistance = getDistance(currentLat, currentLng, destinationLat, destinationLng);

    if (finalDistance < 5) {
      Serial.println("WE'RE DONE, DELAYING");
      delay(200000);
    }

    if (distance < (25)/waypoint_num) {
      if (n == waypoint_num) {
        Serial.println("WE'RE DONE, DELAYING");
        delay(200000);
      }
      else {
        n++;
        Serial.println("NEW WAYPOINT");
      }
    }
    Serial.println("");
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current state.
  }
}
