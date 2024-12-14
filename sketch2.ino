#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <PWMServo.h>

// controls
unsigned long timeToRun = 240000;
float rightdefaultspeed = 40;
float leftdefaultspeed = 35;  // creates slight right tendencies to keep it straight

// control ESC
PWMServo LeftESC;
PWMServo RightESC;
float defaultspeed = 0;
float rightspeed;
float leftspeed;

// difference is

static const int RXPin = 3, TXPin = 4;  // pins actually switched on the arduino
static const uint32_t GPSBaud = 9600;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 5000;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// long lat coordinates
float destinationLng = -71.31245;  // -71.31068;
float destinationLat = 42.29144;   // 42.28984;
float startingLng = 0;
float startingLat = 0;
float waypointLng;
float waypointLat;
float currentLng;
float currentLat;
float prevLng;
float prevLat;
float diff;
float currentHeading;
float oppositeGeneralHeading;
float rightHeadingBoundary;
float leftHeadingBoundary;

// distance and heading
float distance;
float generalHeading;
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
float diflon;
float a;
float b;
float tempHeading;

// waypoint generation
int waypoint_num = 1;
float waypoint_Xdiff;
float waypoint_Ydiff;
float waypoint_Xdist;
float waypoint_Ydist;

int n = 1;

int called = 0;

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  RightESC.attach(10, 1000, 2000);
  LeftESC.attach(9, 1000, 2000);
  RightESC.write(0);
  LeftESC.write(0);
  delay(60000); // load and untether
  rightspeed = rightdefaultspeed;
  leftspeed = leftdefaultspeed;
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

float getHeading(float flat1, float flat2, float flon1, float flon2, int update) {
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2) - (flon1));
  a = sin(diflon) * cos(flat2);
  b = cos(flat1) * sin(flat2) - sin(flat1) * cos(flat2) * cos(diflon);
  a = atan2(a, b);
  tempHeading = degrees(a);
  if (tempHeading < 0) { tempHeading = 360 + tempHeading; }
  if (update == 1) {
    if (tempHeading <= 45 || tempHeading >= 315 || tempHeading <= 225 && tempHeading >= 135) {
      useX = true;
      // Serial.println("using x");
    } else {
      useX = false;
      // Serial.println("using y");
    }
  }

  return tempHeading;
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
  float lat2 = toRadians(waypointLat);
  float lon2 = toRadians(waypointLng);

  // Calculate the distance using the provided formula
  float d = 2 * R * asin(sqrt(sq(sin((lat2 - lat1) / 2)) + cos(lat1) * cos(lat2) * sq(sin((lon2 - lon1) / 2))));

  return d;  // Return distance in meters
}

bool getCloserX(float currentX, float prevX) {
  bool x = getXDifference(currentX) <= getXDifference(prevX);
  // //Serial.println(getXDifference(currentX));
  // //Serial.println(getXDifference(prevX));
  // //Serial.println("x difference:" + x);
  // if (x) {
  //   // Serial.println("Closer X");
  // } else {
  //   //Serial.println("Not closer X");
  // }
  return x;
}

bool getCloserY(float currentY, float prevY) {
  bool y = getYDifference(currentY) <= getYDifference(prevY);
  // //Serial.println(getYDifference(currentY));
  // //Serial.println(getYDifference(prevY));
  // if (y) {
  //   // Serial.println("Closer Y");
  // } else {
  //   //Serial.println("Not closer Y");
  // }
  return y;
}

float getXDifference(float currentX) {
  return waypointX - currentX;
}
float getYDifference(float currentY) {
  return waypointY - currentY;
}

void goLeft() {
  currentDirection = 0;
  rightspeed = 0;
  // if (useX == true) {
  //   rightspeed = 0;
  // } else {
  //   rightspeed = 0;
  // }
  // rightspeed = rightdefaultspeed;
  leftspeed = leftdefaultspeed;
  Serial.println("go left");
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);
  delay(1000);
  // Serial.println("go right 2");

  rightspeed = rightdefaultspeed;
}

void goRight() {
  currentDirection = 1;
  leftspeed = 0;
  // if (useX == true) {
  //   leftspeed = 0;
  //   // 10*getXDifference(currentX);
  // } else {
  //   leftspeed = 0;
  //   // 10*getYDifference(currentY);
  // }
  rightspeed = rightdefaultspeed;
  Serial.println("go right");
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);
  delay(1000);
  // Serial.println("go left 2");

  leftspeed = leftdefaultspeed;
}

void turn180() {
  currentDirection = 0;
  if (useX == true) {
    leftspeed = 0;
  } else {
    leftspeed = 0;
  }
  rightspeed = rightdefaultspeed;
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);
  delay(4000);

  leftspeed = leftdefaultspeed;
}
void toggleDirection() {
  if (currentDirection == 1) {  // 1 = left
    goRight();
  } else {
    goLeft();
  }
}

float gen_waypoint(int n, float startingHeading) {
  waypoint_Xdiff = (destinationLng - startingLng) / waypoint_num;
  waypoint_Xdist = n * waypoint_Xdiff;  // multiply generated waypoint number by x distance between each generated waypointed

  waypoint_Ydiff = (destinationLat - startingLat) / waypoint_num;
  waypoint_Ydist = n * waypoint_Ydiff;
  // float waypoint_Ydist = waypoint_Xdist * tan(startingHeading);
  waypointLng = startingLng + waypoint_Xdist;
  waypointLat = startingLat + waypoint_Ydist;

  return (waypointLng, waypointLat);
}

float wrap(float heading) {
  if (heading >= 360) {
    heading -= 360;
  } else if (heading < 0) {
    heading += 360;
  }

  return heading;
}

void loop() {
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);

  // if (called == 0) {
  //   goRight();
  //   called = 1;
  // }

  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      Serial.print("Latitude= ");
      Serial.print(gps.location.lat(), 6);
      currentLat = gps.location.lat();
      Serial.print(" Longitude= ");
      Serial.println(gps.location.lng(), 6);
      currentLng = gps.location.lng();

      if (startingLng == 0) {
        startingLng = currentLng;
      }

      if (startingLat == 0) {
        startingLat = currentLat;
      }
    }
  }

  currentMillis = millis();                   //get the current "time" (actually the number of milliseconds since the program started)
  if ((currentMillis < timeToRun) && (currentMillis - startMillis >= period))  //test whether the period has elapsed
  {

    generalHeading = getHeading(currentLat, waypointLat, currentLng, waypointLng, 1);

    if (startingHeading == 0) {
      startingHeading = generalHeading;
    } else {
      diff = 360 - generalHeading;
      currentHeading = wrap(getHeading(prevLat, currentLat, prevLng, currentLng, 0) + diff);
      oppositeGeneralHeading = 180.0;
      rightHeadingBoundary = 30.0;
      leftHeadingBoundary = 330.0;

      // Serial.print(useX);
      Serial.print(currentLat, 6);
      Serial.println(currentLng, 6);

      if (currentHeading != diff) {
        if (currentHeading > rightHeadingBoundary && currentHeading < oppositeGeneralHeading) {
          Serial.print("WAY OFF");
          goLeft();
          delay(2000);
        } else if (currentHeading < leftHeadingBoundary && currentHeading > oppositeGeneralHeading) {
          Serial.print("WAY OFF");
          goRight();
          delay(2000);
        }
      }
    }

    if (n >= waypoint_num) {
      waypointLat = destinationLat;
      waypointLng = destinationLng;
    } else {
      waypointLng, waypointLat = gen_waypoint(n, generalHeading);
    }

    currentX = getX(currentLat, currentLng);
    currentY = getY(currentLat);

    waypointX = getX(waypointLat, waypointLng);
    waypointY = getY(waypointLat);

    // logic
    xIsCloser = getCloserX(currentX, prevX);
    yIsCloser = getCloserY(currentY, prevY);

    if (useX && !xIsCloser) {
      Serial.print("toggle");
      toggleDirection();
    } else if (!useX && !yIsCloser) {
      Serial.print("toggle");
      toggleDirection();
    }

    prevX = currentX;
    prevY = currentY;

    prevLat = currentLat;
    prevLng = currentLng;

    distance = getDistance(currentLat, currentLng);

    Serial.print("distance:");
    Serial.println(distance);
    Serial.println("----------------------");

    if (distance < (15)) {
      if (n == waypoint_num) {
        // Serial.println("WE'RE DONE, DELAYING");
        for (int i = 0; i < 10; i++) {
          turn180();
        }
        destinationLat = startingLat;
        destinationLng = startingLng;
        n = 0;
      } else {
        n++;
        Serial.println("NEW WAYPOINT");
      }
    }
    startMillis = currentMillis;
  }

  // stop after 3 min
  if (currentMillis > timeToRun) {
    leftspeed = 0;
    rightspeed = 0;
  }
}
