#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <PWMServo.h>

// control ESCs
PWMServo LeftESC;
PWMServo RightESC;
float rightdefaultspeed = 48;
float leftdefaultspeed = 45;              // old: difference should be around 8-12; left > right;
float rightspeed;
float leftspeed;

// connect to GPS module
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;
static const int RXPin = 4, TXPin = 3;    // pins actually switched on the arduino
SoftwareSerial ss(RXPin, TXPin);          // The serial connection to the GPS device

// create loop for adjustment
unsigned long startMillis; 
unsigned long currentMillis;
const unsigned long period = 4000;

// long lat coordinates
float destinationLng = -71.31139;  // -71.31068;
float destinationLat = 42.29204;   // 42.28984;
float startingLng = 0;
float startingLat = 0;
float waypointLng;                 // current temporary/waypoint destination
float waypointLat;
float currentLng;                  // current position
float currentLat;
float prevLng;                     // position from {period}ms ago
float prevLat;

// distance and heading
float distance;
float startingHeading = 0;         // keep track of initial heading to calculate waypoints
float generalHeading;              // to calculate heading from current location to waypoint
float oppositeGeneralHeading;      // to calculate opposite of the intended heading
float diff;                        // to calculate difference between general heading and 360
float currentHeading;              // to calculate current heading of boat (based on current and previous coordinates)
float rightHeadingBoundary;        // for heading adjustment
float leftHeadingBoundary;         // for heading adjustment
int currentDirection = 1;          // 1 = left; 0 = right;
bool useX;                         // figure out whether to use long or lat coordinates

// intermediate variables for calulations
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
int waypoint_num = 2;
float waypoint_Xdiff;
float waypoint_Ydiff;
float waypoint_Xdist;
float waypoint_Ydist;
int n = 1; //starting waypoint = 1

void setup() {
  Serial.begin(9600);                 // begin serial communication
  RightESC.attach(10, 1000, 2000);    // control ESCs
  LeftESC.attach(9, 1000, 2000);      // control ESCs
  RightESC.write(0);                  // stop motors
  LeftESC.write(0);                   // stop motors
  delay(120000);                      // 2 minutes delay to allow time to place boat in water
  rightspeed = rightdefaultspeed;     // set motor speeds
  leftspeed = leftdefaultspeed;
  ss.begin(GPSBaud);                  // begin GPS transmission
  startMillis = millis();             // begin timer for loop
}

// get x coordinate based on latitude and longtitude
float getX(float lat, float lng) {
  float num = 6371 * cos(radians(lat)) * radians(lng);   
  return num;
}

// get y coordinate based on latitude
float getY(float lat) {
  int num = 6371 * radians(lat);
  return num;
}

// get heading given 2 coordinates. update param is for whether to use this to determine which coordinate to use
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
    } else {
      useX = false;
    }
  }

  return tempHeading;
}


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

// helper function to do modulo for heading
float wrap(float heading) {
  if (heading >= 360) {
    heading -= 360;
  } else if (heading < 0) {
    heading += 360;
  }

  return heading;
}

// get the difference between the target x position and the current x
float getXDifference(float currentX) {
  return waypointX - currentX;
}

// get the difference between the target y position and the current y
float getYDifference(float currentY) {
  return waypointY - currentY;
}

// check if we're getting closer on the x axis
bool getCloserX(float currentX, float prevX) {
  bool x = getXDifference(currentX) <= getXDifference(prevX);
  return x;
}

// check if we're getting closer on the y axis
bool getCloserY(float currentY, float prevY) {
  bool y = getYDifference(currentY) <= getYDifference(prevY);
  return y;
}

// go right for a while then reset back to going straight
void goRight() {
  currentDirection = 0;
  leftspeed = 0;
  rightspeed = rightdefaultspeed;
  Serial.println("go right");
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);
  delay(2000);
  leftspeed = leftdefaultspeed;
}

// go left for a while then reset back to going straight
void goLeft() {
  currentDirection = 1;
  leftspeed = 70;
  rightspeed = rightdefaultspeed;
  Serial.println("go left");
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);
  delay(1300);
  leftspeed = leftdefaultspeed;
}

// turn 180 to begin going back to the origin location
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

// update which direction the boat is going in
void toggleDirection() {
  if (currentDirection == 1) {  // 1 = left; 0 = right
    goRight();
  } else {
    goLeft();
  }
}

// generate waypoints evenly distributed in a line between the target and the starting positions
float gen_waypoint(int n, float startingHeading) {
  waypoint_Xdiff = (destinationLng - startingLng) / waypoint_num;
  waypoint_Xdist = n * waypoint_Xdiff;          // multiply generated waypoint number by x distance between each generated waypointed

  waypoint_Ydiff = (destinationLat - startingLat) / waypoint_num;
  waypoint_Ydist = n * waypoint_Ydiff;
  // float waypoint_Ydist = waypoint_Xdist * tan(startingHeading);
  waypointLng = startingLng + waypoint_Xdist;
  waypointLat = startingLat + waypoint_Ydist;

  return (waypointLng, waypointLat);
}


void loop() {
  // set the motor speeds
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);

  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      currentLat = gps.location.lat();
      currentLng = gps.location.lng();

      if (startingLng == 0 || startingLat == 0) {     //update starting longtitude and latitude so boat can return to it
        startingLng = currentLng;
        startingLat = currentLat;
      }
    }
  }

  currentMillis = millis();                           //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)          //test whether the period has elapsed
  {

    generalHeading = getHeading(currentLat, waypointLat, currentLng, waypointLng, 1);       // get target heading from current location to waypoint

    if (startingHeading == 0) {                                                             // update starting heading if first run
      startingHeading = generalHeading;
    } else {
      diff = 360 - generalHeading;
      currentHeading = wrap(getHeading(prevLat, currentLat, prevLng, currentLng, 0) + diff);
      oppositeGeneralHeading = 180.0;
      rightHeadingBoundary = 45.0;
      leftHeadingBoundary = 315.0;

      Serial.println(useX);
      Serial.println(currentLat, 6);
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

    if (distance < 10) {
      if (n == waypoint_num) {
        Serial.println("WE'RE DONE");

        LeftESC.write(0);
        RightESC.write(0);

        delay(30000);                             // stop for 30 seconds

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
}
