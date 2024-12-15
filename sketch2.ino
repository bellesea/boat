#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <PWMServo.h>

// controls
unsigned long timeToRun = 400000;
float rightdefaultspeed = 40;
float leftdefaultspeed = 34;  // creates slight right tendencies to keep it straight

// control ESC
PWMServo LeftESC;
PWMServo RightESC;
float defaultspeed = 0;
float rightspeed;
float leftspeed;

// connect to GPS module
TinyGPSPlus gps;
static const int RXPin = 3, TXPin = 4;  // pins actually switched on the arduino
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);

// create loop for adjustment
unsigned long startMillis;  
unsigned long currentMillis;
const unsigned long period = 5000;

// long lat coordinates
float destinationLng = -71.31244;  // -71.31068;
float destinationLat = 42.29154;   // 42.28984;
float startingLng = 0;
float startingLat = 0;
float waypointLng;  // current temporary/waypoint destination
float waypointLat;
float currentLng;  // current position
float currentLat;
float prevLng;  // position from {period}ms ago
float prevLat;
float diff;
float currentHeading;
float oppositeGeneralHeading;
float rightHeadingBoundary;
float leftHeadingBoundary;

// distance and heading
float distance;
float generalHeading;         // to calculate heading from current location to waypoint
float startingHeading = 0;    // keep track of initial heading to calculate waypoints
int currentDirection = 1;     // 1 = left; 0 = right;
bool useX;                    // figure out whether to use x or y coordinates

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

int n = 1; //starting waypoint = 1

int called = 0;

void setup() {
  Serial.begin(9600);               // begin serial communication
  ss.begin(GPSBaud);                // begin GPS transmission
  RightESC.attach(10, 1000, 2000);  // control ESCs
  LeftESC.attach(9, 1000, 2000);    // control ESCs
  RightESC.write(0);                // stop motors
  LeftESC.write(0);                 // stop motors
  delay(40000);                     // 40 seconds to load and untether
  rightspeed = rightdefaultspeed;   // set motor speeds
  leftspeed = leftdefaultspeed;
  startMillis = millis();           // begin timer for loop
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

float destinationX = getX(destinationLat, destinationLng);
float destinationY = getY(destinationLat);

// get heading given 2 coordinates. update param is for whether to use this to determine which coordinate to use
float getHeading(float flat1, float flat2, float flon1, float flon2, int update) {
  // Convert latitude and longitude from degrees to radians
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2) - (flon1));
  a = sin(diflon) * cos(flat2);
  b = cos(flat1) * sin(flat2) - sin(flat1) * cos(flat2) * cos(diflon);
  a = atan2(a, b);

  tempHeading = degrees(a); // Convert heading back to degrees from radians

  if (tempHeading < 0) { tempHeading = 360 + tempHeading; } // If heading is negative, wrap around to keep 0 < heading < 360
  if (update == 1) {                                        // if this is the first pass through the code, determine which axis to use in logic
      
    // use x values to check if boat is getting closer if heading is in top or bottom quadrant. Else, check y values
    if (tempHeading <= 45 || tempHeading >= 315 || tempHeading <= 225 && tempHeading >= 135) {
      useX = true;
      // Serial.println("using x");
    } else {
      useX = false;
      // Serial.println("using y");
    }
  }

  return tempHeading; // return heading
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

// check if we're getting closer on the x axis based on current and prev coords
bool getCloserX(float currentX, float prevX) {
  bool x = getXDifference(currentX) <= getXDifference(prevX);
  return x;
}

// check if we're getting closer on the y axis based on current and prev coords
bool getCloserY(float currentY, float prevY) {
  bool y = getYDifference(currentY) <= getYDifference(prevY);
  return y;
}

// get the difference between the target x position and the current x
float getXDifference(float currentX) {
  return waypointX - currentX;
}

// get the difference between the target y position and the current y
float getYDifference(float currentY) {
  return waypointY - currentY;
}

void goLeft() {
  currentDirection = 1;               // set current direction
  rightspeed = 0;                     // set right motor to 0 to turn left
  leftspeed = leftdefaultspeed;
  Serial.println("go left");
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);
  delay(700);
  rightspeed = rightdefaultspeed;     // reset motor speeds
}

// go right for a while then reset back to going straight
void goRight() {
  currentDirection = 0;               // set current direction
  leftspeed = 0;
  rightspeed = rightdefaultspeed;     // set left motor to 0 to turn right
  Serial.println("go right");
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);
  delay(500);
  leftspeed = leftdefaultspeed;       // reset motor speeds
}

// turn in a circle for 20 seconds when reached final destination
void done() {
  leftspeed = 0;
  rightspeed = rightdefaultspeed;
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);
  delay(20000);

  leftspeed = leftdefaultspeed;
}

// update which direction the boat is going in
void toggleDirection() {
  if (currentDirection == 1) {  // 1 = left
    goRight();
  } else {
    goLeft();
  }
}

// generate waypoints evenly distributed in a line between the target and the starting positions
float gen_waypoint(int n, float startingHeading) {
  waypoint_Xdiff = (destinationLng - startingLng) / waypoint_num;  // find x diff between each generated waypoint from start to end
  waypoint_Xdist = n * waypoint_Xdiff;                             // find x dist up until the current generated waypoint number

  waypoint_Ydiff = (destinationLat - startingLat) / waypoint_num;  // find y diff between each generated waypoint from start to end
  waypoint_Ydist = n * waypoint_Ydiff;                             // find y dist up until the current generated waypoint number
  // float waypoint_Ydist = waypoint_Xdist * tan(startingHeading);
  waypointLng = startingLng + waypoint_Xdist;  // determine longitude with x dist of current waypoing and starting lng
  waypointLat = startingLat + waypoint_Ydist;  // determine latitude with y dist of current waypoing and starting lat

  return (waypointLng, waypointLat);
}

// helper function to do modulo for heading
float wrap(float heading) {
  if (heading >= 360) { // if heading is greater than 360, subtract 360 to keep in range [0,360)
    heading -= 360;
  } else if (heading < 0) {  // if heading is less than 9, add 360 to keep in range [0,360)
    heading += 360;
  }

  return heading;
}

void loop() {
  // set the motor speeds
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);

  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      // Serial.print("Latitude= ");
      // Serial.print(gps.location.lat(), 6);
      currentLat = gps.location.lat();  // set current lat with gps reading
      // Serial.print(" Longitude= ");
      // Serial.println(gps.location.lng(), 6);
      currentLng = gps.location.lng();  // set current long with gps reading

      if (startingLng == 0) {
        startingLng = currentLng;
      }

      if (startingLat == 0) {
        startingLat = currentLat;
      }
    }
  }

  currentMillis = millis();                                                             //get the current "time" (actually the number of milliseconds since the program started)
  if ((currentMillis < timeToRun) && (currentMillis - startMillis >= period))           //test whether the period has elapsed
  {

    generalHeading = getHeading(currentLat, waypointLat, currentLng, waypointLng, 1);   // get target heading from current location to waypoint

    if (startingHeading == 0) {                                                         // update starting heading if first run
      startingHeading = generalHeading;
    } else {
      diff = 360 - generalHeading;                                                      // find the offset of the heading from 0
      
      // calculate current heading from current and previous coordinates. Add diff to normalize general heading to 0. Wrap around to keep 0 < heading < 360 degrees
      currentHeading = wrap(getHeading(prevLat, currentLat, prevLng, currentLng, 0) + diff);
      oppositeGeneralHeading = 180.0;                                                 // opposite heading is fixed at 180 after heading is normalized to 0
      rightHeadingBoundary = 30.0;                                                    // define boundary conditions
      leftHeadingBoundary = 330.0;

      // // Serial.print(useX);
      // Serial.print(currentLat, 6);
      // Serial.println(currentLng, 6);

      if (currentHeading != diff) {                                                   // if boat is not moving, heading will be 0 + diff
        // if the current heading is between the right boundary and 180 degrees, correct left
        if (currentHeading > rightHeadingBoundary && currentHeading < oppositeGeneralHeading) {
          Serial.print("WAY OFF ");
          goLeft();
          delay(2000);
        
        // if the current heading is between the left boundary and 180 degrees, correct right
        } else if (currentHeading < leftHeadingBoundary && currentHeading > oppositeGeneralHeading) {
          Serial.print("WAY OFF ");
          goRight();
          delay(2000);
        }
      }
    }

    if (n >= waypoint_num) {
      waypointLat = destinationLat;
      waypointLng = destinationLng;
    } else {
      // otherwise, continue to the next intermediate waypoint
      waypointLng, waypointLat = gen_waypoint(n, generalHeading);
    }

    // gets the current x and y coordinates given gps coordinates
    currentX = getX(currentLat, currentLng);
    currentY = getY(currentLat);

    // gets the x and y coordinates of the waypoints given calculations
    waypointX = getX(waypointLat, waypointLng);
    waypointY = getY(waypointLat);

    // checks if the x and y distances to waypoint are getting smaller
    xIsCloser = getCloserX(currentX, prevX);
    yIsCloser = getCloserY(currentY, prevY);

    //if we use x distances in logic, switch direction if x dist increases
    if (useX && !xIsCloser) {
      // Serial.print("toggle");
      toggleDirection();
    // if using y distances in logic, switch direction if y dist increases
    } else if (!useX && !yIsCloser) {
      // Serial.print("toggle");
      toggleDirection();
    }

    // update previous values to the current values
    prevX = currentX;
    prevY = currentY;

    prevLat = currentLat;
    prevLng = currentLng;

    // find the total distance between current position and final destination
    distance = getDistance(currentLat, currentLng);

    Serial.print("distance:");
    Serial.println(distance);
    Serial.println("----------------------");

    if (distance < (15)) {      // if closer than 15m
      if (n == waypoint_num && destinationLat != startingLat) {
        done();
        // set destination back to starting location
        destinationLat = startingLat;
        destinationLng = startingLng;
        n = 1;
      } else if (n == waypoint_num && destinationLat == startingLat) {
        // stop completely in final destination (has returned home)
        leftspeed = 0;
        rightspeed = 0;
        delay(600000);
      } else {
         // if not on final waypoint, go to next waypoint
        n++;
        Serial.println("NEW WAYPOINT");
      }
    }
    startMillis = currentMillis; // reset millis
  }

  // stop after timeToRun milliseconds
  if (currentMillis > timeToRun) {
    leftspeed = 0;
    rightspeed = 0;
  }
}
