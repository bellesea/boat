#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <PWMServo.h>

// control ESCs
PWMServo LeftESC;
PWMServo RightESC;
float rightdefaultspeed = 48;
float leftdefaultspeed = 45;  // old: difference should be around 8-12; left > right;
float rightspeed;
float leftspeed;

// connect to GPS module
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;
static const int RXPin = 4, TXPin = 3;  // pins actually switched on the arduino
SoftwareSerial ss(RXPin, TXPin);        // The serial connection to the GPS device

// create loop for adjustment
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 4000;

// long lat coordinates
float destinationLng = -71.31139;  // -71.31068;
float destinationLat = 42.29204;   // 42.28984;
float startingLng = 0;
float startingLat = 0;
float waypointLng;  // current temporary/waypoint destination
float waypointLat;
float currentLng;  // current position
float currentLat;
float prevLng;  // position from {period}ms ago
float prevLat;

// distance and heading
float distance;
float startingHeading = 0;     // keep track of initial heading to calculate waypoints
float generalHeading;          // to calculate heading from current location to waypoint
float oppositeGeneralHeading;  // to calculate opposite of the intended heading
float diff;                    // to calculate difference between general heading and 360
float currentHeading;          // to calculate current heading of boat (based on current and previous coordinates)
float rightHeadingBoundary;    // for heading adjustment
float leftHeadingBoundary;     // for heading adjustment
int currentDirection = 1;      // 1 = left; 0 = right;
bool useX;                     // figure out whether to use long or lat coordinates

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
int n = 1;  //starting waypoint = 1

int called = 0;

void setup() {
  Serial.begin(9600);               // begin serial communication
  RightESC.attach(10, 1000, 2000);  // control ESCs
  LeftESC.attach(9, 1000, 2000);    // control ESCs
  RightESC.write(0);                // stop motors
  LeftESC.write(0);                 // stop motors
  delay(120000);                    // 2 minutes delay to allow time to place boat in water
  rightspeed = rightdefaultspeed;   // set motor speeds
  leftspeed = leftdefaultspeed;
  ss.begin(GPSBaud);       // begin GPS transmission
  startMillis = millis();  // begin timer for loop
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
  // Convert latitude and longitude from degrees to radians
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2) - (flon1));
  a = sin(diflon) * cos(flat2);
  b = cos(flat1) * sin(flat2) - sin(flat1) * cos(flat2) * cos(diflon);
  a = atan2(a, b);

  tempHeading = degrees(a);  // Convert heading back to degrees from radians

  if (tempHeading < 0) { tempHeading = 360 + tempHeading; }  // If heading is negative, wrap around to keep 0 < heading < 360
  if (update == 1) {                                         // if this is the first pass through the code, determine which axis to use in logic

    // use x values to check if boat is getting closer if heading is in top or bottom quadrant. Else, check y values
    if (tempHeading <= 45 || tempHeading >= 315 || tempHeading <= 225 && tempHeading >= 135) {
      useX = true;
    } else {
      useX = false;
    }
  }

  return tempHeading;  // return heading
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
  if (heading >= 360) {  // if heading is greater than 360, subtract 360 to keep in range [0,360)
    heading -= 360;
  } else if (heading < 0) {  // if heading is less than 360, add 360 to keep in range [0,360)
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

// go right for a while then reset back to going straight
void goRight() {
  currentDirection = 0;            // set current direction
  leftspeed = 0;                   // set left motor turning speed
  rightspeed = rightdefaultspeed;  // right speed is default
  Serial.println("go right");
  LeftESC.write(leftspeed);  // send motor speeds
  RightESC.write(rightspeed);
  delay(2000);                   // delay for turn
  leftspeed = leftdefaultspeed;  // reset left motor speed to default
}

// go left for a while then reset back to going straight
void goLeft() {
  currentDirection = 1;            // set current direction
  leftspeed = 70;                  // set left motor turning speed
  rightspeed = rightdefaultspeed;  // right speed is default
  Serial.println("go left");
  LeftESC.write(leftspeed);  // send motor speeds
  RightESC.write(rightspeed);
  delay(1300);                   // delay for turn
  leftspeed = leftdefaultspeed;  // reset left motor speed to default
}

// turn 180 to begin going back to the origin location
void turn180() {
  currentDirection = 0;  // set direction to right
  leftspeed = 0;
  rightspeed = rightdefaultspeed;  // keep right motor speed at default value
  LeftESC.write(leftspeed);        // send speeds to the ESC
  RightESC.write(rightspeed);
  delay(4000);  // delay to allow for turn

  leftspeed = leftdefaultspeed;  // reset left motor speed to default
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
  waypoint_Xdiff = (destinationLng - startingLng) / waypoint_num;  // find x diff between each generated waypoint from start to end
  waypoint_Xdist = n * waypoint_Xdiff;                             // find x dist up until the current generated waypoint number

  waypoint_Ydiff = (destinationLat - startingLat) / waypoint_num;  // find y diff between each generated waypoint from start to end
  waypoint_Ydist = n * waypoint_Ydiff;                             // find y dist up until the current generated waypoint number
  // float waypoint_Ydist = waypoint_Xdist * tan(startingHeading);
  waypointLng = startingLng + waypoint_Xdist;  // determine longitude with x dist of current waypoing and starting lng
  waypointLat = startingLat + waypoint_Ydist;  // determine latitude with y dist of current waypoing and starting lat

  return (waypointLng, waypointLat);
}


void loop() {
  // set the motor speeds
  LeftESC.write(leftspeed);
  RightESC.write(rightspeed);

  if (called == 0) {
    goRight();
    called = 1;
  }

  // while (ss.available() > 0) { // ***
  //   gps.encode(ss.read()); // ***
  //   if (gps.location.isUpdated()) { // if there is new gps data
  //     currentLat = gps.location.lat(); // set current lat with gps reading
  //     currentLng = gps.location.lng(); // set current long with gps reading

  //     if (startingLng == 0 || startingLat == 0) {     // set startig coords to current coords if still at initial value of 0
  //       startingLng = currentLng;
  //       startingLat = currentLat;
  //     }
  //   }
  // }

  // currentMillis = millis();                           //get the current "time" (actually the number of milliseconds since the program started)
  // if (currentMillis - startMillis >= period)          //test whether the period has elapsed
  // {

  //   generalHeading = getHeading(currentLat, waypointLat, currentLng, waypointLng, 1);       // get target heading from current location to waypoint

  //   if (startingHeading == 0) {                                                             // update starting heading if first run
  //     startingHeading = generalHeading;
  //   } else {
  //     diff = 360 - generalHeading; // find the offset of the heading from 0

  //     // calculate current heading from current and previous coordinates. Add diff to normalize general heading to 0. Wrap around to keep 0 < heading < 360 degrees
  //     currentHeading = wrap(getHeading(prevLat, currentLat, prevLng, currentLng, 0) + diff);
  //     oppositeGeneralHeading = 180.0; // opposite heading is fixed at 180 after heading is normalized to 0
  //     rightHeadingBoundary = 45.0; // define boundary conditions
  //     leftHeadingBoundary = 315.0;

  //     Serial.println(useX); // print which axis boat uses to navigate
  //     Serial.println(currentLat, 6); // print current latitude
  //     Serial.println(currentLng, 6); // print current longitude

  //     if (currentHeading != diff) { // if boat is not moving, heading will be 0 + diff

  //       // if the current heading is between the right boundary and 180 degrees, correct left
  //       if (currentHeading > rightHeadingBoundary && currentHeading < oppositeGeneralHeading) {
  //         Serial.print("WAY OFF");
  //         goLeft();
  //         delay(2000);

  //       // if the current heading is between the left boundary and 180 degrees, correct right
  //       } else if (currentHeading < leftHeadingBoundary && currentHeading > oppositeGeneralHeading) {
  //         Serial.print("WAY OFF");
  //         goRight();
  //         delay(2000);
  //       }
  //     }
  //   }

  //   // if the boat has passed the final extra waypoint, set the next waypoint to the destination location
  //   if (n >= waypoint_num) {
  //     waypointLat = destinationLat;
  //     waypointLng = destinationLng;
  //   } else {
  //     // otherwise, continue to the next intermediate waypoint
  //     waypointLng, waypointLat = gen_waypoint(n, generalHeading);
  //   }

  //   // gets the current x and y coordinates given gps coordinates
  //   currentX = getX(currentLat, currentLng);
  //   currentY = getY(currentLat);

  //   // gets the x and y coordinates of the waypoints given gps coordinates
  //   waypointX = getX(waypointLat, waypointLng);
  //   waypointY = getY(waypointLat);

  //   // checks if the x and y distances are getting smaller
  //   xIsCloser = getCloserX(currentX, prevX);
  //   yIsCloser = getCloserY(currentY, prevY);

  //   // if we use x distances in logic, switch direction if x dist inc
  //   if (useX && !xIsCloser) {
  //     Serial.print("toggle");
  //     toggleDirection();

  //   // if using y distances in logic, switch direction if y dist inc
  //   } else if (!useX && !yIsCloser) {
  //     Serial.print("toggle");
  //     toggleDirection();
  //   }

  //   // update previous values to the current values
  //   prevX = currentX;
  //   prevY = currentY;

  //   prevLat = currentLat;
  //   prevLng = currentLng;

  //   // find the total distance between current position and final destination
  //   distance = getDistance(currentLat, currentLng);

  //   // print distance
  //   Serial.print("distance:");
  //   Serial.println(distance);
  //   Serial.println("----------------------");

  //   if (distance < 10) { // if closer than 10m
  //     if (n == waypoint_num) { // if we are using the final generated waypoint, set motor speeds to 0
  //       Serial.println("WE'RE DONE");

  //       LeftESC.write(0);
  //       RightESC.write(0);

  //       delay(30000);                             // stop for 30 seconds

  //       destinationLat = startingLat; // set destination back to starting location
  //       destinationLng = startingLng;
  //       n = 0; // reset back to first generated waypoint
  //     } else {
  //       n++; // if not on final waypoint, go to next waypoint
  //       Serial.println("NEW WAYPOINT");
  //     }
  //   }
  //   startMillis = currentMillis; // reset millis
  // }
}
