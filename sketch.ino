#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 5000; 

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

float destinationLng = -71.30584;
float destinationLat = 42.29352;
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
bool useX = true;
bool xIsCloser;
bool yIsCloser;

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

float getHeading() {
  distX = destinationX - currentX;
  distY = destinationY - currentY;
  headingRad = atan2(distX, distY);
  heading = degrees(headingRad);
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
  Serial.println(getXDifference(currentX));
  Serial.println(getXDifference(prevX));
  Serial.println("x difference:" + x);
  if(x) {
    Serial.println("Closer X");
  } else {
    Serial.println("Not closer X");
  }
  return x;
}

bool getCloserY(float currentY, float prevY) {
  bool y = getYDifference(currentY) <= getYDifference(prevY);
  Serial.println(getYDifference(currentY));
  Serial.println(getYDifference(prevY));
  if(y) {
    Serial.println("Closer Y");
  } else {
    Serial.println("Not closer Y");
  }
  return y;
}

float getXDifference(float coordinate) {
  return destinationX - coordinate;
}
float getYDifference(float coordinate) {
  return destinationY - coordinate;
}

void goLeft() {
  currentDirection = 1;
  Serial.println('left');
}

void goRight() {
  currentDirection = 0;
  Serial.println('right');
}

void toggleDirection() {
  if (currentDirection == 1) {  // 1 = left
    goRight();
  } else {
    goLeft();
  }
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
    }
  }

  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    // heading = getHeading();
    // Serial.println(heading);
    // Serial.println("hii");
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

    distance = getDistance(currentLat, currentLng);
    // Serial.println(distance);
    Serial.println(currentDirection);

    if (distance < 50) {
      Serial.println("WE'RE DONE");
    }
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
}
