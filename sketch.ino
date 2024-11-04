float destinationLng = -71.31070;
float destinationLat = 42.29262;
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
int currentDirection; // 1 = left; 0 = right;
bool useX;
bool xIsCloser;
bool yIsCloser;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
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
float R = 6371000; // Earth's radius in meters (can be adjusted if needed)

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

  return d; // Return distance in meters
}

bool getCloserX(float currentX, float prevX) {
  bool x = getXDifference(currentX) < getXDifference(prevX);
  return x;
}

bool getCloserY(float currentY, float prevY) {
  bool y = getYDifference(currentY) < getYDifference(prevY);
  return y;
}


int getXDifference(float coordinate) {
  return destinationX - coordinate;
}
int getYDifference(float coordinate) {
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
  if (currentDirection == 1) { // 1 = left
    goRight();
  } else {
    goLeft();
  }
}

void loop() {
  Serial.println("HII");
  // read from GPS
  currentLng = -71.31117;
  currentLat = 42.29237;

  currentX = getX(currentLat, currentLng);
  currentY = getY(currentLat);
  Serial.println(currentX);
  Serial.println(currentY);
  Serial.println(destinationX);
  Serial.println(destinationY);
  // logic
  xIsCloser = getCloserX(currentX, prevX);
  yIsCloser = getCloserX(currentY, prevY);

  if (useX && !xIsCloser) {
    toggleDirection();
  } else if (!useX && !yIsCloser) {
    toggleDirection();
  }

  prevX = currentX;
  prevY = currentY;

  distance = getDistance(currentLat, currentLng);
  Serial.println(distance);

  if (distance < 3000) {
    Serial.println("WE'RE DONE");
  }
  delay(3000);
}
