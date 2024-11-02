int destinationLng = -71.31070;
int destinationLat = 42.29262;
int currentLng;
int currentLat;
int currentX;
int currentY;
int prevX;
int prevY;
int currentDirection; // 1 = left; 0 = right;
bool useX;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
}

int getHeading() {
  // add code we don't know
    // find quadrants & set useX
}

int getX(lat, lng) {
  int num = 6, 371 * cos(lat) * lng;
  return num % pow(10, 9) 
}

int getY(lat) {
  int num = 6, 371 * lat
  return num % pow(10, 9) 
}

int destinationX = getX(destinationLng);
int destinationY = getY(destinationLat);

int getDistance(currentX, currentY) {
  return srqt(sq(destinationX - currentX) + sq(destinationY - currentY));
}

int getCloser(currentX, currentY, prevX, prevY) {
  bool x = getXDifference(currentX) < getXDifference(prevX)
  bool y = getYDifference(currentY) < getYDifference(prevY)

  return [x, y]
}

int getXDifference(coordinate) {
  return destinationX - coordinate;

int getYDifference(coordinate) {
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
  // read from GPS
  currentLng = 0;
  currentLat = 0;

  currentX = getX(currentLat, currentLng);
  currentY = getY(currentLat);

  // logic
  xIsCloser, yIsCloser = getCloser(currentX, currentY, prevX, prevY);

  if (useX && !xIsCloser) {
    toggleDirection();
  } elif (!useX && !yIsCloser) {
    toggleDirection();
  }

  prevX = currentX;
  prevY = currentY;

  if (getDistance(currentX, currentY) < 3000) {
    Serial.println("WE'RE DONE");
  }
  delay(3000);
  // while (!Serial.available());
  // x = Serial.readString().toInt();
  // Serial.print(x);
}
