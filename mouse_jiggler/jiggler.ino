#include "Mouse.h"

// Pin definitions

int led2 = PIN_LED2;  // Blue LED
int led3 = PIN_LED3;  // Another Blue LED (Really???)
const int LED_PIN =  LED_BUILTIN;  // Internal LED pin

// Movement pattern settings
enum Pattern {

  ZIGZAG,
  CIRCLE,
  ELLIPSE,
  RANDOM,
  REALISTIC,
  PATTERN_COUNT  // Keep track of the number of patterns
};

// Configurable parameters
int minInterval = 15000;       // Minimum interval between movements (ms)
int maxInterval = 50000;       // Maximum interval between movements (ms)
Pattern currentPattern = REALISTIC;
float angle = 0;            // Used for circle/ellipse patterns

// Struct to hold movement distances for each pattern
struct PatternDistances {

  int zigzag;
  int circle;
  int ellipse;
  int random;
  int realistic;
};

// Initialize movement distances for each pattern
PatternDistances moveDistances = {

  .zigzag = 10,
  .circle = 40,
  .ellipse = 30,
  .random = 20,
  .realistic = 500
};

// Bezier curve control points
struct Point {
  float x;
  float y;
};

unsigned long lastMoveTime = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
  Mouse.begin();
  randomSeed(analogRead(0));
}

void randomizePattern() {
  currentPattern = static_cast<Pattern>(random(0, PATTERN_COUNT));
}

void microMove() {
 
  Mouse.move(1, 0, 0);
  Mouse.move(-1, 0, 0);
 
}

bool shouldMove() {
  unsigned long currentTime = millis();
  if (currentTime - lastMoveTime >= random(minInterval, maxInterval)) {
    lastMoveTime = currentTime;
    return true;
  }
  return false;
}


void moveZigZag() {
 
  for(int i = 0; i < 4; i++) {
    for (int j = 0; j < moveDistances.zigzag/4; j++) {
      Mouse.move(1, 1, 0);
      microMove();
    }
    
    for (int j = 0; j < moveDistances.zigzag/4; j++) {
      Mouse.move(1, -1, 0);
      microMove();
    }
  }
 
}

void moveCircle() {
 
  for(int i = 0; i < 360; i += 5) {
    float rad = i * PI / 180.0;
    int x = cos(rad) * (moveDistances.circle/2) / 72;
    int y = sin(rad) * (moveDistances.circle/2) / 72;
    Mouse.move(x, y, 0);
    microMove();
  }

}

void moveEllipse() {

  for(int i = 0; i < 360; i += 5) {
    float rad = i * PI / 180.0;
    int x = cos(rad) * (moveDistances.ellipse/2) / 72;
    int y = sin(rad) * (moveDistances.ellipse/4) / 72;
    Mouse.move(x, y, 0);
    microMove();
  }
  digitalWrite(LED_PIN, LOW);
}

void moveRandom() {
  digitalWrite(LED_PIN, HIGH);
  int targetX = random(-moveDistances.random, moveDistances.random);
  int targetY = random(-moveDistances.random, moveDistances.random);
  
  int steps = max(abs(targetX), abs(targetY));
  float dx = targetX / (float)steps;
  float dy = targetY / (float)steps;
  
  for(int i = 0; i < steps; i++) {
    Mouse.move(round(dx), round(dy), 0);
    microMove();
  }

}

Point bezierPoint(Point p0, Point p1, Point p2, Point p3, float t) {
  float t2 = t * t;
  float t3 = t2 * t;
  float mt = 1 - t;
  float mt2 = mt * mt;
  float mt3 = mt2 * mt;
  
  Point result;
  result.x = mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x;
  result.y = mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y;
  return result;
}

void moveRealistic() {
 
  Point p0 = {0, 0};
  Point p3 = {(float)random(-moveDistances.realistic, moveDistances.realistic), 
              (float)random(-moveDistances.realistic, moveDistances.realistic)};
  Point p1 = {p0.x + random(-20, 20), p0.y + random(-20, 20)};
  Point p2 = {p3.x + random(-20, 20), p3.y + random(-20, 20)};
  
  Point lastP = p0;
  
  for(float t = 0; t <= 1.0; t += 0.02) {
    Point p = bezierPoint(p0, p1, p2, p3, t);
    int dx = round(p.x - lastP.x);
    int dy = round(p.y - lastP.y);
    
    Mouse.move(dx, dy, 0);
    microMove();
    
    lastP = p;
  }
  
}

void loop() {
  if (shouldMove()) {
    randomizePattern();

      digitalWrite(LED_PIN, HIGH);
    switch(currentPattern) {

      case ZIGZAG:
        moveZigZag();
        break;
      case CIRCLE:
        moveCircle();
        break;
      case ELLIPSE:
        moveEllipse();
        break;
      case RANDOM:
        moveRandom();
        break;
      case REALISTIC:
        moveRealistic();
        break;
    }
     digitalWrite(LED_PIN, LOW);
  }
  
 
}