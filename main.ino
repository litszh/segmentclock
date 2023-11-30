#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Timezone.h>

// Define the time zone for Pacific Time (PT) with daylight saving time (DST)
TimeChangeRule usPacific = {"PST", First, Sun, Nov, 2, -480};  // Pacific Standard Time (UTC-8)
TimeChangeRule usPacificDst = {"PDT", Second, Sun, Mar, 2, -420};  // Pacific Daylight Time (UTC-7 during DST)
Timezone usPacificTZ(usPacific, usPacificDst);

// Define NTP settings
//const long utcOffsetInSeconds = usPacificTZ.toUTC(0);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0);

//////////////////////////////
// motor control section
//////////////////////////////

// Motor parameters
#define STEPS_PER_ROTATION 4096 // steps of a single rotation of motor
#define MOTOR_DELAY 1200
#define KILL_BACKLASH 14
#define INITIAL_POS 100

// ports used to control the stepper motor
// if your motor rotates to the opposite direction, 
// change to int port[4] = {2, 0, 4, 12};
int port[4] = {12, 14, 4, 5};

// sequence of stepper motor control
int seq[8][4] = {
  {  LOW, HIGH, HIGH,  LOW},
  {  LOW,  LOW, HIGH,  LOW},
  {  LOW,  LOW, HIGH, HIGH},
  {  LOW,  LOW,  LOW, HIGH},
  { HIGH,  LOW,  LOW, HIGH},
  { HIGH,  LOW,  LOW,  LOW},
  { HIGH, HIGH,  LOW,  LOW},
  {  LOW, HIGH,  LOW,  LOW}
};

// stepper motor rotation
void rotate(int step) {
  String msg = "rotate: " + String(step);
  Serial.println(msg);
  int count = 0;
  static int phase = 0;
  int i, j;
  int delta = (step > 0) ? 1 : 7;

  step = (step > 0) ? step : -step;
  for(j = 0; j < step; j++) {
    Serial.print(".");
    phase = (phase + delta) % 8;
    for(i = 0; i < 4; i++) {
      digitalWrite(port[i], seq[phase][i]);
    }
    count++;
    delayMicroseconds(MOTOR_DELAY);
    if(step - j < 100) delayMicroseconds(MOTOR_DELAY); // deacceleration
    if(step - j < 200) delayMicroseconds(MOTOR_DELAY); // deacceleration
    if(step - j < 400) delayMicroseconds(MOTOR_DELAY); // deacceleration
    if(count < 100) delayMicroseconds(MOTOR_DELAY); // acceleration
    if(count < 50) delayMicroseconds(MOTOR_DELAY); // acceleration
    if(count < 20) delayMicroseconds(MOTOR_DELAY); // acceleration
    if(count < 10) delayMicroseconds(MOTOR_DELAY); // acceleration
  }
  Serial.println("");
  // power cut
  for(i = 0; i < 4; i++) {
    digitalWrite(port[i], LOW);
  }
}

//////////////////////////////
// rorary clock control section
//////////////////////////////

#define POSITION 13 // each wheel has 9 positions

#define DIGIT 4
typedef struct {
  int v[DIGIT];
} Digit;

void printDigit(Digit d);
Digit rotUp(Digit current, int digit, int num);
Digit rotDown(Digit current, int digit, int num);
Digit rotDigit(Digit current, int digit, int num);
Digit setDigit(Digit current, int digit, int num);
int setNumber(Digit n);

// avoid error accumuration of fractional part of 4096 / POSITION
void rotStep(int s) {
  static long currentPos;
  static long currentStep;
  
  currentPos += s;
  long diff = currentPos * STEPS_PER_ROTATION / POSITION - currentStep;
  if(diff < 0) diff -= KILL_BACKLASH;
  else diff += KILL_BACKLASH;
  rotate(diff);
  currentStep += diff;
}

void printDigit(Digit d) {
  String s = "            ";
  int i;

  for(i = 0; i < DIGIT; i++) {
    s.setCharAt(i, d.v[i] + '0');
  }
  Serial.println(s);
}

Digit current = {0};

//increase specified digit
Digit rotUp(Digit current, int digit, int num) {
  int freeplay = 0;
  int i;
  
  for(i = digit; i < DIGIT - 1; i++) {
    int id = current.v[i];
    int nd = current.v[i+1];
    if(id <= nd) id += POSITION;
    freeplay += id - nd - 1;
  }
  freeplay += num;
  rotStep(-1 * freeplay);
  current.v[digit] = (current.v[digit] + num) % POSITION;
  for(i = digit + 1; i < DIGIT; i++) {
    current.v[i] = (current.v[i - 1] + (POSITION - 1)) % POSITION;
  }

  return current;
}

// decrease specified digit
Digit rotDown(Digit current, int digit, int num) {
  int freeplay = 0;
  int i;
  
  for(i = digit; i < DIGIT - 1; i++) {
    int id = current.v[i];
    int nd = current.v[i+1];
    if(id > nd) nd += POSITION;
    freeplay += nd - id;
  }
  freeplay += num;
  rotStep( 1 * freeplay);
  current.v[digit] = (current.v[digit] - num + POSITION) % POSITION;
  for(i = digit + 1; i < DIGIT; i++) {
    current.v[i] = current.v[i - 1];
  }

  Serial.println("down end : ");
  printDigit(current);
  return current;
}

// decrease or increase specified digit 
Digit rotDigit(Digit current, int digit, int num) {
  if(num > 0) {
    return rotUp(current, digit, num);
  }
  else if(num < 0) {
    return rotDown(current, digit, -num);
  }
  else return current;
}

// set single digit to the specified number
Digit setDigit(Digit current, int digit, int num) {
  int pd, cd = current.v[digit];
  if(digit == 0) { // most significant digit
    pd = 0;
  }
  else {
    pd = current.v[digit - 1];
  }
  if(cd == num) return current;
  
  // check if increasing rotation is possible
  int n2 = num;
  if(n2 < cd) n2 += POSITION;
  if(pd < cd) pd += POSITION;
  if(pd <= cd || pd > n2) {
    return rotDigit(current, digit, n2 - cd);
  }
  // if not, do decrease rotation
  if(num > cd) cd += POSITION;
  return rotDigit(current, digit, num - cd);  
}

int setNumber(Digit n) {
  int i;
  
  for(i = 0; i < DIGIT; i++) {
    if(current.v[i] != n.v[i]) {
      break;
    }
  }
  // rotate most significant wheel only to follow current time ASAP
  if(i < DIGIT) {
      current = setDigit(current, i, n.v[i]);
  }
  if(i >= DIGIT - 1) {
    return true; // finished
  }
  else {
    return false;
  }
}
// digit definition. 10 for blank
int table[10] = {0, 10, 6, 2, 12, 7, 5, 11, 4, 3};
void setInt(int num){
//  static int num = 2;
  Digit n;

  n.v[0] = table[(num / 1000) % 10];
  n.v[1] = table[(num /  100) % 10];
  n.v[2] = table[(num /   10) % 10];
  n.v[3] = table[(num /    1) % 10];

  while(!setNumber(n))
    ;
}

//////////////////////////////
// WiFi and NTP section
//////////////////////////////

// switch between 24H (12 rotors) / 12H (10 rotors)
#define HOUR12 false

// NTP settings
#define TIMEZONE "PST8PDT,M3.2.0,M11.1.0"
#define NTP_SERVER "pool.ntp.org"

#define WIFI_SMARTCONFIG false

#if !WIFI_SMARTCONFIG
// if you do not use smartConfifg, please specify SSID and password here
#define WIFI_SSID "" // your WiFi's SSID
#define WIFI_PASS "" // your WiFi's password
#endif

void wifiSetup() {
  int wifiMotion = 400; // while wainting for wifi, large motion
  int smatconfigMotion = 100; // while wainting for smartConfig, small motion

  WiFi.mode(WIFI_STA);
#if WIFI_SMARTCONFIG
  WiFi.begin();
#else
  WiFi.begin(WIFI_SSID, WIFI_PASS);
#endif

  for (int i = 0; ; i++) {
    Serial.println("Connecting to WiFi...");
    rotate(wifiMotion); wifiMotion *= -1;
    delay(1000);
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
#if WIFI_SMARTCONFIG
  if(i > 6)
    break;
#endif    
  }

#if WIFI_SMARTCONFIG
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig();

    //Wait for SmartConfig packet from mobile
    Serial.println("Waiting for SmartConfig.");
    while (!WiFi.smartConfigDone()) {
      Serial.print(".");
      rotate(smatconfigMotion); smatconfigMotion *= -1;
      delay(1000);
    }

    Serial.println("");
    Serial.println("SmartConfig received.");

    //Wait for WiFi to connect to AP
    Serial.println("Waiting for WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      rotate(wifiMotion); wifiMotion *= -1;
      delay(1000);
      Serial.print(",");
    }
  }
  Serial.println("WiFi Connected.");
#endif

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

//////////////////////////////
// setup and main loop
//////////////////////////////

void setup() {
  Serial.begin(115200);
  Serial.println("Start setup.");

  pinMode(port[0], OUTPUT);
  pinMode(port[1], OUTPUT);
  pinMode(port[2], OUTPUT);
  pinMode(port[3], OUTPUT);

  wifiSetup();
  Serial.println("Wifi setup finished.");

  // Initialize the NTP client
  timeClient.begin();

  Serial.println("Got current time. Start reset all digits");

  rotate(STEPS_PER_ROTATION * DIGIT);
  Serial.println("Finish reset all digits using physical end stop");
  delay(500);
  Serial.println("release");  
  rotate(-INITIAL_POS); // release pushing force to align all digits better  

  delay(500);
}

void loop() {
  static int prevhour = -1;
  //struct tm tmtime;
  boolean finished;

  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();

  // Convert epoch time to local time
  time_t lc = usPacificTZ.toLocal(epochTime);
  struct tm *tmtime;
  tmtime = localtime(&lc);

  char buffer[25]; // Buffer to store the formatted time
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", tmtime);
  Serial.println(buffer);

  //printLocalTime();
  Digit n;

#if HOUR12
  tmtime->tm_hour %= 12;
  if(tmtime->tm_hour == 0)
    tmtime->tm_hour = 12;
#endif

  n.v[0] = table[tmtime->tm_hour / 10];
  n.v[1] = table[tmtime->tm_hour % 10];
  n.v[2] = table[tmtime->tm_min / 10];
  n.v[3] = table[tmtime->tm_min % 10];

  setNumber(n);
  delay(1000);
}
