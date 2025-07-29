int jccount;
#include <EEPROM.h>
#include <MicroPOP32.cpp>
#include <SharpIR.h>

short line(); // Forward declaration of line() function
void Track();
void PidTimer(int BaseSpeed, float Kp, float Kd, int Timer, int turnN, int slowSpeedRange);

/* UNILINE 2.2.0 by @superdinmc
 * Universal smart line robot firmware
 * New: Now with junction checking
 *
 * Customizable easy to use line tracking robot firmware
 * Compatible with both POP32 and ATX2
 * Installation:
 * Install SharpIR library(recommended 2.0.x)
 * Configure the bahavior below, all documented
 */
struct Config
{
  //   Developer setting   //
  bool EnableDebugging = true;
  bool useNewFn = true;
  bool JunctionBeep = false;
  //       Settings        //
  short Speed = 68; // สปีด
  float Kp = 0.022; // ส่าย
  float Kd = 0.048; // แก้ส่าย
  short turnN = -40;
  short slowSpeedRange = 31;
  short autoTuneRate = 48;
  short SharpPin = PB0;
  short SharpModel = SharpIR::GP2Y0A41SK0F;
  short ObstacleRange = 4;
  //     Quick manual      //
  // Speed: Speed bruh | Kp: Porpotional gain | Kd: Derilitive gain
  // turnN: (motorout<turnN)&&-90 | slowSpeedRange: Limit of "Slow on turn" feature
  // SharpPin: the pin of sharp obstacle sensor | SharpModel: See SharpIR docs
  // ObstacleRange: if distance less than this then avoid
  // Don't forget ;     ;-; //
};
int sensors[][6] = {
    //    Sensor configuration     // Pin:
    {0, 705, 3790, -4, 2, 0},  // - Analog pin on your board(If it is A5 put "A5"
    {1, 900, 3815, -3, 0, 0},  //     but if it is 17 put "17")
    {2, 260, 2020, -2, 0, 0},  // - Recommended to be 0-7
    {3, 1175, 3832, -1, 1, 0}, // Black:
    {4, 614, 3721, 1, 1, 0},   // - Place your array of sensors on black line
    {5, 1117, 3830, 2, 0, 0},  //     and record the value to that field
    {6, 325, 2275, 3, 0, 0},   // White:
    {7, 1205, 3838, 4, 2, 0},  // - Same like black, but put on white area
                               //{I,BBBB,WWWW,±P,T,0}         // Priority:
                               // │ │    │    │  │ │          // - How much that sensor values, used in calculation
                               // │ │ ;_;│    │  │ [Internal] // - Recommended to be ...,-4,-3,-2,-1,-1,1,1,2,3,4,...
                               // │ │    White│  Type         // Type:
                               // │ Black     Priority        // - This is used in the line recovery process
                               // Pin                         // - 1 for the center sensors
                               //                             // - 2 for sensor at the edge of the sensor array
}; // [Internal]: Memory for calculations
Config config = {}; // /!\ DO NOT MODIFY AFTER THIS /!\ 
//      Junction check      //
void junction_check()
{
  int lin = line(), timer;
  int sensorCount = sizeof(sensors) / sizeof(sensors[0]); /*
       if(s(sensorCount)<20&&s(0)<20) {
         timer = millis();
         while(s(sensorCount)<20&&s(0)<20) Track();
         int totaltime=millis()-timer;
         Serial.println(totaltime);
         if(totaltime<1000&&totaltime>10) turnLeft();
         };*/
  switch (jccount)
  {
    /*case 0:
      while(abs(lin)<30&&s(sensorCount)<20&&s(0)<20) Track();
      break;*/
    /*case 1:
      turnLeft();
      break;*/
    /*case 2:
      while(abs(lin)<30&&s(sensorCount)<20&&s(0)<20) Track();
      break;*/
  }
}
int s(short pin) { return abs(map(a(sensors[pin][0]), sensors[pin][1], sensors[pin][2], 0, 100)); };
bool d(short pin) { return s(pin) > 50; };
void tune(unsigned short sen) { sensors[sen][d(sen) + 1] = (a(sensors[sen][0]) + (sensors[sen][d(sen) + 1] * 2)) / 3; };
unsigned short sc = 0, lastLine, cycle;
short lock;
int all()
{
  unsigned int r = 0;
  for (int i = 0; i < sc; i++)
  {
    r += s(i);
  };
  return r / sc;
};
short line()
{
  int line = 0;
  cycle = cycle + config.autoTuneRate % 1000;
  for (int i = 0; i < sc; i++)
  {
    if (cycle < 1)
      tune(i);
    line += sensors[i][3] * (100 - s(i));
    if (sensors[i][4] == 2 && !d(i))
      lock = sensors[i][3] * 100;
    else if (sensors[i][4] == 1 && !d(i))
      lock = 0;
  };
  return line + lock;
};
void initLineCalc()
{
  sc = sizeof(sensors) / sizeof(sensors[0]);
  for (int i = 0; i < sc; i++)
    sensors[i][5] = (sensors[i][1] + sensors[i][2]) / 2;
}
SharpIR obstacle(config.SharpModel, config.SharpPin);
short sensorCount;
int leftmotor, rightmotor, error, last_error, Power_Motor, Position;
unsigned long LastTime = 0;
void setup()
{
  EEPROM.get(0x0100, config);
  initLineCalc();
  Serial.begin(115200);
  sensorCount = sizeof(sensors) / sizeof(sensors[0]);
  Serial.print(sensorCount);
  if (ok() && config.EnableDebugging)
  {
    while (1)
    {
      // for(int i=0;i<sc;i++) tune(i);
      Serial.print(F("\nSen: "));
      for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++)
      {
        Serial.print(F("A"));
        Serial.print(sensors[i][0]);
        Serial.print(F(" \t"));
      };
      Serial.print(F("| IR: "));
      Serial.print(obstacle.getDistance());
      Serial.print(F("\nRaw: "));
      for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++)
      {
        Serial.print(a(sensors[i][0]));
        Serial.print(F("\t"));
      };
      // Serial.print(F("| Line:"));
      Serial.printf(F("| Line: | Lock: %d\nOut: "), lock);
      // Serial.print(F("\nOut: "));
      for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++)
      {
        Serial.print(F(" "));
        Serial.print(s(i));
        Serial.print(F("\t"));
      };
      Serial.print(F("| "));
      int timestart = micros();
      int li = line();
      int timeend = micros();
      Serial.print(li);
      Serial.print(F("\tline() latency : "));
      Serial.print(timeend - timestart);
      Serial.print(F("us"));
      delay(100);
    }
  }
  while (!ok())
    ;
  beep();
  delay(500);
}

void loop()
{
  junction_check();
  Track();
  jccount++;
  if (config.JunctionBeep)
    beep();
}
void TrackNoLoop(Config c, int duration)
{
  int lin = line(), PreError, Out;
  if (c.useNewFn)
  {
    PidTimer(c.Speed, c.Kp, c.Kd, duration, c.turnN, c.slowSpeedRange);
    return;
  }
  else
    while (!(abs(lin) < 30 && s(0) < 20 && s(0) < 20) && !c.useNewFn)
    {
      lin = line();
      Out = (c.Kp * lin) + (c.Kd * PreError);
      PreError = lin;
      motor(c.Speed + Out, c.Speed - Out);
    }
  delay(100);
}
void Track()
{
  int lin = line(), PreError, Out;
  if (config.useNewFn)
  {
    while (!(/*abs(lin)<30&&*/ s(sensorCount) < 20 && s(0) < 20))
      PidTimer(config.Speed, config.Kp, config.Kd, 1, config.turnN, config.slowSpeedRange);
    return;
  }
  else
    while (!(abs(lin) < 30 && s(sensorCount) < 20 && s(0) < 20) && !config.useNewFn)
    {
      lin = line();
      Out = (config.Kp * lin) + (config.Kd * PreError);
      PreError = lin;
      motor(config.Speed + Out, config.Speed - Out);
    }
}
void Track(Config c)
{
  int lin = line(), PreError, Out;
  if (c.useNewFn)
  {
    while (!(/*abs(lin)<30&&*/ s(sensorCount) < 20 && s(0) < 20))
      PidTimer(c.Speed, c.Kp, c.Kd, 1, c.turnN, c.slowSpeedRange);
    return;
  }
  else
    while (!(abs(lin) < 30 && s(0) < 20 && s(0) < 20) && !c.useNewFn)
    {
      lin = line();
      Out = (c.Kp * lin) + (c.Kd * PreError);
      PreError = lin;
      motor(c.Speed + Out, c.Speed - Out);
    }
  delay(100);
}
//* //wip
void turnLeft()
{
  motor(-30, 30);
  while (line() > 20)
    ;
  motor(-20, 20);
  while (line() < 200)
    ;
} //*/
/**
 * @brief Backend of Track(), taken from romania line tracking code.
 *
 * @param BaseSpeed Base speed for the track
 * @param Kp P setting
 * @param Kd D setting
 * @param Timer milliseconds tracking loop time
 * @param turnN if PID return less than this, set that motor to -90 instead
 * @param slowSpeedRange Range of speed slow-on-curves feature
 */
void PidTimer(int BaseSpeed, float Kp, float Kd, int Timer, int turnN, int slowSpeedRange)
{
  LastTime = millis();
  int StartSpeed = BaseSpeed;
  while ((millis() - LastTime) <= Timer)
  {
    // Filter.Filter(qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 120, 50 ));
    // Position = Filter.Current();
    Position = line() * 7;
    if (!d(0) && !d(7))
      motor(BaseSpeed);
    // if (sensorValue[3] < 600 && sensorValue[4] < 600) if (sensorValues[0] < 600 && sensorValues[1] < 600 && sensorValues[2] < 600 && sensorValues[5] < 600 && sensorValues[6] < 600 && sensorValues[7] < 600) motorControl(100,100);
    error = Position /* - 3500*/;
    /* Remove first slash to mark as comment | Use //* to not mark as comment
    if (obstacle.getDistance(false) < config.ObstacleRange) {
      motor(-25.5,-25.5);
      delay(10);
      motor(0,0);
      delay(190);
      motor(-153,153);
      delay(70);
      motor(25.5,-25.5);
      delay(10);
      motor(0,0);
      delay(290);
      motor(130,65);
      while(error < 1500) {
        error = line()*7;
      }
      motor(0,0);
      delay(200);
    }//*/
    Power_Motor = (Kp * error) + (Kd * (error - last_error));
    last_error = error;
    BaseSpeed = StartSpeed - abs((int)error) / (3500 / slowSpeedRange); // number at 176:48 = 3500 divided by maximum number of auto slow on curve
    //* Remove first slash to mark as comment | Use //* to not mark as comment
    if (Power_Motor > BaseSpeed)
      Power_Motor = BaseSpeed;
    if (Power_Motor < -BaseSpeed)
      Power_Motor = -BaseSpeed;
    leftmotor = BaseSpeed + Power_Motor;
    rightmotor = BaseSpeed - Power_Motor;
    if (leftmotor >= 255)
      leftmotor = 255;
    if (leftmotor <= turnN)
      leftmotor = -30;
    if (rightmotor >= 255)
      rightmotor = 255;
    if (rightmotor <= turnN)
      rightmotor = -30;
    motor(leftmotor, rightmotor);
  }
}