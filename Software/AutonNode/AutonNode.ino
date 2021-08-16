#include <Pixy2.h>
#include <mcp_can.h>
#include <SPI.h>
Pixy2 pixy;

/*! \mainpage ECE-4520 Universal Autonomous Ground Vehicle object tracking Code
 * \brief Software for control node of UAGV in object tracking and seeking scenario.
 * \author Erald Bardhollari
 * \author Matthew Hait
 * \author Shyam Sundar Batagnalli Thimmasetty
 * \version 1.0
 */

// CAN
  /** \brief CAN transceiver pin number. */
  MCP_CAN CAN0(9);
  /** \brief CAN transceiver initialize pin number. */
  #define CAN_INT 2
  
// Distnace Sensor
  /** \brief HC-SR04 echo pin number. */
  #define ECHO_PIN 6
  /** \brief HC-SR04 Trig pin number. */
  #define TRIG_PIN 5

// Misc Vars
  /** \brief Wait period between loosing block and seeking for new block. */
  #define SEEK_DELAY 5000
  /** \brief Vision block aquired logic. */
  bool inSight = false;
  /** \brief Distance to follow block [block area pixles]. */
  int trakDist = 90;

// Seek Vars
  /** \brief Turning speed during seek */
  #define TURNSPEED 170
  /** \brief Last time block was seen [ms] */
  unsigned long lastLost = 0;

/** @brief Analize the biggest block (blocks[0]) that's been around for at least 15 frames (1/4 second).
 * \return int16_t Index of Block[o], else "-1"
 */
int16_t acquireBlock()
{
  if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age > 7) {
    inSight = true;
    return pixy.ccc.blocks[0].m_index;
  } else {
    inSight = false;
    return -1;
  }
}

/** @brief Idle task to look for block if lost longer than SEEK_DELAY.
 * \param Buffer address with motor values = {leftMtr, rightMTR, lDIR, rDir}.
 * \return bool Simple return for a clean exit.
 */
bool seekBlock(uint8_t *buf) {
  // If 0, first time run. Seek block if lost for SEEK_DELAY ms...
  if (lastLost != 0 && (millis()-lastLost) > SEEK_DELAY) {
    // right 5 seconds, left 5 seconds, right 5 seconds, ect...
    if ((millis()-lastLost) < (SEEK_DELAY + 1250)) {
      //Turn left 2.5 seconds.
      buf[0] = TURNSPEED; buf[1] = TURNSPEED; buf[2] = 0; buf[3] = 1;
      return true;
    }
    if ((millis()-lastLost) < (SEEK_DELAY + 3750)) {
      //Turn right 5 seconds.
      buf[0] = TURNSPEED; buf[1] = TURNSPEED; buf[2] = 1; buf[3] = 0;
      return true;
    }
    if ((millis()-lastLost) < (SEEK_DELAY + 6250)) {
      //Turn left 5 seconds.
      buf[0] = TURNSPEED; buf[1] = TURNSPEED; buf[2] = 0; buf[3] = 1;
      return true;
    }
    if ((millis()-lastLost) < (SEEK_DELAY + 7500)) {
      //Turn right 2.5 seconds.
      buf[0] = TURNSPEED; buf[1] = TURNSPEED; buf[2] = 1; buf[3] = 0;
      //Rest lastLost to repeat clean
      lastLost = millis() + SEEK_DELAY + 1;
      return true;
    }
  }
  //Just lost, return no moovement
  buf[0] = 0; buf[1] = 0; buf[3] = 1; buf[4] = 1;
  return false;
}

/** @brief Send structured CAN frame.
 * \param left Left desired wheel speed: (0-1 m/s) * 255 = 0x00-0XFF.
 * \param right Right desired wheel speed: (0-1 m/s) * 255 = 0x00-0XFF.
 * \param leftForward Left wheel direction forward.
 * \param rightForward Right wheel direction forward.
 * \return bool False on CAN error.
 */
bool sendFrame(byte left , byte right, bool leftForward, bool rightForward) {
  byte data[8] = {left, right, (byte)leftForward, (byte)rightForward, 0x00, 0x00, 0x00, 0x00};
  byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
  if (sndStat == CAN_OK) {
    return true;
  } else {
    return false;
  }  
}

/** @brief Measure Ultra Sonic Sensor=
 * \return int Sensor Distance.
 */
int sonicDist() {
  long duration; // variable for the duration of sound wave travel
  int tmpDist;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  tmpDist = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return tmpDist;  
}

/** @brief Extracts block details from index.
 * \param index Pixycam index.
 * \return Block Block Data.
 */
Block *trackBlock(uint8_t index) {
  uint8_t i;
  for (i = 0; i < pixy.ccc.numBlocks; i++) {
    if (index == pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }
  return NULL;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pixy.init();
  pixy.changeProg("color_connected_components");
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK){
    Serial.println("Error Initializing MCP2515...");
  }
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  pinMode(CAN_INT, INPUT);
}

void loop() {
  Block *block = NULL;
  bool lDIR = 1, rDir = 1;
  byte leftMtr = 0x00, rightMTR = 0x00;
  int distance;     // variable for the distance measurement
  int32_t panOffset, heightOffset, headingOffset, left, right;
  float speedMult;
  static int16_t index = -1;
  uint8_t mtrBuf[5];
  
  pixy.ccc.getBlocks();
  index = acquireBlock();
  if (index >= 0) {
    block = trackBlock(index);
  }
  if (block) {
    //Possible Multisample or average inputs from two cameras
    panOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)block->m_x;
    heightOffset = (int32_t)block->m_height;
    //frameWidth = 316 = -158 to 158
    //heightOffset = 0 to 208
  }
  distance = sonicDist();
  if (inSight && distance >= 40) {
    lastLost = millis(); //Update for seek function incase loss.
    //Constrain is here because offset is a large negative value if not in frame
    panOffset = constrain(panOffset, -158, 158);
    heightOffset = constrain(heightOffset, 0, 207);
    //Set speed multiplier to follow at tracking ditance
    speedMult = 1 - (((float)heightOffset + (float)trakDist) / 207);
    //Testing adjustment
      speedMult += 0.6;
    speedMult = constrain(speedMult, 0, 1);
    rightMTR = speedMult*(map(panOffset, -158, 158, 108, -73) + 0x92);
    leftMtr = speedMult*(map(panOffset, -156, 156, -73, 108) + 0x92);
    Serial.print("SpeedMult: ");
    Serial.print(speedMult);
    Serial.print(", Pclean: ");
    Serial.println(panOffset);
  } else {
    seekBlock(mtrBuf);
    leftMtr = mtrBuf[0]; rightMTR = mtrBuf[1]; lDIR = mtrBuf[2]; rDir = mtrBuf[3];
  }
  Serial.print("L: ");
  Serial.print(leftMtr);
  Serial.print(", R: ");
  Serial.println(rightMTR);
    
  // Build CAN Frame
  if (!sendFrame(leftMtr,rightMTR,lDIR,rDir)) {
    Serial.println("Error Sending Message...");
  }
  delay(50);   //Minimum value is 1000ms/60fps ≈ 17
}
