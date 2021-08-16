#include <Pixy2.h>
#include <mcp_can.h>
#include <SPI.h>
Pixy2 pixy;

/*! \mainpage ECE-4520 Universal Autonomous Ground Vehicle Traffic Light Code
 * \brief Software for control node of UAGV in triffuc light scenario.
 * \author Erald Bardhollari
 * \author Matthew Hait
 * \author Shyam Sundar Batagnalli Thimmasetty
 * \version 1.0
 */

// Pixy Block signiture IDs:
// Signatrue 3 = green; range of 3.0
// Signature 2 = red; range of 3.5
// Signature 5 = yello; range 4.4

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
  /** \brief HC-SR04 distance output. */
  int distance;

// Global Vars
  /** \brief HC-SR04 distance safe to drive logic. */
  bool drivePermission = false;
  /** \brief HC-SR04 avoidance logic. */
  bool driveSeek = false;
  /** \brief HC-SR04 reverse logic. */
  bool goBack = false;
  /** \brief Vision block aquired logic. */
  bool inSight = false;
  /** \brief Left desired wheel speed: (0-1 m/s) * 255 = 0x00-0XFF. */
  byte leftMtr = 0x00;
  /** \brief Right desired wheel speed: (0-1 m/s) * 255 = 0x00-0XFF. */
  byte rightMTR = 0x00;

/** @brief Extract current Pixycam block index .
   \return index Pixycam block index.
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

/** @brief Send structured CAN frame.
   \param left Left desired wheel speed: (0-1 m/s) * 255 = 0x00-0XFF.
   \param right Right desired wheel speed: (0-1 m/s) * 255 = 0x00-0XFF.
   \param leftForward Left wheel direction forward.
   \param rightForward Right wheel direction forward.
   \return bool False on CAN error.
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

/** @brief Measure forward object distance using ultrasonic sensor.
   \return int Distance to object [cm].
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

/** @brief Extract block parameters from Pixy library according to index location.
   \return Block Pixy Block details.
*/
Block *trackBlock(uint8_t index) {
  uint8_t i;

  for (i = 0; i < pixy.ccc.numBlocks; i++)
  {
    if (index == pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }

  return NULL;
}

void setup() {
  Serial.begin(9600); //Bluetooth module runs at 9600 atm.
  pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an OUTPUT
  pinMode(ECHO_PIN, INPUT); // Sets the ECHO_PIN as an INPUT
  pixy.init();
  pixy.changeProg("color_connected_components");
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  pinMode(CAN_INT, INPUT);
}

void loop() {
  static int16_t index = -1;
  bool lDIR = 1, rDir = 1;
  int32_t panOffset, heightOffset, headingOffset, left, right, areaBlock;
  float speedMult;
  Block *block = NULL;
  pixy.ccc.getBlocks();
  index = acquireBlock();
  if (index >= 0) {
    block = trackBlock(index);
  }
  Serial.print("signature");
  Serial.println(pixy.ccc.blocks[0].m_signature);
  if (block) {
    //Possible Multisample or average inputs from two cameras
    panOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)block->m_x;
    heightOffset = (int32_t)block->m_height;
    areaBlock = (int32_t)block->m_height * (int32_t)block->m_width;
    //frameWidth = 316 = -158 to 158
    //heightOffset = 0 to 208
  }
  Serial.print("area: ");
  Serial.println(areaBlock);
  distance = sonicDist();
  Serial.print("dist: ");
  Serial.println(distance);
  if (distance >= 40) {
    goBack = false;
    Serial.println("~~~~~~~~~~");
    driveSeek = false;
  }  else if (distance <= 10)  {
    Serial.println("***********");
    goBack = true;
    driveSeek = true;
  } else {
    goBack = false;
    driveSeek = true;
    Serial.println("------------");
  }
  lDIR = 1; rDir = 1;
  if (inSight) {
    switch (pixy.ccc.blocks[0].m_signature) {
      default:
        Serial.println("Green");
        rightMTR = 0xCF;
        leftMtr = 0xDF;
        break;
        // Optional Yellow Light Logic        
//      case 5:
//        Serial.println("Yellow");
//        rightMTR = 0x82;
//        leftMtr = 0x82;
//        break;
      case 2:
        Serial.println("Red");
        if (areaBlock >= 1200) {
          Serial.println("Red Stop");
          rightMTR = 0;
          leftMtr = 0;
        }
        break;
    }
  } else {
    if (!driveSeek) {
      Serial.println("Forward");
      rightMTR = 0xAA;
      leftMtr = 0x9A;
    } else {
      //Turn Right
      if (goBack) {
        //Go back
        rightMTR = 0xDF;
        leftMtr = 0xDF;
        lDIR = 0; rDir = 0;
        Serial.println("GoinBack");
      }  else {
        Serial.println("RightTurn");
        rightMTR = 0xFF;
        leftMtr = 0xFF;
        lDIR = 1; rDir = 0;
      }
    }
  }
  Serial.print("L: ");
  Serial.print(leftMtr);
  Serial.print(", R: ");
  Serial.println(rightMTR);
  // Build CAN Frame
  if (!sendFrame(leftMtr, rightMTR, lDIR, rDir)) {
    Serial.println("Error Sending Message...");
  }
  delay(250);
}
