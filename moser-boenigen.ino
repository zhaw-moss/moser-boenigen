/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <MKRWAN.h>
#include "secrets.h"
#include "ArduinoLowPower.h"
#include <Wire.h>
#include <Adafruit_SHT31.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// num of attempts to join the lora network
#define MAX_ATTEMPTS 5

// waiting time between two join attempts in milliseconds
#define ATTEMPT_DELAY 30000

// waiting time before reset and new attempt in milliseconds
#define RESET_DELAY 600000

// sleep time between two measurements in milliseconds
#define MEASUREMENT_TIMEOUT 120000

// minimum sending interval in minutes
#define MIN_SEND_INTERVAL 120

// temperature hysteresis
#define TEMP_HYSTERESIS 0.5

/*******************************************************************************
 * Pin Definitions
 ******************************************************************************/
#define PIN_RGB_R 8
#define PIN_RGB_G 10
#define PIN_RGB_B 7
#define PIN_RELAIS_1 3
#define PIN_RELAIS_2 2

/*******************************************************************************
 * Object Declarations
 ******************************************************************************/

LoRaModem modem;
Adafruit_SHT31 sht = Adafruit_SHT31();

/*******************************************************************************
 * Functions
 ******************************************************************************/

void (*resetFunc)(void) = 0;  // create a standard reset function

/**
 * \brief resets the board in RESET_DELAY ms time
 */
void delayReset(void) {
  Serial.println("The board will be resetted soon");
  delay(RESET_DELAY);
  resetFunc();
}

/**
 * \brief lets inside led shine in a color
 * \param red brightness of red 0-255
 * \param green brightness of green 0-255
 * \param blue brightness of blue 0-255
 */
void setRgbLed(uint8_t red, uint8_t green, uint8_t blue) {
  red = map(red, 0, 255, 255, 0);
  green = map(green, 0, 255, 255, 0);
  blue = map(blue, 0, 255, 255, 0);
  analogWrite(PIN_RGB_R, red);
  analogWrite(PIN_RGB_G, green);
  analogWrite(PIN_RGB_B, blue);
}

/**
 * \brief converts data into sendable buffer
 * \param temp temperature value
 * \param humi humidity value
 * \param relais1 state of relais 1
 * \param relais2 state of relais2
 * \param buf address to buffer of min size 4
 */
void convertTxData(float temp, float humi, bool relais1, bool relais2, uint8_t* buf) {
  uint16_t _temp = round((temp + 20) * 100);
  uint8_t _humi = round(humi * 2);
  uint8_t _relais = 0;
  _relais |= relais1 ? 0x1 : 0x0;
  _relais |= relais2 ? 0x2 : 0x0;

  // write in buf
  buf[0] = _temp >> 8;
  buf[1] = _temp & 0xFF;
  buf[2] = _humi;
  buf[3] = _relais;
}

void setup() {
  // GPIO Initialization
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  pinMode(PIN_RELAIS_1, OUTPUT);
  pinMode(PIN_RELAIS_2, OUTPUT);

  // Red LED
  setRgbLed(255, 0, 0);

  // Serial port initialization
  Serial.begin(115200);
  delay(2000);  // Wait for serial connection
  if (!Serial) {
    delayReset();
  }

  // LoRa module initialization
  Serial.print("Initialitze the modem...");
  if (!modem.begin(EU868)) {
    Serial.println("- Failed to start module");
    Serial.println("ERROR");
    delayReset();
  };
  Serial.println("OK");

  // Join procedure to the network server
  uint8_t joinAttempts = 0;
  int connected = 0;
  Serial.print("Joining network... ");
  do {
    Serial.print(joinAttempts);
    connected = modem.joinOTAA(APP_EUI, APP_KEY);
    joinAttempts++;
    if (connected) {
      break;
    } else {
      delay(ATTEMPT_DELAY);
    }
  } while (joinAttempts <= MAX_ATTEMPTS);
  if (!connected) {
    Serial.println("ERROR");
    delayReset();
  }
  Serial.println(" OK");

  // Sensor start
  Serial.print("Searching Sensor... ");
  if (sht.begin()) {
    Serial.println(" OK");
  } else {
    Serial.println("ERROR");
    //delayReset();
  }

  // turn off LED
  setRgbLed(0, 0, 0);

  delay(20000);  // make sure that there's enough time to flash
}

void loop() {
  // variables
  static float temp = 0;
  static float lastTemp = 0;
  static float humi = 0;
  static bool relais1 = false;
  static bool relais2 = false;
  static uint32_t timeSent;
  static bool toSend = false;
  uint32_t now = millis();
  uint8_t txBuf[4];
  uint8_t rxBuf[1];

  // Begin Measurement and sending routine
  setRgbLed(0, 255, 0);  // green LED

  // read the sensor values
  sht.readBoth(&temp, &humi);

  Serial.print("Temp: ");
  Serial.println(temp);
  Serial.print("Humi: ");
  Serial.println(humi);

  // check if there was a temperature change
  if (abs(temp - lastTemp) > TEMP_HYSTERESIS) {
    toSend = true;
    lastTemp = temp;
    Serial.println("Message scheduled due to large change");
  }

  // check if there has been no message for a long time
  if (now - timeSent > MIN_SEND_INTERVAL * 60000) {
    toSend = true;
    Serial.println("Message scheduled due to long timeout");
  }

  // send and receive if necessary
  if (toSend) {
    convertTxData(temp, humi, relais1, relais2, txBuf);

    modem.beginPacket();
    modem.write(txBuf, 4);
    toSend = modem.endPacket() ? false : true;

    // check if there is downlonk data availible
    bool dataReceived = modem.available();

    if (dataReceived) {
      rxBuf[0] = (uint8_t)modem.read();
      relais1 = rxBuf[0] & 0x1;
      relais2 = rxBuf[0] & 0x2;

      digitalWrite(PIN_RELAIS_1, relais1);
      digitalWrite(PIN_RELAIS_2, relais2);

      Serial.print("Received: ");
      Serial.println(rxBuf[0], HEX);
    }
  }

  delay(2000); // make sure one can see the led
  setRgbLed(0, 0, 0);          // dark LED
  LowPower.deepSleep(MEASUREMENT_TIMEOUT);
}