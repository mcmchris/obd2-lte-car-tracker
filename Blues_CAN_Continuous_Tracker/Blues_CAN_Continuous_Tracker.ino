/*
   MCM - OBD2 Car Tracker

   @Author: Christopher Mendez | MCMCHRIS
   @Date: 04/09/2024 (mm/dd/yy)
   @Brief:
   This firmware runs on a XIAO ESP32C3 microcontroller, it communicates with the vehicle using a CAN-bus transceiver 
   and sends the read data to the Cloud using a Notecard LTE. 

*/

#include <Notecard.h>
#include <Wire.h>
#include <Serial_CAN_Module_ESP.h>

#define serialDebug Serial

#define PRODUCT_UID "com.hotmail.mcmchris:chevy_tracker"

#define productUID PRODUCT_UID

Notecard notecard;

long previousMillis = 0;
long interval = 60000 * 1;  // 1 minute

Serial_CAN can;

#define STANDARD_CAN_11BIT 1  // That depends on your car. some 1 some 0.

#define PID_ENGIN_PRM 0x0C
#define PID_VEHICLE_SPEED 0x0D
#define PID_COOLANT_TEMP 0x05
#define PID_INTAKE_TEMP 0x0f
#define PID_FUEL_TANK 0x2F  // (100/255)*A
//#define PID_ODOMETER 0xA6

#define LED_A D3
#define LED_B D0


#if STANDARD_CAN_11BIT
#define CAN_ID_PID 0x7DF
#else
#define CAN_ID_PID 0x18db33f1
#endif

int nSpeed = 0;  // speed, 0-200
int nRpm = 0;    // engin speed, 0-999
int nCTemp = 0;  // cooltant temp: 0-999
int nITemp = 0;  //intake temp: 0-999
int nFTank = 0;  // fuel tank level 0-100
float nOdo = 0;

double timeEpoch = 0;
double where_lat = 0;
double where_lon = 0;
double where_when = 0;
double temperature = 0;

bool latch = 0;

/**
 * @brief This function handles the car data send
 */
void sendPid(unsigned char __pid) {
  unsigned char tmp[8] = { 0x02, 0x01, __pid, 0, 0, 0, 0, 0 };
  //Serial.print("SEND PID: 0x");
  //Serial.println(__pid, HEX);

#if STANDARD_CAN_11BIT
  can.send(CAN_ID_PID, 0, 0, 8, tmp);  // SEND TO ID:0X55
#else
  can.send(CAN_ID_PID, 1, 0, 8, tmp);  // SEND TO ID:0X55
#endif
}

/**
 * @brief This function handles the car data requests
 */
unsigned char getPidFromCar(unsigned char __pid, unsigned char *dta) {
  sendPid(__pid);

  unsigned long timer_s = millis();

  while (1) {
    if (millis() - timer_s > 500) {
      return 0;
    }

    unsigned char len = 0;
    unsigned long id = 0;
    if (can.recv(&id, dta))  // check if get data
    {
      if (dta[1] == 0x41 && dta[2] == __pid) {
        return 1;
      }
    }
  }

  return 0;
}

void resetNotecard();
void configureHub();
void configureGPS();
void readFromSensors();

void setup() {
  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);

  serialDebug.begin(115200);

  for (int i = 0; i < 50; i++) {
    digitalWrite(LED_A, !digitalRead(LED_A));
    delay(100);
  }

  can.begin(0, 0, 9600);  // tx, rx

  notecard.setDebugOutputStream(serialDebug);
  notecard.begin();
  
  //resetNotecard();  // you will need to restore your Notecard if its configured with another unwanted project

  configureHub();
  delay(100);
  configureGPS();
  delay(100);

  for (int i = 0; i < 25; i++) {
    digitalWrite(LED_A, !digitalRead(LED_A));
    delay(200);
  }
  digitalWrite(LED_A, LOW);

  
  readFromSensors();
}

void loop() {

  readFromSensors();  // update the car data

  if (nRpm != 0 && latch == 0) {
    carMeasureReq();
    digitalWrite(LED_A, HIGH);
    if (J *req = notecard.newRequest("hub.sync")) {
      notecard.sendRequestWithRetry(req, 5);  // 5 seconds
    }
    latch = 1;
  } else if (nRpm == 0 && latch == 1) {
    carMeasureReq();
    digitalWrite(LED_A, LOW);
    if (J *req = notecard.newRequest("hub.sync")) {
      notecard.sendRequestWithRetry(req, 5);  // 5 seconds
    }
    latch = 0;
  }
  digitalWrite(LED_B, !digitalRead(LED_B)); // constant blink to verify activity
  delay(1000);
  unsigned long currentMillis = millis();

  // Retrieve car data and create the request to send it to the cloud every interval.
  if ((currentMillis - previousMillis > interval)) {
    previousMillis = currentMillis;
    if (nRpm != 0) {
      digitalWrite(LED_B, HIGH);
      carMeasureReq();
      digitalWrite(LED_B, LOW);
    }
  }
}

/**
 * @brief This function request to the car the different engine variables
 */
void readFromSensors() {

  unsigned char dta[8];

  if (getPidFromCar(PID_VEHICLE_SPEED, dta)) {
    nSpeed = dta[3];
  }
  delay(100);
  if (getPidFromCar(PID_ENGIN_PRM, dta)) {
    nRpm = (256.0 * (float)dta[3] + (float)dta[4]) / 4.0;
  }
  delay(100);
  if (getPidFromCar(PID_COOLANT_TEMP, dta)) {
    nCTemp = dta[3];
    nCTemp = nCTemp - 40;
  }
  delay(100);
  if (getPidFromCar(PID_INTAKE_TEMP, dta)) {
    nITemp = dta[3];
    nITemp = nITemp - 40;
  }
  delay(100);
  if (getPidFromCar(PID_FUEL_TANK, dta)) {
    nFTank = dta[3];
    nFTank = nFTank * (100.0 / 255.0);
  }
  delay(100);
  /*if (getPidFromCar(PID_ODOMETER, dta)) {
    nOdo = dta[3] * pow(2, 24) + dta[4] * pow(2, 16) + dta[5] * pow(2, 8) + dta[6];
    nOdo = nOdo * 0.1;
  }*/

  //Serial.println(nRpm);
  //Serial.println(nSpeed);
  //Serial.println(nCTemp);
  //Serial.println(nITemp);
  //Serial.println(nFTank);
}

/**
 * @brief This function can be used to restore the Notecard
 */
void resetNotecard() {
  J *req = notecard.newRequest("card.restore");
  JAddBoolToObject(req, "delete", true);
  notecard.sendRequest(req);
}

/**
 * @brief This function configures the Notecard's product UID, the mode and communication intervals.
 */
void configureHub() {
  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product", productUID);
  JAddStringToObject(req, "mode", "periodic");  // the periodic mode set the Notecard to wait the outbound interval to send data.
  //JAddNumberToObject(req, "align", true);
  JAddNumberToObject(req, "outbound", 5);   // every 5 minutes the Notecard will send every data in the queue.
  JAddNumberToObject(req, "inbound", 720);  // every 12 hours the Notecard will check for any incoming update.
  notecard.sendRequestWithRetry(req, 5);  // 5 seconds
}

/**
 * @brief This function configures the Notecard's GPS.
 */
void configureGPS() {
  J *req = notecard.newRequest("card.location.mode");
  JAddStringToObject(req, "mode", "continuous");  // maintain the GPS turned on. (we don't need to save energy)
  //JAddNumberToObject(req, "seconds", 5);
  notecard.sendRequestWithRetry(req, 5);  // 5 seconds
}

/**
 * @brief This function get the car data, location and environment variables read by the Notecard.
 */
void carMeasureReq() {

  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.temp"));
  if (rsp != NULL) {
    temperature = JGetNumber(rsp, "value");
    notecard.deleteResponse(rsp);
  }


  rsp = notecard.requestAndResponse(notecard.newRequest("card.location"));
  if (rsp != NULL) {
    where_lat = JGetNumber(rsp, "lat");
    where_lon = JGetNumber(rsp, "lon");
    where_when = JGetNumber(rsp, "time");
    notecard.deleteResponse(rsp);
  }


  rsp = notecard.requestAndResponse(notecard.newRequest("card.time"));
  if (rsp != NULL) {
    timeEpoch = JGetNumber(rsp, "time");
    notecard.deleteResponse(rsp);
  }
  readFromSensors();

  J *req = notecard.newRequest("note.add");
  if (req != NULL) {
    JAddStringToObject(req, "file", "sensors.qo");
    J *body = JCreateObject();
    if (body != NULL) {
      JAddNumberToObject(body, "where_lat", where_lat);
      JAddNumberToObject(body, "where_lon", where_lon);
      JAddNumberToObject(body, "where_when", where_when);
      JAddNumberToObject(body, "speed", nSpeed);
      JAddNumberToObject(body, "rpm", nRpm);
      JAddNumberToObject(body, "coolant_temp", nCTemp);
      JAddNumberToObject(body, "intake_temp", nITemp);
      JAddNumberToObject(body, "fuel_level", nFTank);
      JAddNumberToObject(body, "temperature", temperature);
      JAddNumberToObject(body, "time", timeEpoch);
      //JAddNumberToObject(body, "odometer", nOdo);
      JAddItemToObject(req, "body", body);
      
    }
    notecard.sendRequest(req);
  }
}
