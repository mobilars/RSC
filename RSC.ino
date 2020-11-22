/* Based on code from https://github.com/imwitti/FootpodMimic (Thank you!!!)
 *  Modified slightly by lkroland - https://github.com/mobilars to fit my setup
*/

#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

bool        is_inst_stride_len_present = 1;                                 /**< True if Instantaneous Stride Length is present in the measurement. */
bool        is_total_distance_present = 1;                                  /**< True if Total Distance is present in the measurement. */
bool        is_running = 1;                                                 /**< True if running, False if walking. */
uint16_t    inst_speed = 0;                                                 /**< Instantaneous Speed. */
uint8_t     inst_cadence = 0;                                               /**< Instantaneous Cadence. */
uint16_t    inst_stride_length = 50;                                        /**< Instantaneous Stride Length. */
uint32_t    total_distance = 0;
float       total_distance_float = 0;
unsigned long msec_between_steps = 0;

int encoder = 2;
volatile unsigned int counter;
int rpm;

float distance_per_mark = 0.14;
float kmph;
float mps;
byte rscmArray[12] = {0b000001,10,10,10};
byte fakePos[1] = {1};

bool _BLEClientConnected = false;

#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);
//BLECharacteristic sensorFeatureCharacteristic(BLEUUID((uint16_t)0x2A54), BLECharacteristic::PROPERTY_READ);
BLEDescriptor RSCDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

void InitBLE() {
  BLEDevice::init("Footpodmimic_D03");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pRSC = pServer->createService(RSCService);

  pRSC->addCharacteristic(&RSCMeasurementCharacteristics);
  RSCDescriptor.setValue("Rate from 0 to 200");
  RSCMeasurementCharacteristics.addDescriptor(&RSCDescriptor);
  RSCMeasurementCharacteristics.addDescriptor(new BLE2902());

  pRSC->addCharacteristic(&sensorPositionCharacteristic);
  sensorPositionDescriptor.setValue("Position 0 - 6");
  sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);

  pServer->getAdvertising()->addServiceUUID(RSCService);

  pRSC->start();
  // Start advertising
  pServer->getAdvertising()->start();
}

void poop() {
  counter++;
}

void stepSound()
{
 static unsigned long last_interrupt_time = 0;
 static unsigned long last_step_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200)
 {
  digitalWrite(2, LOW);

  msec_between_steps = (interrupt_time - last_step_time);
  Serial.print(msec_between_steps);
  Serial.println();

  if (msec_between_steps > 2000) {
    inst_cadence = 0; 
  } else if (msec_between_steps > 0) {
    inst_cadence = 60000/msec_between_steps;
  } else {
    inst_cadence = 0;  
  }
  last_step_time = interrupt_time;

 }
 last_interrupt_time = interrupt_time;
}

void setup() {
  Serial.begin(115200);
  //Serial.println("Start");
  Serial.print("started");
  pinMode(26, INPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  attachInterrupt(digitalPinToInterrupt(26),poop,RISING);
  attachInterrupt(digitalPinToInterrupt(26),stepSound,RISING);
  InitBLE();

}

static uint32_t previousMillis;

void loop() {
  
  // put your main code here, to run repeatedly:

  uint16_t msSinceLast = millis() - previousMillis;
  float sSinceLast = msSinceLast/1000;

  if (counter > 0) {
    is_running = 1;
  } else {
    is_running = 0;  
  }
  
  float distance_since_last = counter*distance_per_mark;
  total_distance_float = total_distance_float + distance_since_last;
  total_distance = (uint32_t) total_distance_float;
  mps = distance_since_last/sSinceLast;
  counter = 0;
  previousMillis = millis();
  kmph=mps*3.6;
  inst_speed =(256/3.6)*kmph;
  
  Serial.print("Total: ");
  Serial.print(total_distance);
  Serial.print(", Speed(mps): ");
  Serial.print(mps);
  Serial.print(", Speed(kmph): ");
  Serial.print(kmph);
  Serial.print(", inst_speed: ");
  Serial.print(inst_speed);
  Serial.print(", inst_cadence: ");
  Serial.println(inst_cadence);

  byte charArray[10] = {
      3,
      (unsigned byte)inst_speed, (unsigned byte)(inst_speed >> 8),
      (unsigned byte)inst_cadence,
      (unsigned byte)inst_stride_length, (unsigned byte)(inst_stride_length >> 8),
      (unsigned byte)total_distance, (unsigned byte)(total_distance >> 8), (unsigned byte)(total_distance >> 16), (unsigned byte)(total_distance >> 24)};

  RSCMeasurementCharacteristics.setValue(charArray,10);

  RSCMeasurementCharacteristics.notify();

  sensorPositionCharacteristic.setValue(fakePos, 1);


  digitalWrite(2, HIGH);  
  delay(1000);

}
