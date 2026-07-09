/********************************************************************
 *
 *  ECG + Accelerometer BLE Streamer
 *
 *  Board : ESP32
 *
 *  ECG :
 *      256 Hz
 *      256 Samples / Second
 *
 *  ACC :
 *      25 Hz
 *      25 Samples / Second
 *
 *  BLE:
 *      ECG + ACC synchronized by Second ID
 *
 *  Double Buffering
 *
 ********************************************************************/

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//////////////////////////////////////////////////////////////
// GPIO
//////////////////////////////////////////////////////////////

#define ECG_PIN         34
#define BATTERY_PIN     33

#define LO_PLUS         25
#define LO_MINUS        26

#define SDA_PIN         21
#define SCL_PIN         22

//////////////////////////////////////////////////////////////
// Sampling
//////////////////////////////////////////////////////////////

#define ECG_RATE            256
#define ACC_RATE             25

#define ECG_SAMPLES         256
#define ACC_SAMPLES          25

//////////////////////////////////////////////////////////////
// Timing
//////////////////////////////////////////////////////////////

#define ECG_PERIOD_US      3906
#define ACC_PERIOD_US     40000

//////////////////////////////////////////////////////////////
// Battery
//////////////////////////////////////////////////////////////

#define BATTERY_PERIOD_MS   60000

//////////////////////////////////////////////////////////////
// BLE
//////////////////////////////////////////////////////////////

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

#define DATA_UUID           "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define BATTERY_UUID        "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

//////////////////////////////////////////////////////////////
// Packet Types
//////////////////////////////////////////////////////////////

#define PACKET_ECG          1
#define PACKET_ACC          2
#define PACKET_BATTERY      3

//////////////////////////////////////////////////////////////
// Chunk
//////////////////////////////////////////////////////////////

#define BLE_CHUNK_SIZE      500
#define HEADER_SIZE 16
#define PAYLOAD_SIZE (BLE_CHUNK_SIZE - HEADER_SIZE)
//////////////////////////////////////////////////////////////
// ADXL345
//////////////////////////////////////////////////////////////

Adafruit_ADXL345_Unified accel(123);
//////////////////////////////////////////////////////////////

struct AccSample
{
    float x;

    float y;

    float z;
};

//////////////////////////////////////////////////////////////

struct ECGFrame
{
    uint32_t secondID;

    uint32_t timestamp;

    uint8_t leadStatus;

    uint16_t ecg[ECG_SAMPLES];
};

//////////////////////////////////////////////////////////////

struct ACCFrame
{
    uint32_t secondID;

    uint32_t timestamp;

    AccSample acc[ACC_SAMPLES];
};

//////////////////////////////////////////////////////////////

struct ChunkHeader
{
    uint8_t packetType;

    uint32_t secondID;

    uint32_t timestamp;

    uint16_t chunkIndex;

    uint16_t totalChunks;

    uint16_t payloadSize;
};

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

ECGFrame ecgBuffer[2];

ACCFrame accBuffer[2];

volatile uint8_t writeBuffer = 0;

volatile uint8_t sendBuffer = 1;

volatile bool frameReady = false;

portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;
//////////////////////////////////////////////////////////////

volatile uint16_t ecgIndex = 0;

volatile uint8_t accIndex = 0;

volatile uint32_t secondCounter = 0;

//////////////////////////////////////////////////////////////

uint32_t nextECG = 0;

uint32_t nextACC = 0;

uint8_t ecgCorrection = 0;

//////////////////////////////////////////////////////////////

uint32_t lastBatteryMillis = 0;

//////////////////////////////////////////////////////////////

BLECharacteristic *dataCharacteristic;

BLECharacteristic *batteryCharacteristic;

bool deviceConnected = false;

//////////////////////////////////////////////////////////////

uint8_t txBuffer[BLE_CHUNK_SIZE];
class MyServerCallbacks : public BLEServerCallbacks
{

    void onConnect(BLEServer*)
    {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer*)
    {
        deviceConnected = false;

        BLEDevice::startAdvertising();
    }

};
//////////////////////////////////////////////////////////////
// setup()
//////////////////////////////////////////////////////////////

void setup()
{

    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("=================================");
    Serial.println(" ECG + ACC BLE STREAMER ");
    Serial.println("=================================");

    //////////////////////////////////////////////////////////
    // GPIO
    //////////////////////////////////////////////////////////

    pinMode(LO_PLUS, INPUT);

    pinMode(LO_MINUS, INPUT);

    //////////////////////////////////////////////////////////
    // ADC
    //////////////////////////////////////////////////////////

    analogReadResolution(12);

    analogSetPinAttenuation(ECG_PIN, ADC_11db);

    analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

    //////////////////////////////////////////////////////////
    // I2C
    //////////////////////////////////////////////////////////

    Wire.begin(SDA_PIN, SCL_PIN);

    //////////////////////////////////////////////////////////
    // ADXL345
    //////////////////////////////////////////////////////////

    if (!accel.begin())
    {

        Serial.println("ADXL345 NOT FOUND");

        while (1);

    }

    accel.setRange(ADXL345_RANGE_16_G);

    Serial.println("ADXL345 OK");

    //////////////////////////////////////////////////////////
    // BLE
    //////////////////////////////////////////////////////////

    BLEDevice::init("ECG_ACC_ESP32");

    BLEDevice::setMTU(517);

    BLEServer *server = BLEDevice::createServer();

    server->setCallbacks(new MyServerCallbacks());

    BLEService *service =
        server->createService(SERVICE_UUID);

    //////////////////////////////////////////////////////////
    // DATA CHARACTERISTIC
    //////////////////////////////////////////////////////////

    dataCharacteristic =
        service->createCharacteristic(
            DATA_UUID,
            BLECharacteristic::PROPERTY_NOTIFY);

    dataCharacteristic->addDescriptor(
        new BLE2902());

    //////////////////////////////////////////////////////////
    // BATTERY CHARACTERISTIC
    //////////////////////////////////////////////////////////

    batteryCharacteristic =
        service->createCharacteristic(
            BATTERY_UUID,
            BLECharacteristic::PROPERTY_NOTIFY);

    batteryCharacteristic->addDescriptor(
        new BLE2902());

    //////////////////////////////////////////////////////////

    service->start();

    BLEAdvertising *advertising =
        BLEDevice::getAdvertising();

    advertising->addServiceUUID(SERVICE_UUID);

    advertising->start();

    //////////////////////////////////////////////////////////

    nextECG = micros();

    nextACC = micros();

    lastBatteryMillis = millis();

    Serial.println("SYSTEM READY");
}
uint8_t getLeadStatus()
{

    bool plus = digitalRead(LO_PLUS);

    bool minus = digitalRead(LO_MINUS);

    if (!plus && !minus)
        return 0;

    if (plus && !minus)
        return 1;

    if (!plus && minus)
        return 2;

    return 3;

}
inline void sampleECG()
{

    ECGFrame &frame = ecgBuffer[writeBuffer];

    frame.leadStatus = getLeadStatus();

    if(frame.leadStatus == 0)
    {
        frame.ecg[ecgIndex] =
            analogRead(ECG_PIN);
    }
    else
    {
        frame.ecg[ecgIndex] = 0xFFFF;
    }

    ecgIndex++;

}
inline void sampleACC()
{

    ACCFrame &frame =
        accBuffer[writeBuffer];

    sensors_event_t event;

    accel.getEvent(&event);

    frame.acc[accIndex].x =
        event.acceleration.x;

    frame.acc[accIndex].y =
        event.acceleration.y;

    frame.acc[accIndex].z =
        event.acceleration.z;

    accIndex++;

}
void finishFrame()
{
    uint32_t ts = millis();

    ecgBuffer[writeBuffer].secondID = secondCounter;
    ecgBuffer[writeBuffer].timestamp = ts;

    accBuffer[writeBuffer].secondID = secondCounter;
    accBuffer[writeBuffer].timestamp = ts;

    portENTER_CRITICAL(&bufferMux);

    sendBuffer = writeBuffer;

    writeBuffer ^= 1;

    frameReady = true;

    portEXIT_CRITICAL(&bufferMux);

    ecgIndex = 0;

    accIndex = 0;

    secondCounter++;
}
//////////////////////////////////////////////////////////////
// Sampling Scheduler
//////////////////////////////////////////////////////////////

void samplingTask()
{
    uint32_t now = micros();
    //////////////////////////////////////////////////////
    // ECG @256Hz
    //////////////////////////////////////////////////////
    if ((int32_t)(now - nextECG) >= 0)
    {
        nextECG += ECG_PERIOD_US;

        // 3906.25 us correction
        ecgCorrection++;

        if (ecgCorrection >= 4)
        {
            nextECG++;
            ecgCorrection = 0;
        }

        if (ecgIndex < ECG_SAMPLES)
        {
            sampleECG();
        }
    }
    //////////////////////////////////////////////////////
    // ACC @25Hz
    //////////////////////////////////////////////////////
    if ((int32_t)(now - nextACC) >= 0)
    {
        nextACC += ACC_PERIOD_US;

        if (accIndex < ACC_SAMPLES)
        {
            sampleACC();
        }
    }

    //////////////////////////////////////////////////////
    // One complete second collected
    //////////////////////////////////////////////////////

   if (ecgIndex == ECG_SAMPLES &&
     accIndex == ACC_SAMPLES)
    {
      finishFrame();
    }
}
//////////////////////////////////////////////////////////////
// Write Integer To Buffer (Little Endian)
//////////////////////////////////////////////////////////////
inline void put16(uint8_t *buffer, uint16_t value, uint16_t &index)
{
    buffer[index++] = value & 0xFF;
    buffer[index++] = value >> 8;
}

inline void put32(uint8_t *buffer, uint32_t value, uint16_t &index)
{
    buffer[index++] = value & 0xFF;
    buffer[index++] = (value >> 8) & 0xFF;
    buffer[index++] = (value >> 16) & 0xFF;
    buffer[index++] = (value >> 24) & 0xFF;
}
//////////////////////////////////////////////////////////////
// Calculate Number Of Chunks
//////////////////////////////////////////////////////////////

uint16_t getChunkCount(uint32_t totalBytes)
{
    return (totalBytes + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE;
}

//////////////////////////////////////////////////////////////
// Delay Between BLE Notifications
//////////////////////////////////////////////////////////////
inline void bleDelay()
{
    vTaskDelay(pdMS_TO_TICKS(3));
}
//////////////////////////////////////////////////////////////
// Send One BLE Chunk
//////////////////////////////////////////////////////////////

void sendChunk(
        uint8_t packetType,
        uint32_t secondID,
        uint32_t timestamp,
        uint8_t leadStatus,
        uint16_t chunkIndex,
        uint16_t totalChunks,
        const uint8_t *payload,
        uint16_t payloadLength)
{

    if(!deviceConnected)
        return;

    uint16_t index = 0;

    txBuffer[index++] = packetType;

    put32(txBuffer, secondID, index);

    put32(txBuffer, timestamp, index);
    txBuffer[index++] = leadStatus;

    put16(txBuffer, chunkIndex, index);

    put16(txBuffer, totalChunks, index);

    put16(txBuffer, payloadLength, index);

    memcpy(txBuffer + index,
           payload,
           payloadLength);

    index += payloadLength;

    dataCharacteristic->setValue(txBuffer, index);

    dataCharacteristic->notify();

    bleDelay();

}
//////////////////////////////////////////////////////////////
// Send Binary Buffer
//////////////////////////////////////////////////////////////

void sendBinary(
        uint8_t packetType,
        uint32_t secondID,
        uint32_t timestamp,
        uint8_t leadStatus,
        const uint8_t *data,
        uint32_t length)
{

    uint16_t totalChunks = getChunkCount(length);

    uint32_t sent = 0;

    uint16_t chunk = 0;

    while (sent < length)
    {
        uint16_t size =
            min((uint32_t)PAYLOAD_SIZE,
                length - sent);

        sendChunk(
            packetType,
            secondID,
            timestamp,
            leadStatus,
            chunk,
            totalChunks,
            data + sent,
            size);

        sent += size;

        chunk++;
    }
}
//////////////////////////////////////////////////////////////
// Send ECG Frame
//////////////////////////////////////////////////////////////

void sendECGFrame(uint8_t bufferIndex)
{
    ECGFrame &frame = ecgBuffer[bufferIndex];

    sendBinary(
        PACKET_ECG,
        frame.secondID,
        frame.timestamp,
        frame.leadStatus,
        (uint8_t *)frame.ecg,
        sizeof(frame.ecg)
    );
}
//////////////////////////////////////////////////////////////
// Send ACC Frame
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// Send ACC Frame
//////////////////////////////////////////////////////////////

void sendACCFrame(uint8_t bufferIndex)
{
    ACCFrame &frame = accBuffer[bufferIndex];

    sendBinary(
        PACKET_ACC,
        frame.secondID,
        frame.timestamp,
        0,                      // لا يوجد Lead Status للـ ACC
        (uint8_t *)frame.acc,
        sizeof(frame.acc)
    );
}
//////////////////////////////////////////////////////////////
// Read Battery Voltage
//////////////////////////////////////////////////////////////

float readBatteryVoltage()
{
    uint16_t adc = analogRead(BATTERY_PIN);

    // ESP32 ADC
    float vADC = ((float)adc / 4095.0f) * 3.3f;

    // Voltage Divider 100k / 100k
    return vADC * 2.0f;
}

//////////////////////////////////////////////////////////////
// Battery Percentage
//////////////////////////////////////////////////////////////

uint8_t batteryPercent()
{
    float voltage = readBatteryVoltage();

    if(voltage >= 4.20f)
        return 100;

    if(voltage <= 3.00f)
        return 0;

    return (uint8_t)(((voltage - 3.0f) / 1.2f) * 100.0f);
}
//////////////////////////////////////////////////////////////
// Send Battery
//////////////////////////////////////////////////////////////

void sendBattery()
{
    if(!deviceConnected)
        return;

    uint8_t percent = batteryPercent();

    batteryCharacteristic->setValue(&percent,1);

    batteryCharacteristic->notify();

    Serial.print("Battery : ");

    Serial.print(percent);

    Serial.println("%");
}
//////////////////////////////////////////////////////////////
// LOOP
//////////////////////////////////////////////////////////////

void loop()
{
    //////////////////////////////////////////////////////////
    // Sampling Engine
    //////////////////////////////////////////////////////////

    samplingTask();

    //////////////////////////////////////////////////////////
    // Send One Second Frame
    //////////////////////////////////////////////////////////

    if (frameReady)
    {
        uint8_t bufferIndex;

        portENTER_CRITICAL(&bufferMux);

        bufferIndex = sendBuffer;

        frameReady = false;

        portEXIT_CRITICAL(&bufferMux);

        if (deviceConnected)
        {
            sendECGFrame(bufferIndex);

            sendACCFrame(bufferIndex);

            Serial.print("Frame Sent : ");

            Serial.println(ecgBuffer[bufferIndex].secondID);
        }
    }

    //////////////////////////////////////////////////////////
    // Battery Every Minute
    //////////////////////////////////////////////////////////

    if (millis() - lastBatteryMillis >= BATTERY_PERIOD_MS)
    {
        lastBatteryMillis = millis();

        sendBattery();
    }
}