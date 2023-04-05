#include <Arduino.h>
#include <BLEDevice.h>
#include <BLE2902.h>

#define LED 2
boolean led = false;

#define RACECHRONO_UUID "00001ff8-0000-1000-8000-00805f9b34fb"

BLEServer *BLE_server = NULL;
BLEService *BLE_service = NULL;
BLECharacteristic *monitorConfigCharacteristic = NULL;
BLECharacteristic *monitorNotificationCharacteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;

#define CMD_TYPE_REMOVE_ALL 0
#define CMD_TYPE_REMOVE 1
#define CMD_TYPE_ADD_INCOMPLETE 2
#define CMD_TYPE_ADD 3
#define CMD_TYPE_UPDATE_ALL 4
#define CMD_TYPE_UPDATE 5

#define CMD_RESULT_OK 0
#define CMD_RESULT_PAYLOAD_OUT_OF_SEQUENCE 1
#define CMD_RESULT_EQUATION_EXCEPTION 2

#define MAX_REMAINING_PAYLOAD 2048
#define MAX_PAYLOAD_PART 17
#define MONITOR_NAME_MAX 32
#define MONITORS_MAX 255

char monitorNames[MONITORS_MAX][MONITOR_NAME_MAX + 1];
float monitorMultipliers[MONITORS_MAX];
int32_t monitorValues[MONITORS_MAX];
int nextMonitorId = 0;

bool isConfiguring = false;
bool isConfigured = false;

String monitoredChannels[][3] = {
	//{"Stint", "channel(device(gps), elapsed_time)",1.0},
	{"Curr lap", "channel(device(lap), lap_number)", "1.0"},
	{"Curr time", "channel(device(lap), lap_time)*100", "0.01"},
	{"Prev lap", "channel(device(lap), previous_lap_number)", "1.0"},
	{"Prev time", "channel(device(lap), previous_lap_time)*100", "0.01"},
	{"Best lap", "channel(device(lap), best_lap_number)", "1.0"},
	{"Best time", "channel(device(lap), best_lap_time)*100", "0.01"},
	//{"Speed", "channel(device(gps), speed)*10", "0.36"},
	{"Delta Speed", "channel(device(gps), delta_speed)*10", "0.36"},
	{"Delta Time", "channel(device(lap), delta_lap_time)*100", "0.01"}};

void sendConfigCommand(int cmdType, int monitorId, const char *payload, int payloadSequence = 0)
{
	if (!isConfiguring)
		return;

	// Initially use CMD_TYPE_ADD instead of CMD_TYPE_ADD_INCOMPLETE
	cmdType = cmdType == CMD_TYPE_ADD_INCOMPLETE ? CMD_TYPE_ADD : cmdType;

	// Figure out payload
	char *remainingPayload = NULL;
	char payloadPart[MAX_PAYLOAD_PART + 1];
	if (payload && cmdType == CMD_TYPE_ADD)
	{
		// Copy first 17 characters to payload
		strncpy(payloadPart, payload, MAX_PAYLOAD_PART);
		payloadPart[MAX_PAYLOAD_PART] = '\0';

		// If it does not fit to one payload, save the remaining part
		int payloadLen = strlen(payload);
		if (payloadLen > MAX_PAYLOAD_PART)
		{
			int remainingPayloadLen = payloadLen - MAX_PAYLOAD_PART;
			remainingPayload = (char *)malloc(remainingPayloadLen + 1);
			strncpy(remainingPayload, payload + MAX_PAYLOAD_PART, remainingPayloadLen);
			remainingPayload[remainingPayloadLen] = '\0';
			cmdType = CMD_TYPE_ADD_INCOMPLETE;
		}
	}
	else
	{
		payloadPart[0] = '\0';
	}

	// Indicate the characteristic
	byte bytes[20];
	bytes[0] = (byte)cmdType;
	bytes[1] = (byte)monitorId;
	bytes[2] = (byte)payloadSequence;
	memcpy(bytes + 3, payloadPart, strlen(payloadPart));

	// Serial.print("indicate ");
	// for (int i = 0; i < (3 + strlen(payloadPart)); i++)
	// {
	// 	Serial.print(bytes[i]);
	// }
	// Serial.println();

	monitorConfigCharacteristic->setValue(bytes, 3 + strlen(payloadPart));
	monitorConfigCharacteristic->indicate();
	// monitorConfigCharacteristic->register
	delay(10);

	// Handle remaining payload
	if (remainingPayload)
	{
		sendConfigCommand(CMD_TYPE_ADD, monitorId, remainingPayload, payloadSequence + 1);
		free(remainingPayload);
	}
}

void addMonitor(const char *monitorName, const char *filterDef, float multiplier)
{
	if (!isConfiguring)
		return;

	if (nextMonitorId < MONITORS_MAX)
	{
		sendConfigCommand(CMD_TYPE_ADD, nextMonitorId, filterDef);
		strncpy(monitorNames[nextMonitorId], monitorName, MONITOR_NAME_MAX);
		monitorNames[nextMonitorId][MONITOR_NAME_MAX] = '\0';
		monitorMultipliers[nextMonitorId] = multiplier;
		nextMonitorId++;
	}
}

void configureMonitor()
{
	isConfiguring = true;
	nextMonitorId = 0;

	while (isConfiguring)
	{
		for (int i = 0; i < sizeof monitoredChannels / sizeof monitoredChannels[0]; i++)
		{
			addMonitor(monitoredChannels[i][0].c_str(), monitoredChannels[i][1].c_str(), std::stof(monitoredChannels[i][2].c_str()));
		}
		if (isConfiguring)
			isConfigured = true;
		isConfiguring = false;
	}
}

class ConfigCallbacks : public BLECharacteristicCallbacks
{
	void onStatus(BLECharacteristic *pCharacteristic, Status s, uint32_t code)
	{
		switch (s)
		{
		case SUCCESS_NOTIFY:
			// Serial.println("SUCCESS_NOTIFY");
			break;
		case SUCCESS_INDICATE:
			// Serial.println("indicate success");
			break;

		case ERROR_INDICATE_DISABLED:
			Serial.println("ERROR_INDICATE_DISABLED");
			isConfiguring = false;
			isConfigured = false;
			break;
		case ERROR_NOTIFY_DISABLED:
			Serial.println("ERROR_NOTIFY_DISABLED");
			isConfiguring = false;
			isConfigured = false;
			break;
		case ERROR_GATT:
			Serial.println("ERROR_GATT");
			isConfiguring = false;
			isConfigured = false;
			break;
		case ERROR_NO_CLIENT:
			Serial.println("ERROR_NO_CLIENT");
			isConfiguring = false;
			isConfigured = false;
			break;
		case ERROR_INDICATE_TIMEOUT:
			Serial.println("indicate timeout");
			isConfiguring = false;
			isConfigured = false;
			break;
		case ERROR_INDICATE_FAILURE:
			Serial.println("indicate failure");
			isConfiguring = false;
			isConfigured = false;
			break;

		default:
			break;
		}
	};

	void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param)
	{
		if (pCharacteristic->getLength() > 0)
		{
			auto value = pCharacteristic->getValue();

			int result = (int)value[0];
			int monitorId = (int)value[1];
			switch (result)
			{
			case CMD_RESULT_PAYLOAD_OUT_OF_SEQUENCE:
				Serial.print(monitoredChannels[monitorId][0].c_str());
				Serial.print(" : ");
				Serial.println("out-of-sequence");
				break;
			case CMD_RESULT_EQUATION_EXCEPTION:
				Serial.print(monitoredChannels[monitorId][0].c_str());
				Serial.print(" : ");
				Serial.println("exception");
				break;
			default:
				break;
			}
		}
	};
};

class MonitorCallbacks : public BLECharacteristicCallbacks
{
	void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param)
	{

		if (pCharacteristic->getLength() > 0)
		{
			// Serial.println("Received Monitor Value: ");

			auto value = pCharacteristic->getValue();
			int dataPos = 0;
			while (dataPos + 5 <= pCharacteristic->getLength())
			{
				int monitorId = (int)value[dataPos];

				int32_t data = value[dataPos + 1] << 24 | value[dataPos + 2] << 16 | value[dataPos + 3] << 8 | value[dataPos + 4];
				if (monitorId < nextMonitorId && data < 2147483647)
				{

					Serial.print(monitoredChannels[monitorId][0].c_str());
					Serial.print(" : ");
					Serial.print((float)data * std::stof(monitoredChannels[monitorId][2].c_str()));
					Serial.println();
					// Serial.println(data);
					// monitorValues[monitorId] = value;
				}
				dataPos += 5;
			}
		}
	};
};

class ServerCallbacks : public BLEServerCallbacks
{
	void onConnect(BLEServer *BLE_server)
	{
		// delay(2000);

		deviceConnected = true;
		Serial.println("[I] Bluetooth client connected!");
	};

	void onDisconnect(BLEServer *BLE_server)
	{
		deviceConnected = false;
		Serial.println("[I] Bluetooth client disconnected!");
	}
};

// BLE configuration
void configBLE()
{
	BLEDevice::init("RC_DYI_MONITOR_AP");

	BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);
	BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);
	BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_SCAN);

	BLE_server = BLEDevice::createServer();
	BLE_server->setCallbacks(new ServerCallbacks());
	BLE_service = BLE_server->createService(RACECHRONO_UUID);

	monitorConfigCharacteristic = BLE_service->createCharacteristic(
		BLEUUID((uint16_t)0x5), BLECharacteristic::PROPERTY_INDICATE |
									BLECharacteristic::PROPERTY_WRITE);
	monitorConfigCharacteristic->addDescriptor(new BLE2902());
	monitorConfigCharacteristic->setCallbacks(new ConfigCallbacks());
	// monitorConfigCharacteristic->set
	const uint8_t indicationOn[] = {0x2, 0x0};
	monitorConfigCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue((uint8_t *)indicationOn, 2);
	monitorConfigCharacteristic->setIndicateProperty(true);

	monitorNotificationCharacteristic = BLE_service->createCharacteristic(
		BLEUUID((uint16_t)0x6), BLECharacteristic::PROPERTY_WRITE_NR);
	monitorNotificationCharacteristic->addDescriptor(new BLE2902());
	monitorNotificationCharacteristic->setCallbacks(new MonitorCallbacks());

	BLE_service->start();

	BLEAdvertising *BLE_advertising = BLEDevice::getAdvertising();
	BLE_advertising->addServiceUUID(RACECHRONO_UUID);
	BLE_advertising->setScanResponse(false);
	BLE_advertising->setMinInterval(50);
	BLE_advertising->setMaxInterval(100);
	BLEDevice::startAdvertising();
}

void setup()
{
	Serial.begin(115200);
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	configBLE();
}

uint32_t value = 0;

void loop()
{

	if (deviceConnected)
	{
		if (!isConfigured && !isConfiguring)
		{
			configureMonitor();
		}

		if (isConfigured)
		{
			// getUpdatedValues();
			// delay(1000);
		}
		// monitorConfigCharacteristic->setValue((uint8_t *)&value, 4);
		// monitorConfigCharacteristic->indicate();
		// value++;
		// delay(20); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
	}

	// disconnected
	if (!deviceConnected && oldDeviceConnected)
	{
		delay(500);
		BLE_server->startAdvertising();
		Serial.println("[I] Bluetooth device discoverable");
		oldDeviceConnected = deviceConnected;
		isConfigured = false;
	}
	// connected
	if (deviceConnected && !oldDeviceConnected)
	{
		oldDeviceConnected = deviceConnected;
	}
}