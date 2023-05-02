#include <TFT_eSPI.h>
#include "SPI.h"
#include <esp_system.h>
#include <esp_random.h>
#include <math.h>
#include <FS.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLE2902.h>

#define SCREEN1 25
#define SCREEN2 33
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite predictedLapTimeSprite = TFT_eSprite(&tft);
TFT_eSprite currentLapTimeSprite = TFT_eSprite(&tft);
TFT_eSprite prevLapTimeSprite = TFT_eSprite(&tft);
// TFT_eSprite bestLapTimeSprite = TFT_eSprite(&tft);

#define RACECHRONO_UUID "00001ff8-0000-1000-8000-00805f9b34fb"
BLEServer *BLE_server = NULL;
BLEService *BLE_service = NULL;
BLECharacteristic *monitorConfigCharacteristic = NULL;
BLECharacteristic *monitorNotificationCharacteristic = NULL;
bool deviceConnected = false;
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
bool isConfigured = false;
struct MonitoringChannel
{
	const char *name;
	const char *formula;
	const float multiplier;
	boolean configuring;
	boolean configured;
};
MonitoringChannel monitoredChannels[] = {
	{"Lap Number", "channel(device(lap), lap_number)", 1.0, false, false},
	{"Lap Time", "channel(device(lap), lap_time)*1000", 0.001, false, false},
	{"Prev time", "channel(device(lap), previous_lap_time)*1000", 0.001, false, false},
	{"Best lap", "channel(device(lap), best_lap_number)", 1.0, false, false},
	{"Best time", "channel(device(lap), best_lap_time)*1000", 0.001, false, false},
	{"Speed Delta", "channel(device(gps), delta_speed)*100", 0.036, false, false},
	{"Time Delta", "channel(device(lap), delta_lap_time)*1000", 0.001, false, false}
	//{"Stint", "channel(device(gps), elapsed_time)",1.0,false,false},
	//{"Speed", "channel(device(gps), speed)*10", 0.36,false,false},
};
bool receivedData = false;
int lapNumber = 0;
float lapTime = 0;
float prevLapTime = 0;
int bestLapNumber = 0;
float bestLapTime = 0;
float prevSpeedDelta = 0;
float speedDelta = 0;
float prevTimeDelta = 0;
float timeDelta = 0;
int newLapMs = 0;
float prevLapTimeDelta = 0;
float predictedLapTime = 0;

void selectScreen(uint8_t screen)
{
	if (screen == SCREEN1)
	{
		digitalWrite(SCREEN2, 1); // turn off
		digitalWrite(SCREEN1, 0); // turn on
	}
	else if (screen == SCREEN2)
	{
		digitalWrite(SCREEN1, 1); // turn off
		digitalWrite(SCREEN2, 0); // turn on
	}
}

void screenOff(uint8_t screen)
{
	digitalWrite(screen, 1); // turn off
}

void screenOn(uint8_t screen)
{
	digitalWrite(screen, 0); // turn on
}

void screensOn()
{
	// all screens on
	digitalWrite(SCREEN1, 0); // turn on
	digitalWrite(SCREEN2, 0); // turn on
}

void screensOff()
{
	// all screens off
	digitalWrite(SCREEN1, 1); // turn off
	digitalWrite(SCREEN2, 1); // turn off
}

void drawWaitingConnection()
{
	screensOn();
	tft.setTextSize(1);
	tft.setTextPadding(tft.textWidth("waiting for configuration...", 4));
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setTextDatum(TC_DATUM);
	tft.drawString("waiting for connection...", 160, 120, 4);
}

void drawWaitingConfiguration()
{
	screensOn();
	tft.setTextSize(1);
	tft.setTextPadding(tft.textWidth("waiting for configuration...", 4));
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setTextDatum(TC_DATUM);
	tft.drawString("waiting for configuration...", 160, 120, 4);
}

void drawWaitingData()
{
	screensOn();
	tft.setTextSize(1);
	tft.setTextPadding(tft.textWidth(" waiting for configuration... ", 4));
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setTextDatum(TC_DATUM);
	tft.drawString("waiting for data...", 160, 120, 4);
}

void clearScreens()
{
	screensOn();
	tft.fillScreen(TFT_BLACK);
}

void drawTimeDelta()
{
	selectScreen(SCREEN1);

	const float value = (prevLapTimeDelta != 0 && newLapMs > 0 && ((millis() - newLapMs) < 5000)) ? prevLapTimeDelta : timeDelta;

	const int color = value >= 0 ? TFT_RED : TFT_GREEN;
	const float deltaAbs = fabs(value);
	const int digits = (deltaAbs >= 1000 ? 0 : (deltaAbs >= 100 ? 1 : (deltaAbs >= 10 ? 2 : 3)));

	tft.setTextSize(1);
	tft.setTextPadding(tft.textWidth("88.88", 8));
	tft.setTextColor(color, TFT_BLACK);
	tft.setTextDatum(CC_DATUM);
	tft.drawFloat(deltaAbs, digits, 160, 45, 8);

	tft.setTextSize(2);
	tft.setTextPadding(20);
	tft.setTextColor(color, TFT_BLACK);
	tft.setTextDatum(CL_DATUM);
	tft.drawString(value >= 0 ? "+" : "-", 5, 45, 4);

	// tft.setTextSize(1);
	// tft.setTextPadding(tft.textWidth("Delta", 2));
	// tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
	// tft.setTextDatum(TL_DATUM);
	// tft.drawString("Delta", 0, 0, 2);
}

#define SPEED_DELTA_MAX 10
void drawSpeedDelta()
{
	selectScreen(SCREEN1);

	const int color = speedDelta < 0 ? TFT_RED : TFT_GREEN;
	const int width = floor(fabs(speedDelta) * (320 / SPEED_DELTA_MAX));
	const int prevColor = prevSpeedDelta < 0 ? TFT_RED : TFT_GREEN;
	const int prevWidth = floor(fabs(prevSpeedDelta) * (320 / SPEED_DELTA_MAX));

	if (color == prevColor)
	{
		// if same color and same width, skip
		if (width == prevWidth)
			return;
		// if same color and width greater, draw difference on each side
		else if (width > prevWidth)
		{
			int diff = width - prevWidth;

			// left
			tft.fillRect(ceil((320 - width) / 2), 90, ceil(diff / 2) + 2, 150, color);
			// rigth
			tft.fillRect(floor(320 / 2 + prevWidth / 2) - 1, 90, ceil(diff / 2) + 1, 150, color);
		}
		// if same color and width smaller, draw black difference on each side
		else
		{
			int diff = prevWidth - width;

			// left
			tft.fillRect(floor((320 - prevWidth) / 2), 90, ceil(diff / 2) + 1, 150, TFT_BLACK);
			// rigth
			tft.fillRect(ceil(320 / 2 + width / 2) - 1, 90, ceil(diff / 2) + 2, 150, TFT_BLACK);
		}
	}
	else
	{
		// if different color, draw over
		tft.fillRect(ceil((320 - width) / 2), 90, width, 150, color);

		// if different color and width smaller, draw black difference on each side
		if (width < prevWidth)
		{
			int diff = prevWidth - width;

			// left
			tft.fillRect(floor((320 - prevWidth) / 2), 90, ceil(diff / 2) + 1, 150, TFT_BLACK);
			// rigth
			tft.fillRect(floor(320 / 2 + width / 2) - 1, 90, ceil(diff / 2) + 2, 150, TFT_BLACK);
		}
	}

	// tft.setTextSize(1);
	// tft.setTextPadding(0);
	// tft.setTextColor(TFT_LIGHTGREY, color);
	// tft.setTextDatum(BC_DATUM);
	// tft.drawFloat(speedDelta, 1, 160, 241, 2);

	// tft.drawLine(160, 90, 160, 240, TFT_WHITE);

	tft.drawLine(0, 210, 0, 240, TFT_WHITE); // max / 1
	tft.drawLine(20, 222, 20, 240, TFT_WHITE);
	tft.drawLine(40, 222, 40, 240, TFT_WHITE);
	tft.drawLine(60, 222, 60, 240, TFT_WHITE);
	tft.drawLine(80, 210, 80, 240, TFT_WHITE); // max / 2
	tft.drawLine(100, 222, 100, 240, TFT_WHITE);
	tft.drawLine(120, 222, 120, 240, TFT_WHITE);
	tft.drawLine(140, 222, 140, 240, TFT_WHITE);
	tft.drawLine(160, 210, 160, 240, TFT_WHITE); // 0
	tft.drawLine(180, 222, 180, 240, TFT_WHITE);
	tft.drawLine(200, 222, 200, 240, TFT_WHITE);
	tft.drawLine(220, 222, 220, 240, TFT_WHITE);
	tft.drawLine(240, 210, 240, 240, TFT_WHITE); // max / 2
	tft.drawLine(260, 222, 260, 240, TFT_WHITE);
	tft.drawLine(280, 222, 280, 240, TFT_WHITE);
	tft.drawLine(300, 222, 300, 240, TFT_WHITE);
	tft.drawLine(319, 210, 319, 240, TFT_WHITE); // max / 1

	tft.setTextSize(1);
	tft.setTextDatum(BL_DATUM);
	tft.setTextColor(TFT_WHITE);
	tft.setTextPadding(0);
	tft.drawNumber(SPEED_DELTA_MAX, 2, 241, 2);		 // max / 1
	tft.drawNumber(SPEED_DELTA_MAX / 2, 85, 241, 2); // max / 2
	tft.drawNumber(0, 165, 241, 2);					 // 0

	// speedDeltaSprite.fillRect(0, 0, 320, 150, TFT_BLACK);
	// speedDeltaSprite.fillRect((320 - width) / 2, 0, width, 150, color);
	// speedDeltaSprite.drawRect(0, 0, 320, 150, TFT_WHITE);
	// speedDeltaSprite.pushSprite(0, 90);
	// tft.pushImageDMA(0, 90, 320, 150, speedDeltaSpritePtr);
}

std::string getFloatTimeString(float time)
{
	const int minutes = floor(time / 60);
	const int seconds = floor(time - minutes * 60);
	const int milliseconds = fmod(time, 1) * 1000;

	// char timeString[11];
	// sprintf(timeString, "%d:%02d.%02d", minutes, seconds, milliseconds);
	// std::string s(timeString);
	// return s;

	char buffer[9];
	snprintf(buffer, sizeof(buffer), "%01d:%02d.%03d", minutes, seconds, milliseconds);

	return std::string(buffer);
}

void drawLapLabels()
{
	selectScreen(SCREEN2);

	tft.setTextSize(1);
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setTextDatum(TL_DATUM);
	tft.setTextPadding(tft.textWidth("Current Lap", 2));
	tft.drawString("Current Lap", 0, 0, 2);

	tft.setTextPadding(tft.textWidth("Predicted Lap", 2));
	tft.drawString("Predicted Lap", 0, 60, 2);

	tft.setTextPadding(tft.textWidth("Prev Lap", 2));
	tft.drawString("Prev Lap", 0, 120, 2);

	tft.setTextPadding(tft.textWidth("Best Lap", 2));
	tft.drawString("Best Lap", 0, 180, 2);
}

void drawLapTime()
{
	selectScreen(SCREEN2);

	const std::string stringTime = getFloatTimeString(lapTime);

	// tft.setTextSize(1);
	// tft.setTextPadding(tft.textWidth("88:88.88", 6));
	// tft.setTextColor(TFT_WHITE, TFT_BLACK);
	// tft.setTextDatum(TL_DATUM);
	// tft.drawString(stringTime.c_str(), 0, 80, 6);

	currentLapTimeSprite.drawString(stringTime.c_str(), 0, 0, 6);
	currentLapTimeSprite.pushSprite(0, 20);

	// drawLapLabels();
}

void drawPredictedLapTime()
{
	selectScreen(SCREEN2);

	const std::string stringTime = getFloatTimeString(predictedLapTime);

	// tft.setTextSize(1);
	// tft.setTextPadding(tft.textWidth("88:88.888", 6));
	// tft.setTextColor(TFT_WHITE, TFT_BLACK);
	// tft.setTextDatum(TL_DATUM);
	// tft.drawString(stringTime.c_str(), 0, 20, 6);

	predictedLapTimeSprite.drawString(stringTime.c_str(), 0, 0, 6);
	predictedLapTimeSprite.pushSprite(0, 80);

	// drawLapLabels();
}

void drawPrevLapTime()
{
	selectScreen(SCREEN2);

	const std::string stringTime = getFloatTimeString(prevLapTime);

	// tft.setTextSize(1);
	// tft.setTextPadding(tft.textWidth("88:88.88", 6));
	// tft.setTextColor(TFT_WHITE, TFT_BLACK);
	// tft.setTextDatum(TL_DATUM);
	// tft.drawString(stringTime.c_str(), 0, 140, 6);

	prevLapTimeSprite.drawString(stringTime.c_str(), 0, 0, 6);
	prevLapTimeSprite.pushSprite(0, 140);

	// drawLapLabels();
}

void drawBestLapTime()
{
	selectScreen(SCREEN2);

	const std::string stringTime = getFloatTimeString(bestLapTime);

	tft.setTextSize(1);
	tft.setTextPadding(tft.textWidth("88:88.88", 6));
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setTextDatum(TL_DATUM);
	tft.drawString(stringTime.c_str(), 0, 200, 6);

	// tft.drawString(stringTime.c_str(), 0, 200, 6);
	// bestLapTimeSprite.pushSprite(0, 200);

	// drawLapLabels();
}

/* ----------- */

// Define event group handle
EventGroupHandle_t displayEventGroup;

// Define event bits
#define EVENT_LAPTIME (1 << 0)
#define EVENT_TIMEDELTA (1 << 1)
#define EVENT_SPEEDDELTA (1 << 2)
#define EVENT_WAITINGCONNECTION (1 << 3)
#define EVENT_WAITINGCONFIG (1 << 4)
#define EVENT_WAITINGDATA (1 << 5)
#define EVENT_PREDICTEDLAPTIME (1 << 6)
#define EVENT_PREVLAPTIME (1 << 7)
#define EVENT_BESTLAPTIME (1 << 8)
#define EVENT_LAPLABELS (1 << 9)
#define EVENT_CLEAR (1 << 10)

// Task that will run on core 1
void displayTask(void *parameter)
{
	while (1)
	{
		// Wait for any of the event bits to be set
		EventBits_t bits = xEventGroupWaitBits(displayEventGroup,
											   EVENT_LAPTIME | EVENT_TIMEDELTA | EVENT_SPEEDDELTA | EVENT_WAITINGCONNECTION | EVENT_WAITINGCONFIG |
												   EVENT_WAITINGDATA | EVENT_PREDICTEDLAPTIME | EVENT_PREVLAPTIME | EVENT_BESTLAPTIME | EVENT_LAPLABELS |
												   EVENT_CLEAR,
											   pdTRUE, pdFALSE, portMAX_DELAY);

		if (bits & EVENT_WAITINGCONNECTION)
		{
			clearScreens();
			drawWaitingConnection();
		}
		if (bits & EVENT_WAITINGCONFIG)
		{
			drawWaitingConfiguration();
		}
		if (bits & EVENT_WAITINGDATA)
		{
			drawWaitingData();
		}
		if (bits & EVENT_LAPTIME)
		{
			drawLapTime();
		}
		if (bits & EVENT_TIMEDELTA)
		{
			drawTimeDelta();
		}
		if (bits & EVENT_SPEEDDELTA)
		{
			drawSpeedDelta();
		}
		if (bits & EVENT_PREDICTEDLAPTIME)
		{
			drawPredictedLapTime();
		}
		if (bits & EVENT_PREVLAPTIME)
		{
			drawPrevLapTime();
		}
		if (bits & EVENT_BESTLAPTIME)
		{
			drawBestLapTime();
		}
		if (bits & EVENT_LAPLABELS)
		{
			drawLapLabels();
		}
		if (bits & EVENT_CLEAR)
		{
			clearScreens();
		}
	}
}

/* ------------------------ */

void sendConfigCommand(int cmdType, int monitorId, const char *payload, int payloadSequence = 0)
{
	if (!monitoredChannels[monitorId].configuring)
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
		const int payloadLen = strlen(payload);
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
	delay(1);

	// Handle remaining payload
	if (remainingPayload)
	{
		sendConfigCommand(CMD_TYPE_ADD, monitorId, remainingPayload, payloadSequence + 1);
		free(remainingPayload);
	}
	else
	{
		Serial.print(monitoredChannels[monitorId].name);
		Serial.println(" configured");
		monitoredChannels[monitorId].configuring = false;
	}
}

void sendUpdateCommand(int monitorId = -1)
{
	// monitorId < 0 means update all
	byte bytes[2];
	if (monitorId < 0)
		bytes[0] = (byte)4;
	else
	{
		bytes[0] = (byte)4;
		bytes[1] = (byte)monitorId;
	}

	monitorConfigCharacteristic->setValue(bytes, 2);
	monitorConfigCharacteristic->indicate();
}

void checkChannelsConfigured()
{
	bool configured = true;

	// go through channels, check if any not configured
	for (int i = 0; i < (sizeof(monitoredChannels) / sizeof(monitoredChannels[0])) && i < MONITORS_MAX; i++)
	{
		if (monitoredChannels[i].configuring || !monitoredChannels[i].configured)
		{
			// Serial.print(monitoredChannels[i].name);
			// Serial.println(" NOT CONFIGURED");
			configured = false;
		}

		if (!monitoredChannels[i].configured && !monitoredChannels[i].configuring)
		{
			Serial.print("add monitor : ");
			Serial.println(monitoredChannels[i].name);
			monitoredChannels[i].configuring = true;
			sendConfigCommand(CMD_TYPE_ADD, i, monitoredChannels[i].formula);
			delay(1);
		}
	}

	if (configured)
	{
		Serial.println("--CONFIGURED--");
		isConfigured = configured;
		if (!receivedData)
		{
			drawWaitingData();
			// xEventGroupSetBits(displayEventGroup, EVENT_WAITINGDATA);
		}
		sendUpdateCommand();
	}
}

class ConfigCallbacks : public BLECharacteristicCallbacks
{
	void onStatus(BLECharacteristic *pCharacteristic, Status s, uint32_t code)
	{
		const std::string value = pCharacteristic->getValue();
		// for (int i = 0; i < value.length(); i++)
		// {
		// 	Serial.println((int)value[i]);
		// }
		const int command = (int)value[0];
		const int monitorId = (int)value[1];
		Serial.print(command == 4 ? "Update All" : monitoredChannels[monitorId].name);
		Serial.print(" : ");

		switch (s)
		{
		case SUCCESS_NOTIFY:
			Serial.println("SUCCESS_NOTIFY");
			break;
		case SUCCESS_INDICATE:
			Serial.println("SUCCESS_INDICATE");
			break;
		case ERROR_INDICATE_DISABLED:
			Serial.println("ERROR_INDICATE_DISABLED");
			monitoredChannels[monitorId].configured = false;
			monitoredChannels[monitorId].configuring = false;
			break;
		case ERROR_NOTIFY_DISABLED:
			Serial.println("ERROR_NOTIFY_DISABLED");
			monitoredChannels[monitorId].configured = false;
			monitoredChannels[monitorId].configuring = false;
			break;
		case ERROR_GATT:
			Serial.println("ERROR_GATT");
			monitoredChannels[monitorId].configured = false;
			monitoredChannels[monitorId].configuring = false;
			break;
		case ERROR_NO_CLIENT:
			Serial.println("ERROR_NO_CLIENT");
			monitoredChannels[monitorId].configured = false;
			monitoredChannels[monitorId].configuring = false;
			break;
		case ERROR_INDICATE_TIMEOUT:
			Serial.println("indicate timeout");
			monitoredChannels[monitorId].configured = false;
			monitoredChannels[monitorId].configuring = false;
			break;
		case ERROR_INDICATE_FAILURE:
			Serial.println("indicate failure");
			monitoredChannels[monitorId].configured = false;
			monitoredChannels[monitorId].configuring = false;
			break;

		default:
			break;
		}
	};

	void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param)
	{
		if (true || pCharacteristic->getLength() > 0)
		{
			std::string value = pCharacteristic->getValue();
			const int result = (int)value[0];
			const int monitorId = (int)value[1];
			Serial.print(monitoredChannels[monitorId].name);
			Serial.print(" : ");

			switch (result)
			{
			case 0:
				Serial.println("success");
				monitoredChannels[monitorId].configured = true;
				monitoredChannels[monitorId].configuring = false;
				break;

			case CMD_RESULT_PAYLOAD_OUT_OF_SEQUENCE:
				Serial.println("out-of-sequence");
				monitoredChannels[monitorId].configured = false;
				monitoredChannels[monitorId].configuring = false;
				break;

			case CMD_RESULT_EQUATION_EXCEPTION:
				Serial.println("exception");
				break;

			default:
				Serial.println(result);
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

			std::string value = pCharacteristic->getValue();
			int dataPos = 0;
			while (dataPos + 5 <= pCharacteristic->getLength())
			{
				int monitorId = (int)value[dataPos];

				int32_t data = value[dataPos + 1] << 24 | value[dataPos + 2] << 16 | value[dataPos + 3] << 8 | value[dataPos + 4];
				if (monitorId < sizeof monitoredChannels && data < 2147483647)
				{
					if (receivedData == false)
					{
						receivedData = true;
						clearScreens();
						drawLapLabels();
						// xEventGroupSetBits(displayEventGroup, EVENT_CLEAR);
						// xEventGroupSetBits(displayEventGroup, EVENT_LAPLABELS);
					}

					float value = (float)data * monitoredChannels[monitorId].multiplier;
					Serial.print(monitoredChannels[monitorId].name);
					Serial.print("(");
					Serial.print(monitorId);
					Serial.print(")");
					Serial.print(" : ");
					Serial.print(value);
					Serial.println();

					switch (monitorId)
					{
					// 0 - Lap Number
					case 0:
						lapNumber = value;
						break;

					// 1 - Lap Time
					case 1:
						lapTime = value;
						drawLapTime();
						// xEventGroupSetBits(displayEventGroup, EVENT_LAPTIME);
						break;

					// 2 - Prev time
					case 2:
						if (prevLapTime > 0 && bestLapTime > 1)
							newLapMs = millis();
						prevLapTime = value;
						if (prevLapTime != bestLapTime)
							prevLapTimeDelta = prevLapTime - bestLapTime;

						drawPrevLapTime();
						// xEventGroupSetBits(displayEventGroup, EVENT_PREVLAPTIME);
						break;

					// 3 - Best lap
					case 3:
						bestLapNumber = value;

						break;

					// 4 - Best time
					case 4:
						if (value > 1)
						{
							bestLapTime = value;
							if (prevLapTime != bestLapTime)
								prevLapTimeDelta = prevLapTime - bestLapTime;
							drawBestLapTime();
							// xEventGroupSetBits(displayEventGroup, EVENT_BESTLAPTIME);

							predictedLapTime = bestLapTime + timeDelta;
							drawPredictedLapTime();
							// xEventGroupSetBits(displayEventGroup, EVENT_PREDICTEDLAPTIME);
						}
						break;

					// 5 - Speed Delta
					case 5:
						prevSpeedDelta = speedDelta;
						speedDelta = value;
						drawSpeedDelta();
						// xEventGroupSetBits(displayEventGroup, EVENT_SPEEDDELTA);

						break;

					// 6 - Time Delta
					case 6:
						prevTimeDelta = timeDelta;
						timeDelta = value;
						// if (newLapMs > 0 && (millis() - newLapMs < 5000))
						// 	drawTimeDelta();
						// else
						drawTimeDelta();
						// xEventGroupSetBits(displayEventGroup, EVENT_TIMEDELTA);

						predictedLapTime = bestLapTime + timeDelta;
						drawPredictedLapTime();
						// xEventGroupSetBits(displayEventGroup, EVENT_PREDICTEDLAPTIME);

						break;
					}
				}
				dataPos += 5;
				delay(1);
			}
		}
	};
};

class ServerCallbacks : public BLEServerCallbacks
{
	void onConnect(BLEServer *BLE_server)
	{
		// delay(1000);

		deviceConnected = true;
		Serial.println("[I] Bluetooth client connected!");
		drawWaitingConfiguration();
		// xEventGroupSetBits(displayEventGroup, EVENT_WAITINGCONFIG);
	};

	void onDisconnect(BLEServer *BLE_server)
	{
		deviceConnected = false;
		isConfigured = false;
		receivedData = false;
		for (int i = 0; i < (sizeof(monitoredChannels) / sizeof(monitoredChannels[0])); i++)
		{
			monitoredChannels[i].configured = false;
			monitoredChannels[i].configuring = false;
		}
		Serial.println("[I] Bluetooth client disconnected!");
		delay(100);
		BLE_server->startAdvertising();
		Serial.println("[I] Bluetooth device discoverable");
		clearScreens();
		drawWaitingConnection();
		// xEventGroupSetBits(displayEventGroup, EVENT_WAITINGCONNECTION);
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
	// Initialize Serial port
	Serial.begin(115200);

	configBLE();

	pinMode(SCREEN1, OUTPUT);
	pinMode(SCREEN2, OUTPUT);
	screensOn();

	// Initialize TFT display
	tft.init();
	tft.setRotation(1);
	tft.fillScreen(TFT_BLACK);

	predictedLapTimeSprite.createSprite(195, 40);
	predictedLapTimeSprite.setTextSize(1);
	predictedLapTimeSprite.setTextPadding(tft.textWidth("88:88.888", 6));
	predictedLapTimeSprite.setTextColor(TFT_WHITE, TFT_BLACK);
	predictedLapTimeSprite.setTextDatum(TL_DATUM);

	currentLapTimeSprite.createSprite(195, 40);
	currentLapTimeSprite.setTextSize(1);
	currentLapTimeSprite.setTextPadding(tft.textWidth("88:88.888", 6));
	currentLapTimeSprite.setTextColor(TFT_WHITE, TFT_BLACK);
	currentLapTimeSprite.setTextDatum(TL_DATUM);

	prevLapTimeSprite.createSprite(195, 40);
	prevLapTimeSprite.setTextSize(1);
	prevLapTimeSprite.setTextPadding(tft.textWidth("88:88.888", 6));
	prevLapTimeSprite.setTextColor(TFT_WHITE, TFT_BLACK);
	prevLapTimeSprite.setTextDatum(TL_DATUM);

	// bestLapTimeSprite.createSprite(165, 40);
	// bestLapTimeSprite.setTextSize(1);
	// bestLapTimeSprite.setTextPadding(tft.textWidth("88:88.888", 6));
	// bestLapTimeSprite.setTextColor(TFT_WHITE, TFT_BLACK);
	// bestLapTimeSprite.setTextDatum(TL_DATUM);

	// displayEventGroup = xEventGroupCreate();
	//  Create the task and pin it to core 1
	// xTaskCreatePinnedToCore(displayTask, "displayTask", 10000, NULL, 5, NULL, 1);
	// xEventGroupSetBits(displayEventGroup, EVENT_WAITINGCONNECTION);
	drawWaitingConnection();
}

void loop()
{
	if (deviceConnected)
	{
		if (!isConfigured)
		{
			checkChannelsConfigured();
		}
	}

	// uint32_t random_int = esp_random() % 10000;				 // Generate a random integer between 0 and 9999
	// float delta = (((float)random_int / 5000.0) - 1.0) * 10; // Convert the integer to a float between -1 and 1, then scale to -5 to 5
	// prevSpeedDelta = -0.5;
	// speedDelta = 0.25;

	// Serial.print("delta:");
	// Serial.print(speedDelta, 3);
	// Serial.print("(prev:");
	// Serial.print(prevSpeedDelta, 3);
	// Serial.println(")");

	// drawSpeedDelta();

	// delay(1000);
}
