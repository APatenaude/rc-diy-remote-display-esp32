#include "SPI.h"
#include <esp_system.h>
#include <esp_random.h>
#include <math.h>
#include <FS.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <TJpg_Decoder.h>
#include "SPIFFS.h"
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <time.h>
#define LGFX_LOLIN_D32_PRO
#include <config.hpp>
#include <LovyanGFX.hpp>
#include <LGFX_TFT_eSPI.hpp>

#pragma region Data

int lapNumber = 0;
float lapTime = 0;
float prevLapTime = 0;
int bestLapNumber = 0;
float bestLapTime = 0;
float prevSpeedDelta = 0;
float speedDelta = 0;
float prevTimeDelta = 0;
float timeDelta = 0;
float predictedLapTime = 0;
float comparisonLapTime = 0;
bool usingComparisonLap = false;
int satellites = 0;
unsigned long currentTime = 0;
unsigned long previousBatteryIntervalTime = 0;
unsigned long previousTimeIntervalTime = 0;
unsigned long startTime;

float getFastestTime() { return usingComparisonLap && comparisonLapTime > 0 && (bestLapTime == 0 || comparisonLapTime < bestLapTime) ? comparisonLapTime : bestLapTime; }
float getComparisonOrBest() { return usingComparisonLap && comparisonLapTime > 0 ? comparisonLapTime : bestLapTime; }
float isBestFasterThanComparison() { return usingComparisonLap && comparisonLapTime > 0 && bestLapTime > 0 && bestLapTime < comparisonLapTime; }

#pragma endregion Data

#pragma region Config

#define SCREEN1 33
#define SCREEN2 25
#define VBAT_PIN 34
#define RACECHRONO_UUID "00001ff8-0000-1000-8000-00805f9b34fb"
struct MonitoringChannel
{
	bool enabled;
	const char *name;
	const char *formula;
	const float multiplier;
	bool configuring;
	bool configured;
};
MonitoringChannel monitoredChannels[] = {
	/* 0 */ {true, "Lap Number", "channel(device(lap), lap_number)", 1.0, false, false},
	/* 1 */ {true, "Lap Time", "channel(device(lap), lap_time)*1000", 0.001, false, false},
	/* 2 */ {true, "Prev time", "channel(device(lap), previous_lap_time)*1000", 0.001, false, false},
	/* 3 */ {true, "Best lap", "channel(device(lap), best_lap_number)", 1.0, false, false},
	/* 4 */ {true, "Best time", "channel(device(lap), best_lap_time)*1000", 0.001, false, false},
	/* 5 */ {true, "Speed Delta", "channel(device(calc), delta_speed)*100", 0.036, false, false}, // 3.6 multiplier to convert from m/s to km/h
	/* 6 */ {true, "Time Delta", "channel(device(lap), delta_lap_time)*1000", 0.001, false, false},
	/* 7 */ {true, "Comparison time", "channel(device(lap), comparison_lap_time)*1000", 0.001, false, false},
	/* 8 */ {false, "Update Rate", "channel(device(gps), device_update_rate)*10", 0.1, false, false},
	/* 9 */ {true, "Satellites", "channel(device(gps), satellites)", 1.0, false, false},
	/* 10 */ //{false, "Stint", "channel(device(gps), elapsed_time)", 1.0, false, false},
	/* 11 */ //{false, "Speed", "channel(device(gps), speed)*10", 0.36, false, false}, // 3.6 multiplier to convert from m/s to km/h
};
#define SPEED_DELTA_MAX 8
#define SPEED_DELTA_Y 120
const unsigned long batteryLevelDrawInterval = 3000;
const unsigned long stintTimeDrawInterval = 250;
#define SPEED_LINES_COLOR 0xB4B4B4
#define SCREEN2TEXTCOLOR TFT_LIGHTGRAY
const int vref = 1100;
const float voltageCorrectionFactor = 0.975;

#pragma endregion Config

#pragma region Display

TFT_eSPI tft = TFT_eSPI();

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h,
				uint16_t *bitmap)
{
	if (y >= tft.height())
		return 0;
	tft.pushImage(x, y, w, h, bitmap);
	return 1;
}

void selectScreen(uint8_t screen)
{
	if (screen == SCREEN1)
	{
		digitalWrite(SCREEN1, 0); // turn on
		digitalWrite(SCREEN2, 1); // turn off
	}
	else if (screen == SCREEN2)
	{
		digitalWrite(SCREEN1, 1); // turn off
		digitalWrite(SCREEN2, 0); // turn on
	}
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

#define STATUS_CONNECTION 1
#define STATUS_CONFIGURATION 2
#define STATUS_DATA 3
int status = 0;
int statusYPos = 1000;
int statusDrawYInterval = 60;
void drawStatus(bool increaseInterval = false, bool clearPrevious = true, int yPos = -1)
{
	if (status == 0)
		return;

	int textWidth = tft.textWidth("waiting for configuration", &fonts::Font4);

	screensOn();
	tft.setTextSize(1);
	tft.setTextPadding(textWidth);
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setTextDatum(TC_DATUM);

	if (yPos >= 0)
	{
		if (yPos >= tft.height())
		{
			yPos = yPos - tft.height();
			selectScreen(SCREEN2);
		}
		else
			selectScreen(SCREEN1);
		statusYPos = yPos;
	}
	// move the status around to prevent image retention
	else if (increaseInterval)
	{

		if (clearPrevious)
		{
			tft.fillRect(tft.width() / 2 - textWidth / 2, statusYPos, textWidth, 30, TFT_BLACK);
			tft.setTextColor(TFT_WHITE, TFT_BLACK);
		}

		if (statusYPos + statusDrawYInterval > tft.height() - statusDrawYInterval)
			statusYPos = 10;
		else
			statusYPos += statusDrawYInterval;
	}

	switch (status)
	{
	case 1:
		tft.drawString("waiting for connection", tft.width() / 2, statusYPos, &fonts::Font4);
		break;
	case 2:
		tft.drawString("waiting for configuration", tft.width() / 2, statusYPos, &fonts::Font4);
		break;
	case 3:
		tft.drawString("waiting for data", tft.width() / 2, statusYPos, &fonts::Font4);
		break;

	default:
		break;
	}
}

void clearScreens()
{
	screensOn();
	tft.fillScreen(TFT_BLACK);
}

void drawTimeDelta(float value)
{
	selectScreen(SCREEN1);

	const int color = value >= 0 ? TFT_RED : TFT_GREEN;
	const float deltaAbs = fabs(value);
	const int digits = (deltaAbs >= 1000 ? 0 : (deltaAbs >= 100 ? 1 : (deltaAbs >= 10 ? 2 : 3)));

	tft.setTextSize(1.5);
	tft.setTextPadding(tft.textWidth("88.88", &fonts::Font8));
	tft.setTextColor(color, TFT_BLACK);
	tft.setTextDatum(TC_DATUM);
	tft.drawFloat(deltaAbs, digits, tft.width() / 2, 2, &fonts::Font8);

	tft.setTextSize(3);
	tft.setTextPadding(tft.textWidth("+", &fonts::Font4));
	tft.setTextColor(color, TFT_BLACK);
	tft.setTextDatum(CL_DATUM);
	tft.drawString(value >= 0 ? "+" : "-", 2, 65, &fonts::Font4);
}
void drawTimeDelta()
{
	drawTimeDelta(timeDelta);
}

void drawSpeedDelta(float value, float prevValue)
{
	selectScreen(SCREEN1);

	// could use a sprite instead of this algorithm to avoid flickering but at ~240x480 it would take up a lot of ram

	const int color = value < 0 ? TFT_RED : TFT_GREEN;
	const int width = floor(fabs(value) * (tft.width() / SPEED_DELTA_MAX));
	const int prevColor = prevValue < 0 ? TFT_RED : TFT_GREEN;
	const int prevWidth = floor(fabs(prevValue) * (tft.width() / SPEED_DELTA_MAX));

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
			tft.fillRect(ceil((tft.width() - width) / 2), SPEED_DELTA_Y, ceil(diff / 2) + 2, tft.height() - SPEED_DELTA_Y, color);
			// rigth
			tft.fillRect(floor(tft.width() / 2 + prevWidth / 2) - 1, SPEED_DELTA_Y, ceil(diff / 2) + 1, tft.height() - SPEED_DELTA_Y, color);
		}
		// if same color and width smaller, draw black difference on each side
		else
		{
			int diff = prevWidth - width;

			// left
			tft.fillRect(floor((tft.width() - prevWidth) / 2), SPEED_DELTA_Y, ceil(diff / 2) + 1, tft.height() - SPEED_DELTA_Y, TFT_BLACK);
			// rigth
			tft.fillRect(ceil(tft.width() / 2 + width / 2), SPEED_DELTA_Y, ceil(diff / 2) + 2, tft.height() - SPEED_DELTA_Y, TFT_BLACK);
		}
	}
	else
	{
		// if different color, draw over
		tft.fillRect(ceil((tft.width() - width) / 2), SPEED_DELTA_Y, width, tft.height() - SPEED_DELTA_Y, color);

		// if different color and width smaller, draw black difference on each side
		if (width < prevWidth)
		{
			int diff = prevWidth - width;

			// left
			tft.fillRect(floor((tft.width() - prevWidth) / 2), SPEED_DELTA_Y, ceil(diff / 2) + 1, tft.height() - SPEED_DELTA_Y, TFT_BLACK);
			// rigth
			tft.fillRect(floor(tft.width() / 2 + width / 2), SPEED_DELTA_Y, ceil(diff / 2) + 2, tft.height() - SPEED_DELTA_Y, TFT_BLACK);
		}
	}

	int sections = SPEED_DELTA_MAX * 2;
	int spacing = tft.width() / sections;

	for (int i = 0; i <= sections; i++)
	{
		tft.drawLine(i * spacing - (i == sections ? 1 : i == 0 ? -1
															   : 0),
					 tft.height() - (i % (sections / 4) == 0 ? 20 : 12),
					 i * spacing - (i == sections ? 1 : i == 0 ? -1
															   : 0),
					 tft.height(), SPEED_LINES_COLOR);
	}

	tft.setTextSize(1);
	tft.setTextColor(SPEED_LINES_COLOR);
	tft.setTextPadding(0);
	tft.setTextDatum(BL_DATUM);
	tft.drawNumber(SPEED_DELTA_MAX, 1, tft.height() - 20 + 1, &fonts::Font2); // -SPEED_DELTA_MAX
	tft.setTextDatum(BR_DATUM);
	tft.drawNumber(SPEED_DELTA_MAX, tft.width() + 1, tft.height() - 20 + 1, &fonts::Font2); // +SPEED_DELTA_MAX
	tft.setTextDatum(BC_DATUM);
	tft.drawNumber(0, tft.width() / 2 + 1, tft.height() - 20 + 1, &fonts::Font2);								 // 0
	tft.drawNumber(SPEED_DELTA_MAX / 2, (sections / 4) * spacing + 1, tft.height() - 20 + 1, &fonts::Font2);	 // -SPEED_DELTA_MAX/2
	tft.drawNumber(SPEED_DELTA_MAX / 2, (sections / 4) * 3 * spacing + 1, tft.height() - 20 + 1, &fonts::Font2); // +SPEED_DELTA_MAX/2
}
void drawSpeedDelta()
{
	drawSpeedDelta(speedDelta, prevSpeedDelta);
}

std::string getFloatTimeString(float time)
{
	const int minutes = floor(time / 60);
	const int seconds = floor(time - minutes * 60);
	const int milliseconds = fmod(time, 1) * 1000;

	char strBuffer[9];
	snprintf(strBuffer, 9, "%01d:%02d.%03d", minutes, seconds, milliseconds);

	return std::string(strBuffer);
}

std::string getMillisTimeString(int time)
{
	const int minutes = floor(time / 60000);
	const int seconds = floor((time - minutes * 60000) / 1000);
	const int milliseconds = time - minutes * 60000 - seconds * 1000;

	char strBuffer[9];
	snprintf(strBuffer, 9, "%01d:%02d.%03d", minutes, seconds, milliseconds);

	return std::string(strBuffer);
}

void drawLapLabels()
{
	selectScreen(SCREEN2);

	tft.setTextSize(1);
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setTextDatum(TL_DATUM);

	tft.setTextPadding(tft.textWidth("Current Lap", &fonts::Font2));
	tft.drawString("Current Lap", 2, 0, &fonts::Font2);

	tft.setTextPadding(tft.textWidth("Predicted Lap", &fonts::Font2));
	tft.drawString("Predicted Lap", 2, 80, &fonts::Font2);

	tft.setTextPadding(tft.textWidth("Previous Lap", &fonts::Font2));
	tft.drawString("Previous Lap", 2, 160, &fonts::Font2);

	tft.setTextPadding(tft.textWidth("Best Lap", &fonts::Font2));
	tft.drawString("Best Lap", 2, 240, &fonts::Font2);

	tft.setTextPadding(tft.textWidth("Lap #", &fonts::Font2));
	tft.drawString("Lap #", 300, 0, &fonts::Font2);

	tft.setTextPadding(tft.textWidth("Stint time", &fonts::Font2));
	tft.drawString("Stint time", 300, 220, &fonts::Font2);

	tft.setTextPadding(tft.textWidth("Best Lap #", &fonts::Font2));
	tft.drawString("Best Lap #", 300, 140, &fonts::Font2);

	tft.setTextPadding(tft.textWidth("Satellites", &fonts::Font2));
	tft.drawString("Satellites", 395, 140, &fonts::Font2);
}

void drawLapNumber(int value)
{
	selectScreen(SCREEN2);

	tft.setTextSize(1.5);
	tft.setTextPadding(tft.textWidth("88", &fonts::Font8));
	tft.setTextColor(SCREEN2TEXTCOLOR, TFT_BLACK);
	tft.setTextDatum(TC_DATUM);
	tft.drawNumber(value, 388, 20, &fonts::Font8);
}
void drawLapNumber()
{
	drawLapNumber(lapNumber);
}

void drawLapTime(float value)
{
	selectScreen(SCREEN2);

	std::string str = getFloatTimeString(value);

	tft.setTextSize(0.75);
	tft.setTextPadding(tft.textWidth("8:88.88", &fonts::Font8));
	tft.setTextColor(SCREEN2TEXTCOLOR, TFT_BLACK);
	tft.setTextDatum(TL_DATUM);
	tft.drawString(str.c_str(), 1, 20, &fonts::Font8);
}
void drawLapTime()
{
	drawLapTime(lapTime);
}

void drawPredictedLapTime(float value)
{
	selectScreen(SCREEN2);

	std::string str = getFloatTimeString(value);

	tft.setTextSize(0.75);
	tft.setTextPadding(tft.textWidth("8:88.88", &fonts::Font8));
	tft.setTextColor(SCREEN2TEXTCOLOR, TFT_BLACK);
	tft.setTextDatum(TL_DATUM);
	tft.drawString(str.c_str(), 1, 100, &fonts::Font8);
}
void drawPredictedLapTime()
{
	drawPredictedLapTime(predictedLapTime);
}

void drawPrevLapTime(float value, bool hightlight)
{
	selectScreen(SCREEN2);

	std::string str = getFloatTimeString(value);

	tft.setTextSize(0.75);
	tft.setTextPadding(tft.textWidth("8:88.88", &fonts::Font8));
	tft.setTextColor(hightlight && getFastestTime() > 0 && prevLapTime <= getFastestTime() ? TFT_GREEN : SCREEN2TEXTCOLOR, TFT_BLACK);
	tft.setTextDatum(TL_DATUM);
	tft.drawString(str.c_str(), 1, 180, &fonts::Font8);
}
void drawPrevLapTime(bool hightlight)
{
	drawPrevLapTime(prevLapTime, hightlight);
}

void drawBestLapTime(float value, bool hightlight)
{
	selectScreen(SCREEN2);

	std::string str = getFloatTimeString(value);

	tft.setTextSize(0.75);
	tft.setTextPadding(tft.textWidth("8:88.88", &fonts::Font8));
	tft.setTextColor(hightlight && isBestFasterThanComparison() ? TFT_GREEN : SCREEN2TEXTCOLOR, TFT_BLACK);
	tft.setTextDatum(TL_DATUM);
	tft.drawString(str.c_str(), 1, 260, &fonts::Font8);
}
void drawBestLapTime(bool hightlight)
{
	drawBestLapTime(getFastestTime(), hightlight);
}

void drawBestLapNumber(int value)
{
	selectScreen(SCREEN2);

	tft.setTextSize(0.75);
	tft.setTextPadding(tft.textWidth("88", &fonts::Font8));
	tft.setTextColor(SCREEN2TEXTCOLOR, TFT_BLACK);
	tft.setTextDatum(TC_DATUM);
	tft.drawNumber(value, 340, 160, &fonts::Font8);
}
void drawBestLapNumber()
{
	drawBestLapNumber(bestLapNumber);
}

void drawSatellites(int value)
{
	selectScreen(SCREEN2);

	tft.setTextSize(0.75);
	tft.setTextPadding(tft.textWidth("88", &fonts::Font8));
	tft.setTextColor(SCREEN2TEXTCOLOR, TFT_BLACK);
	tft.setTextDatum(TL_DATUM);
	tft.drawNumber(value, 395, 160, &fonts::Font8);
}
void drawSatellites()
{
	drawSatellites(satellites);
}

void drawStintTime()
{
	selectScreen(SCREEN2);

	unsigned long time = millis() - startTime;

	const int minutes = floor(time / 60000);
	const int seconds = floor((time - minutes * 60000) / 1000);

	char strBuffer[6];
	snprintf(strBuffer, 6, "%02d:%02d", minutes, seconds);

	tft.setTextSize(0.7);
	tft.setTextPadding(tft.textWidth("88:88", &fonts::Font8));
	tft.setTextColor(SCREEN2TEXTCOLOR, TFT_BLACK);
	tft.setTextDatum(TL_DATUM);
	tft.drawString(strBuffer, 302, 240, &fonts::Font8);
}

#pragma endregion Display

#pragma region Battery

// Array with voltage - charge definitions
float voltageTable[] = {
	3.200, // 0
	3.250, // 1
	3.300, // 2
	3.350, // 3
	3.400, // 4
	3.450, // 5
	3.500, // 6
	3.550, // 7
	3.600, // 8
	3.650, // 9
	3.700, // 10
	3.703, // 11
	3.706, // 12
	3.710, // 13
	3.713, // 14
	3.716, // 15
	3.719, // 16
	3.723, // 17
	3.726, // 18
	3.729, // 19
	3.732, // 20
	3.735, // 21
	3.739, // 22
	3.742, // 23
	3.745, // 24
	3.748, // 25
	3.752, // 26
	3.755, // 27
	3.758, // 28
	3.761, // 29
	3.765, // 30
	3.768, // 31
	3.771, // 32
	3.774, // 33
	3.777, // 34
	3.781, // 35
	3.784, // 36
	3.787, // 37
	3.790, // 38
	3.794, // 39
	3.797, // 40
	3.800, // 41
	3.805, // 42
	3.811, // 43
	3.816, // 44
	3.821, // 45
	3.826, // 46
	3.832, // 47
	3.837, // 48
	3.842, // 49
	3.847, // 50
	3.853, // 51
	3.858, // 52
	3.863, // 53
	3.868, // 54
	3.874, // 55
	3.879, // 56
	3.884, // 57
	3.889, // 58
	3.895, // 59
	3.900, // 60
	3.906, // 61
	3.911, // 62
	3.917, // 63
	3.922, // 64
	3.928, // 65
	3.933, // 66
	3.939, // 67
	3.944, // 68
	3.950, // 69
	3.956, // 70
	3.961, // 71
	3.967, // 72
	3.972, // 73
	3.978, // 74
	3.983, // 75
	3.989, // 76
	3.994, // 77
	4.000, // 78
	4.008, // 79
	4.015, // 80
	4.023, // 81
	4.031, // 82
	4.038, // 83
	4.046, // 84
	4.054, // 85
	4.062, // 86
	4.069, // 87
	4.077, // 88
	4.085, // 89
	4.092, // 90
	4.100, // 91
	4.111, // 92
	4.122, // 93
	4.133, // 94
	4.144, // 95
	4.156, // 96
	4.167, // 97
	4.178, // 98
	4.189, // 99
	4.200, // 100
};

int batteryLevel = 0;
int batteryIconIndex = 0;
float batteryVoltage = 0;
const char *batteryImages[] = {
	"/battery_01.jpg",
	"/battery_02.jpg",
	"/battery_03.jpg",
	"/battery_04.jpg",
};

float getVolatge()
{
	return (analogRead(VBAT_PIN) / 4095.0) * 2.0 * 3.3 * (vref / 1000.0) * voltageCorrectionFactor;
}

int getChargeLevel(float volts)
{
	int idx = 50;
	int prev = 0;
	if (volts >= 4.2)
	{
		return 100;
	}
	if (volts <= 3.2)
	{
		return 0;
	}
	while (true)
	{
		int half = abs(idx - prev) / 2;
		prev = idx;
		if (volts >= voltageTable[idx])
		{
			idx = idx + half;
		}
		else
		{
			idx = idx - half;
		}
		if (prev == idx)
		{
			break;
		}
	}
	return idx;
}

void drawBatteryIcon()
{
	selectScreen(SCREEN2);
	TJpgDec.drawFsJpg(tft.width() - 35, tft.height() - 18, batteryImages[batteryIconIndex]);
}

void drawBatteryLevel()
{
	selectScreen(SCREEN2);
	tft.setTextSize(1);
	tft.setTextPadding(tft.textWidth("8.88v", &fonts::Font2));
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setTextDatum(BR_DATUM);
	tft.drawString(String(batteryVoltage) + "v", tft.width() - 35, tft.height(), &fonts::Font2);
}

void drawBattery()
{

	batteryVoltage = getVolatge();

	batteryLevel = getChargeLevel(batteryVoltage);
	if (batteryLevel >= 75)
	{
		batteryIconIndex = 3;
	}
	else if (batteryLevel >= 50)
	{
		batteryIconIndex = 2;
	}
	else if (batteryLevel >= 25)
	{
		batteryIconIndex = 1;
	}
	else
	{
		batteryIconIndex = 0;
	}

	drawBatteryIcon();
	drawBatteryLevel();
}

TaskHandle_t drawBatteryAndStatusTaskHandle;
void drawBatteryAndStatusTask(void *pvParameters)
{
	clearScreens();
	while (1)
	{
		if (status > 0)
		{
			drawStatus(true);
		}

		drawBattery();

		vTaskDelay(pdMS_TO_TICKS(batteryLevelDrawInterval));
	}
}

#pragma endregion Battery

TaskHandle_t demoModeTaskHandle;
void demoModeTask(void *pvParameters)
{
	int drawInterval = 100;
	randomSeed(millis());
	float _speedDelta = 0;
	float _prevSpeedDelta = 0;
	int direction = 1;
	int prevBatteryIntervalTime = millis();

	clearScreens();
	drawLapLabels();
	drawBestLapTime(random(0, 600000) / 1000.0, false);
	drawPrevLapTime(random(0, 600000) / 1000.0, false);
	drawBestLapNumber(random(0, 100));
	drawSatellites(random(0, 100));
	drawLapNumber(random(0, 100));
	drawStatus(false, false, 610);

	while (1)
	{

		drawLapTime(random(0, 600000) / 1000.0);
		drawPredictedLapTime(random(0, 600000) / 1000.0);

		drawTimeDelta(random(-10000, 10000) / 1000.0);

		_prevSpeedDelta = _speedDelta;
		if ((_speedDelta >= SPEED_DELTA_MAX) || ((_speedDelta * -1) >= SPEED_DELTA_MAX))
			direction = direction * -1;
		_speedDelta += direction;
		drawSpeedDelta(_speedDelta, _prevSpeedDelta);

		int batteryIntervalDelta = millis() - prevBatteryIntervalTime;
		if (batteryIntervalDelta >= 1000)
		{
			prevBatteryIntervalTime = millis();
			drawBattery();
			drawStintTime();
		}

		// vTaskDelay(pdMS_TO_TICKS(drawInterval));
	}
}

#pragma region BLE

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
bool receivedData = false;

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
		bytes[0] = (byte)CMD_TYPE_UPDATE_ALL;
	else
	{
		bytes[0] = (byte)CMD_TYPE_UPDATE;
		bytes[1] = (byte)monitorId;
	}

	monitorConfigCharacteristic->setValue(bytes, 2);
	monitorConfigCharacteristic->indicate();
}

void sendRemoveCommand(int monitorId = -1)
{
	// monitorId < 0 means remove all
	byte bytes[2];
	if (monitorId < 0)
		bytes[0] = (byte)CMD_TYPE_REMOVE_ALL;
	else
	{
		bytes[0] = (byte)CMD_TYPE_REMOVE;
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
		if (!monitoredChannels[i].enabled)
		{
			// skip if not enabled
			monitoredChannels[i].configuring = false;
			monitoredChannels[i].configured = true;
			continue;
		}

		if (monitoredChannels[i].configuring || !monitoredChannels[i].configured)
		{
			// not done
			configured = false;
		}

		if (!monitoredChannels[i].configured && !monitoredChannels[i].configuring)
		{
			// add if not configured
			Serial.print("add monitor : ");
			Serial.println(monitoredChannels[i].name);
			monitoredChannels[i].configuring = true;
			sendConfigCommand(CMD_TYPE_ADD, i, monitoredChannels[i].formula);
			delay(1);
		}
	}

	if (configured)
	{
		// all configured
		Serial.println("--CONFIGURED--");
		isConfigured = configured;
		if (!receivedData)
		{
			status = STATUS_DATA;
			drawStatus();
		}
	}
}

// callbacks for monitoring configuration
class ConfigCallbacks : public BLECharacteristicCallbacks
{
	// status of the sent payload from ble stack
	void onStatus(BLECharacteristic *pCharacteristic, Status s, uint32_t code)
	{
		const std::string value = pCharacteristic->getValue();
		const int command = (int)value[0];
		const int monitorId = (int)value[1];
		Serial.print(command == CMD_TYPE_UPDATE_ALL ? "Update All" : monitoredChannels[monitorId].name);
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
			if (command == CMD_TYPE_ADD || command == CMD_TYPE_ADD_INCOMPLETE)
			{
				monitoredChannels[monitorId].configured = false;
				monitoredChannels[monitorId].configuring = false;
			}
			break;
		case ERROR_NOTIFY_DISABLED:
			Serial.println("ERROR_NOTIFY_DISABLED");
			if (command == CMD_TYPE_ADD || command == CMD_TYPE_ADD_INCOMPLETE)
			{
				monitoredChannels[monitorId].configured = false;
				monitoredChannels[monitorId].configuring = false;
			}
			break;
		case ERROR_GATT:
			Serial.println("ERROR_GATT");
			if (command == CMD_TYPE_ADD || command == CMD_TYPE_ADD_INCOMPLETE)
			{
				monitoredChannels[monitorId].configured = false;
				monitoredChannels[monitorId].configuring = false;
			}
			break;
		case ERROR_NO_CLIENT:
			Serial.println("ERROR_NO_CLIENT");
			if (command == CMD_TYPE_ADD || command == CMD_TYPE_ADD_INCOMPLETE)
			{
				monitoredChannels[monitorId].configured = false;
				monitoredChannels[monitorId].configuring = false;
			}
			break;
		case ERROR_INDICATE_TIMEOUT:
			Serial.println("indicate timeout");
			if (command == CMD_TYPE_ADD || command == CMD_TYPE_ADD_INCOMPLETE)
			{
				monitoredChannels[monitorId].configured = false;
				monitoredChannels[monitorId].configuring = false;
			}
			break;
		case ERROR_INDICATE_FAILURE:
			Serial.println("indicate failure");
			if (command == CMD_TYPE_ADD || command == CMD_TYPE_ADD_INCOMPLETE)
			{
				monitoredChannels[monitorId].configured = false;
				monitoredChannels[monitorId].configuring = false;
			}
			break;
		default:
			break;
		}
	};

	// response notification from racechrono
	void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param)
	{
		if (pCharacteristic->getLength() > 0)
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

// main callbacks for monitored values
class MonitorCallbacks : public BLECharacteristicCallbacks
{
	void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param)
	{
		if (pCharacteristic->getLength() > 0)
		{
			// 20 bytes, 4 values max
			// byte 0 = monitor id, byte 1-4 = value
			std::string value = pCharacteristic->getValue();
			// int x = 0;
			// while (x + 5 <= pCharacteristic->getLength())
			// {
			// 	Serial.print((int)value[x]);
			// 	Serial.print(" : ");
			// 	int32_t data = value[x + 1] << 24 | value[x + 2] << 16 | value[x + 3] << 8 | value[x + 4];
			// 	Serial.print(data * monitoredChannels[(int)value[x]].multiplier);
			// 	Serial.println(" | ");
			// 	x += 5;
			// }
			// Serial.println(" -- ");
			int dataPos = 0;
			while (dataPos + 5 <= pCharacteristic->getLength())
			{
				int monitorId = (int)value[dataPos];

				int32_t data = value[dataPos + 1] << 24 | value[dataPos + 2] << 16 | value[dataPos + 3] << 8 | value[dataPos + 4];
				if (monitorId < sizeof monitoredChannels && data < INT_MAX) // weird stuff with values of int max sometimes received
				{
					if (receivedData == false)
					{
						receivedData = true;
						status = 0;
						vTaskDelete(drawBatteryAndStatusTaskHandle);

						clearScreens();
						drawLapLabels();
					}

					float value = (float)data * monitoredChannels[monitorId].multiplier;

					Serial.print(monitoredChannels[monitorId].name);
					Serial.print(" : ");
					Serial.print(value);
					Serial.println();

					switch (monitorId)
					{
					// 0 - Lap Number
					case 0:
						lapNumber = value;
						drawLapNumber();
						break;

					// 1 - Lap Time
					case 1:
						lapTime = value;
						drawLapTime(false);
						break;

					// 2 - Prev time
					case 2:
					{
						// dont hightlight first lap
						bool hightlight = prevLapTime > 0 || comparisonLapTime > 0;
						prevLapTime = value;
						drawPrevLapTime(hightlight);
						break;
					}
					// 3 - Best lap
					case 3:
						bestLapNumber = value;
						drawBestLapNumber();
						break;

					// 4 - Best time
					case 4:
						if (value > 1)
						{
							// dont hightlight first lap
							bool hightlight = bestLapTime > 0;
							bestLapTime = value;
							drawBestLapTime(hightlight);
							// if prev received before new best, wont be highlighted, so draw again
							drawPrevLapTime(true);

							predictedLapTime = getComparisonOrBest() + timeDelta;
							drawPredictedLapTime();
						}
						break;

					// 5 - Speed Delta
					case 5:
						prevSpeedDelta = speedDelta;
						speedDelta = value;
						drawSpeedDelta();

						break;

					// 6 - Time Delta
					case 6:
						prevTimeDelta = timeDelta;
						timeDelta = value;
						drawTimeDelta();

						predictedLapTime = getComparisonOrBest() + timeDelta;
						drawPredictedLapTime();

						break;

					// 7 - comparison lap time
					case 7:
						if (lapNumber < 2)
						{
							// if received comparison lap on 1st lap or before, then def. using seperate comparison lap
							// event if not set in app, best lap is also sent as comparison lap, which is why this is needed
							usingComparisonLap = true;
						}
						comparisonLapTime = value;
						drawBestLapTime(true);

						break;

					// 8 - update rate
					case 8:

						break;

					// 9 - Satellites
					case 9:
						satellites = value;
						drawSatellites();

						break;

					// 10 - Speed
					case 10:

						break;

					// 11 - speed
					case 11:
						// satellites = value;
						// drawSatellites();

						break;
					}
				}
				dataPos += 5;

				delay(1);
			}

			if (receivedData)
			{
				// draw stint time and battery here
				// tried to seperate it so its not tied to the reception of ble notifications
				// but absolutely nothing else worked, weird concurrency issues with the TFTs
				currentTime = millis();
				if (currentTime - previousTimeIntervalTime >= stintTimeDrawInterval)
				{
					drawStintTime();
				}
				if (currentTime - previousBatteryIntervalTime >= batteryLevelDrawInterval)
				{
					previousBatteryIntervalTime = currentTime;

					drawBattery();
				}
			}
		}
	}
};

// callbacks for clients connection status
class ServerCallbacks : public BLEServerCallbacks
{
	void onConnect(BLEServer *BLE_server)
	{
		deviceConnected = true;
		startTime = millis();
		Serial.println("[I] Bluetooth client connected!");
		Serial.println("set status");
		status = STATUS_CONFIGURATION;
		// drawStatus();
		if (demoModeTaskHandle != NULL)
		{
			Serial.println("delete demo");
			vTaskDelete(demoModeTaskHandle);
			demoModeTaskHandle = NULL;
			Serial.println("deleted demo");
		}
		if (drawBatteryAndStatusTaskHandle == NULL)
		{
			Serial.println("start task");
			xTaskCreatePinnedToCore(drawBatteryAndStatusTask, "Battery & Status Task", 2048, NULL, 0, &drawBatteryAndStatusTaskHandle, 0);
			Serial.println("started task");
		}
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

		status = STATUS_CONNECTION;
		if (drawBatteryAndStatusTaskHandle == NULL)
		{
			xTaskCreatePinnedToCore(drawBatteryAndStatusTask, "Battery & Status Task", 2048, NULL, 0, &drawBatteryAndStatusTaskHandle, 0);
		}
	}
};

// BLE configuration
void configBLE()
{
	// name of device
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
	BLE_advertising->setMinInterval(35); // 35 is lowest value that works
	BLE_advertising->setMaxInterval(100);
	BLEDevice::startAdvertising();

	status = STATUS_CONNECTION;
	// xTaskCreatePinnedToCore(drawBatteryAndStatusTask, "Battery & Status Task", 2048, NULL, 0, &drawBatteryAndStatusTaskHandle, 0);
	xTaskCreatePinnedToCore(demoModeTask, "Demo Mode Task", 2048, NULL, 0, &demoModeTaskHandle, 0);
}

#pragma endregion BLE

void setup()
{
	// Initialize Serial port
	Serial.begin(115200);

	// need spiffs for images
	if (!SPIFFS.begin())
	{
		Serial.println("SPIFFS initialisation failed!");
		while (1)
			yield(); // Stay here twiddling thumbs waiting
	}
	Serial.println("SPIFFS available!");

	pinMode(SCREEN1, OUTPUT);
	pinMode(SCREEN2, OUTPUT);
	screensOn();

	// Initialize TFT display
	tft.init();
	tft.invertDisplay(1);
	tft.setRotation(3);
	tft.fillScreen(TFT_BLACK);
	tft.setSwapBytes(true);

	// display logo while configuring ble
	TJpgDec.setJpgScale(1);
	TJpgDec.setCallback(tft_output);
	TJpgDec.drawFsJpg(0, tft.height() / 2 - 45, "/timerdisplaylogo.jpg");

	configBLE();
}

void loop()
{
	if (deviceConnected && !isConfigured && status == STATUS_CONFIGURATION)
	{
		// continually check if all the channels are configured or configuring
		checkChannelsConfigured();
	}
}
