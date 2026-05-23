/* ESP32 Weather Display and an e-paper 1.54" Display, obtains data from Open Weather Map, decodes and then displays it.
  ####################################################################################################################################
  This software, the ideas and concepts is Copyright (c) David Bird 2019. All rights to this software are reserved.

  Any redistribution or reproduction of any part or all of the contents in any form is prohibited other than the following:
  1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
  2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird as the source of the material.
  3. You may not, except with my express written permission, distribute or commercially exploit the content.
  4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.

  The above copyright ('as annotated') notice and this permission notice shall be included in all copies or substantial portions of the Software and where the
  software use is visible to an end-user.

  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY
  OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  See more at http://www.dsbird.org.uk
*/

/* Changes made by @mattcuk
   ========================
   I've made some small changes to David's code (https://github.com/G6EJD/ESP32-e-Paper-Weather-Display) to enable this to 
   run on the WeAct 1.54" ePaper Module & ESP32 C3 Super without issues.
   
   You'll be able to take this and make it work with WeAct's other displays without much hassle.

   a. Solved my ESP32 C3 Super not connecting to wifi by capping the max wifi power (defaults to 40 & configurable in code)
   b. Added more info re. which WeAct pins go to which C3 Super pins
   c. Added the exact GxEPD2_BW initialisation needed for the WeAct 1.54" SSD1681 module
   d. Increased the number of API retry attempts to 5, and put a delay of 5sec between retries
   e. Added a 60 second delay before deep sleep, allowing you to re-flash the board more easily
   f. OpenWeather v3 API doesn't have High/Low temps so I've changed the display to show 'Feels Like' instead
   g. OpenWeather v3 API doesn't have the Forecast so I've changed the display to show the current conditions
   h. Removed the software version from the top of the display
   i. Removed (commented out) the SPI commands from the screen initialisation function.. these were crashing the C3 Super board
   j. Added a board reboot after 100 cycles to help with it stopping working after a few days for no reason
*/

#include "owm_credentials.h"   // See 'owm_credentials' tab and enter your OWM API key and set the Wifi SSID and PASSWORD
#include "Micro_Wx_Icons.h"    // Weather Icons
#include <ArduinoJson.h>       // https://github.com/bblanchon/ArduinoJson
#include <WiFi.h>              // Built-in
#include "esp_wifi.h"          // Used to set wifi power level on ESP32 C3 Super
#include "time.h"              // Built-in
#include <SPI.h>               // Built-in 
#define  ENABLE_GxEPD2_display 0
#include <GxEPD2_BW.h>         // GxEPD2 from Sketch, Include Library, Manage Libraries, search for GxEDP2
#include <Fonts/FreeMonoBold12pt7b.h>
#include "epaper_fonts.h"

#define SCREEN_WIDTH  200
#define SCREEN_HEIGHT 200

#define ESP32_WIFI_MAX_POWER 40

#define BUILTIN_LED 8

enum alignment {LEFT, RIGHT, CENTER};

static const uint8_t EPD_BUSY = 1;  // WeAct BUSY (Purple) -> ESP C3 GPIO 1 (Configurable in this code)
static const uint8_t EPD_RST  = 3;  // WeAct RES (Orange) -> ESP C3 GPIO 3 (Configurable in this code)
static const uint8_t EPD_DC   = 2;  // WeAct D/C (White) -> ESP C3 GPIO 2 (Configurable in this code)
static const uint8_t EPD_CS   = 10; // WeAct CS (Blue) -> ESP C3 GPIO 10 (Configurable in this code)

static const uint8_t EPD_SCK  = 4;  // WeAct SCL (Green) -> ESP C3 GPIO 4 (SCK)
static const uint8_t EPD_MOSI = 6;  // WeAct SDA (Yellow) -> ESP C3 GPIO 6 (MOSI)

static const uint8_t EPD_MISO = 19; // Master-In Slave-Out not used, as no data from display

// Try one of these for your display
//GxEPD2_3C<GxEPD2_154c, GxEPD2_154c::HEIGHT> display(GxEPD2_154c(/*CS=5*/ SS, /*DC=17*/ 17, /*RST=16*/ 16, /*BUSY=4*/ 4));     // 3-Colour display
//GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS*/ EPD_CS, /*DC*/ EPD_DC, /*RST*/ EPD_RST, /*BUSY*/ EPD_BUSY)); // New version of 2-Colour display (B/W) GDEH0154D67 or Waveshare 1.54 V2
//GxEPD2_BW<GxEPD2_154_M09, GxEPD2_154_M09::HEIGHT> display(GxEPD2_154_M09(/*CS*/ EPD_CS, /*DC*/ EPD_DC, /*RST*/ EPD_RST, /*BUSY*/ EPD_BUSY)); // GDEW0154M09 200x200
// GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT> display(GxEPD2_154(/*CS=5*/ EPD_CS, /*DC=17*/ EPD_DC, /*RST=16*/ EPD_RST, /*BUSY=4*/ EPD_BUSY)); // 2-Colour display (B/W)

// WeAct 1.54 ePaper Module SSD1681
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// Lolin D32 ProGxEPD2_3C<GxEPD2_154c, GxEPD2_154c::HEIGHT> display(GxEPD2_154c(/*CS*/ EPD_CS, /*DC*/ EPD_CS, /*RST*/ EPD_RST, /*BUSY*/ EPD_BUSY)); // 3-Colour display
// GxEPD2_3C<GxEPD2_154c, GxEPD2_154c::HEIGHT> display(GxEPD2_154c(/*CS*/ EPD_CS, /*DC*/ EPD_DC, /*RST*/ EPD_RST, /*BUSY*/ EPD_BUSY)); // 3-Colour display

// For 3-Colour displays use: // If you want to colour text use GxEPD_RED or GxEPD_YELLOW in place of GxEPD_BLACK
//GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT> display(GxEPD2_154(/*CS*/ EPD_CS, /*DC*/ EPD_DC, /*RST*/ EPD_RST, /*BUSY*/ EPD_BUSY));
// For 2-Colour displays (B/W) use:
//GxEPD2_3C<GxEPD2_154, GxEPD2_154c::HEIGHT> display(GxEPD2_154c(/*CS*/ EPD_CS, /*DC*/ EPD_DC, /*RST*/ EPD_RST, /*BUSY*/ EPD_BUSY));

//################  VERSION  ##########################
String version = "1.4";      // Version of this program
//################ VARIABLES ###########################

bool LargeIcon = true, SmallIcon = false, RxWeather = false, RxForecast = false;
#define Large  10
#define Small  4
String  Time_str, Date_str, rxtext; // strings to hold time and received weather data;
int     StartTime, CurrentHour = 0, CurrentMin = 0, CurrentSec = 0;

//################ PROGRAM VARIABLES and OBJECTS ################

#define max_readings 4

#include "common.h"

#define autoscale_on  true
#define autoscale_off false
#define barchart_on   true
#define barchart_off  false

float pressure_readings[max_readings]    = {0};
float temperature_readings[max_readings] = {0};
float rain_readings[max_readings]        = {0};

long SleepDuration = 30; // Sleep time in minutes, aligned to minute boundary, so if 30 will always update at 00 or 30 past the hour
int  WakeupTime    = 7;  // Don't wakeup until after 07:00 to save battery power
int  SleepTime     = 23; // Sleep after (23+1) 00:00 to save battery power
int  CyclesBeforeReboot = 100; // Reboot after 100 cycles to help prevent memory leaks and other issues that can arise from long-term use of ESP32 in deep sleep mode. Adjust as needed based on your specific use case and stability requirements.
int  CycleCount = 0; // Initialize cycle count

//#########################################################################################

void setup() {
  StartTime = millis();
  Serial.begin(115200);
  if (StartWiFi() == WL_CONNECTED && SetupTime() == true) {
    if ((CurrentHour >= WakeupTime && CurrentHour <= SleepTime)) {
      InitialiseDisplay(); // Give screen time to initialise by getting weather data!
      byte Attempts = 1;
      WiFiClient client;   // wifi client object
      while (RxWeather == false && Attempts <= 5) { // Try up-to 5x for Weather and Forecast data
        if (RxWeather == false) RxWeather  = ReceiveOneCallWeather(client, true);
        delay(5000); // Delay between attempts to retrieve weather from API
        Attempts++;
      }
      if (RxWeather || RxForecast) { // If received either Weather or Forecast data then proceed, report later if either failed
        StopWiFi(); // Reduces power consumption
        DisplayWeather();
        display.display(false); // Full screen update mode
      }
    }
    BeginSleep();
  }
}

//#########################################################################################

void loop() { // this will never run!
}

//#########################################################################################

void BeginSleep() {
  delay(90000); // Wait a 90sec before turning off.. gives time to reflash the firmware
  display.powerOff();
  long SleepTimer = (SleepDuration * 60 - ((CurrentMin % SleepDuration) * 60 + CurrentSec)); //Some ESP32 are too fast to maintain accurate time
  esp_sleep_enable_timer_wakeup((SleepTimer+20) * 1000000LL); // Added +20 seconnds to cover ESP32 RTC timer source inaccuracies
#ifdef BUILTIN_LED
  pinMode(BUILTIN_LED, INPUT); // If it's On, turn it off and some boards use GPIO-5 for SPI-SS, which remains low after screen use
  digitalWrite(BUILTIN_LED, HIGH);
#endif
  Serial.println("Entering " + String(SleepTimer) + "-secs of sleep time");
  Serial.println("Awake for : " + String((millis() - StartTime) / 1000.0, 3) + "-secs");
  Serial.println("Starting deep-sleep period...");
  CycleCount++; // Track the number of sleep/wake cycles
  if (CycleCount >= CyclesBeforeReboot) {
    Serial.println("Rebooting...");
    ESP.restart();
  }
  esp_deep_sleep_start();      // Sleep for e.g. 30 minutes
}
//#########################################################################################
void DisplayWeather() {                                    // 1.54" e-paper display is 200x200 resolution
  DisplayHeadingSection();                                 // Top line of the display
  DisplayTempHumiSection(0, 12);                           // Current temperature with Max/Min
  DisplayWxPerson(114, 12, WxConditions[0].Icon);          // Weather person depiction of weather
  DisplayMainWeatherSection(0, 112);                       // Weather forecast text
  DisplayForecastSection(0, 135);                          // 3hr interval forecast boxes
}
//#########################################################################################
void DisplayTempHumiSection(int x, int y) {
  display.drawRect(x, y, 115, 97, GxEPD_BLACK);
  display.setFont(&DSEG7_Classic_Bold_21);
  display.setTextSize(2);
  drawString(x + 17, y + 5, String(WxConditions[0].Temperature, 0) + "'", LEFT);                                   // Show current Temperature
  display.setTextSize(1);
  drawString(x + 90, y + 30, (Units == "M" ? "C" : "F"), LEFT); // Add-in smaller Temperature unit
  display.setTextSize(2);
  display.setFont(&DejaVu_Sans_Bold_11);
  // High&Low aren't in v3.0 of the API, so we'll show 'Feels Like' instead
  //drawString(x + 57, y + 59, String(WxConditions[0].High, 0) + "'/" + String(WxConditions[0].Low, 0) + "'", CENTER); // Show forecast high and Low, in the font ' is a °
  drawString(x + 55, y + 56, "f" + String(WxConditions[0].FeelsLike, 0), RIGHT);
  display.setTextSize(1);
  drawString(x + 57,  y + 57, "o  " + String(WxConditions[0].Humidity, 0) + "%", LEFT);                               // Show Humidity
  drawString(x + 60,  y + 83, "H" + String(Daily[0].High, 0) + " / L" + String(Daily[0].Low, 0), CENTER);                // Show day high/low temps from daily forecast array

}
//#########################################################################################
void DisplayHeadingSection() {
  drawString(2, 2, Time_str, LEFT);
  drawString(SCREEN_WIDTH - 2, 0, Date_str, RIGHT);
  // Don't need to show the version of this code
  //drawString(SCREEN_WIDTH / 2, 0, version, CENTER);
  display.drawLine(0, 12, SCREEN_WIDTH, 12, GxEPD_BLACK);
}
//#########################################################################################
void DisplayMainWeatherSection(int x, int y) {
  display.drawRect(x, y - 4, SCREEN_WIDTH, 28, GxEPD_BLACK);
  // A nice forecast string isn't in the v3.0 API, so we'll just show the current weather condition instead :(
  //String Wx_Description1 = WxConditions[0].Forecast0;
  String Wx_Description1 = WxConditions[0].Description;
  display.setFont(&DejaVu_Sans_Bold_11);
  String Wx_Description2 = WindDegToDirection(WxConditions[0].Winddir) + " " + String(WxConditions[0].Windspeed, 1) + (Units == "M" ? "m/s" : "mph") + " Gust " + String(Daily[0].Windgust, 1) + (Units == "M" ? "m/s" : "mph") + "";
  drawStringMaxWidth(x + 2, y - 2, 27, TitleCase(Wx_Description1), LEFT);
  drawStringMaxWidth(x + 2, y +10, 27, TitleCase(Wx_Description2), LEFT);  
}
//#########################################################################################
void DisplayForecastSection(int x, int y) {
  int offset = 50;
  DisplayForecastWeather(x + offset * 0, y, offset, 0);
  DisplayForecastWeather(x + offset * 1, y, offset, 1);
  DisplayForecastWeather(x + offset * 2, y, offset, 2);
  DisplayForecastWeather(x + offset * 3, y, offset, 3);
  int r = 0;
  do {
    if (Units == "I") pressure_readings[r] = WxForecast[r].Pressure * 0.02953;
    else              pressure_readings[r] = WxForecast[r].Pressure;
    temperature_readings[r]                = WxForecast[r].Temperature;
    if (Units == "I") rain_readings[r]     = WxForecast[r].Rainfall * 0.0393701;
    else              rain_readings[r]     = WxForecast[r].Rainfall;
    r++;
  } while (r < max_readings);
}
//#########################################################################################
void DisplayForecastWeather(int x, int y, int offset, int index) {
  display.drawRect(x, y, offset, 65, GxEPD_BLACK);
  display.drawLine(x, y + 13, x + offset, y + 13, GxEPD_BLACK);
  DisplayWxIcon(x + offset / 2 + 1, y + 35, WxForecast[index].Icon, SmallIcon);
  drawString(x + offset / 2, y  + 3, String(ConvertUnixTime(WxForecast[index].Dt + WxConditions[0].Timezone).substring(0,5)), CENTER);
  // High&Low aren't in v3.0 of the API, so we'll show 'Feels Like' instead
  //drawString(x + offset / 2, y + 50, String(WxForecast[index].High, 0) + "/" + String(WxForecast[index].Low, 0), CENTER);
  //drawString(x + offset / 2, y + 50, String(WxForecast[index].Temperature, 0) + "/" + String(WxForecast[index].FeelsLike, 0), CENTER);
  drawString(x + offset / 2, y + 50, String(WxForecast[index].FeelsLike, 0), CENTER);
}
//#########################################################################################
String WindDegToDirection(float winddirection) {
  int dir = int((winddirection / 22.5) + 0.5);
  String Ord_direction[16] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  return Ord_direction[(dir % 16)];
}
//#########################################################################################
void DisplayRain(int x, int y) {
  if (WxForecast[1].Rainfall > 0) drawString(x, y, String(WxForecast[1].Rainfall, 3) + (Units == "M" ? "mm" : "in") + " Rain", LEFT); // Only display rainfall if > 0
}
//#########################################################################################
void DisplayWxIcon(int x, int y, String IconName, bool LargeSize) {
  Serial.println(IconName);
  if      (IconName == "01d" || IconName == "01n") Sunny(x, y,       LargeSize, IconName);
  else if (IconName == "02d" || IconName == "02n") MostlySunny(x, y, LargeSize, IconName);
  else if (IconName == "03d" || IconName == "03n") Cloudy(x, y,      LargeSize, IconName);
  else if (IconName == "04d" || IconName == "04n") MostlyCloudy(x, y, LargeSize, IconName);
  else if (IconName == "09d" || IconName == "09n") ChanceRain(x, y,  LargeSize, IconName);
  else if (IconName == "10d" || IconName == "10n") Rain(x, y,        LargeSize, IconName);
  else if (IconName == "11d" || IconName == "11n") Tstorms(x, y,     LargeSize, IconName);
  else if (IconName == "13d" || IconName == "13n") Snow(x, y,        LargeSize, IconName);
  else if (IconName == "50d")                      Haze(x, y,        LargeSize, IconName);
  else if (IconName == "50n")                      Fog(x, y,         LargeSize, IconName);
  else                                             Nodata(x, y,      LargeSize);
}
//#########################################################################################
uint8_t StartWiFi() {
  Serial.print(F("\r\nConnecting to: ")); Serial.println(String(ssid));
  IPAddress dns(8, 8, 8, 8); // Google DNS
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(ESP32_WIFI_MAX_POWER); // Bring ESP32 wifi power level down to solve connection issues
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  uint8_t connectionStatus;
  bool AttemptConnection = true;
  while (AttemptConnection) {
    connectionStatus = WiFi.status();
    if (millis() > start + 15000) { // Wait 15-secs maximum
      AttemptConnection = false;
    }
    if (connectionStatus == WL_CONNECTED || connectionStatus == WL_CONNECT_FAILED) {
      AttemptConnection = false;
    }
    delay(100);
  }
  if (connectionStatus == WL_CONNECTED) {
    Serial.println("WiFi connected at: " + WiFi.localIP().toString());
  }
  else Serial.println("WiFi connection *** FAILED ***");
  return connectionStatus;
}
//#########################################################################################
void StopWiFi() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}
//#########################################################################################
boolean SetupTime() {
  configTime(0, 0, "0.uk.pool.ntp.org", "time.nist.gov");
  setenv("TZ", Timezone, 1);
  tzset(); // Set the TZ environment variable
  delay(100);
  bool TimeStatus = UpdateLocalTime();
  return TimeStatus;
}
//#########################################################################################
boolean UpdateLocalTime() {
  struct tm timeinfo;
  char output[30], day_output[30];
  while (!getLocalTime(&timeinfo, 5000)) { // Wait for 5-sec for time to synchronise
    Serial.println(F("Failed to obtain time"));
    return false;
  }
  strftime(output, 30, "%H", &timeinfo);
  CurrentHour = timeinfo.tm_hour;
  CurrentMin  = timeinfo.tm_min;
  CurrentSec  = timeinfo.tm_sec;
  //See http://www.cplusplus.com/reference/ctime/strftime/
  //Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
  Serial.println(&timeinfo, "%H:%M:%S");                 // Displays: 14:05:49
  if (Units == "M") {
    strftime(day_output, 30, "%d-%b-%y", &timeinfo);     // Displays: 24-Jun-17
    strftime(output, 30, "%H:%M", &timeinfo);            // Creates: '14:05'
  }
  else {
    strftime(day_output, 30, "%b-%d-%y", &timeinfo);     // Creates: Jun-24-17
    strftime(output, 30, "%I:%M%p", &timeinfo);          // Creates: '2:05pm'
  }
  Date_str = day_output;
  Time_str = output;
  return true;
}
//#########################################################################################
// Symbols are drawn on a relative 10x10grid and 1 scale unit = 1 drawing unit
void addcloud(int x, int y, int scale, int linesize) {
  //Draw cloud outer
  display.fillCircle(x - scale * 3, y, scale, GxEPD_BLACK);                      // Left most circle
  display.fillCircle(x + scale * 3, y, scale, GxEPD_BLACK);                      // Right most circle
  display.fillCircle(x - scale, y - scale, scale * 1.4, GxEPD_BLACK);            // left middle upper circle
  display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75, GxEPD_BLACK); // Right middle upper circle
  display.fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1, GxEPD_BLACK); // Upper and lower lines
  //Clear cloud inner
  display.fillCircle(x - scale * 3, y, scale - linesize, GxEPD_WHITE);           // Clear left most circle
  display.fillCircle(x + scale * 3, y, scale - linesize, GxEPD_WHITE);           // Clear right most circle
  display.fillCircle(x - scale, y - scale, scale * 1.4 - linesize, GxEPD_WHITE); // left middle upper circle
  display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize, GxEPD_WHITE); // Right middle upper circle
  display.fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9, scale * 2 - linesize * 2 + 2, GxEPD_WHITE); // Upper and lower lines
}
//#########################################################################################
void addrain(int x, int y, int scale) {
  y = y + scale / 2;
  for (int i = 0; i < 6; i++) {
    display.drawLine(x - scale * 4 + scale * i * 1.3 + 0, y + scale * 1.9, x - scale * 3.5 + scale * i * 1.3 + 0, y + scale, GxEPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 4 + scale * i * 1.3 + 1, y + scale * 1.9, x - scale * 3.5 + scale * i * 1.3 + 1, y + scale, GxEPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.3 + 2, y + scale * 1.9, x - scale * 3.5 + scale * i * 1.3 + 2, y + scale, GxEPD_BLACK);
    }
  }
}
//#########################################################################################
void addsnow(int x, int y, int scale) {
  int dxo, dyo, dxi, dyi;
  for (int flakes = 0; flakes < 5; flakes++) {
    for (int i = 0; i < 360; i = i + 45) {
      dxo = 0.5 * scale * cos((i - 90) * 3.14 / 180); dxi = dxo * 0.1;
      dyo = 0.5 * scale * sin((i - 90) * 3.14 / 180); dyi = dyo * 0.1;
      display.drawLine(dxo + x + 0 + flakes * 1.5 * scale - scale * 3, dyo + y + scale * 2, dxi + x + 0 + flakes * 1.5 * scale - scale * 3, dyi + y + scale * 2, GxEPD_BLACK);
    }
  }
}
//#########################################################################################
void addtstorm(int x, int y, int scale) {
  y = y + scale / 2;
  for (int i = 0; i < 5; i++) {
    display.drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0, y + scale, GxEPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1, y + scale, GxEPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2, y + scale, GxEPD_BLACK);
    }
    display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0, GxEPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1, GxEPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2, GxEPD_BLACK);
    }
    display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5, GxEPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5, GxEPD_BLACK);
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5, GxEPD_BLACK);
    }
  }
}
//#########################################################################################
void addsun(int x, int y, int scale) {
  int linesize = 1;
  int dxo, dyo, dxi, dyi;
  display.fillCircle(x, y, scale, GxEPD_BLACK);
  display.fillCircle(x, y, scale - linesize, GxEPD_WHITE);
  for (float i = 0; i < 360; i = i + 45) {
    dxo = 2.2 * scale * cos((i - 90) * 3.14 / 180); dxi = dxo * 0.6;
    dyo = 2.2 * scale * sin((i - 90) * 3.14 / 180); dyi = dyo * 0.6;
    if (i == 0   || i == 180) {
      display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
      if (scale != Small) {
        display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y, GxEPD_BLACK);
        display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y, GxEPD_BLACK);
      }
    }
    if (i == 90  || i == 270) {
      display.drawLine(dxo + x, dyo + y - 1, dxi + x, dyi + y - 1, GxEPD_BLACK);
      if (scale != Small) {
        display.drawLine(dxo + x, dyo + y + 0, dxi + x, dyi + y + 0, GxEPD_BLACK);
        display.drawLine(dxo + x, dyo + y + 1, dxi + x, dyi + y + 1, GxEPD_BLACK);
      }
    }
    if (i == 45  || i == 135 || i == 225 || i == 315) {
      display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
      if (scale != Small) {
        display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y, GxEPD_BLACK);
        display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y, GxEPD_BLACK);
      }
    }
  }
}
//#########################################################################################
void addfog(int x, int y, int scale, int linesize) {
  y -= 10;
  linesize = 1;
  for (int i = 0; i < 6; i++) {
    display.fillRect(x - scale * 3, y + scale * 1.5, scale * 6, linesize, GxEPD_BLACK);
    display.fillRect(x - scale * 3, y + scale * 2.0, scale * 6, linesize, GxEPD_BLACK);
    display.fillRect(x - scale * 3, y + scale * 2.7, scale * 6, linesize, GxEPD_BLACK);
  }
}
//#########################################################################################
void MostlyCloudy(int x, int y, bool IconSize, String IconName) {
  int scale = Large, linesize = 3;
  if (IconSize == SmallIcon) {
    scale = Small;
    linesize = 1;
  }
  if (IconName.endsWith("n")) addmoon(x, y, scale);
  addcloud(x, y, scale, linesize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale);
  addcloud(x, y, scale, linesize);
}
//#########################################################################################
void MostlySunny(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addsun(x - scale * 1.8, y - scale * 1.8 + offset, scale);
}
//#########################################################################################
void Rain(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  if (LargeSize) {
    scale = Large;
    offset = 12;
  }
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addrain(x, y + offset, scale);
}
//#########################################################################################
void Cloudy(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
}
//#########################################################################################
void Sunny(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  scale = scale * 1.5;
  addsun(x, y + offset, scale);
}
//#########################################################################################
void ExpectRain(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  addsun(x - scale * 1.8, y - scale * 1.8 + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addrain(x, y + offset, scale);
}
//#########################################################################################
void ChanceRain(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  addsun(x - scale * 1.8, y - scale * 1.8 + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addrain(x, y + offset, scale);
}
//#########################################################################################
void Tstorms(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addtstorm(x, y + offset, scale);
}
//#########################################################################################
void Snow(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addsnow(x, y + offset, scale);
}
//#########################################################################################
void Fog(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addfog(x, y + offset, scale, linesize);
}
//#########################################################################################
void Haze(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n")) addmoon(x, y + offset, scale);
  addsun(x, y + offset, scale * 1.4);
  addfog(x, y + offset, scale * 1.4, linesize);
}
//#########################################################################################
void addmoon (int x, int y, int scale) {
  display.fillCircle(x - 20, y - 15, scale, GxEPD_BLACK);
  display.fillCircle(x - 15, y - 15, scale * 1.6, GxEPD_WHITE);
}
//#########################################################################################
void Nodata(int x, int y, bool LargeSize) {
  int scale = Small, offset = 0;
  if (LargeSize) {
    scale = Large;
    offset = 7;
  }
  if (scale == Large)  display.setFont(&FreeMonoBold12pt7b); else display.setFont(&DejaVu_Sans_Bold_11);
  drawString(x - 20, y - 10 + offset, "N/A", LEFT);
}
//#########################################################################################
void drawString(int x, int y, String text, alignment align) {
  int16_t  x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
  uint16_t w, h;
  display.setTextWrap(false);
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  if (align == RIGHT)  x = x - w;
  if (align == CENTER) x = x - w / 2;
  display.setCursor(x, y + h);
  display.print(text);
}
//#########################################################################################
void drawStringMaxWidth(int x, int y, int text_width, String text, alignment align) {
  int16_t  x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
  uint16_t w, h;
  if (text.length() > text_width * 2) text = text.substring(0, text_width * 2); // Truncate if too long for 2 rows of text
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  if (align == RIGHT)  x = x - w;
  if (align == CENTER) x = x - w / 2;
  display.setCursor(x, y + h);
  display.println(text.substring(0, text_width));
  if (text.length() > text_width) {
    display.setCursor(x, y + h * 2);
    display.println(text.substring(text_width));
  }
}
//#########################################################################################
void DisplayWxPerson(int x, int y, String IconName) {
  display.drawRect(x, y, 86, 97, GxEPD_BLACK);
  x = x + 3;
  y = y + 7;
  // NOTE: Using 'drawInvertedBitmap' and not 'drawBitmap' so that images are WYSIWYG, otherwise all images need to be inverted
  if      (IconName == "01d" || IconName == "01n")  display.drawInvertedBitmap(x, y, uWX_Sunny,       80, 80, GxEPD_BLACK);
  else if (IconName == "02d" || IconName == "02n")  display.drawInvertedBitmap(x, y, uWX_MostlySunny, 80, 80, GxEPD_BLACK);
  else if (IconName == "03d" || IconName == "03n")  display.drawInvertedBitmap(x, y, uWX_Cloudy,      80, 80, GxEPD_BLACK);
  else if (IconName == "04d" || IconName == "04n")  display.drawInvertedBitmap(x, y, uWX_MostlySunny, 80, 80, GxEPD_BLACK);
  else if (IconName == "09d" || IconName == "09n")  display.drawInvertedBitmap(x, y, uWX_ChanceRain,  80, 80, GxEPD_BLACK);
  else if (IconName == "10d" || IconName == "10n")  display.drawInvertedBitmap(x, y, uWX_Rain,        80, 80, GxEPD_BLACK);
  else if (IconName == "11d" || IconName == "11n")  display.drawInvertedBitmap(x, y, uWX_TStorms,     80, 80, GxEPD_BLACK);
  else if (IconName == "13d" || IconName == "13n")  display.drawInvertedBitmap(x, y, uWX_Snow,        80, 80, GxEPD_BLACK);
  else if (IconName == "50d")                       display.drawInvertedBitmap(x, y, uWX_Haze,        80, 80, GxEPD_BLACK);
  else if (IconName == "50n")                       display.drawInvertedBitmap(x, y, uWX_Fog,         80, 80, GxEPD_BLACK);
  else                                              display.drawInvertedBitmap(x, y, uWX_Nodata,      80, 80, GxEPD_BLACK);
}

void InitialiseDisplay() {
  display.init(115200, true, 50, false);
  //// display.init(); for older Waveshare HAT's
  //SPI.end();
  //SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, EPD_CS);
  display.setRotation(3);
  display.setTextSize(0);
  display.setFont(&DejaVu_Sans_Bold_11);
  display.setTextColor(GxEPD_BLACK);
  display.fillScreen(GxEPD_WHITE);
  display.setFullWindow();
}

/*
  Version 1.0 Initial release

  Version 1.1 Added support for Waveshare ESP32 Driver board

  Version 1.2 Changed GxEPD2 initialisation from 115200 to 0
  1.  Display.init(115200); becomes display.init(0); to stop blank screen following update to GxEPD2
  
  Version 1.3 
  1.  Added extra 20-secs to sleep delay to allow for slower ESP32 RTC timers
  
  Version 1.4
  1. Modified for GxEPD2
  
*/
