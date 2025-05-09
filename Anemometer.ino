/*
  Anemometer, rain Sensor and WindVane

  A headless - no display - sketch to interface (optionally)

  > Anemometer  - uses a hall effect sensor to count rotations
  > Rain Gauge  - uses a tipping bucket and hall effect to count rain "buckets"
  > Rain Sensor - uses a Hydreon RG-15 rain sensor. Note - this is a 12V device and will require s "buck" board to produce 5V for the Wemos D1 
  > Wind Direction Indicator - uses a 360 degree variable resistor.
  
  For the Anemometer is counts hall effect pulses and send the rate up to emoncms for logging.

  Featuring the LOLIN D1 Mini - ESP8266
  https://lolin.aliexpress.com/store

  You will need a file "data.h" - copy the template below.
  if you have lots of these keep the individual configs in myDeviceName.h for easy reference to save remembering config and board versions.
  Occasionaly you may need to add an extra setting from the template below if you are enabling new features.
  e.g. create a "kitchen.h" and change the local code to just include kitchen.h?

  Also add that file.h or *.h to the .gitignore so you dont upload your wifi password to github!

   -------------------------------------
  // template for data.h

  #define ANEMOMETER                                  // you may or may not any of these installed
  #define RAINSENSOR                                  // Experimental support for Hydreon RG-11 Rain gauge
  
  #define WINDVANE                                    // Installed / configured
  // Use the "VaneCalibrate.ino" Sketch to get these values. 
  // Different VRs and bias resistors will give you different values to these. 
  // Your support rod will almost certainly be pointed at a different orientation to mine
  #define WV_ANALOG_MAX 557
  #define WV_ANALOG_MIN 19
  #define WV_ROD_OFFSET 70     // actually a combination to the zero on the VR and the actual orientation  
  
  #define RAINGAUGE                                   // Installed / connected?
  #define RG_MILLILITRES_PER_BUCKET  1.875     // ml per bucket
  #define RG_BUCKETS_PER_RAIN_ML   6.03
  // RG funnel is 60mm radius so area is Pi R Squared so 3.1415 * 60 * 60 = 11310 sq mm
  // 1mm of rain is therefore 11310 cubic mm 
  // 1 bucket from my calibration ia 1.875 ml which is 1875 cubic millimeters
  // 1mm of rain is therefore 11310 / 1.875 = 6.031 buckets (rounded )
  
  // Node and Network Setup

  #define NODENAME "<Your NodeName>";                 // eg "Kitchen"  Required and UNIQUE per site.  Also used to find mdns eg NODENAME.local

  #define APARRAY {<"ap1">,<"ap2">,<"ap....">}        // An array of possible Access points - usually just 1!
  #define PASSARRAY {<"password1">,<"password2">,<"password...">}  // Array of paswords matching you APs
  #define APCOUNT 4  // How many you have defined

  #define HOST "<Your emoncms host fqdn>"            // eg  "emoncms.org" Required for logging. Note:just the host not the protocol
  #define MYAPIKEY "<Your emoncms API write key>";    // Required Get it from your MyAccount details in your emoncms instance
  #define GUSTPERCENT                                 // How many RPM above the 5 minute average defines a "gust"
  #define CALIBRATION                                 // Future - a multiplier to convert RPM into whatever you are going to log - Knots, M/s, MPH etc
  #define TARGETUNITS  "M/s"                          // Future - once calibration is done, what will we be using as a unit.
  
  If required, enable the following block to your data.h to set fixed IP addresses

  #define STATIC_IP
  IPAddress staticIP( 192,168,1,22 );
  IPAddress gateway( 192,168,1,1 );
  IPAddress subnet(255,255,255,0 );
  IPAddress dns1( 8,8,8,8 );

  -------------------------------------

*/
#define VERSION 1.06            // Rain Gauge code, some fixes for wind vane
// 1.05 EEPROM save of Max wind speed.
// 1.04 Calibration values, reboot remote
// 1.03 Time Client, uptimes
// 1.02 AP Arrays, 5 minute averages
// 1.00 Initial version

// #define C3MINI

#include <EEPROM.h>           // Going to save some Max/Min stuff
#define EEPROM_SIZE 32        // 4 bytes each for largestGust, timeofLargestGust

//Node and Network Setup

#ifdef C3MINI
 #include <WiFi.h>
 #include <WifiMulti.h>
 #include <ESPmDNS.h>
 #include <WebServer.h>
#else
 #include <ESP8266WiFi.h>
 #include <ESP8266WiFiMulti.h>   // Include the Wi-Fi-Multi library
 #include <ESP8266mDNS.h>
 #include <ESP8266WebServer.h>   // Include the WebServer library
#endif

#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Needed to move this here as the IPAddress types aren't declared until the WiFi libs are loaded
// This means we dont keep uploading API key+password to GitHub. (data.h should be ignored in repository)
#include "data.h"             // Create this file from template above.  

const char* nodeName = NODENAME;
const char* host = HOST;
const char* APIKEY = MYAPIKEY;
const char* ssid = LOCALSSID;
const char* password = WIFIPASSWORD;
//const char* passwords[] = PASSARRAY;
//const char* accessPoints[] = APARRAY;
const int gustPercent = GUSTPERCENT;

#ifdef C3MINI
 WiFiMulti wifiMulti;
 WebServer server(80);
#else
//  ESP8266WiFiMulti wifiMulti;      // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'
 ESP8266WebServer server(80);     // Create a webserver object that listens for HTTP request on port 80
#endif
WiFiClient client;                // Instance of WiFi Client

void handleRoot();                // function prototypes for HTTP handlers
void handleNotFound();            // Something it don't understand
void rebootDevice();              // Kick it over remotely

const uint32_t waitForWiFi = 5000 ;         // How long to wait for the WiFi to connect - 5 Seconds should be enough
int startWiFi;

int connectMillis = millis();     // this gets reset after every successful data push

int poll = 60000;                       // Poll the sensor every 60 seconds (or so)
long unsigned int lastRun = millis() - (poll + 1);    // Force a run on boot. Includes connect to WiFi
float elapsedMinutes = 0;               // How much "minutes" have passed
unsigned int numberOfPolls;             // Total # of polls since rebooted

#ifdef ANEMOMETER
const int anemometerPin = D2;
unsigned int an_triggered = 1;              // Count of the number of Hall triggers
unsigned int an_totalTrigs;                 // Just for fun - number of triggerings since boot.
unsigned int gustSpeed = 0;                 // define gusts as + 20%
unsigned int largestGust = 0;               // The ubiquitoius self explanatory variable
unsigned long timeOfLargestGust;            // how long ago the gust ran

float an_RPM = 0;                               // there are 2 transitions per magnet per rev
float an_fiveMinuteAverage = 0.0 ;              // should be obvious what is going on
int an_fiveMinuteSamples[5] = {0, 0, 0, 0, 0} ; // roughly one poll every minute
#endif

#ifdef WINDVANE
int windVanePin = A0;           // An analog pin
int readVane;                   // Analog read value
int vaneOutput = 0;             // is in degrees in the final bit
String vaneDirection = "XX";    // Undefined at start
// Use the "VaneCalibrate.ino" Sketch to get these values

float wvPerDegree = ( ( WV_ANALOG_MAX - WV_ANALOG_MIN) / 360.0 );  
                                //  + if > 180 and less than 360
                                //  i.e. if your vane support rod is West then +90 is the offest
#endif

#ifdef RAINSENSOR
int rainSensorPin = D6;
int rs_triggered;                                // Count of the number of Hall triggers
unsigned int rs_totalTrigs;                      // Just for fun - number of triggerings since boot.
float rs_trigsPerMinute = 0.0;
float rs_fiveMinuteAverage = 0.0 ;               // should be obvious what is going on
int rs_fiveMinuteSamples[5] = {0, 0, 0, 0, 0} ;  // roughly one poll every minute
#endif

#ifdef RAINGAUGE
int rainGaugePin = D5;
int rg_triggered;                                // Count of the number of Hall triggers
unsigned int rg_totalTrigs;                      // Just for fun - number of triggerings since boot.
float rg_trigsPerMinute = 0.0;                   // we have a rough 1 minute poll / upload.
float rg_fiveMinuteAverage = 0.0 ;               // should be obvious what is going on
int rg_fiveMinuteSamples[5] = {0, 0, 0, 0, 0} ;  // roughly one poll every minute  - gives trends
int rg_barrel_sq_mm = (RG_BARREL_RADIUS * RG_BARREL_RADIUS) * 3.1451;  // 
#endif

const long utcOffsetInSeconds = 36000;       // Sydney is 10 hours ahead - you will have to readjust
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

#ifndef WINDVANE   // May as well measure the VCC
ADC_MODE(ADC_VCC);
#endif

String timeString;
String startTime;
unsigned long startAbsoluteTime;    // How long have we been running for?

const int ledPin = LED_BUILTIN;
int ledState = LOW;

//-----------------------------------------
void setup()
{
  
  Serial.begin(115200);     // baud rate
  millisDelay(1) ;          // allow the serial to init
  Serial.println();         // clean up a little

//  for (int i = 0; i < APCOUNT; i++) {
//    wifiMulti.addAP( accessPoints[i], passwords[i] );     // Add all the APs and passwords from the arrays
//  }

  WiFi.mode(WIFI_STA);  // Station Mode
  connectWiFi();        // This thing isn't any use without WiFi

#ifdef ANEMOMETER  
  EEPROM.begin(32);                       // this number in "begin()" is ESP8266. EEPROM is emulated so you need buffer to emulate.
  EEPROM.get(0,largestGust);              // Should read 4 bytes for each of these thangs
  if ( isnan( largestGust ) ) { largestGust = 0; }  // Having issues during devel with "nan" (not a number) being written to EEPROM
  EEPROM.get(4,timeOfLargestGust);
  if (largestGust > 20000) {              // Hurricane Strength - sometimes first value is stupid
   largestGust = 0;
   EEPROM.put( 0, largestGust );
   EEPROM.commit();
  }
#endif

  pinMode( ledPin, OUTPUT );
  
#ifdef ANEMOMETER
  pinMode( anemometerPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(anemometerPin), anemometer_ISR, HIGH);
#endif

#ifdef RAINSENSOR
  pinMode( rainSensorPin, INPUT_PULLUP );
  attachInterrupt(digitalPinToInterrupt(rainSensorPin), rainSensor_ISR, HIGH);
#endif

#ifdef RAINGAUGE
  pinMode( rainGaugePin, INPUT_PULLUP );
  attachInterrupt(digitalPinToInterrupt(rainGaugePin), rainGauge_ISR, HIGH);
#endif

 if (MDNS.begin( nodeName )) {              // Start the mDNS responder for <nodeName>.local
    Serial.println("mDNS responder started");
  }
  else
  {
    Serial.println("Error setting up MDNS responder!");
  }

  server.on("/", handleRoot);                   // Call the 'handleRoot' function when a client requests URI "/"
  server.onNotFound(handleNotFound);            // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"
  server.on("/reboot", handleRebootDevice);     // Kick over remotely
  server.on("/resetgusts", handleResetGusts);   // Fix the occasional stupid gust value
  server.begin();                               // Actually start the server
  
  Serial.println("HTTP server started");
  ArduinoOTA.begin();                           // Remote updates
  ArduinoOTA.setHostname( nodeName );

  timeClient.begin();
  startTime = getInternetTime();
  startAbsoluteTime = timeClient.getEpochTime();

}       // Setup

//------------------------------------------------------

void loop() {

  server.handleClient();                         // Listen for HTTP requests from clients
  ArduinoOTA.handle();
#ifdef DEBUG
//  digitalWrite(ledPin, ledState);           // only useful for demo. If running on battery, this should be commented out
#endif
  if ( millis() > lastRun + 60000 ) {             // only want this happening every minute (or so) 
    lastRun = millis();                           // don't want to add Wifi Connection latency to the poll
    numberOfPolls += 1;                           // Another poll recorded

#ifdef ANEMOMETER
    an_RPM = an_triggered / 2 ;   // 1 transition per magnet (2 of) per rev
    an_triggered = 0;             // How many trigs in the last "minute"        
    an_fiveMinuteAverage = 0.0;

    for (int i = 3; i >= 0; i--) {                 // Only pushing 4 samples along here
      an_fiveMinuteSamples[i + 1] = an_fiveMinuteSamples[i];
      an_fiveMinuteAverage += ( an_fiveMinuteSamples[i + 1] );
    }
    an_fiveMinuteSamples[0] = an_RPM;
    an_fiveMinuteAverage += an_fiveMinuteSamples[0];
    an_fiveMinuteAverage /= 5;
    
    // Calculate if a gust has occured
    if (an_RPM > ( an_fiveMinuteAverage + ( an_fiveMinuteAverage / (100 / gustPercent) ) ) ) {
      gustSpeed = an_RPM;
    }
    if (gustSpeed > largestGust) {
      largestGust = gustSpeed;
      timeOfLargestGust = timeClient.getEpochTime();
      EEPROM.put( 0, largestGust );
      EEPROM.put( 4, timeOfLargestGust );
      EEPROM.commit();

    }
    Serial.print( "AN Triggers : " );
    Serial.println( an_triggered );
    Serial.print("Elapsed Minutes : ");
    Serial.println( elapsedMinutes );
    Serial.print("Revs per minute : ");
    Serial.println( an_RPM );

#endif
#ifdef WINDVANE
    readVane = analogRead(A0);

    readVane -= WV_ANALOG_MIN;       // delete the offset value of the VR
    readVane = (readVane/wvPerDegree) + WV_ROD_OFFSET;  // Calculate the offset and figure out the true compass
    if (readVane > 360 ) {readVane -= 360;}
 
    if ((readVane >= 338) & (readVane < 22) ) {      // North
      vaneOutput = 0;
      vaneDirection = "North";
    }
    else if ((readVane >= 23) & (readVane < 68)) {  // NE
      vaneOutput = 45;
      vaneDirection = "North East";
    }
    else if ((readVane >= 68) & (readVane < 112)) { // E
      vaneOutput = 90;
      vaneDirection = "East";
    }
    else if ((readVane >= 112) & (readVane < 157)) { // SE
      vaneOutput = 135;
      vaneDirection = "South East";
    }
    else if ((readVane >= 157) & (readVane < 202 )) { // S
      vaneOutput = 180;
      vaneDirection = "South";
    }
    else if ((readVane >= 202) & (readVane < 247)) { // SW
      vaneOutput = 225;
      vaneDirection = "South West";
    }
    else if ((readVane >= 247) & (readVane < 292)) { // W
      vaneOutput = 270;
      vaneDirection = "West";
    }
    else {               // (readVane >=832 & readVane < 960)
      vaneOutput = 315;
      vaneDirection = "North West";
    }               // NW

#endif
#ifdef RAINSENSOR
    rs_trigsPerMinute = rs_triggered; // How many triggers of the rg/minute

    rs_triggered = 0;                                // Still gunna be counting in the background
    rs_fiveMinuteAverage = 0.0;                      // Calculate the latest 5 minute average

    for (int i = 3; i >= 0; i--) {                 // Only pushing 4 samples along here
      rs_fiveMinuteSamples[i + 1] = rs_fiveMinuteSamples[i];
      rs_fiveMinuteAverage += ( rs_fiveMinuteSamples[i + 1] / 5 );

    }
    rs_fiveMinuteSamples[0] = rs_trigsPerMinute;
    rs_fiveMinuteAverage += ( rs_trigsPerMinute / 5 );

#endif
#ifdef RAINGAUGE
    rg_trigsPerMinute = rg_triggered; // How many triggers of the rg/minute

    rg_triggered = 0;                                // Still gunna be counting in the background
    rg_fiveMinuteAverage = 0.0;                      // Calculate the latest 5 minute average

    for (int i = 3; i >= 0; i--) {                 // Only pushing 4 samples along here
      rg_fiveMinuteSamples[i + 1] = rg_fiveMinuteSamples[i];
      rg_fiveMinuteAverage += ( rg_fiveMinuteSamples[i + 1] / 5 );

    }
    rg_fiveMinuteSamples[0] = rg_trigsPerMinute;
    rg_fiveMinuteAverage += ( rg_trigsPerMinute / 5 );

    Serial.println();
    Serial.print("Rain Gauge Triggered : ");
    Serial.println( rg_trigsPerMinute );
    Serial.print("Millilitres per Bucket: ");
    Serial.println( RG_MILLILITRES_PER_BUCKET );
    Serial.print("Bucket Millitres in last minute : ");
    Serial.println( RG_MILLILITRES_PER_BUCKET * rg_trigsPerMinute );
    Serial.print("Bucket Millitres since boot : ");
    Serial.println( RG_MILLILITRES_PER_BUCKET * rg_totalTrigs );
    Serial.print("Rain Mils since boot : ");
    Serial.println( rg_totalTrigs / RG_BUCKETS_PER_RAIN_ML  );
    Serial.print("Rain Mils last minute : ");
    Serial.println( rg_trigsPerMinute / RG_BUCKETS_PER_RAIN_ML );
    Serial.print("Rain Mils 5 minute Average : ");
    Serial.println( rg_fiveMinuteAverage / RG_BUCKETS_PER_RAIN_ML );

#endif
    Serial.println();
    Serial.print("Free Heap : ");
    Serial.println(ESP.getFreeHeap());

    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
    if (WiFi.status() != WL_CONNECTED ) {
      Serial.println("Connection failed!]");
      for (int i = 0; i <= 2; i++) {          // flash 3 times.
        digitalWrite(ledPin, HIGH);      // This is crap. Should be something async like Bob overeasy does it.
        millisDelay(.5);
        digitalWrite(ledPin, LOW);
        millisDelay(.5);
      }

    }
    else
    {
      Serial.println("Connecting to ");
      Serial.print( host );
      if (client.connect(host, 80))     {
        Serial.println("Connected]");
        Serial.println("[Sending a request]");

        String request  = "GET " ;
        request += "/input/post?node=";
        request += nodeName;
#ifdef ANEMOMETER
        request += "&fulljson={\"anemometer\":";
        request += an_fiveMinuteAverage;        // attempt to smooth the graph
#endif
#ifdef RAINSENSOR
        request += ",\"rainSensorTriggers\":";
        request += rs_trigsPerMinute;           // absolute number of triggers
        request += ",\"rainSensor5MinAvg\":";
        request += rs_fiveMinuteAverage;        // average rate over 5 minutes
#endif
#ifdef RAINGAUGE
        request += ",\"rainGaugeMils\":";
        request += rg_trigsPerMinute / RG_BUCKETS_PER_RAIN_ML;           // absolute value of rain in last minute
        request += ",\"rainGauge5MinAvgMils\":";
        request += rg_fiveMinuteAverage / RG_BUCKETS_PER_RAIN_ML;        // average rate over 5 minutes
        request += ",\"rainGaugeTotalTrigs\":";
        request += rg_totalTrigs;                                        // Number of triggers
#endif
#ifdef WINDVANE
        request += ",\"windvane\":";
        request += vaneOutput;                // time on this isn't critical
#endif
        request += "}&apikey=";
        request += APIKEY;

        Serial.println( request );
        client.println( request );

        Serial.println("[Response:]");

        while (client.connected()) {
          if (client.available()) {
            String resp = "Null";
            resp = client.readStringUntil('\n');  // See what the host responds with.
            Serial.println( resp );

          }  // Client available

        }   // client connected

      }    // client connect

    }     // else

  }      // Millis Loop

}       // Loop

void connectWiFi() {

#ifdef STATIC_IP  
 WiFi.config( staticIP, gateway, subnet, dns1 );
#endif
  String newHostName = NODENAME;
  WiFi.setHostname( newHostName.c_str() );     // This will show up in your DHCP server
  WiFi.begin(ssid, password);

  String strDebug = ssid ;
  strDebug += "  ";
  strDebug +=  password;
  Serial.println( strDebug );
  
  startWiFi = millis() ;        // When we started waiting
  // Loop and wait 
  while ((WiFi.status() != WL_CONNECTED) && ( (millis() - startWiFi) < waitForWiFi ))
  {
    delay(500);
    Serial.print(".");
  }

//  tft.print("");
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println( WiFi.localIP());
  Serial.printf("Connection status: %d\n", WiFi.status());

}

void handleRoot() {
  String url = "<td><a href=http://" + String(host) + ">" + host + "</a><td></b>";
  Serial.println( url );
  String response =  "<h2>You have reached the Weather Station</h2>";
  response += "<b>This device polls at approximately 1 minute intervals.</b>";
  response += "<p></p><table style=\"width:600\">";
#ifndef WINDVANE   // If we aren't using the ADC then why not do the voltage thing
  response += "<tr><td>VCC </td><td><b>" + String( ESP.getVcc() / 1000.00 ) + "</b></td></tr>";
#endif
  response += "<tr><td>Current time </td><td><b>" + getInternetTime() + "</b></td></tr>";
#ifdef ANEMOMETER
  response += "<tr><td>Last calculated RPM </td><td><b>" + String(an_RPM) + "</b></td></tr>";
  response += "<tr><td>an_triggered value </td><td><b>" + String(an_triggered) + "</b></td></tr>";
  response += "<tr><td>Rolling 5 minute average RPM </td><td><b>" + String(an_fiveMinuteAverage) + "</b></td></tr>";
  response += "<tr><td>5 minute counts </td><td><b>" + String(an_fiveMinuteSamples[0]);
  response += "," + String(an_fiveMinuteSamples[1]);
  response += "," + String(an_fiveMinuteSamples[2]);
  response += "," + String(an_fiveMinuteSamples[3]);
  response += "," + String(an_fiveMinuteSamples[4]) + "</b></td></tr>";
  response += "<tr><td>Calibration value is </td><td><b>" + String(CALIBRATION) + "</b></td></tr>";
  response += "<tr><td>Target Units is </td><td><b>" + String(TARGETUNITS) + "</b></td></tr>";
  response += "<tr><td>Calculated Wind Speed (5 minute Avg) is </td><td><b>" + String(an_fiveMinuteAverage / CALIBRATION) + String(TARGETUNITS) + "</b></td></tr>";
  response += "<tr><td>Total trigs since boot </td><td><b>" + String(an_totalTrigs) + "</b></td></tr>";

  response += "<tr><td>Largest gust RPM detected </td><td><b>" + String(largestGust) + "</b> on <b>" + fullDate( timeOfLargestGust ) + "</b></td></tr>";
  response += "<tr><td>Total Polls (or minutes) </td><td><b>" + String(numberOfPolls) + "</b></td></tr>";
#endif
#ifdef RAINSENSOR
  response += "<tr><td>Rain Sensor 5 minute triggers </td><td><b>" + String(rs_fiveMinuteAverage) + "</b></td></tr>";
  response += "<tr><td>Rain Sensor total triggers </td><td><b>" + String(rs_totalTrigs) + "</b></td></tr>";
#endif
#ifdef RAINGAUGE
  response += "<tr><td>Rain Gauge last minute </td><td><b>" + String(rg_trigsPerMinute) + "</b></td></tr>";
  response += "<tr><td>Rain Gauge 5 minute triggers </td><td><b>" + String(rg_fiveMinuteAverage) + "</b></td></tr>";
  response += "<tr><td>Rain Gauge total triggers </td><td><b>" + String(rg_totalTrigs) + "</b></td></tr>";
  response += "<tr><td>Rain Gauge Rain mils last minute </td><td><b>" + String(rg_trigsPerMinute / RG_BUCKETS_PER_RAIN_ML) + "</b></td></tr>";
  response += "<tr><td>Rain Gauge 5 Avg rain mils </td><td><b>" + String(rg_fiveMinuteAverage/ RG_BUCKETS_PER_RAIN_ML) + "</b></td></tr>";
  response += "<tr><td>Rain Gauge total rain mils </td><td><b>" + String(rg_totalTrigs/ RG_BUCKETS_PER_RAIN_ML) + "</b></td></tr>";
#endif
#ifdef WINDVANE
 response += "<tr><td>Current Wind direction </td><td><b>" + vaneDirection + "</b></td></tr>";
  response += "<tr><td>Vane read </td><td><b>" + String( readVane ) + "</b></td></tr>";
  response += "<tr><td>Offset </td><td><b>" + String( WV_ROD_OFFSET ) + "</b></td></tr>";
#endif
  int runSecs = timeClient.getEpochTime() - startAbsoluteTime;
  int upDays = abs( runSecs / 86400 );
  int upHours = abs( runSecs - ( upDays * 86400 ) ) / 3600;
  int upMins = abs( ( runSecs - (upDays * 86400) - ( upHours * 3600 ) ) / 60 ) ;
  int upSecs = abs( runSecs - (upDays * 86400) - ( upHours * 3600 ) - ( upMins * 60 ) );
  String upTime = String(upDays) + "d " + String( upHours ) + "h " + String(upMins) + "m " + String(upSecs) + "s";

  response += "<tr><td>Uptime  </td><td><b>" + upTime + "</b></td></tr>";
  response += "<tr><td>Node Name </td><td><b>" + String(nodeName) + "</b></td></tr>";
  response += "<tr><td>Currently logging to</td>" + url + "</td></tr>";
  response += "<tr><td>Local IP is: </td><td><b>" + WiFi.localIP().toString() + "</b></td></tr>";
  response += "<tr><td>Connected via AP:</td><td> <b>" + WiFi.SSID() + "</b></td></tr>";
  response += "<tr><td>Free Heap Space </td><td><b>" + String(ESP.getFreeHeap()) + " bytes</b></td></tr>";
  response += "<tr><td>Software Version</td><td> <b>" + String(VERSION) + "</b></td></tr></table>";

  server.send(200, "text/html", response );   // Send HTTP status 200 (Ok) and send some text to the browser/client
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

void handleResetGusts() {                           // Sometimes the max gust number is stupid.
  EEPROM.put( 0, 0 );
  EEPROM.put( 4, timeOfLargestGust );
  EEPROM.commit();
  server.send(200, "text/plain", "Max Gust and Time set to 0"); // Let em know
  millisDelay(5000);      // Hang around
}

void handleRebootDevice() {
  Serial.println( "In reboot Device" );
  server.send(200, "text/html", "<h1>Rebooting " + String(nodeName) + " in 5 seconds</h1>"); // Warn em
  millisDelay( 5000 );
  ESP.restart();
}

#ifdef ANEMOMETER
IRAM_ATTR void anemometer_ISR()
{
  an_triggered += 1;
  an_totalTrigs += 1;

  if( ledState == 1 )            // LED is active LOW
   { 
      ledState = 0;
   }
    else
   { 
      ledState = 1;
   } 
  digitalWrite( ledPin, ledState ); 

}
#endif

#ifdef RAINSENSOR
IRAM_ATTR void rainSensor_ISR()
{
  rs_triggered += 1;
  rs_totalTrigs += 1;
}
#endif

#ifdef RAINGAUGE
IRAM_ATTR void rainGauge_ISR()
{
  rg_triggered += 1;
  rg_totalTrigs += 1;

  if( ledState == 1 )            // LED is active LOW
   { 
      ledState = 0;
   }
    else
   { 
      ledState = 1;
   } 
  digitalWrite( ledPin, ledState ); 

}
#endif

String getInternetTime() {
  timeClient.update();
  return String( timeClient.getFormattedTime() );

}

String fullDate ( unsigned long epoch ) {
  static unsigned char month_days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  static unsigned char week_days[7] = {4, 5, 6, 0, 1, 2, 3}; //Thu=4, Fri=5, Sat=6, Sun=0, Mon=1, Tue=2, Wed=3
  unsigned char ntp_hour, ntp_minute, ntp_second, ntp_week_day, ntp_date, ntp_month, leap_days=0 ;
  String dow, sMonth;
  unsigned short temp_days;
  unsigned int ntp_year, days_since_epoch, day_of_year;

  ntp_second = epoch % 60;
  epoch /= 60;
  ntp_minute = epoch % 60;
  epoch /= 60;
  ntp_hour  = epoch % 24;
  epoch /= 24;

  days_since_epoch = epoch;      //number of days since epoch
  ntp_week_day = week_days[days_since_epoch % 7]; //Calculating WeekDay

  ntp_year = 1970 + (days_since_epoch / 365); // ball parking year, may not be accurate!

  unsigned int i;
  for (i = 1972; i < ntp_year; i += 4) // Calculating number of leap days since epoch/1970
    if (((i % 4 == 0) && (i % 100 != 0)) || (i % 400 == 0)) leap_days++;

  ntp_year = 1970 + ((days_since_epoch - leap_days) / 365); // Calculating accurate current year by (days_since_epoch - extra leap days)
  day_of_year = ((days_since_epoch - leap_days) % 365) + 1;


  if (((ntp_year % 4 == 0) && (ntp_year % 100 != 0)) || (ntp_year % 400 == 0))
  {
    month_days[1] = 29;   //February = 29 days for leap years
//    leap_year_ind = 1;    //if current year is leap, set indicator to 1
  }
  else month_days[1] = 28; //February = 28 days for non-leap years

  temp_days = 0;

  for (ntp_month = 0 ; ntp_month <= 11 ; ntp_month++) //calculating current Month
  {
    if (day_of_year <= temp_days) break;
    temp_days = temp_days + month_days[ntp_month];
  }

  temp_days = temp_days - month_days[ntp_month - 1]; //calculating current Date
  ntp_date = day_of_year - temp_days;


  switch (ntp_week_day) {

    case 0: dow = "Sunday";
      break;
    case 1: dow = "Monday" ;
      break;
    case 2: dow = "Tuesday";
      break;
    case 3: dow = "Wednesday";
      break;
    case 4: dow = "Thursday";
      break;
    case 5: dow = "Friday";
      break;
    case 6: dow = "Saturday";
      break;
    default: break;
  }

  switch (ntp_month) {

    case 1: sMonth = "January";
      break;
    case 2: sMonth = "February";
      break;
    case 3: sMonth = "March";
      break;
    case 4: sMonth = "April";
      break;
    case 5: sMonth = "May";
      break;
    case 6: sMonth = "June";
      break;
    case 7: sMonth = "July";
      break;
    case 8: sMonth = "August";
      break;
    case 9: sMonth = "September";
      break;
    case 10: sMonth = "October";
      break;
    case 11: sMonth = "November";
      break;
    case 12: sMonth = "December";
    default: break;
  }
  return String( dow + " " + ntp_date + " " + sMonth + " " + ntp_hour + ":" + ntp_minute + ":" + ntp_second );
}


// Wait around for a bit
void millisDelay ( long unsigned int mDelay )
{
  long unsigned int now = millis();
  do {
    // Do nothing
  } while ( millis() < now + mDelay);

}
