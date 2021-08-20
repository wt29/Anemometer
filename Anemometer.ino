/* 
Anemometer

A headless - no display - sketch to count hall effect pulses and sent the rate up to emoncms for logging

Featuring the LOLIN D1 Mini - ESP8266
https://lolin.aliexpress.com/store

You will need a file "data.h" - copy the template below.
if you have lots of these keep the individual configs in myDeviceName.h for easy reference to save remembering config and board versions.
Occasionaly you may need to add an extra setting from the template below if you are enabling new features.
e.g. create a "kitchen.h" and change the local code to just include kitchen.h?

Also add that file.h or *.h to the .gitignore so you dont upload your wifi password to github!

-------------------------------------
//template for data.h

//** Node and Network Setup

#define NODENAME "<Your NodeName>";                 // eg "Kitchen"  Required and UNIQUE per site.  Also used to find mdns eg NODENAME.local

#define WIFI                                        // Default is true to enable WiFi
#define LOCALSSID "<Your WiFi SSID>";               // eg "AwesomeWifi"  Required Can't use plain SSID as the WiFi library now defines it.
#define PASSWORD "<Your WiFI Password>";            // eg "HorseStapleSomething"  Required

#define HOST "<Your emoncms host fqdn>";            // eg  "emoncms.org" Required for logging. Note:just the host not the protocol
#define MYAPIKEY "<Your emoncms API write key>";    // Required Get it from your MyAccount details in your emoncms instance

If required, enable the following block to your data.h to set fixed IP addresses 
#define STATIC_IP
IPAddress staticIP( 192,168,1,22 );
IPAddress gateway( 192,168,1,1 );
IPAddress subnet(255,255,255,0 );
IPAddress dns1( 8,8,8,8 );

-------------------------------------

*/
#define VERSION 1.01            // First working version 
                                // 1.00 Initial version

#warning Setup your data.h.  Refer to template in code.

//debug mode
#define DEBUG
// If we define then DEBUG_LOG will log a string, otherwise
// it will be ignored as a comment.
#ifdef DEBUG
#  define DEBUG_LOG(x) Serial.print(x)
#else
#  define DEBUG_LOG(x)
#endif

//Node and Network Setup
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>   // Include the WebServer library

// Needed to move this here as the IPAddress types aren't declared until the WiFi libs are loaded
#include "data.h"             // Create this file from template above.  
                              // This means we dont keep uploading API key+password to GitHub. (data.h should be ignored in repository)

const char* nodeName = NODENAME;
const char* ssid = LOCALSSID;
const char* password = PASSWORD;
const char* host = HOST;
const char* APIKEY = MYAPIKEY;

WiFiClient client;              // Instance of WiFi Client
ESP8266WebServer server(80);    // Create a webserver object that listens for HTTP request on port 80
void handleRoot();              // function prototypes for HTTP handlers
void handleNotFound();

int waitForWiFi = 20000 ;         // How long to wait for the WiFi to connect - 10 Seconds should be enough 
int startWiFi;
int connectMillis = millis();     // this gets reset after every successful data push

int poll = 60000;                        // Poll the sensor every 60 seconds (or so)
int lastRun = millis() - (poll + 1);     // Force a run on boot. Includes connect to WiFi
int triggered;                           // Count of the number of Hall triggers

float elapsedMinutes = 0; // How much "minutes" have passed
float revsPerMinute = 0;  // there are 2 transitions per magnet per rev
  
const int hallPin = D4;
volatile bool ledState = LOW;

void setup()
{
  Serial.begin(115200); //baud rate
  Serial.println();

  pinMode( LED_BUILTIN, OUTPUT);
  pinMode( hallPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin), hall_ISR, HIGH);

if (MDNS.begin( nodeName )) {              // Start the mDNS responder for <nodeName>.local
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error setting up MDNS responder!");
  }

server.on("/", handleRoot);               // Call the 'handleRoot' function when a client requests URI "/"
server.onNotFound(handleNotFound);        // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"

server.begin();                           // Actually start the server
Serial.println("HTTP server started");
}       // Setup

void loop() {

 if ( millis() > 14400000) {   // Reboot every 4 hours - I have crappy internet. You may not need this
      Serial.println("Rebooting");
      ESP.restart();           // Kick it over and try from the beginning
  }

 server.handleClient();                         // Listen for HTTP requests from clients
 
 digitalWrite(LED_BUILTIN, ledState);           // only useful for demo. If running on battery, this should be commented out
 
if ( millis() > lastRun + poll ) {              // only want this happening every so often - see poll value
  
  elapsedMinutes = (millis()-lastRun)/1000/60;  // How much "minutes" have passed - this isn't totally accurate but sufficient for this purpose
  revsPerMinute = triggered/2/elapsedMinutes;   // there is 1 transition per magnet (2 of) per rev
  
  lastRun = millis();                           // don't want to add Wifi Connection latency to the poll
  triggered = 0;                                // Still gunna be counting in the background
  
  Serial.println();
  Serial.print("Elapsed Minutes : ");
  Serial.println( elapsedMinutes );
  Serial.print("Revs per minute : ");
  Serial.println( revsPerMinute);
  Serial.print("Free Heap : ");
  Serial.println(ESP.getFreeHeap());

  if (WiFi.status() != WL_CONNECTED){
    connectWiFi();
  }
  if (WiFi.status() != WL_CONNECTED ) {
    Serial.println("Connection failed!]");
    for (int i = 0; i <= 2; i++) {          // flash 3 times. 
      digitalWrite(LED_BUILTIN, true);      // This is crap. Should be something async like Bob overeasy does it.
      delay(.5);
      digitalWrite(LED_BUILTIN, false);
      delay(.5);
    }
 
  }
  else
  {
    Serial.printf("\n[Connecting to %s ... ", host, "\n");
      
    if (client.connect(host, 80))     {
      Serial.println("Connected]");
      Serial.println("[Sending a request]");

    String request  = "GET " ;
           request += "/input/post?node=";
           request += nodeName;
           request += "&fulljson={\"anemometer\":";
           request += revsPerMinute;
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
  WiFi.begin(ssid, password);
  WiFi.hostname( nodeName );     // This will show up in your DHCP server

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

  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println( WiFi.localIP());
  Serial.printf("Connection status: %d\n", WiFi.status());

}

void handleRoot() {
  String response = "<h1>You have reached the Anemometer</h1>";
         response += "<br>";
         response += "Number of triggerings <b>" + String(triggered) + "</b><br>";
         response += "Current RPM is <b>" + String(revsPerMinute) + "</b><br>";
         response += "Node Name <b>" + String(nodeName) + "</b><br>"; 
         response += "Local IP is: <b>" + WiFi.localIP().toString() + "</b><br>";
         response += "Free Heap Space <b>" + String(ESP.getFreeHeap()) + " bytes</b><br>";
         response += "Software Version <b>" + String(VERSION) + "</b>";
         
  server.send(200, "text/html", response );   // Send HTTP status 200 (Ok) and send some text to the browser/client
}

void handleNotFound(){
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

ICACHE_RAM_ATTR void hall_ISR() 
{
triggered += 1;
ledState = !ledState;
}
