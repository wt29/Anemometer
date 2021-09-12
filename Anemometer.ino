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

#define APARRAY {<"ap1">,<"ap2">,<"ap3">,<"ap....">}   // An array of possible Access points - usually just 1!
#define PASSARRAY {<"password1">,<"password2">,<"password3">,<"password...">}  // Array of paswords matching you APs
#define APCOUNT 4  // How many you have defined

#define HOST "<Your emoncms host fqdn>";            // eg  "emoncms.org" Required for logging. Note:just the host not the protocol
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
#define VERSION 1.03            // Time Client, uptimes
                                // AP Arrays, 5 minute averages 
                                // 1.00 Initial version

#warning Setup your data.h.  Refer to template in code.

//Node and Network Setup
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>   // Include the Wi-Fi-Multi library
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>   // Include the WebServer library
#include <ArduinoOTA.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Needed to move this here as the IPAddress types aren't declared until the WiFi libs are loaded
#include "data.h"             // Create this file from template above.  
                              // This means we dont keep uploading API key+password to GitHub. (data.h should be ignored in repository)

const char* nodeName = NODENAME;
const char* ssid = LOCALSSID;
const char* password = PASSWORD;
const char* host = HOST;
const char* APIKEY = MYAPIKEY;
char* passwords[] = PASSARRAY;
char* accessPoints[] = APARRAY;
const int gustPercent = GUSTPERCENT;

ESP8266WiFiMulti wifiMulti;     // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'
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
unsigned int totalTrigs;                 // Just for fun - number of triggerings since boot.
unsigned int numberOfPolls;              // start of trend analysis
int gustSpeed = 0;                       // define gusts as + 20%
int largestGust = 0;                     // The ubiquitoius self explanitory variable
unsigned long timeOfLargestGust;         // how long ago the gust ran

float elapsedMinutes = 0;               // How much "minutes" have passed
float revsPerMinute = 0;                // there are 2 transitions per magnet per rev

float fiveMinuteAverage = 0.0 ;               // should be obvious what is going on
int fiveMinuteSamples[5] = {0,0,0,0,0} ;      // roughly one poll every minute
  
const int hallPin = D2;
volatile bool ledState = LOW;

const long utcOffsetInSeconds = 36000;       // Sydney is 10 hours ahead
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
String timeString;
String startTime;
unsigned long startAbsoluteTime;    // How long have we been running for?

void setup()
{
  Serial.begin(115200);     // baud rate
  delay(1) ;                // allow the serial to init
  Serial.println();         // clean up a little

  pinMode( LED_BUILTIN, OUTPUT);
  pinMode( hallPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin), hall_ISR, HIGH);

  for (int i=0; i<=APCOUNT; i++) {
    wifiMulti.addAP( accessPoints[i], passwords[i] );     // Add all the APs and passwords from the arrays
  }

  connectWiFi();        // This thing isn't any use without WiFi
 
  if (MDNS.begin( nodeName )) {              // Start the mDNS responder for <nodeName>.local
      Serial.println("mDNS responder started");
    } 
    else
    {
      Serial.println("Error setting up MDNS responder!");
    }
  
  server.on("/", handleRoot);               // Call the 'handleRoot' function when a client requests URI "/"
  server.onNotFound(handleNotFound);        // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"

  server.begin();                           // Actually start the server
  Serial.println("HTTP server started");

  ArduinoOTA.begin();                       // Remote updates
  ArduinoOTA.setHostname( nodeName );

  timeClient.begin();
  startTime = getInternetTime();
  startAbsoluteTime = timeClient.getEpochTime();

  }       // Setup

void loop() {

 server.handleClient();                         // Listen for HTTP requests from clients
 ArduinoOTA.handle();
 
 digitalWrite(LED_BUILTIN, ledState);           // only useful for demo. If running on battery, this should be commented out
 
if ( millis() > lastRun + poll ) {              // only want this happening every so often - see poll value
  
  elapsedMinutes = (millis()-lastRun)/1000/60;  // How much "minutes" have passed - this isn't totally accurate but sufficient for this purpose
  revsPerMinute = triggered/2/elapsedMinutes;   // there is 1 transition per magnet (2 of) per rev
  
  lastRun = millis();                           // don't want to add Wifi Connection latency to the poll
  triggered = 0;                                // Still gunna be counting in the background
  numberOfPolls += 1;                            // Another poll recorded
  fiveMinuteAverage = 0.0;

  for (int i=3; i>=0; i--) {                     // Only pushing 4 samples along here
   fiveMinuteSamples[i+1] = fiveMinuteSamples[i]; 
   fiveMinuteAverage += ( fiveMinuteSamples[i+1] / 5 );     
  }
  
  fiveMinuteSamples[0] = revsPerMinute;
  fiveMinuteAverage += (revsPerMinute / 5 );
  
  // Calculate if a gust has occured
  if (revsPerMinute > ( fiveMinuteAverage + ( fiveMinuteAverage / (100 / gustPercent) ) ) ) {
    gustSpeed = revsPerMinute;
  }
  if (gustSpeed > largestGust) {
    largestGust = gustSpeed;
    timeOfLargestGust = timeClient.getEpochTime();
  }
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
           request += fiveMinuteAverage;     // attempt to smooth the graph
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
  WiFi.hostname( nodeName );     // This will show up in your DHCP server
  while (wifiMulti.run() != WL_CONNECTED) {
    String strDebug = ssid ;
    strDebug += "  ";
    strDebug +=  password;
    Serial.println( strDebug );
  
    startWiFi = millis() ;        // When we started waiting

    while ((WiFi.status() != WL_CONNECTED) && ( (millis() - startWiFi) < waitForWiFi ))
    {
      delay(500);
      Serial.print(".");
    }
  }
  
  if (WiFi.status() == WL_CONNECTED){
    Serial.println("");
    Serial.print("IP Address: ");
    Serial.println( WiFi.localIP());
    Serial.printf("Connection status: %d\n", WiFi.status());
  }

}

void handleRoot() {
  String url = "<td><a href=http://" + String(host) + ">"+host+"</a><td></b>";
  Serial.println( url );
  String response =  "<h2>You have reached the Anemometer</h2>";
         response += "<b>This device polls at roughly 1 minute intervals. It does not keep accurate time</b>";
         response += "<p></p><table style=\"\width:600\"\>";
         response += "<tr><td>Device started at </td><td><b>" + startTime + "</b></td></tr>";
         response += "<tr><td>Current time </td><td><b>" + getInternetTime() + "</b></td></tr>";
         response += "<tr><td>Last calculated RPM </td><td><b>" + String(revsPerMinute) + "</b></td></tr>";
         response += "<tr><td>Rolling 5 minute average RPM </td><td><b>" + String(fiveMinuteAverage) + "</b></td></tr>";
         response += "<tr><td>Last gust RPM was </td><td><b>" + String(gustSpeed) + "</b></td></tr>";
         response += "<tr><td>Largest gust RPM detected </td><td><b>" + String(largestGust) + "</b> at <b>" + fullDate( timeOfLargestGust ) + "</b></td></tr>";
         response += "<tr><td>Total Polls/minutes </td><td><b>" + String(numberOfPolls) + "</b></td></tr>";
         
         int runSecs = timeClient.getEpochTime() - startAbsoluteTime;
         int upDays = abs( runSecs / 86400 );
         int upHours = abs( runSecs - ( upDays * 86400 ) ) / 3600;
         int upMins = abs( ( runSecs - (upDays * 86400) - ( upHours*3600 ) ) / 60 ) ;
         int upSecs = abs( runSecs - (upDays * 86400) - ( upHours*3600 ) - ( upMins*60 ) );
         String upTime = String(upDays) + "d " + String( upHours ) + "h " + String(upMins) + "m " +String(upSecs) + "s";

         response += "<tr><td>Uptime  </td><td><b>" + upTime + "</b></td></tr>";
         response += "<tr><td>Total trigs since boot </td><td><b>" + String(totalTrigs) + "</b></td></tr>";
         response += "<tr><td>Node Name </td><td><b>" + String(nodeName) + "</b></td></tr>"; 
         response += "<tr><td>Currently logging to</td>" + url + "</td></tr>"; 
         response += "<tr><td>Local IP is: </td><td><b>" + WiFi.localIP().toString() + "</b></td></tr>";
         response += "<tr><td>Connected via AP:</td><td> <b>" + WiFi.SSID() + "</b></td></tr>";
         response += "<tr><td>Free Heap Space </td><td><b>" + String(ESP.getFreeHeap()) + " bytes</b></td></tr>";
         response += "<tr><td>Software Version</td><td> <b>" + String(VERSION) + "</b></td></tr></table>";
         
  server.send(200, "text/html", response );   // Send HTTP status 200 (Ok) and send some text to the browser/client
}

void handleNotFound(){
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

ICACHE_RAM_ATTR void hall_ISR() 
{
triggered += 1;
totalTrigs += 1;
ledState = !ledState;
}

String getInternetTime() {
  timeClient.update();
  return String( timeClient.getFormattedTime() );
  
}


String fullDate ( unsigned long epoch ){

static unsigned char month_days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
static unsigned char week_days[7] = {4,5,6,0,1,2,3}; //Thu=4, Fri=5, Sat=6, Sun=0, Mon=1, Tue=2, Wed=3

unsigned char ntp_hour, ntp_minute, ntp_second, ntp_week_day, ntp_date, ntp_month, leap_days, leap_year_ind ;
String dow, sMonth;
unsigned short temp_days;

unsigned int ntp_year, days_since_epoch, day_of_year; 

    leap_days=0; 
    leap_year_ind=0;
    
    ntp_second = epoch%60;
    epoch /= 60;
    ntp_minute = epoch%60;
    epoch /= 60;
    ntp_hour  = epoch%24;
    epoch /= 24;
        
    days_since_epoch = epoch;      //number of days since epoch
    ntp_week_day = week_days[days_since_epoch%7];  //Calculating WeekDay
     
    ntp_year = 1970+(days_since_epoch/365); // ball parking year, may not be accurate!
 
    int i;
    for (i=1972; i<ntp_year; i+=4)      // Calculating number of leap days since epoch/1970
       if(((i%4==0) && (i%100!=0)) || (i%400==0)) leap_days++;
            
    ntp_year = 1970+((days_since_epoch - leap_days)/365); // Calculating accurate current year by (days_since_epoch - extra leap days)
    day_of_year = ((days_since_epoch - leap_days)%365)+1;
  
   
    if(((ntp_year%4==0) && (ntp_year%100!=0)) || (ntp_year%400==0))  
     {
        month_days[1]=29;     //February = 29 days for leap years
        leap_year_ind = 1;    //if current year is leap, set indicator to 1 
       }
        else month_days[1]=28; //February = 28 days for non-leap years 

         temp_days=0;
   
    for (ntp_month=0 ; ntp_month <= 11 ; ntp_month++) //calculating current Month
       {
           if (day_of_year <= temp_days) break; 
           temp_days = temp_days + month_days[ntp_month];
        }
    
    temp_days = temp_days - month_days[ntp_month-1]; //calculating current Date
    ntp_date = day_of_year - temp_days;
    
   
    switch(ntp_week_day) {
                         
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
  
  switch(ntp_month) {
                         
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
/*  
  printf(" %2d",ntp_date);
  printf(", %d\n",ntp_year);
  printf("TIME = %2d : %2d : %2d\n\n", ntp_hour,ntp_minute,ntp_second)  ;
  printf("Days since Epoch: %d\n",days_since_epoch);
  printf("Number of Leap days since EPOCH: %d\n",leap_days);
  printf("Day of year = %d\n", day_of_year);
  printf("Is Year Leap? %d\n",leap_year_ind);
*/
  return String( dow + " " + ntp_date + " " + sMonth + " " + ntp_hour + ":" + ntp_minute + ":" + ntp_second );
}
