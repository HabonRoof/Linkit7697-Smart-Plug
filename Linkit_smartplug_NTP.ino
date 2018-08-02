/*  Linkit7697 V1.0
 *  Smart Plug use MCS with Wifi Clock Synchronization
 *  Author : JohnsonChen ,aka, HabonRoof
 *  https://www.instagram.com/habonroof/
 *  2018-07-26 17:05
 */


#include <LWiFi.h>
#include <WiFiClient.h>
#include "MCS.h"
#include<LRTC.h>
#include<WiFiUdp.h>

#define _SSID "Your WiFi SSID"                                                            // Assign AP ssid / password here
#define _KEY  "Your WiFi password"                                                            

MCSDevice mcs("DD2AJMsb", "D4HNvUCYUu7VzJan");           // Assign device id / key of your test device
MCSControllerOnOff Relay("Relay");
MCSDisplayOnOff Relay_State("Relay_State");
MCSDisplayFloat Voltage("Volt");
MCSDisplayFloat Current("Curr");
MCSDisplayFloat Energy("Eng");
MCSControllerInteger Hour("SetHour");
MCSControllerInteger Minute("SetMinute");
MCSControllerInteger OpeningTime("OpeningTime");
MCSControllerString Ctrl_Mode("Ctrl_Mode");
WiFiUDP Udp;

int status = WL_IDLE_STATUS;
int keyIndex = 0;
const int Volt = 110;
const int currentSensorPin =14;         //define current sensor pin
const int mVperAmp = 66;                // use 185 for 5A Module, and 66 for 30A Module
byte RelayPin = 3;
float calibration_current = 0;
float CurrentValue = 0;
float Eng = 0;

unsigned long LastTime = 0;
unsigned int localPort = 2390;         //local port to listen for UDP packets
IPAddress timeServer(129,6,15,28);
const char *NTP_server = "time-a.nist.gov";
const int NTP_PACKET_SIZE = 48;       // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE];  //buffer to hold incoming and outgoing packets
int TimeZone = 8;

int SetHour = 0;    int SetMinute = 0;
int DesHour = 0;    int DesMinute = 0;
int NowHour = 0;    int NowMinute = 0;
int NowSecond = 0;
bool Udpconnected = false;    //Insure the package was send from with NTP server

void setup() {
   Serial.begin(9600);
   pinMode(RelayPin,OUTPUT);
   LRTC.begin();              //Start LRTC Module
   // Calibration process，simple 10 time the initial current error value，and averge the real error current
   for(int i =0;i<11;i++)
    calibration_current = calibration_current + readACCurrent(currentSensorPin);
    calibration_current = calibration_current / 10 ;
   delay(1);
   
   //Turn on the socket for test
   digitalWrite(RelayPin,HIGH);
   delay(500);
   digitalWrite(RelayPin,LOW);  

    //Linkit 7697 wifi Setup
    while(WL_CONNECTED != WiFi.status())
      WiFi.begin(_SSID, _KEY);    //Connect to WiFi   
    
    // setup MCS connection
    mcs.addChannel(Relay);
    mcs.addChannel(Relay_State);
    mcs.addChannel(Voltage);
    mcs.addChannel(Current);
    mcs.addChannel(Energy);
    mcs.addChannel(Hour);
    mcs.addChannel(Minute);
    mcs.addChannel(OpeningTime);
    mcs.addChannel(Ctrl_Mode);
    while(!mcs.connected())
      mcs.connect();
    digitalWrite(RelayPin,Relay.value() ? HIGH : LOW);
    Serial.println("\nStarting connection to server...");
    //Get NTP time on webserver
    while(!Udpconnected){
      Udp.begin(localPort);
      sendNTPpacket(NTP_server);
      delay(5000);
      Serial.println("1");
      if (Udp.parsePacket()) {
        Serial.println("packet received");
        // We've received a packet, read the data from it
        Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer  
             
        //the timestamp starts at byte 40 of the received packet and is four bytes,
        // or two words, long. First, esxtract the two words:         
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        // combine the four bytes (two words) into a long integer
        // this is NTP time (seconds since Jan 1 1900):
        unsigned long secsSince1900 = highWord << 16 | lowWord;
        Serial.println(secsSince1900);
        // now convert NTP time into everyday time:
        // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
        const unsigned long seventyYears = 2208988800UL;
        // subtract seventy years:
        unsigned long epoch = secsSince1900 - seventyYears;  //epoch:Seconds from 1970/1/1
        // print the hour, minute and second:
        Serial.print("The UTC time is ");         // UTC is the time at Greenwich Meridian (GMT)
        Serial.print((epoch  % 86400L) / 3600);   // print the hour (86400 equals secs per day)
        Serial.print(':');
        if (((epoch % 3600) / 60) < 10)
          Serial.print('0');
        Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
        Serial.print(':');
        if ((epoch % 60) < 10)
          Serial.print('0');
        Serial.println(epoch % 60); // print the second
        Serial.print("The local time is ");
        Serial.print(NowHour=(epoch % 86400L)/3600+TimeZone);
        Serial.print(":");
        if (((epoch % 3600+TimeZone) / 60) < 10)
          Serial.print('0');
        Serial.println(NowMinute=(epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
        Serial.print(':');
        if (NowSecond = (epoch % 60) < 10)
          Serial.print('0');
        Udpconnected = true;        
      } 
  }
    //RTC correction process
    LRTC.set(2018,7,26,NowHour,NowMinute,NowSecond);    //Set RTC year-month-day-hour-minute-second,
                                                        //I ignore the year,month and date because we dont use 
                                                        //that information to control our relay for now
    for(int i ;i++;i<4){
    digitalWrite(1,HIGH);
    delay(200);
    digitalWrite(1,LOW);
    delay(200);
    }
  digitalWrite(7,HIGH);
}



void loop() {
    mcs.process(30);     //need this command to process the data from MCS server
    if(millis() - LastTime >= 1000){
      LastTime = millis();
      Voltage.set(Volt);
      CurrentValue = readACCurrent(currentSensorPin) - calibration_current;
      if(CurrentValue < 0.18)                    //Neglet current smaller than 0.18A
       CurrentValue = 0;
       Current.set(CurrentValue);    
       Eng += CurrentValue*Volt/3600;
       Energy.set(Eng);
     }    
     switch(Ctrl_Mode.value()[0]){
      case '1':
        Manual_Relay();
        Serial.println("Manual mode");
        break;
      case '2':
        RTC_Relay();
        Serial.println("Auto mode");
        break;
      default:
        Manual_Relay();
        Serial.println("Default mode");
     }
     connectcheck();
     delay(500);
}


/*read AC Current Value and ruturn the RMS*/
float readACCurrent(int Pin)
{ 
    int analogValue;             //analog value read from the sensor output pin
    int maxValue = 0;            // store max value
    int minValue = 4096;         // store min value 
    unsigned long start_time = millis();
    while((millis()-start_time) < 200) //sample for 0.2s
    {
        analogValue = analogRead(Pin);
        if (analogValue > maxValue)
            maxValue = analogValue;
        if (analogValue < minValue)
            minValue = analogValue;
   }
   float Vpp = (maxValue - minValue) * 5000 /4096;    //    Vcc/4096
   float Vrms = Vpp / 2.0 * 0.707 / mVperAmp; //Vpp -> Vrms
   return Vrms;
}


void Manual_Relay(){
    if(Relay.updated()){
    Serial.println(Relay.value());
    digitalWrite(RelayPin,Relay.value() ? HIGH : LOW);
    Relay_State.set(Relay.value());
    }
}


void GetSetTime(){
  mcs.process(30);
  //Serial.println("Getting Set Time...");
  SetHour = Hour.value();
  SetMinute = Minute.value();
  int Hr = OpeningTime.value()/60;
  int Min = OpeningTime.value()%60;
  DesHour = SetHour + Hr;           //Set destination hour
  DesMinute = SetMinute + Min;      //Set destination minutes   
  DesHour = DesHour + DesMinute/60; //
  DesHour = DesHour%24;             //Day Cross check
  DesMinute = DesMinute%60;         //Hour Cross check
  //Serial.println("Getting Set Time Done!");        
  delay(200);
}


void RTC_Relay(){
      GetSetTime();
      //Serial.print("Set Time:");
      //Serial.print(SetHour);
      //Serial.print(":");
      //Serial.print(SetMinute);
      //Serial.print("Duration:");
      //Serial.println(OpeningTime.value());      
      LRTC.get();
      NowHour = LRTC.hour();
      NowMinute = LRTC.minute();
      //Serial.print("  Now:");
      //Serial.print(NowHour);
      //Serial.print(":");
      //Serial.println(NowMinute);
      //Serial.print("Destination Time ");
      //Serial.print(DesHour);
      //Serial.print(":");
      //Serial.println(DesMinute);
      if(NowHour == SetHour && NowMinute == SetMinute){
          digitalWrite(RelayPin,HIGH);
          Serial.println("Relay ON");
          Relay_State.set(HIGH);
      }
      if(NowHour == DesHour && NowMinute == DesMinute){
          digitalWrite(RelayPin,LOW);
          Serial.println("Relay Off");
          Relay_State.set(LOW);
      }
}

void connectcheck(){
   while(!mcs.connected())
        mcs.connect();        
}

unsigned long sendNTPpacket(const char* host) {
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(host, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
  return 0;
}
