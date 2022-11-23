// Implemented on Arduino UNO Metro with Ethernet shield W5500
// PID not tuned yet
#include <Adafruit_MAX31865.h>
#include "Ethernet5500.h"
#include <SPI.h>
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(1, 2, 3, 4);
//Ethernet Shield use pin 9,10,11,12 for standard SPI

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

#define SSR_TEMP    8
#define SSR_MAGNETS 7

#define VERBOSE   2 //0 = No Serial Output  1 = Standard output  2 = Full output
//Variables used for PID tunning
float aerror[20]; // 2 point should be enough
float ainterval[20];
float temp = 0;
float ratio = 0;
int k = 0;
float error = 0;
float derror = 0;
float ierror = 0;
float time_tot = 1;
float time_0[20];
float interval = millis();
bool heatingEnable = true;
bool magnetEnable = true;
float Kp = 1.0;
float Kd = 150.0; //PID parameters to be tuned
float Ki = 0.01;
float setpoint = 37;

bool stringComplete = false;
String dataString = "";
String inputString = "";

//Ethernet related settings
byte mac[] = {0x2C, 0xF7, 0xF1, 0x08, 0x3F, 0xAE};  //W5500 Mac address

int8_t ip_addr[4] = {192,168,1,91};
String strIP = "192.168.1.91";
IPAddress ip(192,168,1,91);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255, 255, 255, 0);
IPAddress myDns(8, 8, 8, 8); // google puble dns
EthernetServer server = EthernetServer(80);
#define TIMEOUT_ETH 1000

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);

  if(!initEthernet()) Serial.println("Error on Ethernet");
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  
  pinMode(SSR_TEMP,OUTPUT);
  digitalWrite(SSR_TEMP,LOW);
  pinMode(SSR_MAGNETS,OUTPUT);
  digitalWrite(SSR_MAGNETS,LOW);
  delay(500);
}


void loop() {

  uint16_t rtd = thermo.readRTD();
  ratio = rtd;
  temp = thermo.temperature(RNOMINAL,RREF);
  error = setpoint-temp;
  interval = millis()-interval;
  ratio /= 32768;

  //Temperature Control PID ----------------------------------------------------------------
  if(k<20){
    aerror[k] = error; //array_error °C
    ainterval[k] = interval; //array_interval ms
    //time_0[k] = time_0[k-1] + interval;
    k++;
  }
  else{
    for(int i = 0; i<19;i++){
      aerror[i] = aerror[i+1];
      ainterval[i] = ainterval[i+1];
      time_tot += ainterval[i];
      time_0[i] = time_tot;
    }
    aerror[19] = error;
    ainterval[19] = interval;
    time_tot += interval;
    time_0[19] = time_tot;
    derror = (aerror[19]-aerror[10])/(time_0[19]-time_0[10])*1000;
    ierror = (aerror[19]*ainterval[19]+aerror[18]*ainterval[18])/(ainterval[19]+ainterval[18]);
  }
  // PID controller = Kp*e + Ki*integral(e) + Kd*de/dt
  // error > 0 when temp < setpoint
  //&& !digitalRead(PIN_ENABLE)
  #ifdef VERBOSE
    Serial.print("Setpoint : ");Serial.print(Kp*error + Kd*derror + Ki*ierror + temp);
    Serial.print(" / Temperature : ");Serial.print(temp);
    Serial.print(" / derror : ");Serial.print(Kd*derror);
    Serial.print(" / error : ");Serial.print(error);
    Serial.println();
  #endif
  //Temperature control ---------------------------------------------------------------------------
  if(Kp*error + Kd*derror + Ki*ierror > 0 && heatingEnable){
    digitalWrite(SSR_TEMP,HIGH);
    Serial.print("Heating On / ");
  }
  else{digitalWrite(SSR_TEMP,LOW);
  Serial.print("Heating Off / ");
  }
  delay(250); //Wait 250 ms to be sure that the SSR was activated or deactivated
  //Magnet control ----------------------------------------------------------------------------
  if(magnetEnable){
    digitalWrite(SSR_MAGNETS,HIGH);
    Serial.println("Magnets ON");
  }
  else {
    digitalWrite(SSR_MAGNETS,LOW);
    Serial.println("Magnets OFF");
  }
  //Ethernet Client check
  EthernetClient client = server.available();
  EthernetClient *client_pntr = &client;
  if(client){
    Serial.println("New client connected");
    String currentLine = "";
    long time_connection = millis();
    while(client.connected()){
      if(client.available()) {
        currentLine = ""; //reset currentLine
        //Read the first Line
        char c = client.read();
        while(!(c== '\n' || c == ' ' || c == '/' )){
          currentLine += c;
          c = client.read();
        }
        if(currentLine == "home"){ homePage(client_pntr);}
        if(currentLine == "MagnetsON"){ magnetEnable = true;}
        if(currentLine == "MagnetsOFF"){ magnetEnable  = false;}
        if(currentLine == "HeatingON"){ heatingEnable = true;}
        if(currentLine == "HeatingOFF"){ heatingEnable = false;}
        if(currentLine == "ResetController"){ software_reset();}
      }
      //Serial.println("client connected");
      if(millis()-time_connection> TIMEOUT_ETH)
        client.stop();
      delay(20);
    }
  }
}

//Reset Arduino
void software_reset() {
  asm volatile (" jmp 0");  
}

bool checkFaultRTD(){
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
    return true;
  }
  return false;
}

bool initEthernet(){
  Serial.println(F("Ethernet Coms starting..."));
  Ethernet.begin(mac, ip, myDns, gateway, subnet);  //Start the Ethernet coms
  // Check for Ethernet hardware present
  // Start the server
  server.begin();           //"server" is the name of the object for comunication through ethernet
  Serial.print(F("Ethernet server connected. Server is at "));
  //Serial.println(Ethernet.localIP());         //Gives the local IP through serial com
  Serial.println(strIP);
  return true;
}

void homePage(EthernetClient *client_pntr){
  Serial.println("home");
  int state = getStatus();
  long current_time = millis();
  int seconds = (int) (current_time / 1000) % 60 ;
  int minutes = (int) ((current_time / (1000*60)) % 60);
  int hours   = (int) ((current_time / (1000*60*60)) % 24);
  char c[30];
  int l = sprintf(c, "%02d:%02d:%02d",hours,minutes,seconds);
  client_pntr->println(F("HTTP/1.1 200 OK"));
  client_pntr->println(F("Content-Type: text/html"));
  client_pntr->println(F("Connection: close"));  // the connection will be closed after completion of the response
  client_pntr->println(F("Refresh: 5"));  // refresh the page automatically every 5 sec
  client_pntr->println();
  client_pntr->println(F("<!DOCTYPE HTML>"));
  client_pntr->println(F("<html>"));
  client_pntr->println(F("<body>"));
  client_pntr->print("<h1 style=\"text-align:center\">IKA Controller  </h1>");
  client_pntr->print("<p> Heating : ");
  if(heatingEnable) client_pntr->print("<strong style= \"background-color:#00ff00\"> ON </strong>");
  else client_pntr->print("<strong style= \"background-color:#ff0000\"> OFF </strong>");
  client_pntr->print("</p> <p> Magnets : ");
  if(magnetEnable) client_pntr->print("<strong style= \"background-color:#00ff00\"> ON </strong>");
  else client_pntr->print("<strong style= \"background-color:#ff0000\"> OFF </strong>");
  String temp_below_color = "<strong style= \"background-color:#0000ff\">";
  String temp_above_color = "<strong style= \"background-color:#ff0000\">";
  client_pntr->print("</p> <p> Current Temperature : " + (temp<setpoint ? temp_below_color:temp_above_color));client_pntr->print(temp);client_pntr->print("</strong> Setpoint : " + String(setpoint)+ "</p>");
  client_pntr->print(F("<p><a href=\"http://192.168.1.91/HeatingON\">Enable Heating</a></p>"));
  client_pntr->print(F("<p><a href=\"http://192.168.1.91/HeatingOFF\">Disable Heating</a></p>"));
  client_pntr->print(F("<p><a href=\"http://192.168.1.91/MagnetsON\">Enable Magnets</a></p>"));
  client_pntr->print(F("<p><a href=\"http://192.168.1.91/MagnetsOFF\">Disable Magnets</a></p>"));
  
  client_pntr->print(F("Connection closed by the server at internal time : "));client_pntr->print(millis());
  //Close the connection
  client_pntr->print("</body>");
  client_pntr->print("</html>");
  delay(10);
  //client_pntr->flush();
  while (client_pntr->read() != -1);
  ////Serial.println("Client stop called");
  client_pntr->stop();
}

int getStatus() {

  if(magnetEnable && heatingEnable) return 3;
  if(magnetEnable && !heatingEnable) return 2;
  if(!magnetEnable && heatingEnable) return 1;
  return 0;
}
