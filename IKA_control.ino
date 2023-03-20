// Implemented on Arduino UNO Metro with Ethernet shield W5500
// Version live on the IKA shaker, upload 30.01.2023
// PID not tuned yet
#include <Adafruit_MAX31865.h>
#include "Ethernet5500.h"
#include <SPI.h>
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(4, 5, 6, 7);
//Ethernet Shield use pin 10,11,12,13 for standard SPI

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

#define SSR_TEMP    3 //12V relay
#define SSR_MAGNETS 2 //24V relay

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
bool heatingEnable = false;
bool magnetEnable = false;
float Kp = 1.0;
float Kd = 10.0; //PID parameters to be tuned
float Ki = 0.01;
float setpoint = 43;
float p_temp = 0;
#define PID_TIME 1000 //ms

bool stringComplete = false;
String dataString = "";
String inputString = "";

//Ethernet related settings
byte mac[] = {0x2C, 0xF7, 0xF1, 0x08, 0x3F, 0xAE};  //W5500 Mac address

int8_t ip_addr[4] = {192,168,1,50};
String strIP = "192.168.1.50";
IPAddress ip(192,168,1,50);
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
  delay(PID_TIME);
  if(millis()> 86400000) software_reset(); //Reset every day 86400000
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
    derror = (aerror[19]-aerror[15])/(time_0[19]-time_0[15])*1000;
    ierror = (aerror[19]*ainterval[19]+aerror[18]*ainterval[18])/(ainterval[19]+ainterval[18]);
  }
  derror = p_temp-temp;
  p_temp = temp;
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
  //delay(250); //Wait 250 ms to be sure that the SSR was activated or deactivated
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
    String currentLine = "";
    long time_connection = millis();
    while(client.connected()){
      if(client.available()) {
        currentLine = ""; //reset currentLine
        //Read the first Line
        char c = client.read();
        while(!(c== '\n' || c == ' ' || c == '/' || c == -1)){
          currentLine += c;
          c = client.read();
        }
        if(currentLine == "home"){ homePage(client_pntr);}
        if(currentLine == "magnetsON"){ magnetEnable = true; homePage(client_pntr);}
        if(currentLine == "magnetsOFF"){ magnetEnable  = false; homePage(client_pntr);}
        if(currentLine == "heatingON"){ heatingEnable = true; homePage(client_pntr);}
        if(currentLine == "heatingOFF"){ heatingEnable = false; homePage(client_pntr);}
        if(currentLine == "resetController"){homePage(client_pntr); software_reset();}
        if(currentLine == "getStatus") { AnswerHttp(client_pntr);}
        if(currentLine == "bothON"){magnetEnable = true; heatingEnable = true; homePage(client_pntr);}
        if(currentLine == "bothOFF"){magnetEnable = false; heatingEnable = false; homePage(client_pntr);}
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


int getStatus() {

  if(magnetEnable && heatingEnable) return 3; //Both ON : status = 3
  if(magnetEnable && !heatingEnable) return 2; //Magnet On but not heating : status = 2
  if(!magnetEnable && heatingEnable) return 1; // Heating ON but not magnets : status = 1
  return 0; //Both disabled : status = 0
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
