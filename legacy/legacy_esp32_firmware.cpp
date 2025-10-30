#include <Arduino.h>
uint32_t timer_led=0;



#include <queue> 


const char * networkName = "TP-LINK_KROSS";
const char * networkPswd = "GracefullY";


const char * udpAddress = "10.10.0.134";
const int udpPort = 7777;

#include <GyverNTP.h>
GyverNTP ntp(3);


void WiFiEvent(WiFiEvent_t event);
void connectToWiFi(const char * ssid, const char * pwd);

boolean connected = false;

WiFiUDP udp;

uint32_t timer_ping=0;

uint32_t timer_debug=0;



struct s_m_event {
  uint8_t type;
  uint8_t sensor_id;
  uint32_t timestamp;
  
}  __attribute__ ((packed));

struct s_m_sensor{
  uint8_t id=0;
  uint8_t type=0;
  uint8_t pin=0;
  uint8_t last_state=0;
  uint8_t datas[6];
  uint32_t last_update=0;
  uint32_t last_update_status=0;
};
s_m_event m_event;
s_m_event m_event_g;
std::queue<s_m_event> v_events; 
#define MAX_SENSORS_COUNT 5

s_m_sensor m_sensors[MAX_SENSORS_COUNT];


uint8_t light=false;

uint32_t time_timer=0;




void setup(){
  Serial.begin(115200);
  
  analogReadResolution(8);
  light=true;
  timer_led=millis();
  m_sensors[0].id=0;
  m_sensors[0].type=1;
  m_sensors[0].pin=10;
  m_sensors[1].id=1;
  m_sensors[1].type=2;
  m_sensors[1].pin=12;
  m_sensors[1].datas[0]=11;
  m_sensors[1].datas[1]=0;
  m_sensors[1].datas[2]=25;
  m_sensors[1].datas[3]=10;
  m_sensors[1].datas[4]=9*2;
  m_sensors[1].datas[5]=25; //25.5 скунд макс 

  for (uint8_t s_id=0;s_id<MAX_SENSORS_COUNT;s_id++) {
    if (m_sensors[s_id].type==1) {
      pinMode(m_sensors[s_id].pin, INPUT_PULLUP);
    }
    if (m_sensors[s_id].type==2 && m_sensors[s_id].datas[0]!=0) {
      pinMode(m_sensors[s_id].datas[0], OUTPUT);
      digitalWrite(m_sensors[s_id].datas[0], m_sensors[s_id].datas[1]);
    }

  }
  

  connectToWiFi(networkName, networkPswd);
  ntp.begin();
}

//state btn  0,1
//state line 2,3,4,5,6
//state 
//state 
//state 


void loop(){
    for (uint8_t s_id=0;s_id<MAX_SENSORS_COUNT;s_id++) {
      if (m_sensors[s_id].type==1) {
        uint8_t state=digitalRead(m_sensors[s_id].pin);
        if (state!=m_sensors[s_id].last_state) {
          m_sensors[s_id].last_state=state;
          m_event.sensor_id=s_id;
          m_event.type=state;
          m_event.timestamp=ntp.unix()+3*3600;
          v_events.push(m_event);
          neopixelWrite(48,10,0,0);
          light=true;
          timer_led=millis();
          Serial.println("new btn");
          
        }
      }else if (m_sensors[s_id].type==2) {
        if (millis()-m_sensors[s_id].last_update>200) {
          int analogValue = analogRead(m_sensors[s_id].pin);
          if (millis()-timer_debug>2000) {
            Serial.println("ANALOG "+String(analogValue));
            timer_debug=millis();
          }
          uint8_t state=2;
          //prepare state
          if (m_sensors[s_id].last_state==4) {
            state=m_sensors[s_id].last_state;
            if (millis()-m_sensors[s_id].last_update_status>m_sensors[s_id].datas[4]*500){
              digitalWrite(m_sensors[s_id].datas[0], !m_sensors[s_id].datas[1]);
              state=5;
              m_sensors[s_id].last_update_status=millis();
            }

          }else if (m_sensors[s_id].last_state==5) {
            state=m_sensors[s_id].last_state;
            if (millis()-m_sensors[s_id].last_update_status>m_sensors[s_id].datas[4]*100){
              digitalWrite(m_sensors[s_id].datas[0], m_sensors[s_id].datas[1]);
              state=6;
              m_sensors[s_id].last_update_status=millis();
            }
            
          }else{

          
            if (analogValue<5){
              state=2+1;
            } else if (analogValue<m_sensors[s_id].datas[2]+m_sensors[s_id].datas[3] && analogValue>m_sensors[s_id].datas[2]-m_sensors[s_id].datas[3]) {
              state=2+0;
            } else {
              state=2+2;
              m_sensors[s_id].last_update_status=millis();
              
              //Добавить еще можно кз state=2+3
            }
          }


          if (state!=m_sensors[s_id].last_state) {
            m_sensors[s_id].last_state=state;
            m_event.sensor_id=s_id;
            m_event.type=state;
            m_event.timestamp=ntp.unix()+3*3600;
            v_events.push(m_event);

            neopixelWrite(48,0,0,10);
            light=true;
            timer_led=millis();
            Serial.println("new smoke "+String(m_event.type));
          
          }


          m_sensors[s_id].last_update=millis();
        }
      }
    
    } 
    
    if (light && millis()-timer_led>100){
      neopixelWrite(48,0,0,0);
      light=false;
    }


  // Serial.println( );

  // only send data when connected
  if(connected){
    ntp.tick();
    //Send a packet
    if (millis()-timer_ping>5000) {
      udp.beginPacket(udpAddress,udpPort);
      udp.write(1);
      udp.write(255);
      udp.endPacket();
      timer_ping=millis();
    }else if(v_events.size()!=0) {
      Serial.println("try send");
      
      
      if (udp.beginPacket(udpAddress,udpPort)==1) {
        m_event_g=v_events.front();
        
        uint8_t tmp=udp.write(1);
        if (tmp!=1) Serial.println("error1 "+String(tmp));
        tmp=udp.write((uint8_t*)&m_event_g,sizeof(m_event_g));
        if (tmp!=6) Serial.println("error2 "+String(tmp));
        tmp=udp.endPacket();
        if (tmp!=1) Serial.println("error3 "+String(tmp));
        v_events.pop();
      }else{
        Serial.println("error send");
      }
      
      
    }
    
  }
}



void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));


  WiFi.disconnect(true);

  WiFi.onEvent(WiFiEvent);
  

  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}



