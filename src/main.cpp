
#define OTA_ENABLED 1
#define DEBUG_ENABLED 0

#include "esp_camera.h"
#include <WiFi.h>

#if OTA_ENABLED
#include <ArduinoOTA.h>
#endif

#include "config.h"

#if DEBUG_ENABLED
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINTF(...)
#endif

#define LED_BUILTIN_PIN 4


//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

char currentWifiName[32];
char currentWifiPassword[64];
char currentServerIpAddress[32];

#ifndef WIFI_DEFAULT_STA_NAME
#define WIFI_DEFAULT_STA_NAME "LocalNetwork"
#endif

#ifndef WIFI_DEFAULT_STA_PASSWORD
#define WIFI_DEFAULT_STA_PASSWORD "LocalNetworkPassword"
#endif

uint32_t backupStaConnectionTime = 0;

#define MASSAGE_ASCII 1
#if MASSAGE_ASCII
    #include <AsciiMassagePacker.h>
    #include <AsciiMassageParser.h>
    AsciiMassagePacker massageOutbound;
    AsciiMassageParser massageInbound;
#else
    #include <SlipMassagePacker.h>
    #include <SlipMassageParser.h>
    SlipMassagePacker massageOutbound;
    SlipMassageParser massageInbound;
#endif




void startCameraServer();

#if OTA_ENABLED
void otaInit(){
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        {
            type = "sketch";
        }
        else
        { // U_FS
            type = "filesystem";
        }

        // NOTE: if updating FS this would be the place to unmount FS using FS.end()
        DEBUG_PRINTLN("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        DEBUG_PRINTLN("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        DEBUG_PRINTF("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        DEBUG_PRINTF("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
        {
            DEBUG_PRINTLN("Auth Failed");
        }
        else if (error == OTA_BEGIN_ERROR)
        {
            DEBUG_PRINTLN("Begin Failed");
        }
        else if (error == OTA_CONNECT_ERROR)
        {
            DEBUG_PRINTLN("Connect Failed");
        }
        else if (error == OTA_RECEIVE_ERROR)
        {
            DEBUG_PRINTLN("Receive Failed");
        }
        else if (error == OTA_END_ERROR)
        {
            DEBUG_PRINTLN("End Failed");
        }
    });
    ArduinoOTA.begin();
}
#endif

void setup() {

  WiFi.hostname("Rover-Turret");

  memset(currentWifiName,0,sizeof(currentWifiName));
  memset(currentWifiPassword,0,sizeof(currentWifiPassword));
  memset(currentServerIpAddress,0,sizeof(currentServerIpAddress));
  
  Serial.begin(9600);
  Serial.setDebugOutput(DEBUG_ENABLED?true:false);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DEBUG_PRINTF("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  if(psramFound()){
    DEBUG_PRINTLN("psramFound");
  } else {
    DEBUG_PRINTLN("NO psramFound");
  }

  DEBUG_PRINTLN("Setup complete");
  
}

bool connectToWifi(){

  #define CONNECT_TIMEOUT 30000

  if (WiFi.status()==WL_CONNECTED){
    return true;
    //already connected
  }

  if (strlen(currentWifiName)==0 || strlen(currentWifiPassword)==0){
    //invalid credentials
    return false;
  }

  WiFi.begin(currentWifiName, currentWifiPassword);

  uint32_t timeout=millis() + CONNECT_TIMEOUT;

  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
    //Connecting
    delay(500);
  }

  bool connected = WiFi.status() == WL_CONNECTED;

  if (connected){
    DEBUG_PRINTLN("Starting camera");
    startCameraServer();

#if OTA_ENABLED
    DEBUG_PRINTLN("OTA init");
    otaInit();
#endif

    //report the new local ip
    massageOutbound.beginPacket("gIP");
    massageOutbound.addString(WiFi.localIP().toString().c_str());
    massageOutbound.streamPacket(&Serial);

  }
  else{
    DEBUG_PRINTLN("Connection failed");
  }

  return connected;

}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(10000);
#if OTA_ENABLED
  ArduinoOTA.handle();
#endif
  if ( massageInbound.parseStream( &Serial ) ) {

      if ( massageInbound.fullMatch ("WiFi") ) {

        DEBUG_PRINTLN("Wifi message received");

        int mode = massageInbound.nextInt();
        char wifiName[32];
        char wifiPassword[64];

        massageInbound.nextString(wifiName,32);
        massageInbound.nextString(wifiPassword,64);
        massageInbound.nextString(currentServerIpAddress,32);

        if (strlen(wifiName)>0 && strlen(wifiPassword)>0 && strcmp(wifiName,currentWifiName)!=0 && strcmp(wifiPassword,currentWifiPassword)!=0 && WiFi.status()!=WL_CONNECTED){
          strcpy(currentWifiName,wifiName);
          strcpy(currentWifiPassword,wifiPassword);
          connectToWifi();
        }
        else{
          //report the new local ip
          massageOutbound.beginPacket("gIP");
          massageOutbound.addString(WiFi.localIP().toString().c_str());
          massageOutbound.streamPacket(&Serial);
        }



      }

      if ( massageInbound.fullMatch ("sLP") ) {

          int bottomLed = massageInbound.nextInt(); //used by ECU
          int turretLed = massageInbound.nextInt(); //used by ESP32CAM, this board!
          int turretLaser = massageInbound.nextInt(); //used by UNO
          
          digitalWrite(LED_BUILTIN_PIN,turretLed?HIGH:LOW);

      }

  }

  if (millis() > 60000 && backupStaConnectionTime == 0 && WiFi.status()!=WL_CONNECTED){
    DEBUG_PRINTLN("Connecting to backup network");
    strcpy(currentWifiName,WIFI_DEFAULT_STA_NAME);
    strcpy(currentWifiPassword,WIFI_DEFAULT_STA_PASSWORD);
    backupStaConnectionTime = millis();
    connectToWifi();
  }

  

}
