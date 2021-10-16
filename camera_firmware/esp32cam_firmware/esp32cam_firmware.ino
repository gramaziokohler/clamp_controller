/****************************************************************************************************************************************************
    TITLE: HOW TO BUILD A $9 RSTP VIDEO STREAMER: Using The ESP-32 CAM Board || Arduino IDE - DIY #14
    DESCRIPTION: This sketch creates a video streamer than uses RTSP. You can configure it to either connect to an existing WiFi network or to create
    a new access point that you can connect to, in order to stream the video feed.

    By Frenoy Osburn
    YouTube Video: https://youtu.be/1xZ-0UGiUsY
    BnBe Post: https://www.bitsnblobs.com/rtsp-video-streamer---esp32
 ****************************************************************************************************************************************************/

/********************************************************************************************************************
   Board Settings:
   Board: "ESP32 Wrover Module"
   Upload Speed: "921600"
   Flash Frequency: "80MHz"
   Flash Mode: "QIO"
   Partition Scheme: "Hue APP (3MB No OTA/1MB SPIFFS)"
   Core Debug Level: "None"
   COM Port: Depends *On Your System
*********************************************************************************************************************/

#include "src/OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#include "src/SimStreamer.h"
#include "src/OV2640Streamer.h"
#include "src/CRtspSession.h"

#include "wifikeys.h"

//#define ENABLE_OLED //if want use oled ,turn on thi macro

WebServer server(80);

// Set Static IP address (WIFI connection to host)
IPAddress local_IP(192, 168, 0, 160);
// Set Gateway IP address (WIFI connection to host)
IPAddress gateway(192, 168, 1, 1);

#ifdef ENABLE_OLED
#include "SSD1306.h"
#define OLED_ADDRESS 0x3c
#define I2C_SDA 14
#define I2C_SCL 13
SSD1306Wire display(OLED_ADDRESS, I2C_SDA, I2C_SCL, GEOMETRY_128_32);
bool hasDisplay; // we probe for the device at runtime
#endif

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5CAM
#include "camera_pins.h"

// Camera Flash light and Status LED
#define FLASH_LED_PIN 4
#define FLASH_LED_CHANNEL 4
#define FLASH_LED_BRIGHTNESS 20
#define WIFI_LED_PIN 33
OV2640 cam;

void handle_jpg_stream(void)
{
  Serial.println("Entered handle_jpg_stream()");
  ledcWrite(FLASH_LED_CHANNEL, FLASH_LED_BRIGHTNESS); // FLASH LED On
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (1)
  {
    cam.run();
    if (!client.connected()) {
      ledcWrite(FLASH_LED_CHANNEL, 0); // FLASH LED Off
      break;
    }
    response = "--frame\r\n";
    response += "Content-Type: image/jpeg\r\n\r\n";
    server.sendContent(response);

    client.write((char *)cam.getfb(), cam.getSize());
    server.sendContent("\r\n");
    if (!client.connected()) {
      ledcWrite(FLASH_LED_CHANNEL, 0); // FLASH LED Off
      break;
    }
  }
}

void handle_jpg(void)
{
  Serial.println("Entered handle_jpg()");
  ledcWrite(FLASH_LED_CHANNEL, FLASH_LED_BRIGHTNESS); // FLASH LED On
  delay(100);
  WiFiClient client = server.client();

  cam.run();
  if (!client.connected())
  {
    return;
  }
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-disposition: inline; filename=capture.jpg\r\n";
  response += "Content-type: image/jpeg\r\n\r\n";
  server.sendContent(response);
  client.write((char *)cam.getfb(), cam.getSize());
  ledcWrite(FLASH_LED_CHANNEL, 0); // FLASH LED Off
}

void handleNotFound()
{
  Serial.println("Entered handleNotFound()");
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text/plain", message);
}


void lcdMessage(String msg)
{
#ifdef ENABLE_OLED
  if (hasDisplay) {
    display.clear();
    display.drawString(128 / 2, 32 / 2, msg);
    display.display();
  }
#endif
}

void setup()
{
#ifdef ENABLE_OLED
  hasDisplay = display.init();
  if (hasDisplay) {
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
  }
#endif
  lcdMessage("booting");

  Serial.begin(115200);
  //while (!Serial);            //wait for serial connection.

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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, HIGH); //LED Off

  cam.init(config);

  ledcSetup(FLASH_LED_CHANNEL, 5000, 8);
  ledcAttachPin(FLASH_LED_PIN, FLASH_LED_CHANNEL);
  ledcWrite(FLASH_LED_CHANNEL, 0); // FLASH LED Off

  lcdMessage(String("join ") + ssid);
  WiFi.mode(WIFI_STA);

  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);
  IPAddress secondaryDNS(8, 8, 4, 4);
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Wifi Static IP Failed to configure");
  }
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  IPAddress ip;
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.println(ip);
  Serial.print("Stream Link: rtsp://");
  Serial.print(ip);
  Serial.println(":8554/mjpeg/1");


  lcdMessage(ip.toString());

  server.on("/", HTTP_GET, handle_jpg_stream);
  server.on("/jpg", HTTP_GET, handle_jpg);
  server.onNotFound(handleNotFound);
  server.begin();

}

CStreamer *streamer;
CRtspSession *session;
WiFiClient client; // FIXME, support multiple clients

void loop()
{
  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(WIFI_LED_PIN, HIGH); //LED Off
  } else {
    digitalWrite(WIFI_LED_PIN, LOW); //LED On
  }

}
