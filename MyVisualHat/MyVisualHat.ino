#include "esp_camera.h"
#include <WiFi.h>
#include "SPIFFS.h"
#include "ESPmDNS.h"
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"

//const char* ssid = "Ming-Ming";
//const char* password = "mingming";

// Variable to store the HTTP request
String header;

String outpin15State = "off";  //設定字串變數 outpin15State，顯示 GPIO15 狀態。如果您要增加可控制的 GPIO 數目，請在這裡增加
String outpin4State = "off";

// Assign output variables to GPIO pins
//const int output26 = 26;
//const int output27 = 27;
#define OUTPIN15 15
#define OUTPIN4 4

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void startCameraServer();

WiFiServer serverCtrl(82);

void setup() {
  // my code for setting ssid and pw
  char ssid[33] = { '\0' };
  char password[64] = { '\0' };
  Serial.begin(115200);
  pinMode(OUTPIN15, OUTPUT);
  digitalWrite(OUTPIN15, LOW);
  pinMode(OUTPIN4, OUTPUT);
  digitalWrite(OUTPIN4, LOW);
  //delay(1);

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  Serial.println("openning WiFi setting...");
  File file_ssid = SPIFFS.open("/ssid.txt", "r");
  File file_pw = SPIFFS.open("/password.txt", "r");
  if (!file_ssid || !file_pw) {
    if (!file_ssid) {
      Serial.println("Failed to open ssid file for reading");
    }
    if (!file_pw) {
      Serial.println("Failed to open password file for reading");
    }
    return;
  } else {
    //const char* ssid = file_ssid.read();
    //const char* password = file_pw.read();

    //寫入ssid
    uint16_t i = 0;
    while (file_ssid.available()) {
      ssid[i] = file_ssid.read();
      //Serial.print (ssid [i]); //use for debug
      i++;
    }
    ssid[i] = '\0';
    Serial.println(ssid);  //use for debug

    //寫入password
    i = 0;
    while (file_pw.available()) {
      password[i] = file_pw.read();
      //Serial.print (password [i]); //use for debug
      i++;
    }
    password[i] = '\0';
    Serial.println(password);  //use for debug
  }
  file_ssid.close();
  file_pw.close();

  // default code-----
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif
  if (password[0] == '\0'){
    Serial.println("Connect without password.");
    WiFi.begin(ssid);
    }
  else
    WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.println("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    for(int i=0; i<11; i++){
      delay(500);
      Serial.print(".");
      if(WL_CONNECTED) break;
    }
    Serial.println("");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // for my own remoter
  serverCtrl.begin();
  // set a name for finding on network
  AdvertiseServices("MyVisualHat");
}
//sensor_t * s = esp_camera_sensor_get();
void loop() {
  // my code for making a remoter, not using. commended out.
  WiFiClient client = serverCtrl.available();  // 看有沒有外部裝置 client (如手機)，藉著瀏覽器 browser 連上 ESP32 web server

  if (client) {  // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    //Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                                                   // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {  // if there's bytes to read from the client,
        char c = client.read();  // read a byte, then
        Serial.write(c);         // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // 接收 client  的指令，點亮或關閉 連上 GPIO 的 LED。瀏覽器上，不同的按鈕，會發出不同的指令 (URLs requests)
            if (header.indexOf("GET /15/on") >= 0) {
              //Serial.println("GPIO 15 on");
              outpin15State = "on";
              digitalWrite(OUTPIN15, HIGH);
            } else if (header.indexOf("GET /15/off") >= 0) {
              //Serial.println("GPIO 15 off");
              outpin15State = "off";
              digitalWrite(OUTPIN15, LOW);
            } else if (header.indexOf("GET /4/on") >= 0) {
              //Serial.println("GPIO 4 on");
              outpin4State = "on";
              digitalWrite(OUTPIN4, HIGH);
            } else if (header.indexOf("GET /4/off") >= 0) {
              //Serial.println("GPIO 4 off");
              outpin4State = "off";
              digitalWrite(OUTPIN4, LOW);
            }

            // 設計 client 上的瀏覽器網頁格式，包括顏色、字型大小、有無邊框、字元等，這一部分是用 HTML 的語言寫成的
            /*
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\"></head>");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button {background-color: #555555; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 { background-color: #4CAF50;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 15  
            client.println("<p>GPIO 15 - State " + outpin15State + "</p>");
            // If the outpin15State is off, it displays the ON button       
            if (outpin15State=="off") {
              client.println("<p><a href=\"/15/on\"><button class=\"button\" name=\"btn1\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/15/off\"><button class=\"button button2\" name=\"btn2\">ON</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 4  
            client.println("<p>GPIO 4 - State " + outpin4State + "</p>");
            // If the outpin4State is off, it displays the ON button       
            if (outpin4State=="off") {
              client.println("<p><a href=\"/4/on\"><button class=\"button\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/4/off\"><button class=\"button button2\">ON</button></a></p>");
            }
            client.println("</body></html>");
            */
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    //Serial.println("Client disconnected.");
    //Serial.println("");
  }
}
 
void AdvertiseServices(const char *MyName)
{
  if (MDNS.begin(MyName))
  {
    Serial.println(F("mDNS responder started"));
    Serial.print(F("mDNS: "));
    Serial.println(MyName);
 
    // Add service to MDNS-SD
    MDNS.addService("n8i-mlp", "tcp", 23);
  }
  else
  {
    Serial.println(F("Error setting up MDNS responder"));
    delay(1000);
  }
}
