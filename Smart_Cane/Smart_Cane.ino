//Wifi library
// #include "WiFi.h" //SysFile Name changed from C:\Program Files (x86)\Arduino\libraries\WiFi\src : WiFi.h -> SysWiFi.h

//BLE(Bluetooth Low Energy) libray
// #include "BLEDevice.h"

//BlueTooth library
#include <BluetoothSerial.h>

//SPIFFS library
#include "SPIFFS.h"   //儲存小檔案用

//MPU6050 library
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
//定義腳位
#define Trig    12 //發出聲波
#define Echo    14 //接收聲波

#define Motor   27 //震動馬達
#define Buzzer  26 //蜂鳴器
//硬體接腳定義
//SDA 21
//SCL 22

bool finding = false;

//BlueTooth-----------------------------------------------------
BluetoothSerial SerialBT;
//--------------------------------------------------------------

//BLE-----------------------------------------------------------
// //我們希望連上的服務
// static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331915c");
// //我們希望連上的特徵
// static BLEUUID   charUUID1("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
// //判斷是否連線、完成掃描
// static boolean doConnect = false;
// static boolean connected = false;
// static boolean doScan = false;


// static BLERemoteCharacteristic* pRemoteCharacteristic;
// static BLEAdvertisedDevice* myDevice;

// //BLE 通知的回調函式，如果有資料則儲存並列印
// static void notifyCallback(
//   BLERemoteCharacteristic* pBLERemoteCharacteristic,
//   uint8_t* pData,
//   size_t length,
//   bool isNotify) {
    
//     Serial.print("Notify callback for characteristic ");
//     Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    
//     Serial.print("資料長度");
//     Serial.println(length);
//     String Collect_Date = (char*)pData;
//     Serial.print("收到資料: ");
//     Serial.println(Collect_Date);
//     if(Collect_Date=="finding"){
//       finding = true;
//     }
//     Collect_Date = "";
    
// }
// //連線之回調函式，會返回連線狀態
// class MyClientCallback : public BLEClientCallbacks {
//   void onConnect(BLEClient* pclient) {

//   }

//   void onDisconnect(BLEClient* pclient) {
//     connected = false;
//     Serial.println("onDisconnect");
//   }
// };
// //與Server連線，並判斷是否成功，回傳bool
// bool connectToServer() {
  
//     Serial.print("Forming a connection to ");
//     Serial.println(myDevice->getAddress().toString().c_str());
//     //啟動BLE GATT Client
//     BLEClient*  pClient  = BLEDevice::createClient();
//     Serial.println("啟動Client!");
//     //設置連線之回調函數
//     pClient->setClientCallbacks(new MyClientCallback());
//     //與BLE Server連線
//     pClient->connect(myDevice);  
//     Serial.println(" - Connected to server");
//     //設置最大資料傳輸長度517位元組
//     //set client to request maximum MTU from server (default is 23 otherwise)
//     pClient->setMTU(517); 
  
//     //Obtain a reference to the service we are after in the remote BLE server.
//     //從BLE Server取得所需要的服務
//     BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    
//     if (pRemoteService == nullptr) {
//       Serial.print("Failed to find our service UUID: ");
//       Serial.println(serviceUUID.toString().c_str());
//       pClient->disconnect();
//       return false;
//     }
//     //找到該Service之UUID
//     Serial.println(" - Found our service");


//     // Obtain a reference to the characteristic in the service of the remote BLE server.
//     //從BLE Service取得所需要的特徵
//     pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID1);
//     if (pRemoteCharacteristic == nullptr) {
//       Serial.print("Failed to find our characteristic UUID: ");
//       Serial.println(charUUID1.toString().c_str());
//       pClient->disconnect();
//       return false;
//     }
//     //找到該特徵之UUID
//     Serial.println(" - Found our characteristic");

//     // Read the value of the characteristic.
//     // 由特徵讀取值
//     if(pRemoteCharacteristic->canRead()) {
//       std::string value = pRemoteCharacteristic->readValue();
//       //Serial.print("The characteristic value was: ");
//       //Serial.println(value.c_str());
//     }


//     if(pRemoteCharacteristic->canNotify())
//       pRemoteCharacteristic->registerForNotify(notifyCallback);
//     //連線成功
//     connected = true;
//     return true;
// }
// //回調BLE SERVER狀況
// class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
//  /**
//    * Called for each advertising BLE server.
//    */
//   void onResult(BLEAdvertisedDevice advertisedDevice) {
//     Serial.print("BLE Advertised Device found: ");
//     Serial.println(advertisedDevice.toString().c_str());

//     // We have found a device, let us now see if it contains the service we are looking for.
//     if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
//       //找到該SERVER，則停止廣播
//       BLEDevice::getScan()->stop();
//       myDevice = new BLEAdvertisedDevice(advertisedDevice);
//       doConnect = true;
//       doScan = true;

//     } // Found our server
//   } // onResult
// }; // MyAdvertisedDeviceCallbacks
//--------------------------------------------------------------

//超音波感測器---------------------------------------------------
  float echoTime; //傳回時間(前)
  float distance;//轉換成距離(前)
//--------------------------------------------------------------

//MPU6050宣告初始值----------------------------------------------
MPU6050 accelgyro;
// MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

//#define LED_PIN 13
//bool blinkState = false;

String str=""; //儲存結果為字串

//int accel_1[3]; // 三軸感測
//int accel_2[3]; // 變化後(變化量)
int gyro[3];
//--------------------------------------------------------------

//Bluetooth----------------------------------------------------------
String msg;
//--------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1);
  //BlueTooth-----------------------------------------------------
  SerialBT.begin("SmartCane");
  //--------------------------------------------------------------
  //BLE-----------------------------------------------------------
  // Serial.println("開始BLE Client....");
  // BLEDevice::init("SmartCane"); //初始化，並啟動BLE裝置

  //掃描附近BLE SERVER
  // BLEScan* pBLEScan = BLEDevice::getScan();
  // pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  // pBLEScan->setInterval(1349);
  // pBLEScan->setWindow(449);
  // pBLEScan->setActiveScan(true);
  // pBLEScan->start(5, false);
  //--------------------------------------------------------------

  //震動模組-------------------------------------------------------
  pinMode(Motor, OUTPUT);
  //--------------------------------------------------------------
  //蜂鳴器--------------------------------------------------------
  pinMode(Buzzer, OUTPUT);
  //--------------------------------------------------------------

  //超音波感測器---------------------------------------------------

  //超音波前
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);

  //--------------------------------------------------------------
  //MPU6050-------------------------------------------------------
  delay(1);
  Serial.println("MPU6050: Wire.begin()");
  Wire.begin();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
    
  // verify connection
  Serial.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  while(!accelgyro.testConnection()){
    Serial.println("MPU6050 connection failed");
    delay(500);
  }
  Serial.println("MPU6050 connection successful");

  //G-sensor offsets //初始值誤差補償
  Serial.println("Updating internal sensor offsets...");
  accelgyro.setXGyroOffset(50);
  accelgyro.setYGyroOffset(20);
  accelgyro.setZGyroOffset(-35);
//----------------
}

void loop() {
  //BlueTooth------------------------------------------------
  //讀取並發送藍芽訊號
  /* //暫時沒用到
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  */
  //接收藍芽訊號
  if (SerialBT.available()) {
    msg = SerialBT.readString();
    Serial.println(msg);
    if(msg == "finding"){
      finding = true;
    }
  }
  //---------------------------------------------------------
  //BLE------------------------------------------------------
  // if (doConnect == true) {
  //   if (connectToServer()) {
  //     Serial.println("We are now connected to the BLE Server.");
  //   } else {
  //     Serial.println("We have failed to connect to the server; there is nothin more we will do.");
  //   }
  //   doConnect = false;
  //   }
  //   if (connected) {
      
  //   }
  //   else if(doScan){
  //     BLEDevice::getScan()->start(0);
  //   }
  //---------------------------------------------------------
  //主功能程式------------------------------------------------
  if (finding){
    Serial.println(msg);
    beep();
  }else{
    digitalWrite(Buzzer, LOW);
  }
  if(isMoving()){  //if MPU6050晃動中
    finding = false;
    //超音波前
    digitalWrite(Trig, LOW);  //關閉
    delayMicroseconds(5);
    digitalWrite(Trig, HIGH); //啟動
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);  //關閉
    echoTime = pulseIn(Echo, HIGH); //傳回時間
    distance = echoTime * 34 / 1000 / 2; //轉換成距離

    Serial.print("Distance: "); //莫名壞掉，改用兩行
    Serial.println(distance);
    delay(50);

    if(distance <= 100){    //距離1公尺
      digitalWrite(Motor, HIGH);
    }
    else{
      digitalWrite(Motor, LOW);
      Serial.println("None detected");
    }//超音波結尾
  
  }else{ //if not moving
    digitalWrite(Motor, LOW);
    Serial.println("Sleeping. (Not moving)");
      delay(1000);
  }
  //---------------------------------------------------------
}//loop結尾

//函式宣告
bool isMoving(){  //MPU6050是否晃動中
//void gsensorPrint(){
  if (!accelgyro.testConnection()){
    Serial.println("MPU6050 connection failed");
    delay(5000);
  }
  else{
    // read raw accel/gyro measurements from device

    //全讀
    //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

/*  //用a偵測 (三軸角度)
    accelgyro.getAcceleration(&ax, &ay, &az); //只抓a

    accel_1[0]=ax;
    accel_1[1]=ay;
    accel_1[2]=ay;
*/

    //用g偵測 (加速度)
    accelgyro.getRotation(&gx, &gy, &gz); //只抓g
    //#ifdef OUTPUT_READABLE_ACCELGYRO
    // display tab-separated accel/gyro x/y/z values

/*  //使用其他值時的列印函式
        //存成字串一起輸出
        //str ="a/g= " + String(ax) +' '+ String(ay) +' '+ String(az) +' '+ String(gx) +' '+ String(gy) +' '+ String(gz);
        //用"\t"分隔:
        //str = String(ax) +"\t"+ String(ay) +"\t"+ String(az) +"\t"+ String(gx) +"\t"+ String(gy) +"\t"+ String(gz);
        
        //只抓Acceleration:
        //str = "a/g= " + String(ax) +' '+ String(ay) +' '+ String(az);
*/
        //只抓G時的print
        str = "a/g= " + String(gx) +' '+ String(gy) +' '+ String(gz);

        Serial.println(str);

    // G-sensor 作動值
    int limit = 200;

    //使用G偵測，setup需設定補償
    if((gx < limit * -1 || gx > limit) || (gy < limit * -1 || gy > limit) || (gz<limit * -1 || gz > limit)) {
      return true;
    }else{
      return false;
    }

/*  //測試a變化量，需有明顯的瞬間角度改變
    accelgyro.getAcceleration(&ax,  &ay,  &az); //只抓a
    str = "a/g= " + String(ax) +' '+ String(ay) +' '+ String(az);
    Serial.println(str);
    
    accel_2[0]=ax;
    accel_2[1]=ay;
    accel_2[2]=ay;

    //用accel測試晃動
    for(int i=0; i<3; i++){
      if(accel_2[i]>accel_1[i]+250 || accel_2[i]<accel_1[i]-250){
        return true;
      }
     return false;
    }
*/
  } // end else
}// G-sonsor func

void beep(){
    digitalWrite(Buzzer, HIGH);
    delay(1000);
    digitalWrite(Buzzer, LOW);
    delay(100);
}