/*
Base Station Program

Sources:
https://github.com/educ8s/Arduino-ST7920-Temperature-and-Humidity-Monitor

NOTE:
 For this example you'll need SimpleTimer library:
   https://github.com/jfturcot/SimpleTimer
 Visit this page for more information:
   http://playground.arduino.cc/Code/SimpleTimer

*/

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <stdio.h>

// SPI SD card and LCD libraries
#include <U8g2lib.h>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <Wire.h>

// Libraries to get time from NTP Server
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "time.h"
#include <string.h>

//MQTT Cayenne includes
#define CAYENNE_PRINT Serial     // Comment this out to disable prints and save space
#include <CayenneMQTTESP32.h> // Change this to use a different communication device. See Communications examples.
#include <SimpleTimer.h>

// Define ALTERNATE_PINS to use non-standard GPIO pins for SPI bus
#ifdef ALTERNATE_PINS
  #define VSPI_MISO   2
  #define VSPI_MOSI   4
  #define VSPI_SCLK   0
  #define VSPI_SS     33

  #define HSPI_MISO   26
  #define HSPI_MOSI   27
  #define HSPI_SCLK   25
  #define HSPI_SS     32
#else
  #define VSPI_MISO   MISO
  #define VSPI_MOSI   MOSI
  #define VSPI_SCLK   SCK
  #define VSPI_SS     SS

  #define HSPI_MISO   26 
  #define HSPI_MOSI   13
  #define HSPI_SCLK   27
  #define HSPI_SS     16
#endif

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif

static const int spiClk = 4000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

//Input Pins
//hw_timer_t *My_timer = NULL;// Interrupt Timer for button
constexpr int Kill_Switch = 22;
constexpr int board_Select = 35;
constexpr int Switch_relay = 39;
//Output Pins
constexpr int Plug1_PIN = 32;
constexpr int Plug2_PIN = 25;

/* Cayenne credentials setup*/
// WiFi network info.
char ssid[] = "Wi-Fi name herer";
char wifiPassword[] = "Wi-Fi password here";
//uint8_t start_ap = 0; // 1 for True then start wifi AP
//String  IPAdress;

//WebServer server(80);

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 1;
const int   daylightOffset_sec = 7200;
char measTime[11];

// Save SD card reading number on RTC memory
RTC_DATA_ATTR int readingID = 0;
String dataMessage;
// Define NTP Client to get time
//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP);
// Variables to save date and time
//String formattedDate;
//String dayStamp;
//String timeStamp;

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char username[] = "Cayenne authentication key #1";
char password[] = "Cayenne authentication key #2";
char clientID[] = "Cayenne authentication key #3";

// Use Virtual Channel for uptime display.
//Plug 1 variable channels
#define Voltage_1 0
#define Current_1 1
#define Power_1 2
#define PF_1 3
#define frequency_1 4
#define PowerGraph_1 12

//Plug 2 variable channels
#define Voltage_2 5
#define Current_2 6
#define Power_2 7
#define PF_2 8
#define frequency_2 9
#define PowerGraph_2 13

//Plug relay channels
#define Relay_1 10
#define Relay_2 11

volatile bool meas1 = false;
volatile bool meas2 = false;

/*
ESP NOW Setup
*/
// REPLACE WITH THE MAC Address of your Smart Plug
uint8_t broadcastAddress_1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};// Board 1
uint8_t broadcastAddress_2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02};// Board 2

// Structure example to send data
// Must match the receiver structure
typedef struct struct_measurement {
    int id; // must be unique for each sender board
    float Voltage; // Voltage measurement to be sent
    float Current; // Current measurement to be sent
    float Power; // Calculated power consumption
    float PF; // Power factor, range 0~90
    float Sample_frequency; // Calculated sampling frequency
    float frequency; // Calculated Mains frequency
    char TimeStamp[11]; // Time Data is received
} struct_measurement;

// Create a structure to process the readings from each board
struct_measurement Measurement;

// Create a structure to hold the readings from each board
struct_measurement board_1;
struct_measurement board_2;

//Struct to contol State: Relay state and clock time
typedef struct struct_state {
    int id; // must be unique for each sender board
    int request; //Maesurement request variable
    char TimeStamp[11]; // Time Data is received
    int relay_state; // State of Relay switch: ON(true)/OFF(false)
} struct_state;

// Create a structure to process the state from each board
volatile struct_state State;

// Create a structure to hold the state for each board
struct_state State_1;
struct_state State_2;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Create an array with all the structures
struct_measurement boardsStruct[2] = {board_1, board_2};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  readingID++;
  //getTimeStamp();
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&Measurement, incomingData, sizeof(Measurement));
  Serial.printf("Board ID %u: %u bytes\n", Measurement.id, len);
  // Update the structures with the new incoming data
  boardsStruct[Measurement.id-1].Voltage = Measurement.Voltage;
  boardsStruct[Measurement.id-1].Current = Measurement.Current;
  boardsStruct[Measurement.id-1].Power = Measurement.Power;
  boardsStruct[Measurement.id-1].PF = Measurement.PF;
  boardsStruct[Measurement.id-1].frequency = Measurement.frequency;
  boardsStruct[Measurement.id-1].Sample_frequency = Measurement.Sample_frequency;
  Serial.printf("Voltage: %f \n", boardsStruct[Measurement.id-1].Voltage);
  Serial.printf("Current: %f \n", boardsStruct[Measurement.id-1].Current);
  Serial.printf("Apperant Power: %f \n", boardsStruct[Measurement.id-1].Power);
  Serial.printf("PF: %f \n", boardsStruct[Measurement.id-1].PF);
  Serial.printf("frequency: %f \n", boardsStruct[Measurement.id-1].frequency);
  Serial.printf("Sampling frequency: %f \n", boardsStruct[Measurement.id-1].Sample_frequency);
  Serial.println();
  if(Measurement.id==1){
    sendData_1();
    meas1 = true;
  }
  if(Measurement.id==2){
    sendData_2();
    meas2 = true;
  }
  logSDCard();
}

/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
*/

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// U8g2 Contructor List (Frame Buffer) The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ HSPI_SCLK, /* data=*/ HSPI_MOSI, /* CS=*/ HSPI_SS, /* reset=*/ HSPI_MISO); // ESP32

char voltage_1[4] = "***";
char current_1[6] = "**.**";
char Com_pow_1[5] = "****";
char Sig_freq_1[6] = "**.**";
char plug_on_1[4] = "***";

char voltage_2[4] = "***";
char current_2[6] = "**.**";
char Com_pow_2[5] = "****";
char Sig_freq_2[6] = "**.**";
char plug_on_2[4] = "***";

/*
void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.enableUTF8Print();
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}
*/
void draw(){
  
  u8g2.drawFrame(0, 0, SCREEN_WIDTH/2, SCREEN_HEIGHT);         
  u8g2.drawFrame(SCREEN_WIDTH/2, 0, SCREEN_WIDTH/2, SCREEN_HEIGHT);        
  
  u8g2.drawStr(2, 9, "Plug 1:");
  u8g2.drawStr(2, 19, "Volt: "); u8g2.drawStr(SCREEN_WIDTH/2 + 32, 19, voltage_1);
  u8g2.drawStr(2, 29, "Curr: "); u8g2.drawStr(SCREEN_WIDTH/2 + 32, 29, current_1);
  u8g2.drawStr(2, 39, "Pow: "); u8g2.drawStr(SCREEN_WIDTH/2 + 32, 39, Com_pow_1);
  u8g2.drawStr(2, 49, "Freq: "); u8g2.drawStr(SCREEN_WIDTH/2 + 32, 49, Sig_freq_1);
  u8g2.drawStr(2, 59, "ON/OFF: "); u8g2.drawStr(SCREEN_WIDTH/2 + 45, 59, plug_on_1); 
    
  u8g2.drawStr(SCREEN_WIDTH/2 + 2, 9, "Plug 2:");
  u8g2.drawStr(SCREEN_WIDTH/2 + 2, 19, "Volt: "); u8g2.drawStr(32, 19, voltage_2);
  u8g2.drawStr(SCREEN_WIDTH/2 + 2, 29, "Curr: "); u8g2.drawStr(32, 29, current_2);
  u8g2.drawStr(SCREEN_WIDTH/2 + 2, 39, "Pow: "); u8g2.drawStr(32, 39, Com_pow_2);
  u8g2.drawStr(SCREEN_WIDTH/2 + 2, 49, "Freq: "); u8g2.drawStr(32, 49, Sig_freq_2);
  u8g2.drawStr(SCREEN_WIDTH/2 + 2, 59, "ON/OFF: "); u8g2.drawStr(45, 59, plug_on_2);
}

// This function sends board 1 measurements to Cayenne on Virtual Channels.
void sendData_1()
{
	// Send values using the virtualWrite function. Cayenne currently accepts int and float values.
	// Please don't send more that 10 values per second.
	Cayenne.virtualWrite(Voltage_1, boardsStruct[0].Voltage, "Voltage", "Vrms");
  Cayenne.virtualWrite(Current_1, boardsStruct[0].Current, "Current", "Arms");
  Cayenne.virtualWrite(Power_1, boardsStruct[0].Power, "Complex Power", "VA");
  Cayenne.virtualWrite(PF_1, boardsStruct[0].PF);
  Cayenne.virtualWrite(frequency_1, boardsStruct[0].frequency, "Frequency", "Hz");
  Cayenne.virtualWrite(PowerGraph_1, boardsStruct[0].Power, "Complex Power", "VA");
  //Cayenne.virtualWrite(5, millis() / 1000);
}
// This function sends board 2 measurements to Cayenne on Virtual Channels.
void sendData_2()
{
	// Send values using the virtualWrite function. Cayenne currently accepts int and float values.
	// Please don't send more that 10 values per second.
  Cayenne.virtualWrite(Voltage_2, boardsStruct[1].Voltage, "Voltage", "Vrms");
  Cayenne.virtualWrite(Current_2, boardsStruct[1].Current, "Current", "Arms");
  Cayenne.virtualWrite(Power_2, boardsStruct[1].Power,"Complex Power", "VA");
  Cayenne.virtualWrite(PF_2, boardsStruct[1].PF);
  Cayenne.virtualWrite(frequency_2, boardsStruct[1].frequency, "Frequency", "Hz");
  Cayenne.virtualWrite(PowerGraph_2, boardsStruct[1].Power, "Complex Power", "VA");
  //Cayenne.virtualWrite(5, millis() / 1000);
}
void toggle_plug_1(){Cayenne.virtualWrite(Relay_1, State_1.relay_state);}
void toggle_plug_2(){Cayenne.virtualWrite(Relay_2, State_2.relay_state);}

// Timer flags
#define TIMER1_INTERVAL_MS        20
#define DEBOUNCING_INTERVAL_MS    100

volatile bool SWPressed = false;

volatile bool killSWPressedNoted = false;
volatile bool boardSWPressedNoted = false;
volatile bool relaySWPressedNoted = false;

static unsigned int debounceCountSWPressed  = 0;
static unsigned int debounceCountSWReleased = 0;

hw_timer_t *task_timer = NULL;
uint32_t timer_1_ms = 0;
uint8_t DoCheckWifiConnection = 0;
uint8_t makeRequest = 0;
uint8_t board_id = 1;
uint8_t boardNr = 0;
uint8_t ledFlash = 0;

// Timers
void IRAM_ATTR KillSWPressed()
{
  killSWPressedNoted  = true;
}

void IRAM_ATTR BoardSWPressed()
{
  boardSWPressedNoted  = true;
}

void IRAM_ATTR RelaySWPressed()
{
  relaySWPressedNoted  = true;
}

void IRAM_ATTR onTimer() {
  timer_1_ms++;

  //button debonce
  if(digitalRead(Kill_Switch)){
    // Start debouncing counting debounceCountSWPressed and clear debounceCountSWReleased
    debounceCountSWPressed++;
    if (debounceCountSWPressed >= DEBOUNCING_INTERVAL_MS / TIMER1_INTERVAL_MS)
    {
      // Call and flag SWPressed
      if (!SWPressed)
      {
        SWPressed = true;
        KillSWPressed();
      }
      debounceCountSWPressed = 0;
    }
  }else
  {
    debounceCountSWReleased++;
    // Start debouncing counting debounceCountSWReleased and clear debounceCountSWPressed
    if ( SWPressed && (debounceCountSWReleased >= DEBOUNCING_INTERVAL_MS / TIMER1_INTERVAL_MS))
    {
      SWPressed = false;
      // Call and flag SWPressed
      debounceCountSWPressed = 0;
    }
  }
  
  if(digitalRead(board_Select)){
    // Start debouncing counting debounceCountSWPressed and clear debounceCountSWReleased
    debounceCountSWPressed++;
    if (debounceCountSWPressed >= DEBOUNCING_INTERVAL_MS / TIMER1_INTERVAL_MS)
    {
      // Call and flag SWPressed
      if (!SWPressed)
      {
        SWPressed = true;
        BoardSWPressed();
      }
      debounceCountSWPressed = 0;
    }
  }else
  {
    debounceCountSWReleased++;
    // Start debouncing counting debounceCountSWReleased and clear debounceCountSWPressed
    if ( SWPressed && (debounceCountSWReleased >= DEBOUNCING_INTERVAL_MS / TIMER1_INTERVAL_MS))
    {
      SWPressed = false;
      // Call and flag SWPressed
      debounceCountSWPressed = 0;
    }
  }
  
  if(digitalRead(Switch_relay)){
    // Start debouncing counting debounceCountSWPressed and clear debounceCountSWReleased
    debounceCountSWPressed++;
    if (debounceCountSWPressed >= DEBOUNCING_INTERVAL_MS / TIMER1_INTERVAL_MS)
    {
      // Call and flag SWPressed
      if (!SWPressed)
      {
        SWPressed = true;
        RelaySWPressed();
      }
      debounceCountSWPressed = 0;
    }
  }else
  {
    debounceCountSWReleased++;
    // Start debouncing counting debounceCountSWReleased and clear debounceCountSWPressed
    if ( SWPressed && (debounceCountSWReleased >= DEBOUNCING_INTERVAL_MS / TIMER1_INTERVAL_MS))
    {
      SWPressed = false;
      // Call and flag SWPressed
      debounceCountSWPressed = 0;
    }
  }

  //timer conditions
  if ((timer_1_ms % 5000) == 0)// 5 second timer
  {
    Serial.println("request measurement");
    //request measurements
    makeRequest = 1;
  }
  if ((timer_1_ms % 180000) == 0) // Check wifi connection every 3 minutes
  {
      //DoCheckWifiConnection = 1;
  }
  if (timer_1_ms > 0xFFFFFFFE) {
      timer_1_ms = 0;
  }  
}

void RequestMeasurement(){
    get_ntp_time();
    // Send message via ESP-NOW to Plug 1
    if(board_id==1){
      for(int i = 0; i<11;i++){
        State_1.TimeStamp[i] = measTime[i];
      }
      esp_err_t result = esp_now_send(broadcastAddress_1, (uint8_t *) &State_1, sizeof(State_1));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
      board_id++;
      return;
    }
    if(board_id==2){
      for(int i = 0; i<11;i++){
        State_2.TimeStamp[i] = measTime[i];
      }
      esp_err_t result = esp_now_send(broadcastAddress_2, (uint8_t *) &State_2, sizeof(State_2));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
      board_id=1;
     return;
    }
}
char timeMonth[10];
char timeDay[3];
void get_ntp_time() {
  struct tm timeinfo; // used to store time
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  //Serial.println(&timeinfo, "%H:%M:%S");  
  //storing measurement request time
  int length = 0; 
  int j=0;
  Serial.println("Time variables");
  
  strftime(timeMonth,10, "%B", &timeinfo);
  strftime(timeDay,3, "%d", &timeinfo);
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  char timeMinutes[3];
  strftime(timeMinutes,3, "%M", &timeinfo);
  char timeSeconds[3];
  strftime(timeSeconds,3, "%S", &timeinfo);
  //strcat(timeHour,":");
  
  for(j=0;timeHour[j] != '\0'; j++,length++){
    measTime[length] = timeHour[j];
  }
  measTime[length] = ':';
  length++;
  for(j=0;timeMinutes[j] != '\0'; j++,length++){
    measTime[length] = timeMinutes[j];
  }
  measTime[length] = ':';
  length++;
  for(j=0;timeSeconds[j] != '\0'; j++,length++){
    measTime[length] = timeSeconds[j];
  }
  measTime[length] = '\0';
  Serial.println(measTime);
}

void initWifi(){
  //Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  get_ntp_time();
}

void initESPNOW(){
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }else{
    Serial.println("ESP-NOW Initialized");
  }
  
  // Once ESPNow is successfully Init, we will register for Send and recv CB to
  // get recv packer info,
  esp_now_register_recv_cb(OnDataRecv);
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress_1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress_2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void LCD_Setup(){
  /*LCD setup*/
  //u8g2.begin();
  //u8g2_prepare();
  /*LCD initialization*/
  u8g2.begin();
  u8g2.enableUTF8Print();
  //u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.setFont(u8g2_font_6x10_tf); 
  u8g2.setColorIndex(1); 
}

void SD_Setup(){
  if(!SD.begin(VSPI_SS, *vspi, spiClk)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  /*
  listDir(SD, "/", 0);
  createDir(SD, "/mydir");
  listDir(SD, "/", 0);
  removeDir(SD, "/mydir");
  listDir(SD, "/", 2);
  writeFile(SD, "/hello.txt", "Hello ");
  appendFile(SD, "/hello.txt", "World!\n");
  readFile(SD, "/hello.txt");
  deleteFile(SD, "/foo.txt");
  renameFile(SD, "/hello.txt", "/foo.txt");
  readFile(SD, "/foo.txt");
  testFileIO(SD, "/test.txt");
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  */

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
}

void SPI_Setup(){
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  
  //clock miso mosi ss
  #ifndef ALTERNATE_PINS
    //initialise vspi with default pins
    //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
    vspi->begin();
  #else
    //alternatively route through GPIO pins of your choice
    vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); //SCLK, MISO, MOSI, SS
  #endif

  #ifndef ALTERNATE_PINS
    //initialise hspi with default pins
    //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
    hspi->begin();
  #else
    //alternatively route through GPIO pins
    hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS
  #endif
  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(vspi->pinSS(), OUTPUT); //VSPI SS
  pinMode(hspi->pinSS(), OUTPUT); //HSPI SS
}

/*
Deivce Setup Parameters
*/
void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello!");

  initWifi();

  /*ESP NOW setup*/
  initESPNOW();  
  Cayenne.begin(username, password, clientID);

  // Initialize a NTPClient to get time
  //timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  //timeClient.setTimeOffset(3600);

  SPI_Setup();
  /*SD card setup*/
  SD_Setup();
  /*LCD screen setup*/
  LCD_Setup();

  //Interrupt timer setup
  task_timer = timerBegin(0, 80, true);//Scales clock to 1MHz
  timerAttachInterrupt(task_timer, &onTimer, true);
  timerAlarmWrite(task_timer, 1000, true); ///interrupt every 1ms
  timerAlarmEnable(task_timer);//Enables interrupt

  pinMode(Kill_Switch, INPUT);
  pinMode(board_Select, INPUT);
  pinMode(Switch_relay, INPUT);
  // initialize the LED pin as an output
  pinMode(Plug1_PIN, OUTPUT);
  pinMode(Plug2_PIN, OUTPUT);

  //initialize Plug state
  State_1.id = 1;
  State_1.relay_state = 1;
  State_1.request = 1;

  State_2.id = 2;
  State_2.relay_state = 1;
  State_2.request = 1;
}

void loop(void) {
  /*
  if (DoCheckWifiConnection) {
    DoCheckWifiConnection = 0;
    CheckWifiConnection();
  }  
  */
  //boardsStruct[0].Voltage = 230.00;
  //boardsStruct[0].Current = 10.00;
  //boardsStruct[0].Power = 1500.00;
  //boardsStruct[0].frequency = 50.00;

  if(makeRequest==1)
  {
    RequestMeasurement();
    makeRequest = 0;
  }
  if (boardSWPressedNoted)
  {
    boardSWPressedNoted = false;
    //Serial.println("Board switch pressed");
    if(State_2.relay_state == 1){
      State_2.relay_state = 0;
    }else{
      State_2.relay_state = 1;
    }
  }
  
  if (relaySWPressedNoted)
  {
    relaySWPressedNoted = false;
    //Serial.println("Select switch pressed");
    if(State_1.relay_state == 1){
      State_1.relay_state = 0;
    }else{
      State_1.relay_state = 1;
    }
  }
  if (killSWPressedNoted)
  {
    killSWPressedNoted = false;
    //Serial.println("Kill switch pressed");
    State_1.relay_state = 0;
    digitalWrite(Plug1_PIN, LOW);
    //toggle_plug_1();

    State_2.relay_state = 0;
    digitalWrite(Plug2_PIN, LOW);
    //toggle_plug_2();
  }

  Cayenne.loop(); // Runs main loop

  if(State_1.relay_state == 1){
    digitalWrite(Plug1_PIN, HIGH);
    toggle_plug_1();
  }else{
    digitalWrite(Plug1_PIN, LOW);
    toggle_plug_1();
  }

  if(State_2.relay_state == 1){
    digitalWrite(Plug2_PIN, HIGH);
    toggle_plug_2();
  }else{
    digitalWrite(Plug2_PIN, LOW);
    toggle_plug_2();
  }

  updateDisplayValues();
  // picture loop  
  u8g2.clearBuffer();
  draw();
  u8g2.sendBuffer();
    
  // deley between each page
  delay(100);
}

// This function is called when data is sent from Cayenne.
CAYENNE_IN(Relay_1)
{
	// Write value to turn the relay switch on or off.
	  State_1.relay_state = getValue.asInt();
    Serial.println(State_1.relay_state);
}

CAYENNE_IN(Relay_2)
{
	// Write value to turn the relay switch on or off.
	  State_2.relay_state = getValue.asInt();
    Serial.println(State_2.relay_state);
  
}

// This function is called at intervals to send data to Cayenne and keep the device online.
// Will create a temporary green widget on Channel 0, make it permanent by clicking on '+'. 
/*
CAYENNE_OUT(Relay_1)
{
	CAYENNE_LOG("Send data for Relay_1");
	// This command writes the device's uptime in seconds to the Virtual Channel. 
	Cayenne.virtualWrite(Relay_1, State_1.relay_state);
}

CAYENNE_OUT(Relay_2)
{
	CAYENNE_LOG("Send data for Relay_2");
	// This command writes the device's uptime in seconds to the Virtual Channel. 
	Cayenne.virtualWrite(Relay_2, State_2.relay_state);
}
*/
void updateDisplayValues(){
  if(meas1 == true){
    sprintf(voltage_1, "%3.0f", boardsStruct[0].Voltage);
    //dtostrf(boardsStruct[0].Voltage, 5, 0, voltage_1); // Convert float to string
    sprintf(current_1, "%2.2f", boardsStruct[0].Current);
    //dtostrf(boardsStruct[0].Current, 4, 1, current_1); // Convert float to string
    sprintf(Com_pow_1, "%4.0f", boardsStruct[0].Power);
    //dtostrf(boardsStruct[0].Power, 4, 0, Com_pow_1); // Convert float to string
    sprintf(Sig_freq_1, "%2.2f", boardsStruct[0].frequency);
    //dtostrf(boardsStruct[0].frequency, 4, 2, Sig_freq_1); // Convert float to string
    if(State_1.relay_state == 1){
      strcpy(plug_on_1, "ON");
    }else{
      strcpy(plug_on_1, "OFF");
    }
    meas1 = false;
  }

  if(meas2 == true){
    sprintf(voltage_2, "%3.0f", boardsStruct[1].Voltage);
    //dtostrf(boardsStruct[1].Voltage, 5, 0, voltage_2); // Convert float to string
    sprintf(current_2, "%2.2f", boardsStruct[1].Current);
    //dtostrf(boardsStruct[1].Current, 4, 1, current_2); // Convert float to string
    sprintf(Com_pow_2, "%4.0f", boardsStruct[1].Power);
    //dtostrf(boardsStruct[1].Power, 4, 0, Com_pow_2); // Convert float to string
    sprintf(Sig_freq_2, "%2.2f", boardsStruct[1].frequency);
    //dtostrf(boardsStruct[1].frequency, 4, 2, Sig_freq_2); // Convert float to string
    if(State_2.relay_state == 1){
      strcpy(plug_on_2, "ON");
    }else{
      strcpy(plug_on_2, "OFF");
    }
    meas2 = false;
  }
}
/*
int8_t WifiNetworkScan() { // returns number of networks found
    Serial.println("Wi-Fi network scan start");

    // WiFi.scanNetworks will return the number of networks found
    int8_t n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n <= 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (uint8_t i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            delay(10);
        }
    }
    Serial.println("");
    return (n);
}

void CheckWifiConnection() { // attempt to reconnect to stored network
    if ((WiFi.status() != WL_CONNECTED) && (start_ap == 0)) {
        uint8_t MatchingNetworkFound = 0;
        Serial.println("Wifi disconnected, attempting to reconnect now");
        WiFi.disconnect(true);
        // WiFi.mode(WIFI_STA);
        int8_t n = WifiNetworkScan();
        if (n < 0) {
            n = 0;
        }
        for (int8_t i = 0; i < n; ++i) {
            char temp[4];
            WiFi.SSID(i).toCharArray(temp, sizeof(temp));
            if (temp[0] == ssid[0])
                if (temp[1] == ssid[1])
                    if (temp[2] == ssid[2]) {
                        i = n;
                        Serial.println(
                            "Network found that matched stored credentials, "
                            "attempting connection now");
                        WiFi.disconnect();
                        // WiFi.mode(WIFI_STA);
                        WiFi.begin(ssid, wifiPassword);
                        Serial.println("Connecting to local network : ");
                        Serial.println(ssid);
                        Serial.println(wifiPassword);
                        MatchingNetworkFound = 1;
                    }
        }
        if (MatchingNetworkFound == 0) {
            Serial.println("No matching networks found, starting AP");
            // Print our IP address
            WiFi.disconnect();
            Serial.println("AP running");
            Serial.print("My IP address: ");
            Serial.println(WiFi.softAPIP());
            IPAdress = WiFi.softAPIP().toString();
        }
    }
}
*/

/*
SD card functions
*/

// Function to get date and time from NTPClient
/*
void getTimeStamp() {
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);

  // Extract date
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  Serial.println(dayStamp);
  // Extract time
  timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
  Serial.println(timeStamp);
}
*/

// Write the sensor readings on the SD card
void logSDCard() {
  dataMessage = String(readingID) + "," + String(timeMonth) + "," + String(timeDay) + ","+ String(measTime) + "," + " Plug id: " + String(Measurement.id) + "," + " Voltage: " +
                String(Measurement.Voltage) + "," + " Current: " + String(Measurement.Current) + "," + " Apparent power: " + String(Measurement.Power) + "," + 
                " Frequency: " + String(Measurement.frequency) + "," + " Sample frequency: " + String(Measurement.Sample_frequency) + "," + " Power factor: " + 
                String(Measurement.PF) + "," + "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/data.txt", dataMessage.c_str());
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}