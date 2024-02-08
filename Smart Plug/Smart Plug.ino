/*
Main Smart Plug Program
*/
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include <ADS1115.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <math.h>
#include <complex.h>
#include <stdio.h>
#include <stdlib.h>

/*
    ADS1115 ADC Setup
*/
//Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
ADS1115 adc0(ADS1115_DEFAULT_ADDRESS);

// Use 860 SPS or 475 SPS
/*
SAMPLE RATE 860 SPS:
- 86 Samples for 5 wave cycles at 50Hz
- 0.1s time lapse
- Comes close to peak: 0.994

SAMPLE RATE 475 SPS:
- 19 samples for 2 wave cycles at 50Hz or 38 samples for 4 wave cycles at 50Hz
- 0.04s or 0.08s time lapse
- Far from peak
*/
const int SAMPLE_ARRAY_LENGTH = 128;
const int ARRAY_LENGTH = 256;
//float time_period = 0.1;//seconds

int16_t Gain_level;
int16_t Data_rate;

// Pin connected to the ALERT/RDY signal for new sample notification.
// Wire ADS1115 ALERT/RDY pin to Arduino pin 18
constexpr int READY_PIN = 18;
//Input Pins
//hw_timer_t *My_timer = NULL;// Interrupt Timer for button
constexpr int BUTTON_PIN = 34;
//constexpr int OPTO_4N25_PIN = 25;
//Output Pins
constexpr int FAN_PIN = 32;
constexpr int RELAY_PIN = 33;

const unsigned long SampleIntervalMicroseconds = 6600;//Sampling time period, in milliseconds
float scaler = 4/(float)ARRAY_LENGTH;//FFT scaler to get real amplitude

typedef __complex__ double cplx;

//Voltage measurement variables
int16_t adc_0[ARRAY_LENGTH];
float Voltage[ARRAY_LENGTH];
float rVoltage[ARRAY_LENGTH];
float iVoltage[ARRAY_LENGTH];
cplx cVoltage[ARRAY_LENGTH];
unsigned long voltageMicros[SAMPLE_ARRAY_LENGTH];

//Current measurement variables
int16_t adc_1[ARRAY_LENGTH];
float Current[ARRAY_LENGTH];
float rCurrent[ARRAY_LENGTH];
float iCurrent[ARRAY_LENGTH];
cplx cCurrent[ARRAY_LENGTH];
unsigned long currentMicros[SAMPLE_ARRAY_LENGTH];

float sampleFrequency;

// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

/*
    ESP NOW Setup
*/
// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};//Base Station

// Structure example to send data
// Must match the receiver structure
//Struct to send Measurements
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

// Create a struct_message called Measurement
struct_measurement Measurement;

//Struct to contol State: Relay state and clock time
typedef struct struct_state {
    int id; // must be unique for each sender board
    int request; //Maesurement request variable
    char TimeStamp[11]; // Time Data is received
    int relay_state; // State of Relay switch: ON(true)/OFF(false)
} struct_state;

// Create a struct_state called struct_state
struct_state State;

// Insert your SSID
constexpr char WIFI_SSID[] = "Wi-Fi name here";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
/*
volatile bool new_data = false;
void IRAM_ATTR NewDataReadyISR() {
  new_data = true;
}

void IRAM_ATTR onTimer(){
  if(){
  button_press = true;
  }
}*/

/*
void IRAM_ATTR ChangeState() {
  delay(40);
  if(digitalRead(BUTTON_PIN)==HIGH){
  State.relay_state = !State.relay_state;
  digitalWrite(FAN_PIN, !digitalRead(FAN_PIN));
  digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
  }
}
*/

/*FFT code START: Source = https://rosettacode.org/wiki/Fast_Fourier_transform#C*/
void _fft(cplx buf[], cplx out[], int n, int step)
{
	if (step < n) {
		_fft(out, buf, n, step * 2);
		_fft(out + step, buf + step, n, step * 2);
 
		for (int i = 0; i < n; i += 2 * step) {
			cplx t = cexp(-I * PI * i / n) * out[i + step];
			buf[i / 2]     = out[i] + t;
			buf[(i + n)/2] = out[i] - t;
		}
	}
}

void fft(cplx buf[], int n)
{
	cplx out[n];
	for (int i = 0; i < n; i++) out[i] = buf[i];
 
	_fft(buf, out, n, 1);
}
 
 
void show(const char * s, cplx buf[]) {
	printf("%s", s);
	for (int i = 0; i < 8; i++)
		if (!cimag(buf[i]))
			printf("%g ", creal(buf[i]));
		else
			printf("(%g, %g) ", creal(buf[i]), cimag(buf[i]));
}
/*FFT code END: Source = https://rosettacode.org/wiki/Fast_Fourier_transform#C*/

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&State, incomingData, sizeof(State));
  Serial.printf("Sent to Board %u: %u bytes\n", State.id, len);
  // Update the structures with the new incoming data
  Serial.printf("Id: %d \n", State.id);
  Serial.printf("Request State: %d \n", State.request);
  Serial.printf("Time of request: %c \n", State.TimeStamp);
  Serial.printf("Relay state: %d \n", State.relay_state);
  Serial.println();
}

void setup(void)
{
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Hello!");
  
  Serial.println("Getting single ended readings from AIN0 (Voltage) and AIN1 (Current)");
  Serial.println("ADC Range: 1x gain +/- 4.096V  1 bit = 0.125mV/ADS1115");

  // The ADC input range (or gain) is defined above, be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting this value incorrectly may destroy your ADC!
  
  Serial.println("Testing device connections...");
  Serial.println(adc0.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");
    
  adc0.initialize(); // initialize ADS1115 16 bit A/D chip

  // We're going to do single shot sampling
  adc0.setMode(ADS1115_MODE_SINGLESHOT);
    
  // Slow things down so that we can see that the "poll for conversion" code works
  adc0.setRate(ADS1115_RATE_860);
      
  // Set the gain (PGA) +/- 4.096V
  // Note that any analog input must be higher than –0.3V and less than VDD +0.3
  adc0.setGain(ADS1115_PGA_4P096);

  // ALERT/RDY pin will indicate when conversion is ready
  pinMode(READY_PIN,INPUT_PULLUP);
  adc0.setConversionReadyPinMode();

  // To get output from this method, you'll need to turn on the 
  //#define ADS1115_SERIAL_DEBUG // in the ADS1115.h file
  #ifdef ADS1115_SERIAL_DEBUG
  adc0.showConfigRegister();
  Serial.print("HighThreshold="); Serial.println(adc0.getHighThreshold(),BIN);
  Serial.print("LowThreshold="); Serial.println(adc0.getLowThreshold(),BIN);
  #endif

  // initialize digital pins as output.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // We get a falling edge every time a new sample is ready.
  //attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ChangeState, RISING);
  //pinMode(OPTO_4N25_PIN, INPUT);
  // initialize digital pins as input.
  pinMode(FAN_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  int32_t channel = getWiFiChannel(WIFI_SSID);
  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send and recv CB to
  // get recv packer info,
  esp_now_register_recv_cb(OnDataRecv);
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //Measurement.id = 1;//Board 1 identifier, change for each board
  //State.id = 1;//Board identifier, change for each board
  Measurement.id = 2;//Board 2 identifier, change for each board
  State.id = 2;//Board identifier, change for each board
  State.relay_state = 1;// Set State of Relay switch: ON(true)
  State.request = 0;
  digitalWrite(RELAY_PIN, HIGH);// Turn on Relay
  digitalWrite(FAN_PIN, HIGH);// Turn on fan

  //Interrupt timer setup
  /*
  My_timer = timerBegin(0, 80, true);//Scales clock to 1MHz
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 10000, true);//interrupt every 10ms
  timerAlarmEnable(My_timer);//Enables interrupt
  */
}

/** Poll the assigned pin for conversion status 
 */
void pollAlertReadyPin() {
  for (uint32_t i = 0; i<100000; i++)
    if (!digitalRead(READY_PIN)) return;
   Serial.println("Failed to wait for AlertReadyPin, it's stuck high!");
}

void loop(void)
{
  if(State.relay_state == 1){
    digitalWrite(RELAY_PIN, HIGH);// Turn on Relay
    digitalWrite(FAN_PIN, HIGH);// Turn on fan
  }else{
    digitalWrite(RELAY_PIN, LOW);// Turn on Relay
    digitalWrite(FAN_PIN, LOW);// Turn on fan
  }
  if(State.request==1){
    //Sample data
    Serial.print("Start Sampling:");Serial.println();
    unsigned long startTime = micros();
    unsigned long start = micros();
    for(int n = 0; n< SAMPLE_ARRAY_LENGTH; n++){
      while (micros() - startTime < SampleIntervalMicroseconds)
      {
        /* DOING NOTHING */
      }
      startTime += SampleIntervalMicroseconds;
      currentMicros[n] = micros();
      adc_0[n] = adc0.getConversionP0GND();
      pollAlertReadyPin();
      voltageMicros[n] = micros();
      adc_1[n] = adc0.getConversionP1GND();
      pollAlertReadyPin();
    }
    unsigned long end = micros();
    for(int n = 0; n< SAMPLE_ARRAY_LENGTH; n++){
      //ADC 1:
      //Voltage[n] = (float)(adc_1[n] * 0.0001) + 0.3286 - 1.637; //Add DC offset
      //Current[n] = (float)(adc_0[n] * 0.0001) + 0.3506 - 1.637; //Add DC offset
      //ADC 2:
      Voltage[n] = (float)(adc_1[n] * 0.0001) + 0.3044 - 1.642; //Add DC offset
      Current[n] = (float)(adc_0[n] * 0.0001) + 0.3049 - 1.642; //Add DC offset
    }
    sampleFrequency = (end - start)/SAMPLE_ARRAY_LENGTH;
    sampleFrequency = (float)(1000000/sampleFrequency);
    Measurement.Sample_frequency = sampleFrequency;
    Serial.print("End Sampling:");Serial.println();

    //Calculate peak voltage and store value
    float abs_voltage[SAMPLE_ARRAY_LENGTH];
    for(int i=0; i<SAMPLE_ARRAY_LENGTH-1; i++){
      abs_voltage[i] = sqrt( pow(Voltage[i+1],2));
    }
    Measurement.Voltage = Voltage[0];
    for(int i=0; i<SAMPLE_ARRAY_LENGTH-1; i++){
      if(Measurement.Voltage < abs_voltage[i+1]){
        Measurement.Voltage = abs_voltage[i+1];
      }
    }
    float hold_Volt = Measurement.Voltage;
  
    //used to calculate power factor
    float y = Voltage[0];
    float x = Current[0];    
      
    //append zeros
    for(int i = SAMPLE_ARRAY_LENGTH; i<ARRAY_LENGTH;i++){
      Voltage[i] = 0;
      Current[i] = 0;
    }

    //convert samples to complex numbers
    for(int i=0;i<ARRAY_LENGTH;i++){
      cVoltage[i] = (cplx)Voltage[i];
      cCurrent[i] = (cplx)Current[i];
    }

    // Fast Fourier Transform
    Serial.print("Start FFT:");Serial.println();
    fft(cVoltage, ARRAY_LENGTH);
    for(int i=0;i<ARRAY_LENGTH;i++){
      rVoltage[i] = creal(cVoltage[i]);
      iVoltage[i] = cimag(cVoltage[i]);
      Voltage[i] = sqrt( pow(rVoltage[i], 2)+pow(iVoltage[i], 2) );
    }
    
    fft(cCurrent, ARRAY_LENGTH);
    for(int i=0;i<ARRAY_LENGTH;i++){
      rCurrent[i] = creal(cCurrent[i]);
      iCurrent[i] = cimag(cCurrent[i]);
      Current[i] = sqrt( pow(rCurrent[i], 2)+pow(iCurrent[i], 2) );
    }
    Serial.print("End FFT:");Serial.println();

    //Calculate peak current and store value
    Measurement.Current = Current[0];
    for(int i=0; i<ARRAY_LENGTH-1; i++){
      if(Measurement.Current < Current[i+1]){
        Measurement.Current = Current[i+1];
      }
    }
    //Calculate signal frequency
    //float Voltage_frequency = Voltage[0];
    Measurement.Voltage = Voltage[0];
    int k = 0;
    for(int i=0; i<ARRAY_LENGTH-1; i++){
      if(Measurement.Voltage < Voltage[i+1]){
        Measurement.Voltage = Voltage[i+1];
        k = i+1;        
      }
    }
    //Scale FFT output to actual amplitude
    Measurement.Current = scaler*Measurement.Current; //256 Samples
    //Measurement.Voltage = 1.52*scaler*Measurement.Voltage; //256 Samples
    //add FFT scale offset
    Measurement.Current = Measurement.Current + 0.0142*Measurement.Current;
    //Measurement.Voltage = Measurement.Voltage + 0.0142*Measurement.Voltage;
    Measurement.Voltage = hold_Volt;

    //Calculate frequency using FFT results
    Measurement.frequency = ((float)k/(float)ARRAY_LENGTH)*sampleFrequency;
    //Measurement.frequency = (float)k;

    //Calculate power factor, need to change: uses different amplitudes and has phase differernce
    float t1 = voltageMicros[0]/1000000;
    float t0 = currentMicros[0]/1000000;
    y = (y - x + Measurement.Current)/Measurement.Voltage;
    Measurement.PF = (acos(y) - 2*PI*Measurement.frequency*(t1-t0)); // Calculate frequency using arccos, might need to add 90 degree phase
    
    //Board 1:
    //CT1: Linear calibration equation
    //Measurement.Current = 16.946*(Measurement.Current);
    //VT1: Linear calibration equation
    //Measurement.Voltage = 350*Measurement.Voltage-28;

    //Board 2:
    //CT2: Linear calibration equation
    //Measurement.Current = 17.304*(Measurement.Current-0.036);
    Measurement.Current = 14.301*Measurement.Current-0.3227;
    //if(Measurement.Current < 0){Measurement.Current = 0;}
    //VT2: Linear calibration equation
    Measurement.Voltage = 350*Measurement.Voltage - 33;
    //Measurement.Voltage = 148.94*Measurement.Voltage + 93.486;

    //Calcute RMS power using RMS values
    Measurement.Power = Measurement.Voltage*Measurement.Current;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Measurement, sizeof(Measurement));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    State.request = 0;
  }
}