#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "FreeRTOS.h"
#include "RTClib.h"

// Pin definitions
// Define 8 channel analog multiplexer control pins
#define MUX1 D8
#define MUX2 D7
#define MUX3 D6
// Define 2 negative voltage switch control pins
#define PHASEP D10
#define PHASEN D9
// Define 3 circuit control pins
#define CTEST D1
#define CSLEEP D2
#define CRESET D3
// Define one ADC pin
#define ADC_SIG A0
// Define battery
#define VBAT_DIVIDER      (0.332888F)   // 1M + 0.499M voltage divider on VBAT
#define VBAT_DIVIDER_COMP (3.004008F)        // Compensation factor for the VBAT divider

//************************ Signal ************************
// Base frequency for D8
const int baseFrequency = 1024 / 8;  // 8Hz Frequency in milliseconds
// Buffer for ADC readings
const int bufferSize = 16;
int adcBuffer[bufferSize] = {0};
bool bufferOverflow = false;
SemaphoreHandle_t bufferSemaphore;

//************************ Battery ************************
const int batterySampleNum = 8;
int batteryValues[batterySampleNum] = {0}; // buffer to store the last 10 battery readings
int percentage; // average battery buffer
int currentSampleIndex = 0; // index to keep track of the current sample
float mv_per_lsb = 3600.0F/1024.0F; // 10-bit ADC with 3.6V input range


typedef struct {
    float voltage;
    int percentage;
} BatteryState;

BatteryState battery_states[] = {
    {4.16, 100}, {4.15, 99}, {4.14, 98}, {4.13, 97}, {4.12, 96}, {4.11, 95}, {4.10, 94}, {4.09, 92}, 
    {4.08, 91}, {4.07, 90}, {4.06, 89}, {4.05, 88}, {4.04, 87}, {4.03, 86}, {4.02, 85}, {4.01, 84}, 
    {4.00, 83}, {3.99, 82}, {3.98, 81}, {3.97, 80}, {3.96, 79}, {3.95, 78}, {3.94, 77}, {3.93, 76}, 
    {3.92, 75}, {3.91, 74}, {3.9, 73}, {3.89, 72}, {3.88, 71}, {3.87, 70}, {3.86, 69}, {3.85, 68}, 
    {3.84, 67}, {3.83, 66}, {3.82, 65}, {3.81, 64}, {3.8, 63}, {3.79, 62}, {3.78, 61}, {3.77, 60}, 
    {3.76, 59}, {3.75, 58}, {3.74, 57}, {3.73, 56}, {3.72, 55}, {3.71, 54}, {3.7, 53}, {3.69, 52}, 
    {3.68, 51}, {3.67, 50}, {3.66, 49}, {3.65, 48}, {3.64, 47}, {3.63, 46}, {3.62, 45}, {3.61, 44}, 
    {3.6, 43}, {3.59, 42}, {3.58, 41}, {3.57, 40}, {3.56, 39}, {3.55, 38}, {3.54, 37}, {3.53, 36}, 
    {3.52, 35}, {3.51, 34}, {3.5, 33}, {3.49, 32}, {3.48, 31}, {3.47, 30}, {3.46, 29}, {3.45, 28}, 
    {3.44, 27}, {3.43, 26}, {3.42, 25}, {3.41, 24}, {3.4, 23}, {3.39, 22}, {3.38, 21}, {3.37, 20}, 
    {3.36, 19}, {3.35, 18}, {3.34, 17}, {3.33, 16}, {3.32, 15}, {3.31, 14}, {3.3, 13}, {3.29, 12}, 
    {3.28, 11}, {3.27, 10}, {3.26, 9}, {3.25, 8}, {3.24, 7}, {3.23, 6}, {3.22, 5}, {3.21, 4}, 
    {3.19, 3}, {3.17, 2}, {3.15, 1}, {0.00, 0}
}; 

int getBatteryPercentage(float voltage) {
  for (int i = 0; i < sizeof(battery_states)/sizeof(BatteryState) - 1; i++) {
    if (voltage >= battery_states[i].voltage) {
      return battery_states[i].percentage;
    }
  }
  return 0; // Return 0% if the voltage is below the lowest defined voltage
}

//************************ RTC ************************
RTC_Millis rtc;
// Initial date and time values
volatile int year = 2023;
volatile int month = 9;
volatile int day = 30;
volatile int hour = 15;
volatile int minute = 0;
volatile int second = 0;
// Update date and time every second
TickType_t Second_Time_Delay = 1024; 

//************************ BLE Service ************************
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery
char central_name_global[32] = { 0 };
String receivedString;         // Variable to store the received string
String lastProcessedString;    // Variable to store the last processed string

// This function converts the pin readings into a number
int getBufferIndex() {
  int index = 0;
  index += digitalRead(PHASEN) * 8; // D9 is the MSB
  index += digitalRead(MUX3) * 4; 
  index += digitalRead(MUX2) * 2;
  index += digitalRead(MUX1);     // D8 is the LSB
  return index;
}

// This function updates the software-based clock every second
void updateClock() 
{
    DateTime now = rtc.now();
    year = now.year();
    month = now.month();
    day = now.day();
    hour = now.hour();
    minute = now.minute();
    second = now.second();
    // Print the current date and time
/*     Serial.print(year);
    Serial.print('/');
    Serial.print(month);
    Serial.print('/');
    Serial.print(day);
    Serial.print(' ');
    Serial.print(hour);
    Serial.print(':');
    Serial.print(minute);
    Serial.print(':');
    Serial.print(second);
    Serial.println();     */
}

// Task functions for each pin
void SignalD8(void *pvParameters) {
  for (;;) {
    digitalWrite(MUX1, !digitalRead(MUX1));  // Toggle the pin
    vTaskDelay(pdMS_TO_TICKS(baseFrequency));    
  }
}

void SignalD7(void *pvParameters) {
  for (;;) {
    digitalWrite(MUX2, !digitalRead(MUX2));  // Toggle the pin
    vTaskDelay(pdMS_TO_TICKS(baseFrequency * 2));

  }
}

void SignalD6(void *pvParameters) {
  for (;;) {
    digitalWrite(MUX3, !digitalRead(MUX3));  // Toggle the pin
    digitalWrite(LED_RED, !digitalRead(LED_RED));  // Toggle the pin
    vTaskDelay(pdMS_TO_TICKS(baseFrequency * 4));
  }
}

void SignalD9(void *pvParameters) {
  for (;;) {
    digitalWrite(PHASEN, !digitalRead(PHASEN));  // Toggle the pin
    //digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));  // Toggle the pin
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));  // Toggle the pin
    digitalWrite(PHASEP, !digitalRead(PHASEP));  // Toggle the pin
    vTaskDelay(pdMS_TO_TICKS(baseFrequency * 8));
  }
}

// Task for sampling the battery
void TaskSampleBattery(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Sample the ADC value
    batteryValues[currentSampleIndex] = analogRead(PIN_VBAT);
    // Move to the next index, wrapping around if necessary
    currentSampleIndex = (currentSampleIndex + 1) % batterySampleNum;
    // Delay for next sample based on the defined sample rate
    vTaskDelay(pdMS_TO_TICKS(baseFrequency));
  }
}

// Task for displaying the battery information
void TaskDisplayBattery(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Calculate the average ADC value
    int sum = 0;
    for(int i = 0; i < batterySampleNum; i++) {
      sum += batteryValues[i];
    }
    int avgBatValues = sum / batterySampleNum;

    // Calculate the battery value based on the average ADC value
    float batteryvalue = avgBatValues * mv_per_lsb * VBAT_DIVIDER_COMP / 1000;
    percentage = getBatteryPercentage(batteryvalue);

    // Display the results
/*     Serial.print(avgBatValues);
    Serial.print(" [");
    Serial.print(batteryvalue, 2); // Print with 2 decimal places
    Serial.print(" V], Battery Percentage: ");
    Serial.print(percentage);
    Serial.print("%\n"); */
    
    // Delay until it's time to display again
    vTaskDelay(pdMS_TO_TICKS(baseFrequency * 8));
  }
} 

// This task will read ADC value and place it into the buffer based on the index
void TaskADCReader(void *pvParameters) {
  (void) pvParameters; // Just to avoid compiler warnings

  for (;;) {
    // Get the current index from the pins
    int index = getBufferIndex();
    // Read from ADC and store in buffer at the position indicated by the pins
    adcBuffer[index] = analogRead(ADC_SIG);
    // Set the overflow flag if we've just written to the last index
    bufferOverflow = (index == (bufferSize - 1));
    // Wait before reading the next value
    vTaskDelay(pdMS_TO_TICKS(baseFrequency / 2));
  }
}

// The RTC thread
void TaskDateTime(void *pvParameters) {
    (void) pvParameters; // Silence unused parameter warning

    while (true) 
    {
      updateClock();
      // Delay for 1 second to match our software clock update.
      vTaskDelay(Second_Time_Delay);
    }
}

// This task checks for buffer overflow and prints the buffer if overflow occurs forward data from HW Serial to BLEUART
void ble_uart_task(void *pvParameters)
{
    (void) pvParameters; // Just to avoid compiler warnings

  for (;;) {
    // Wait for the overflow flag to be set
    if (bufferOverflow) {
      // Take the semaphore to ensure no conflict on buffer access
      if (xSemaphoreTake(bufferSemaphore, (TickType_t)10) == pdTRUE) {

        unsigned long currentTime = millis();
        uint8_t buf[500] = {0};

        String timeString = String(currentTime);
        timeString += ",";
        timeString += String(percentage);
        timeString += "%,";
        timeString += String(year);
        timeString += "/";
        timeString += String(month);
        timeString += "/";
        timeString += String(day);


        for (int i = 0; i < bufferSize; i++) {
          timeString += ",";
          timeString += String(adcBuffer[i]);
          //Serial.println(adcBuffer[i]);
        }
        //Serial.println(timeString);

        int count1 = timeString.length();
        //Serial.println(count1);
        
        for(int i = 0; i < count1; i++){
          buf[i] = timeString[i];
          //Serial.print((char)buf[i]);
        }

        bleuart.write(buf, count1);
      
        // Reset the overflow flag
        bufferOverflow = false;
        
        // Give the semaphore back
        xSemaphoreGive(bufferSemaphore);
      }
    }
    
    // Slight delay to prevent this task from hogging the CPU
    vTaskDelay(pdMS_TO_TICKS(baseFrequency));
  }
}

void ble_receive_task(void *pvParameters)
{
  while(true) 
  {
    // Check if there's data available
    if (bleuart.available()) 
    {
      receivedString = bleuart.readString();
      // TODO: Handle or process the receivedString if needed
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to prevent busy-waiting
  }
} 

void processReceivedStringTask(void *pvParameters) {
    while(1) {
        if (receivedString != lastProcessedString) {
            if (isInCorrectFormat(receivedString)) {
                int year   = receivedString.substring(0, 4).toInt();
                int month  = receivedString.substring(5, 7).toInt();
                int day    = receivedString.substring(8, 10).toInt();
                int hour   = receivedString.substring(11, 13).toInt();
                int minute = receivedString.substring(14, 16).toInt();
                int second = receivedString.substring(17, 19).toInt();

                DateTime newTime(year, month, day, hour, minute, second);
                rtc.adjust(newTime);
            }
            
            lastProcessedString = receivedString;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
  // Initialize digital pins as outputs
  pinMode(MUX1, OUTPUT);
  pinMode(MUX2, OUTPUT);
  pinMode(MUX3, OUTPUT);
  pinMode(PHASEN, OUTPUT);
  pinMode(PHASEP, OUTPUT);
  pinMode(VBAT_ENABLE, OUTPUT);
  // Initialize analog pins as input
  pinMode(ADC_SIG, INPUT);

  // Initialize pin phases
  digitalWrite(MUX1, HIGH);
  digitalWrite(MUX2, HIGH);
  digitalWrite(MUX3, HIGH);
  digitalWrite(PHASEN, HIGH);
  digitalWrite(PHASEP, LOW);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(VBAT_ENABLE, LOW);
  
  //Serial.begin(115200);
  //while(!Serial);  

  // initialize BLE
  setupBLE();

  delay(1000);
  //Serial.println("******************************");
  //Serial.println("        Program start         ");
  //Serial.println("******************************");

  // initialize RTC
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  rtc.adjust(DateTime(year, month, day, hour, minute, second));

  // Initialize the semaphore
  bufferSemaphore = xSemaphoreCreateMutex();

  // Now set up two tasks to run independently.
  xTaskCreate(SignalD8, "Pin D8", 128, NULL, 9, NULL);
  xTaskCreate(SignalD7, "Pin D7", 128, NULL, 9, NULL);
  xTaskCreate(SignalD6, "Pin D6", 128, NULL, 9, NULL);
  xTaskCreate(SignalD9, "Pin D9 and D10", 128, NULL, 9, NULL);
  // Create the ADC reader task
  xTaskCreate(TaskADCReader, "ADCReader", 128, NULL, 8, NULL);
  // Create the BLE send task
  xTaskCreate(ble_uart_task, "BLE UART Task", 1000, NULL, 5, NULL);
  // Create RTC task
  xTaskCreate(TaskDateTime, "RTC Task", 256, NULL, 7, NULL); 
  // Create battery voltage tasks
  xTaskCreate(TaskSampleBattery, "SampleBattery", 100, NULL, 6, NULL);
  xTaskCreate(TaskDisplayBattery, "DisplayBattery", 256, NULL, 4, NULL);
  // Create BLE receive tasks
  xTaskCreate(ble_receive_task, "BLE RE Task", 1000, NULL, 3, NULL);
  xTaskCreate(processReceivedStringTask, "Process Received String Task", 256, NULL, 1, NULL);
}


void loop() {
  // Empty. Things are managed by tasks.
}


// Start Advertising Setting
void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

// Setup the BLE LED to be enabled on CONNECT
void setupBLE(void) 
{
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("TTR"); // useful testing with multiple central connections getMcuUniqueID()
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("University of Queensland");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  strncpy(central_name_global, central_name, 32);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

/*   Serial.println();
  Serial.print("Disconnected from ");
  Serial.print(central_name_global);
  Serial.print(", reason = 0x");
  Serial.println(reason, HEX); */
}

bool isInCorrectFormat(const String &str) {
    // Simple check for the format "YYYY/MM/DD HH:MM:SS"
    if (str.length() != 19) return false;
    if (str.charAt(4) != '/' || str.charAt(7) != '/' || 
        str.charAt(10) != ' ' || str.charAt(13) != ':' || str.charAt(16) != ':') return false;

    // Additional checks like valid month, day, hour, etc., can be added if needed.

    return true;
}