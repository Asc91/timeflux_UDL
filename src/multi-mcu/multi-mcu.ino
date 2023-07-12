
/* Uncomment one of the following line depending on the MCU being used*/
#define arduino
//#define esp32

bool debug = false;// Boolean used to turn debug mode on

// Usable ADC Pins for ESP32 and Arduino
#ifdef esp32
#include <driver/adc.h>
int adc_1_Pins[6] = {4, 5, 6, 7, 0, 3}; //GPIO 32, 33, 34, 35, 36, 39
int adc_2_Pins[6] = {5, 4, 6, 8, 9, 7}; //GPIO 12, 13, 14, 25, 26, 27
#endif

#ifdef arduino
int adcPins[6] = {A0, A1, A2, A3, A4, A5};
#endif

unsigned long present = 0; // Keeps track of current time, in micro sec
int rate = 1; // Default rate
unsigned long period = 0; // Time between two samples for given rate
int channelNo = 1;  // Default number of channles
uint16_t seq = 0;  // Sample No.
uint16_t * data;   // Pointer to array which will store data

// Some constants for recieving commands from python
const int COMMAND_PARAM_MAX = 4;
const char COMMAND_PARAM_SEP = ',';
const char COMMAND_END = '\n';
String command = "";
String params[COMMAND_PARAM_MAX];
int current_param = -1;

bool streaming = true; // Boolean to start streaming


void setup() {
  Serial.begin(115200);
  period = interval(rate);
  cmd_channels(channelNo);
}


void loop() {

  receive(); // Check for commands from user

  // if required period has elapsed and streaming is true, take ADC samples
  if (tick() && streaming) {
#ifdef arduino
    for (int i = 0; i < channelNo; i++) {
      data[i] = analogRead(adcPins[i]) & 0x0FFF;
    }
    data[channelNo] = seq;
#endif

    /* ESP32 has 2 ADC ports, ADC1 and ADC2.  ADC2 is used with WiFi. So it won't work when user is using WiFi of ESP32.
       Furthermore configuration methods and sampling methods are different for both of these ports. More info can be found
       at https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/adc.html#analog-to-digital-converter */
#ifdef esp32
    for (int i = 0; i < channelNo; i++)
    {
      if (i < 6) {
        data[i] = adc1_get_raw((adc1_channel_t)adc_1_Pins[i]) & 0x0FFF;
      }
      else {
        int temp;
        adc2_get_raw( (adc2_channel_t)adc_2_Pins[i - 6], ADC_WIDTH_12Bit, &temp);
        data[i] = temp & 0x0FFF;
      }
      data[channelNo] = seq;
    }
#endif

    send(); // Send data through serial
    seq = seq + 1; // Update sample number
  }
}

// Checks if sufficient time has elapsed according to given rate
bool tick() {
  unsigned long now = micros();
  if (now - present > period) {
    present  = now;
    return true;
  }
  else {
    return false;
  }
}

// Returns period between two ticks according to given rate
unsigned long interval(int rate) {
  unsigned long period = 1e6 / rate;
  if (debug) {
    Serial.print("Period : ");
    Serial.println(period);
  }
  return period;
}

// Initialises ADC channes for ESP32
void cmd_channels(int ch) {
  channelNo = ch;
  if (debug) {
    Serial.print("channelNo: ");
    Serial.println(channelNo);
  }

#ifdef esp32
  for (int i = 0; i < channelNo; i++)
  {
    if (i < 6) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten((adc1_channel_t)(adc_1_Pins[i]), ADC_ATTEN_DB_11);
    }
    else
    {
      adc2_config_channel_atten((adc2_channel_t)(adc_2_Pins[i - 6]), ADC_ATTEN_DB_11);
    }
  }
#endif

  // Allocate memroy to data pointer previously created according to channels number
  // Extra block is added for sample number
  data = new uint16_t[channelNo + 1];
}

// Execute helper functions according to command recieved
void exec() {
  if (command == "rate") {
    period = interval(params[0].toInt());
  } else if (command == "channels") {
    cmd_channels(params[0].toInt());
  }  else if (command == "start") {
    streaming = true;
  } else if (command == "stop") {
    streaming = false;
    delete[]data;
    data = NULL;
  }
}

// Check if user has sent commands
void receive() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case COMMAND_PARAM_SEP:
        current_param ++;
        break;
      case COMMAND_END:
        exec();
        command = "";
        memset(params, 0, sizeof(params));
        current_param = -1;
        break;
      default:
        if (current_param == -1) {
          command += c;
        } else {
          params[current_param] += c;
        }
    }
  }
}

void send() {

  // Check if there is space in output buffer of serial
  int available = Serial.availableForWrite();

  if (available > sizeof(data)) {
    if (!debug) {

      // Data is sent in 8bit packets over serial
      uint8_t buff[2 * (channelNo + 1)];

      // Split 16bit data in 8bit packets
      for (int i = 0; i < (channelNo + 1); i++)
      {
        int j = i * 2;
        buff[j] = (uint8_t)(data[i] & 0x00ff);
        buff[j + 1] = (uint8_t)(data[i] >> 8 & 0x00ff);
      }

      // Send 8bit data over serial
      Serial.write(buff, sizeof(buff));
    }
    if (debug) {
      String str;
      for (int i = 0; i < channelNo + 1; i++) {
        if (i > 0) {
          str += ",";
        }
        str += data[i];
      }
      Serial.println(str);
    }
  }
}
