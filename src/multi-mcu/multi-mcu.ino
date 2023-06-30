#include <ArduinoJson.h>

#define arduino
//#define esp32

#ifdef esp32
#include <driver/adc.h>
int adcPins[12] = {32, 33, 34, 35, 36, 39, 12, 13, 14, 25, 26, 27};
#endif

#ifdef arduino
int adcPins[6] = {A0, A1, A2, A3, A4, A5};
#endif

const int DEFAULT_RATE = 1;
const int COMMAND_PARAM_MAX = 4;
const char COMMAND_PARAM_SEP = ',';
const char COMMAND_END = '\n';
uint16_t * data;

unsigned long interval = 0;
unsigned long utime = 0;
String command = "";
String params[COMMAND_PARAM_MAX];
int current_param = -1;
unsigned int seq = 0;
bool led = false;
bool streaming = false;
int channelNo = 1;

void setup() {
  Serial.begin(115200);
  cmd_rate(DEFAULT_RATE);
  Serial.write("ready\n");
}

void loop() {

  // Receive commands from the serial input
  receive();

  // Push data from buffer to the serial output

  // Run at the defined rate
  if (tick() && streaming) {
    Serial.println("streaming");
    data[0] = seq;
    data[1] = utime;
#ifdef arduino
    for (int i = 0; i < channelNo; i++) {
      data[i + 2] = analogRead(adcPins[i]) & 0x0FFF;
    }
#endif

#ifdef esp32
    for (int i = 0; i < channelNo; i++)
    {
      if (i < 6) {
        data[i + 2] = adc1_get_raw((adc1_channel_t)adcPins[i]) & 0x0FFF;
      }
      else {
        int temp;
        adc2_get_raw( (adc2_channel_t)adcPins[i], ADC_WIDTH_12Bit, &temp);
        data[1 + 2] = temp;
      }
    }
#endif
  
    send();
  }
}

void cmd_rate(int rate) {
  interval = 1e6 / rate;
  Serial.print("interval: ");
  Serial.println(interval);
}

void cmd_channels(int ch) {
  channelNo = ch;

#ifdef esp32
  for (int i = 0; i < channelNo; i++)
  {
    if (i < 6) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten((adc1_channel_t)(adcPins[i]), ADC_ATTEN_DB_11);
    }
    else
    {
      adc2_config_channel_atten((adc2_channel_t)(adcPins[i]), ADC_ATTEN_DB_11);
    }
  }
#endif
  Serial.print("ch: ");
  Serial.println(channelNo);
  data = (uint16_t *)calloc(channelNo + 2, sizeof(uint16_t));
}
void cmd_start() {
  streaming = true;
}

void cmd_stop() {
  streaming = false;
}

void cmd_ping() {
  Serial.write(micros());
}

void exec() {
  if (command == "rate") {
    cmd_rate(params[0].toInt());
  } else if (command = "channles") {
    cmd_channels (params[0].toInt());
  } else if (command == "start") {
    cmd_start();
  } else if (command == "stop") {
    cmd_stop();
  } else if (command == "ping") {
    cmd_ping();
  }
}

bool tick() {
  // TODO: overflow after 70 minutes
  unsigned long now = micros();
  if (now - utime >= interval) {
    utime = now;
    seq ++;

    return true;

  }
  return false;
}

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
  int available = Serial.availableForWrite();
  if (available) {
    Serial.println((int)data);
    Serial.write((uint8_t *)data, (channelNo + 2)*sizeof(uint16_t));
    Serial.write("gibberish");
    Serial.write('\n');
  }
}
