#include <Arduino.h>

// include drivers vor i2s mic
#include <driver/i2s.h>
// libraries for spectrum analysis
#include <arduinoFFT.h>

#include <NeoPixelBrightnessBus.h>

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

#include <ESPmDNS.h>

#include <WebServer.h>

#include <ArduinoJson.h>

#include <EEPROM.h>

#include "html_page.h"

// define pins for i2s mic
#define I2S_SCK 26
#define I2S_WS 25
#define I2S_SD 22

const i2s_port_t I2S_PORT = I2S_NUM_0;

// definitions for FFT
#define SAMPLING_FREQUENCY 5000
#define AMPLITUDE 200 
#define BLOCK_SIZE 512
#define NUM_BANDS 8
#define REFRESH_CYCLE 30000
// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

// define pin for led connection
#define LED_PIN         27
#define NUM_LEDS        232
// init led array
NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> leds(NUM_LEDS, LED_PIN);

WebServer server;

arduinoFFT FFT = arduinoFFT();

// global variables needed for animation
uint8_t briVals[NUM_LEDS];
uint8_t colVals[NUM_LEDS];
uint8_t randomNumber = 0;
uint8_t colorBuffer = 0;
int brightnessBuffer = 0;

int direction = 1;
int counter = 0;

// global variables for controlling the lamp
uint8_t globalArgs[] = {0, 30, 240, 127, 100, 24, 40, 0, 64};
const String ARG_NAMES[] = {"state", "color", "saturation", "brightness", "base", "steps", "ticklength", "frame", "spectrum"};

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
bool stateChanged = true;
bool on = true;
bool listening = true;


//global variables needed for FFT
double vReal[BLOCK_SIZE];
double vImag[BLOCK_SIZE];
int16_t samples[BLOCK_SIZE];

int momVal[BLOCK_SIZE];
int maxVal = 1;

int bands[NUM_BANDS] = {0};
const int amplitude = 100000000;

// factor for transforming 0-256 int to 0-1 float
float f = 0.00390625;

// setting up the i2s connection to the microphone
void setupMic() {
  Serial.println("Configuring I2S...");
  esp_err_t err;

  // The I2S config as per the example
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
    .sample_rate = SAMPLING_FREQUENCY,                        
    .bits_per_sample = i2s_bits_per_sample_t(32), // could only get it to work with 32bits
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // although the SEL config should be left, it seems to transmit on right
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,     // Interrupt level 1
    .dma_buf_count = 8,                           // number of buffers
    .dma_buf_len = BLOCK_SIZE,                     // samples per buffer
    .use_apll = false
  };

  // The pin config as per the setup
    i2s_pin_config_t pin_config = {
      bck_io_num:   I2S_SCK,  
      ws_io_num:    I2S_WS,    
      data_out_num: -1,     // not used
      data_in_num:  I2S_SD   
    };

  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }
  Serial.println("I2S driver installed.");  
  
  i2s_start(I2S_PORT);
}




// get specific global variable
int getArg(String name)
{
  if(server.arg(name)!= "")
  {
    return server.arg(name).toInt();
  } else 
  {
    return -1;
  }
}

// send global variables to client
void sendArgs()
{
  EEPROM.write(0, (byte)on);
  EEPROM.commit();
  
  DynamicJsonDocument doc(JSON_OBJECT_SIZE(sizeof(globalArgs) + 10));

  for(int i = 0; i < sizeof(globalArgs); i++)
  {
    doc[ARG_NAMES[i]] = globalArgs[i];
  }
  doc["power"] = on;
  doc["ip"] = WiFi.localIP().toString();

  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);
  server.send(200, "application/json", jsonString);
}

// receive and set global variables
void setArgs()
{
  int val = 0;
  for(int i = 0; i < sizeof(globalArgs); i++)
  {
    val = getArg(ARG_NAMES[i]);

    if(val >= 0)
    {

      globalArgs[i] = val;
      Serial.print(ARG_NAMES[i]);
      Serial.print(" set to ");
      Serial.println(val);

      if(ARG_NAMES[i] == "state")
      {
        stateChanged = true;
      } else if(ARG_NAMES[i] == "steps" && val == 0)
      {
        globalArgs[i] = 1;
      }
    }
  }
  sendArgs();
}

// initialize new state on state change
void initState()
{

  Serial.println("init state " + (String)globalArgs[0]);

  leds.ClearTo(RgbColor(0, 0, 0));
  listening = false;

  switch (globalArgs[0])
  {
    case 0:
      for(int i = 0; i < NUM_LEDS; i++)
        {
          leds.SetPixelColor(i, HsbColor(globalArgs[1], globalArgs[2], 255));
        }
    break;

    case 1:
      
      break;

    case 2:
      randomSeed(analogRead(A0) + currentMillis);
      for(int i = 0; i < NUM_LEDS; i++)
        {
          leds.SetPixelColor(i, HsbColor(globalArgs[1], globalArgs[2], globalArgs[4]));
          briVals[i] = globalArgs[4];
          colVals[i] = globalArgs[1];
        }
      break;

    case 3:
      randomSeed(analogRead(A0) + currentMillis);
      globalArgs[7] = 0;
      for(int i = 0; i < NUM_LEDS; i++)
        {
          leds.SetPixelColor(i, HsbColor(globalArgs[1], globalArgs[2], globalArgs[4]));
        }
        brightnessBuffer = globalArgs[4];
      break;

    case 4:
      globalArgs[7] = 0;
      for(int i = 0; i < NUM_LEDS; i++)
      {
        leds.SetPixelColor(i, HsbColor(globalArgs[1], globalArgs[2], globalArgs[4]));
      }
      brightnessBuffer = globalArgs[3];
      break;
    
    case 5:
      listening = true;
      for (int i = 0; i < BLOCK_SIZE; i++)
      {
        maxVal = 1;
        momVal[i] = 0;
      }
      briVals[0] = 0;
      globalArgs[6] = 5;
      break;

    case 6:
      randomSeed(analogRead(A0) + currentMillis);
      for(int i = 0; i < NUM_LEDS; i++)
      {
        colVals[i] = globalArgs[1];
        briVals[i] = globalArgs[4];
      }
      listening = true;
      for (int i = 0; i < BLOCK_SIZE; i++)
      {
        maxVal = 1;
        momVal[i] = 0;
      }
      globalArgs[6] = 5;
      break;

    default:
      break;
  }
}

// util to set new color for one pixel
void setColorHSV(int i, int h, int s, int v) {

    leds.SetPixelColor(i, HsbColor(f * (float)h, f * (float)s, f * (float)v));
}

// ------------------------
// 
// main setup and loop starts here
//
// ------------------------

void setup() {
  // put your setup code here, to run once:

  leds.Begin();
  leds.SetBrightness(globalArgs[3]);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  // it is a good practice to make sure your code sets wifi mode how you want it.

  Serial.begin(115200);
  
  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
  // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
  // then goes into a blocking loop awaiting configuration and will return success result

  bool res;
  // res = wm.autoConnect(); // auto generated AP name from chipid
  // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
  res = wm.autoConnect("infinilamp","einszweidreivier"); // password protected ap

  if(!res) {
      Serial.println("Failed to connect");
      // ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("connected...yeey :)");
  }

  Serial.println("Setting up mic");
  setupMic();
  Serial.println("Mic setup completed");

  // init http server endpoints
  server.on("/", [](){server.send_P(200, "text/html", html_string);});
  server.on("/toggle", [](){on = !(on);sendArgs();});
  server.on("/on", [](){on = true;sendArgs();});
  server.on("/off", [](){on = false;sendArgs();});
  server.on("/default", [](){});
  server.on("/setargs", setArgs);
  server.on("/getargs", sendArgs);
  server.on("/reinit", initState);
  server.on("/reboot", [](){ESP.restart();});
  server.begin();

  if (MDNS.begin("infinilamp")) { // Start the mDNS responder
    Serial.println("MDNS responder started");
  }

  MDNS.addService("http", "tcp", 80);  
}

// ------------------------

void loop() {
  // put your main code here, to run repeatedly:

  currentMillis = millis();
  
  server.handleClient();

  if(stateChanged)
  {
    stateChanged = false;
    initState();
  }

  if(on && listening)
  {

    /*SAMPLING I2S MIC*/
    size_t bytesIn = 0;
    esp_err_t result = i2s_read(I2S_PORT, 
                                (char *)samples, 
                                BLOCK_SIZE, 
                                &bytesIn, 
                                portMAX_DELAY);

    if (result == ESP_OK)
    {

        // Serial.println(samples[0]);
        
      for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
        vReal[i] = samples[i] << 8;
        vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
      }
  
      /*FFT*/
      FFT.Windowing(vReal, BLOCK_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, BLOCK_SIZE, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, BLOCK_SIZE);
      // double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

      for (int i = 0; i < 8; i++) {
      bands[i] = 0;
      }
      
      for (int i = 0; i < (BLOCK_SIZE/2); i++){ // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
        if (vReal[i] > 2000) { // Add a crude noise filter, 10 x amplitude or more
          if (i<=2 )             bands[0] = max(bands[0], min(255, (int)(vReal[i]/amplitude))); // 125Hz
          if (i >3   && i<=5 )   bands[1] = max(bands[1], min(255, (int)(vReal[i]/amplitude))); // 250Hz
          if (i >5   && i<=7 )   bands[2] = max(bands[2], min(255, (int)(vReal[i]/amplitude))); // 500Hz
          if (i >7   && i<=15 )  bands[3] = max(bands[3], min(255, (int)(vReal[i]/amplitude))); // 1000Hz
          if (i >15  && i<=30 )  bands[4] = max(bands[4], min(255, (int)(vReal[i]/amplitude))); // 2000Hz
          if (i >30  && i<=53 )  bands[5] = max(bands[5], min(255, (int)(vReal[i]/amplitude))); // 4000Hz
          if (i >53  && i<=200 ) bands[6] = max(bands[6], min(255, (int)(vReal[i]/amplitude))); // 8000Hz
          if (i >200           ) bands[7] = max(bands[7], min(255, (int)(vReal[i]/amplitude))); // 16000Hz
        }
      }
      
        Serial.println(bands[0]); 

      // this dynamically calibrates the values from each band
      for (int i = 0; i < NUM_BANDS; i++)
      {
        // upper bound that decays over time
        maxVal = max(max(maxVal - 1, 1), (int)bands[i]);
        // momentary value maped according to upper bound
        momVal[i] = max((long)momVal[i], map((long)bands[i], 0, (long)maxVal, 0, 255));
      }
    }
    else
    {
      Serial.println("Something went wrong while recording");
      on = false;
    }
    
  }
    
  // do a animation tick
  if (currentMillis - previousMillis >= globalArgs[6])
  {
    
    
    previousMillis = currentMillis;    

    if (on)
    {
      leds.SetBrightness(globalArgs[3]);

      // increment timeout counter
      counter++;
      if(counter > REFRESH_CYCLE)
      {
        counter = 0;
        initState();
      }
      // jump to animation section according to active state
      switch (globalArgs[0])
      {
        case 0:
          for(int i = 0; i < NUM_LEDS; i++)
          {
            setColorHSV(i, globalArgs[1], globalArgs[2], 255);
          }
          break;

        case 1:
          for(int i = 0; i < NUM_LEDS; i++) 
          {
            setColorHSV(i, (globalArgs[1] + (int)(i * 1.33 * 8)) % 256, 
                            globalArgs[2], 255);            
          }
          globalArgs[1] += 1;
          break;

        case 2:
          randomNumber = random(0, NUM_LEDS);
          briVals[randomNumber] = 255;
          colVals[randomNumber] = (globalArgs[1] + random(15) - 7) % 256;
          
          for(int i = 0; i < NUM_LEDS; i++)
          {
            setColorHSV(i, colVals[i], globalArgs[2], briVals[i]); 

            if(briVals[i] > globalArgs[4])
            {
              briVals[i] -= (int)((255 - globalArgs[4])/globalArgs[5]);
            } else
            {
              briVals[i] = globalArgs[4];
            }
          }
          break;
        
        case 3:

          if (globalArgs[7] == 0){
            randomNumber = random(0, 23);
          }

          if (globalArgs[7] == (24 + randomNumber) || globalArgs[7] == (120 + randomNumber))
          {
            brightnessBuffer = 255;
          } else if(brightnessBuffer > globalArgs[4])
          {
            brightnessBuffer -= max((255 - globalArgs[4])/globalArgs[5], 1);
          } else
          {
            brightnessBuffer = globalArgs[4];
          }

          for(int i = 0; i < NUM_LEDS; i++)
          {
            setColorHSV(i ,globalArgs[1], globalArgs[2], brightnessBuffer);
          }
          globalArgs[7] ++;
          break;

        case 4:
          brightnessBuffer = brightnessBuffer - 
            (direction * max((255 - globalArgs[4])/globalArgs[5], 1));
          if (brightnessBuffer >= 255)
          {
            brightnessBuffer = 255;
            direction *= -1;
          } else if (brightnessBuffer <= globalArgs[4])
          {
            brightnessBuffer = globalArgs[4];
            direction *= -1;
          }
          
          for(int i = 0; i < NUM_LEDS; i++)
          {
            setColorHSV(i, globalArgs[1], globalArgs[2], brightnessBuffer);
          }        
          break;

        case 5:

            // Serial.println(momVal[0]);

          if (momVal[0] < globalArgs[4])
          {
            momVal[0] = 0;
          } else 
          {
            briVals[0] = momVal[0];
            counter = 0;
          }
          
          for(int i = 0; i < NUM_LEDS; i++)
          {
            setColorHSV(i, globalArgs[1], globalArgs[2], briVals[0]);
          }
          
          briVals[0] = max(0, (int)briVals[0] - ((255 - globalArgs[4])/globalArgs[5]));

          momVal[0] = 0;
          break;

        case 6:
          for (int i = 0; i < NUM_BANDS; i++)
          {
            if (momVal[i] < globalArgs[4])
            {
              momVal[i] = 0;
              counter = 0;
            } else
            { 
              randomNumber = random(0, NUM_LEDS);
              colVals[randomNumber] = (globalArgs[1] + (i * (globalArgs[8] / NUM_BANDS))) % 256;
              briVals[randomNumber] = momVal[i];
              momVal[i] = 0;
            }
          }
          for(int i = 0; i < NUM_LEDS; i++)
          {
            setColorHSV(i, (int)colVals[i], (int)globalArgs[2], (int)briVals[i]);
            briVals[i] = max((int)briVals[i] - ((256 - globalArgs[4])/globalArgs[5]), (int)globalArgs[4]);
          }
          break;

        default:
          break;
      }
    } else 
    {
      leds.SetBrightness(0);
    }
    leds.Show();
  }
}



