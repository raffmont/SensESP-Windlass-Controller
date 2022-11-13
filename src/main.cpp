// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include "sensesp/sensors/digital_output.h"
#include "sensesp/signalk/signalk_listener.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp_app.h"
#include "sensesp_app_builder.h"

#include <math.h>

using namespace sensesp;

#define DEBOUNCE_DELAY 250
#define WINDLASS_GO_DOWN +1
#define WINDLASS_GO_UP -1


reactesp::ReactESP app;

// The chain counter pin
const uint8_t chainCounterPin = 14;
const uint8_t chainSpeedPin = 27;

// Actuate the relay for going up
const uint8_t goUpPin = 2;

// Actuate the relay for going down
const uint8_t goDownPin = 4;

// Sense if the windlass is going down
const uint8_t goingUpPin = 16;

// Sense if the windalss is going up
const uint8_t goingDownPin = 17;     


// Counter for chain events
int chainCounter = 0;     

// 1 =  Chain down / count up, -1 = Chain up / count backwards
int upDown = WINDLASS_GO_DOWN;       

// Stores last ChainCounter value to allow storage to nonvolatile storage in case of value changes
int lastSavedCounter = 0;

#define ENABLE_DEMO 1                // Set to 1 to enable Demo Mode with up/down counter
#define SAFETY_STOP 0                // Defines safety stop for chain up. Stops defined number of events before reaching zero
#define MAX_CHAIN_LENGTH 40          // Define maximum chan length. Relay off after the value is reached

// Translates counter impuls to meter 0,33 m per pulse
float chainCalibrationValue = 0.33f; 
float chainCalibrationOffset = 0.00f;

unsigned long lastChainSpeedMills = millis();

// The setup function performs one-time application initialization.
void setup() {

  // Delays
  int goingUpDownSensorReadDelay = 10;
  int goingUpDownSensorDebounceDelay = 15;
  int chainCounterSensorDebounceDelay = 10;

  // Configuraion paths
  String goingUpDownSensorReadDelayConfigPath = "/sensor_going_up_down/read_delay";
  String windlassStatusSKPathConfigPath = "/windlass_status/sk";
  String goingUpDownSensorDebounceDelayConfigPath = "/sensor_going_up_down/debounce_delay";
  String chainCounterSKPathConfigPath = "/rodeDeployed/sk";
  String chainCounterSensorDebounceDelayConfigPath = "/chain_counter_sensor/debounce_delay";
  String chainSpeedSKPathConfigPath = "/chainSpeed/sk";
  String chainCalibrationSKPathConfigPath = "/chain_counter_sensor/calibration_value";

  // Signal K paths
  String windlassStatusSKPath = "navigation.anchor.windlass.status";
  String chainCounterSKPath = "navigation.anchor.rodeDeployed";
  String chainSpeedSKPath = "navigation.anchor.windlass.speed";

  // Chain counter metadata
  SKMetadata* chainCounterMetadata = new SKMetadata();
  chainCounterMetadata->units_ = "m";
  chainCounterMetadata->description_ = "Anchor Chain Deployed";
  chainCounterMetadata->display_name_ = "Chain Deployed";
  chainCounterMetadata->short_name_ = "Chain Out";

  // Chain counter speed metadata
  SKMetadata* chainSpeedMetadata = new SKMetadata();
  chainSpeedMetadata->units_ = "m/s";
  chainSpeedMetadata->description_ = "Windlass chain speed";
  chainSpeedMetadata->display_name_ = "Windlass speed";
  chainSpeedMetadata->short_name_ = "Chain speed";
  


#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("signalk-windlass-controller")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();


  // Set GPIO pin 15 to output and toggle it every 650 ms

  const uint8_t kDigitalOutputPin = 15;
  const unsigned int kDigitalOutputInterval = 650;
  pinMode(kDigitalOutputPin, OUTPUT);
  app.onRepeat(kDigitalOutputInterval, [kDigitalOutputPin]() {
    digitalWrite(kDigitalOutputPin, !digitalRead(kDigitalOutputPin));
  });

  
  
  

  // Connect the digital input to a lambda consumer that prints out the
  // value every time it changes.

  // Test this yourself by connecting pin 15 to pin 14 with a jumper wire and
  // see if the value changes!

  

  
  pinMode(goUpPin, OUTPUT);
  pinMode(goDownPin, OUTPUT);

  digitalWrite(goUpPin, LOW );
  digitalWrite(goDownPin, LOW );
  
  /**
   * DigitalInputChange monitors a physical button connected to BUTTON_PIN.
   * Because its interrupt type is CHANGE, it will emit a value when the button
   * is pressed, and again when it's released, but that's OK - our
   * LambdaConsumer function will act only on the press, and ignore the release.
   * DigitalInputChange looks for a change every read_delay ms, which can be
   * configured at read_delay_config_path in the Config UI.
   */

  
  auto* goingUpSensor = new DigitalInputChange(
      goUpPin, INPUT_PULLDOWN, goingUpDownSensorReadDelay, goingUpDownSensorReadDelayConfigPath);

  auto* goingDownSensor = new DigitalInputChange(
      goingDownPin, INPUT_PULLDOWN, goingUpDownSensorReadDelay, goingUpDownSensorReadDelayConfigPath);


  

  auto* windlassStatusSKOutput = new SKOutputString(windlassStatusSKPath, windlassStatusSKPathConfigPath);
  windlassStatusSKOutput->emit("off");

  /**
   * Create a DebounceInt to make sure we get a nice, clean signal from the
   * button. Set the debounce delay period to 15 ms, which can be configured at
   * debounce_config_path in the Config UI.
   */
  
  auto* goingUpDownSensorDebounce = new DebounceInt(goingUpDownSensorDebounceDelay, goingUpDownSensorDebounceDelayConfigPath); 

  // Manage the going up sensor
  goingUpSensor
    // Debounce the signal
    ->connect_to(goingUpDownSensorDebounce)

    // Update the windlass status according to the going up sensor
    ->connect_to(new LambdaConsumer<int>([windlassStatusSKOutput](int input) {

      // Check if the windlass is going up (press)
      if (input == HIGH) {

        // The windlass is going up, update the upDown variable
        upDown = WINDLASS_GO_UP;

        // Update the windlass status
        windlassStatusSKOutput->emit("up");
      } else {

        // The windlass is not going up anymore (releae), update the windlass status
        windlassStatusSKOutput->emit("off");
      }
    }));

  // Manage the going down sensor
  goingDownSensor
    // Debounce the signal
    ->connect_to(goingUpDownSensorDebounce)

    // Update the windlass status according to the going down sensor
    ->connect_to(new LambdaConsumer<int>([windlassStatusSKOutput](int input) {

      // Check if the windlass is going down (press)
      if (input == HIGH) {

        // The windlass is going down, update the upDown variable
        upDown = WINDLASS_GO_DOWN;

        // Update the windlass status
        windlassStatusSKOutput->emit("down");
      } else {
        // The windlass is not going down anymore (releae), update the windlass status
        windlassStatusSKOutput->emit("off");
      }
    }));

  // Define a digital input sensor with debouncer as a counter
  auto* chainCounterSensor = new DigitalInputChange(chainCounterPin, INPUT_PULLUP,CHANGE);

  /**
   * Create a DebounceInt to make sure we get a nice, clean signal from the
   * button. Set the debounce delay period to 15 ms, which can be configured at
   * debounce_config_path in the Config UI.
   */
  auto* chainCounterSensorDebounce = new DebounceInt(chainCounterSensorDebounceDelay, chainCounterSensorDebounceDelayConfigPath); 
  
  // Manage the chain counter sensor
  chainCounterSensor

    // Debounce the signal
    ->connect_to(chainCounterSensorDebounce)

    // Transform the signal in deployed rod in meters
    ->connect_to(

      // Create a lambda transform function
      new LambdaTransform<int,int>(

        // Catch the counter and the status output instances, input is HIGH when the pin status changes
        [windlassStatusSKOutput](int input) {
        
          // Check if it is the rising front of the pin status change
          if (input == HIGH) {

            // Increase or decrease the couter
            chainCounter = chainCounter + upDown;
            
            // Safety stop counter reached while chain is going up
            if (
              // If the windlass is going up...
              digitalRead(goingUpPin) == HIGH
              
              // ...and the chain counter reached the safety limit, stop the windlass
              && (chainCounter <= SAFETY_STOP) 
            ) {  
              // Shutdown the relay on up
              digitalWrite(goUpPin, LOW );

              // Update windlass status
              windlassStatusSKOutput->emit("off");

            }
            // Maximum chain lenght reached   
            else if (
              // If the windlass is going up...
              digitalRead(goingDownPin) == HIGH

              // ...and the rod is deployed, stop the windlass
              && (chainCounter >= MAX_CHAIN_LENGTH) 
            ) {  
              // Shutdown the relay on up
              digitalWrite(goUpPin, LOW );

              // Update windlass status
              windlassStatusSKOutput->emit("off");
            }
            // Check if the chain if free falling
            else if (
              // If the windlass is going down...
              upDown == WINDLASS_GO_DOWN

              // ...but the going down sensor is not sensing it, the windlass is in free down status
              && digitalRead(goingDownPin)==LOW) {

              // Update windlass status
              windlassStatusSKOutput->emit("freeDown");  
            }
            // Check if the chain is free rising (for example manually)
            else if (
              // If the windlass is going up...
              upDown == WINDLASS_GO_UP
              
              // ...but the going up sensor is not sensing it, the windlass is in free up status
              && digitalRead(goingUpPin)==LOW) {
              
              // Update windlass status
              windlassStatusSKOutput->emit("freeUp");  
            }
          }

          // Returm thr chain counter value
          return chainCounter;
        }
        )
      )
    ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
    ->connect_to(new SKOutputFloat(chainCounterSKPath, chainCounterSKPathConfigPath, chainCounterMetadata));
    
  
  // Define a counter sensor with debouncer as for measuring the chain speed
  auto* chainSpeedSensor = new DigitalInputDebounceCounter(chainSpeedPin, INPUT_PULLUP, RISING, 250, chainCounterSensorDebounceDelay, chainCounterSensorDebounceDelayConfigPath);

  // Manage the chain speed sensor
  chainSpeedSensor
      ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
      ->connect_to(new MovingAverage(4, 4))                                          
      ->connect_to(new SKOutputFloat(chainSpeedSKPath, chainSpeedSKPathConfigPath, chainSpeedMetadata));  

  

  // Create a linstener for the SignalK windlass status path
  auto* windlassStatusListener = new StringSKListener(windlassStatusSKPath);

  // Connect the listener
  windlassStatusListener->connect_to(

      // Create a lambda consumer function
      new LambdaConsumer<String>(
        [](String input) {

          // Check if the status is up and the windlass is not going up
          if (input == "up" && digitalRead(goingUpPin) == LOW) {

            // Activate the relay up
            digitalWrite(goUpPin,HIGH); 
          }
          // Check if the status is down and the windlass is not going down
          else if (input == "down" && digitalRead(goingDownPin) == LOW) {

            // Activate the relay down
            digitalWrite(goDownPin,HIGH); 
          }
          // Check if the status is of and the ralay is up pr down
          else if (input == "off" && (digitalRead(goingUpPin) == HIGH || digitalRead(goingDownPin) == HIGH)) {

            // Switch off the relay on up
            digitalWrite(goUpPin,LOW); 

            // Switch off the relay on down
            digitalWrite(goDownPin,LOW); 
          }
        }));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
