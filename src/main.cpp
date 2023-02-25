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
#include "sensesp/controllers/smart_switch_controller.h"
#include "sensesp/signalk/signalk_listener.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/signalk/signalk_put_request_listener.h"
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

#include <Preferences.h>

#include <math.h>

using namespace sensesp;


#define WINDLASS_GO_DOWN +1
#define WINDLASS_GO_UP -1


reactesp::ReactESP app;

// Define the Pins for chain counter & chain speed
const uint8_t chainCounterPin = 23;
const uint8_t chainSpeedPin = 22;

// Define the PIN for Relay goUP (output)
const uint8_t goUpPin = 19;

// Define the PIN for Relay goDown (output)
const uint8_t goDownPin = 4;

// Define the PIN for detecting windlass direction goingUp
const uint8_t goingUpPin = 16;

// Define the PIN for detecting windlass direction goingDown
const uint8_t goingDownPin = 17;     
 
 //Define the PIN for chain counter reset button
const uint8_t resetPin = 21;

// upDown is chain direction:  1 =  Chain down / count up, -1 = Chain up / count backwards
// initialise to zero 
int upDown = 0;       


//  initialise chainCounter 
  int chainCounter ;


// Define windlass (anchor winch)  gypsy details
// example used: Muir 4200 Thor horizontal windlass 10mm chain
// Chain wheel reference #   ;  1 rotation = 0.405m
// Translates one sensor  impulse to meters ( 0.405 m per pulse)

float chainCalibrationValue = 0.405f; 
float chainCalibrationOffset = 0.00f;

// initiate an instance of the Preferences library. Here its called preferences

  Preferences preferences;


// The setup function performs one-time application initialization.
void setup() {


 // Open Preferences with "my_app" namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. 
  // We will open storage in RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.

  preferences.begin("my_app", false);

  // Retrieves stored_ChainCounter value from nonvolatile storage
  //  if the key does not exist, it will return a default value of 0.
  // Note: Key name is limited to 15 chars.
  // "counter" is the name of the key used to store chainCounter in ESP32 flash (aka Non Vol)

 int  stored_chainCounter = preferences.getInt("counter", 0);

// retrieve the stored counter and update chainCounter 
 chainCounter=stored_chainCounter;

// test ...  Store a dummy  counter value =45 to the Preferences
// confirmed workin ok... 
/// needs some logic for when/how to do the put 
// after a wait period to let chain deployment settle
//  (to reduce frequency/number of writes to flash memory)
              
    preferences.putInt("counter", 45);

// Close the Preferences...
     preferences.end();

// 


// Delays
 int goingUpSensorDebounceDelay = 30;
 int goingDownSensorDebounceDelay = 30;
 int chainCounterSensorDebounceDelay = 10;
 int resetbtnDebounceDelay = 20;

  // Configuration paths
  String goingUpDownSensorReadDelayConfigPath = "/sensor_going_up_down/read_delay";
  String windlassStatusSKPathConfigPath = "/windlass_status/sk";
  String goingUpSensorDebounceDelayConfigPath = "/sensor_going_up/debounce_delay";
  String goingDownSensorDebounceDelayConfigPath = "/sensor_going_down/debounce_delay";
  String chainCounterSKPathConfigPath = "/rodeDeployed/sk";
  String chainCounterSensorDebounceDelayConfigPath = "/chain_counter_sensor/debounce_delay";
  String chainSpeedSKPathConfigPath = "/chainSpeed/sk";
  String chainCalibrationSKPathConfigPath = "/chain_counter_sensor/calibration_value";
  String resetbtnDebounceDelaySKPathConfigPath = "/chain_counter_resetbtn_debounce/delay";

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
                    ->set_hostname("sensESP windlass-chain-counter")
                   ->set_wifi("ssid", "password")
                    ->set_sk_server("192.168.1.69", 80)
                    ->get_app();

  // define output PINs    
  pinMode(goUpPin, OUTPUT);
  pinMode(goDownPin, OUTPUT);

//set initial state of outputs
  digitalWrite(goUpPin, LOW );
  digitalWrite(goDownPin, LOW );
  
  /**
   * DigitalInputChange monitors a physical button/solenoid connected to BUTTON_PIN.
   * The input PIN is set to PULLDOWN (LOW) & the button or solenoid must take the input voltage HIGH when operated.
   * Because its interrupt type is CHANGE, it will emit a value when the button
   * is pressed, and again when it's released.
   * The LambdaConsumer function determines the action based on HIGH (pressed) or LOW (released) state of input
   */
  
  auto* goingUpSensor = new DigitalInputChange(goingUpPin, INPUT_PULLDOWN, CHANGE);

  auto* goingDownSensor = new DigitalInputChange(goingDownPin, INPUT_PULLDOWN, CHANGE);


  auto* windlassStatusSKOutput = new SKOutputString(windlassStatusSKPath, windlassStatusSKPathConfigPath);

  // initialise SKPath with "off" status
  
    windlassStatusSKOutput->emit("off");

  /**
   * Create a Debounce for each direction sensor to a clean signal from the
   * button/solenoid. The debounce delay period (setup initially under // Delays above) 
   *  can also be configured by debounce_config_path in the Config UI.
   */
    auto* goingUpSensorDebounce = new DebounceInt(goingUpSensorDebounceDelay, goingUpSensorDebounceDelayConfigPath); 

    auto* goingDownSensorDebounce = new DebounceInt(goingDownSensorDebounceDelay, goingDownSensorDebounceDelayConfigPath); 


  //  Manage the going down sensor

  goingDownSensor
    // Debounce the signal
    ->connect_to(goingDownSensorDebounce)

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


  // Manage the going up sensor 

  goingUpSensor
    // Debounce the signal
    ->connect_to(goingUpSensorDebounce)

    // Update the windlass status according to the going up sensor
    ->connect_to(new LambdaConsumer<int>([windlassStatusSKOutput](int input) {

      // Check if the windlass is going up (press)
      if (input == HIGH) {

        // The windlass is going up, update the upDown variable
        upDown = WINDLASS_GO_UP;

        // Update the windlass status
        windlassStatusSKOutput->emit("up");
      } else {

        // The windlass is not going up anymore (release), update the windlass status
        windlassStatusSKOutput->emit("off");
      }
    }));


  // Define a digital input sensor for detecting windlass rotations (from hall effect sensor or reed relay)
  //  PULLDOWN input type, assumes its a POSITIVE pulse,  RISING for leading edgetrigger
  //NOTE: if connecting sensor to BOTH chainCounterPIN  AND chainSensorPIN .. 
  //  then BOTH DigitalInputChange functions (INPUT_PULLDOWN and RISING) MUST be set the same!!
  auto* chainCounterSensor = new DigitalInputChange(chainCounterPin, INPUT_PULLDOWN,RISING);

  /**
   * Create a DebounceInt to ensure a clean signal from the button. 
   * Set the debounce delay period initially (from //Delays above), 
   * which can also be configured via debounce_config_path in the Config UI.
   */
  auto* chainCounterSensorDebounce = new DebounceInt(chainCounterSensorDebounceDelay, chainCounterSensorDebounceDelayConfigPath); 
  
  // ******  Manage the chain counter sensor *******
  chainCounterSensor
    // Debounce the signal
    ->connect_to(chainCounterSensorDebounce)

    // Transform the signal in deployed rode in meters
    ->connect_to(

      // Create a lambda transform function
      new LambdaTransform<int,int>(

        // Catch the counter and the status output instances, input is HIGH when the pin status changes
        [windlassStatusSKOutput](int input) {
        
          // Check if it is the rising front of the pin status change
          if (input == HIGH) {

            // Increase or decrease the couter
            chainCounter = chainCounter + upDown;
            
          }

          // Returm the chain counter value
          return chainCounter;
        }
        )
      )
    ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
    ->connect_to(new SKOutputFloat(chainCounterSKPath, chainCounterSKPathConfigPath, chainCounterMetadata));
    
  
  // Define a counter sensor with debouncer  for measuring the chain speed
  // in the above  windlass example, the wheel typically rotates between 1 to 2 seconds per revolution ..
  //  so the read timer is set to cover a 2000mS window
  // PULLDOWN input type, assumes its a POSITIVE pulse (detection = RISING for leading edge)

  auto* chainSpeedSensor = new DigitalInputDebounceCounter(chainSpeedPin, INPUT_PULLDOWN, RISING, 2000,
                                     chainCounterSensorDebounceDelay, chainCounterSensorDebounceDelayConfigPath);

  // ****** Manage the chain speed sensor ******
  // Counter result is put thru linear transform to get meters for every result period (2000mS)
  // Then thru a MOving average calculation over a 4 sec window, 
  // & with above 2000mS counter period, the moving average output needs to be scaled by 0.5 to get counts per second 
 
  chainSpeedSensor
      ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
      ->connect_to(new MovingAverage(4, 0.5F))                                          
      ->connect_to(new SKOutputFloat(chainSpeedSKPath, chainSpeedSKPathConfigPath, chainSpeedMetadata));  

  
  // Create a listener for the SignalK windlass status path

 /**  
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

*/


  // Connect a physical button that will reset chainCounter = 0
  // a momentary push button pulls the PIN low when operated 
  // there is probably a more efficient way to code this section?
  
  DigitalInputState* resetbtn = new DigitalInputState(resetPin, INPUT_PULLUP, 100);
  
auto* resetbtnDebounce = new DebounceInt(resetbtnDebounceDelay, resetbtnDebounceDelaySKPathConfigPath); 
  
  // ******  reset the chain counter  *******
  resetbtn
    // Debounce the signal
    ->connect_to(resetbtnDebounce)

    // Transform 
    ->connect_to(

      // Create a lambda transform function
      new LambdaTransform<int,int>(

        // Catch the counter and the resetbtn state, input is LOW when button pressed 
        [windlassStatusSKOutput](int input) {
        
          // Check if button pressed (input =LOW )
          if (input == LOW) {

            // reset couter =0
            chainCounter = 0;
            
          }

          // Returm the updated chain counter value
          return chainCounter;
        }
        )
      )
  ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
  ->connect_to(new SKOutputFloat(chainCounterSKPath, chainCounterSKPathConfigPath, chainCounterMetadata));



// *********** Reset chain counter from NodeRed or similar external program/application  **************
// monitor an SKPath that is toggled  "true" by external NodeRed flow 
// there is probably a better way to achive this using a Put? style message

// Configuration Paths
String chainCounterResetSKPath = "navigation.anchor.chainCounterReset";

auto* windlassResetListener = new BoolSKListener(chainCounterResetSKPath);


     windlassResetListener 
    ->connect_to(

      // Create a lambda transform function
      new LambdaTransform<int,int>(

        // Catch the counter and the windlassResetListener state, input is LOW to reset chainCounter 
        [windlassStatusSKOutput](int input) {
        
          // Check if remote skpath set = true )
          if (input == true) {

            // reset couter =0
            chainCounter = 0;
            
          }

          // Returm the updated chain counter value
          return chainCounter;
        }
        )
      )
  ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
  ->connect_to(new SKOutputFloat(chainCounterSKPath, chainCounterSKPathConfigPath, chainCounterMetadata));


//  ********************

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
