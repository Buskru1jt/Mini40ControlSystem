#include <SBUS.h>                   //SBUS Library by Brian R Taylor (Bolder Flight Systems)
#include <MPU9250.h>                //MPU9250 Library by Brian R Taylor (Bolder Flight Systems)
#include <Servo.h>                  //Default Arduino servo library

uint16_t channels[16];              //Array to store data from channels (1-16)
bool failSafe;                      //Boolean to store failsafe state (failsafe activated)
bool lostFrame;                     //Boolean to store lostframe state (out of range)

int status;                         //Variable to store the MPU9250 status


SBUS xmplus(Serial1);               //Define XM+ SBUS receiver on Hardware serial port 1
MPU9250 IMU(Wire,0x68);             //Define MPU9250 IMU on I2C address 0x68
Servo rudderServo;                  //Define rudder servo
Servo sailServo;                    //Define sail servo

int modeSelect;                     //Integer to store the selected control mode (1 = manual, 2 = anti capsize, 3 = anti capsize + course hold)
float currentHeading;               //Float to store the current heading for comparisson
float previousHeading;              //Float to store the previous heading for comparisson
float setHeading;                   //Float to store the set heading for comparisson
float currentHeadingDeviation;      //Float to store teh current heading deviation

uint16_t toggleSwitchLow = 750;     //Low boundry value for toggle switch on transmitter
uint16_t toggleSwitchHigh = 1350;   //High boundry value for toggle switch on transmitter

float headingAmplification = 5;     //Amplification factor from degrees to micro seconds
float maxHeadingDeviation = 1;          //Float to store the maximum deviation in heading before the auto rudder control kicks in

void setup() {

  Serial.begin(115200);             //Start serial communication with an attached PC
  Serial.println("Booting...");

  xmplus.begin();                   //Start the communication with the SBUS receiver
  Serial.println("Receiver initialised...");


  status = IMU.begin();             //Start the communication with the IMU and get the status of the connection

  Serial.println("Booting IMU...");
  //Wait for the IMU to return a status and print the status to the serial port in case of an error for debug purposes
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  Serial.println("IMU initialised...");
  rudderServo.attach(8);            //Attach rudder servo to PWM pin 8
  sailServo.attach(9);              //Attach sail servo to PWM pin 9

  rudderServo.writeMicroseconds(1500);    //Center rudder servo

  IMU.readSensor();                 //Do a initial read to set a baseline heading
  setHeading = IMU.getMagZ_uT();    //Set a baseline set heading
  
  Serial.println("Ready");          //Send ready signal over serial port for debug purposes
}

void loop() {

  //Read the current state of the IMU Sensor and store in a buffer
  IMU.readSensor();

  currentHeading = IMU.getMagZ_uT();
  Serial.println("Mag = " + String(currentHeading));

  //Check for a valid frame from the receiver, store the values in a variable and send the data to the serial connection for debug purposes
  if(xmplus.read(&channels[0], &failSafe, &lostFrame)){     
    Serial.print(channels[0]);
    Serial.print(", ");
    Serial.print(channels[2]);
    Serial.print(", ");
    Serial.println(channels[4]);


    //Set desired control mode according to the toggle switch on the transmitter (AUX.3)
    if(channels[4] < toggleSwitchLow) {

      //Steering mode = 1, manual, so directly send reciever values to servo's to minimize delays and work as if it would work without control box 
      modeSelect = 1;
      setServos(channels[0], channels[2], 1); 
      Serial.println("Mode 1");
      
    }
    else if(channels[4] > toggleSwitchHigh) {
      modeSelect = 3;
      Serial.println("Mode 3");

      if(channels[0] < 1000 && channels[0] > 950 ) {
        if(abs(currentHeading - setHeading) > maxHeadingDeviation) {
          setServos((channels[0] + (setHeading - currentHeading)) * headingAmplification, channels[2], 1);
        }
        
      }
      else {
         setServos(channels[0], channels[2], 3); 
         setHeading = currentHeading;
      }
    }
    else
    {
      modeSelect = 2;
      Serial.println("Mode 2");

      
    }
  }
  
}


void setServos(int rudderServoValue, int sailServoValue, int mode) {

  switch(mode) {
    case 1:
      rudderServo.writeMicroseconds(map(rudderServoValue,172,1811,1000,2000));
      break;

    case 2:
      break;

    case 3:
      break;

    default:
      Serial.println("No mode selected");
      break;
  }
  
  
}


