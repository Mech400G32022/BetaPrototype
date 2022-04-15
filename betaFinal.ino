#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SD.h>
#include <EEPROM.h>
#include <Encoder.h>
#include <LiquidCrystal.h>

//Custom Headers
#include "PololuMotorDriver.h"
#include "TimerExecuter.h"
#include "PID.h"
#include "LeadComp.h"

//Pin Definitions
#define DIR1 34
#define SLP1 35
#define CS1 36
#define M1_PWM 33
#define ENCA_1 30
#define ENCB_1 29

#define DIR2 38
#define SLP2 39
#define CS_1 40
#define M2_PWM 37
#define ENCA_2 31
#define ENCB_2 32

#define POTENTIOMETER 27

#define LCD_RS 0
#define LCD_ENABLE 1
#define LCD_D4 2
#define LCD_D5 3
#define LCD_D6 4
#define LCD_D7 5

//Hardware Definition
Adafruit_BNO055 imuOne = Adafruit_BNO055(1, BNO055_ADDRESS_A, &Wire1);
Adafruit_BNO055 baseImu = Adafruit_BNO055(2, BNO055_ADDRESS_A, &Wire);
Adafruit_BNO055 imuTwo = Adafruit_BNO055(3, BNO055_ADDRESS_A, &Wire2);

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);


//Motor loop constants
#define KP  2                                                              
#define KI  0.001
#define KD  0

PololuDcMotor motor1 = PololuDcMotor(DIR1, SLP1, M1_PWM);
Encoder enc1(ENCA_1, ENCB_1);
PID motor1Gains(KP, KI, KD);
LeadCompensator motor1Lead(KP, 5, 0.01, 0.1);

PololuDcMotor motor2 = PololuDcMotor(DIR2, SLP2, M2_PWM);
Encoder enc2(ENCA_2, ENCB_2);
PID motor2Gains(0.75, KI, KD);

#define SPOOL_PLATE_RATIO 41.57
#define ANGLE_LIMIT 16

#define F_MC_LOOP_HZ 1000
IntervalTimer motorControlTimer;

#define F_ANALOG_READ 10
void checkMode();
IntervalExecuter modeCheckTimer(1000/F_ANALOG_READ, checkMode);

//Operation modes. INIT is only used at startup.
#define HOLD_MODE 0
#define STABLE_MODE 1
#define CAL_MODE 2
#define INIT 3
volatile short mode = INIT;

#define F_IMU_LOOP_HZ 100
void positionLoop();
IntervalExecuter positionControlTimer(1000/F_IMU_LOOP_HZ, positionLoop);

#define IMU_CALIB_SIZE (sizeof(adafruit_bno055_offsets_t) + sizeof(long))

#define DEBUG_SERIAL
#define FATAL(error) Serial.println(error);

//If we don't want serial all calls will be removed by preprocessor
#ifdef DEBUG_SERIAL
  #define SerialPrintLn(m) Serial.println(m)
#else
  #define SerialPrintLn(m)
#endif

#define DEBUG_SD
#define FILENAME_PATTERN "enc_imu%d.csv"
#define CSV_HEADER "time (ms),enc1 pos,enc2 pos,z1,z2,y1,y2,cmd1,cmd2,z3,y3"

//Define the index of the data to write to the file
//Ensure that the CSV_HEADER and CSV_LEN of the buffer match.
#define CSV_TIME  0
#define CSV_ENC1_POS    1
#define CSV_ENC2_POS    2
#define CSV_Z1    3
#define CSV_Z2    4
#define CSV_Y1    5
#define CSV_Y2    6
#define CSV_COMMAND_1 7
#define CSV_COMMAND_2 8
#define CSV_Z3 9
#define CSV_Y3 10

#define CSV_LEN   11
volatile double csv_buffer[CSV_LEN];

//If the buffer has no length we shouldn't be writing to file.
//Save computation by removing need for bounds checking
#if CSV_LEN <= 0
  #undef DEBUG_SD
#endif


//We are willing to accept losing the last reading from the IMU on power down.
#define FLUSH_PERIOD_MS 1000 / F_IMU_LOOP_HZ / 2
#define F_SAVE_CSV_HZ 100
void saveCSV();
IntervalExecuter saveCSVTimer(1000 / F_SAVE_CSV_HZ, saveCSV);

void flushCSV();
IntervalExecuter flushCSVTimer(FLUSH_PERIOD_MS, flushCSV);

File dataFile = NULL;
volatile bool writeToFile = false;

void initDataLog(){
  if (SD.begin(BUILTIN_SDCARD)) {
    int i = 0;
    char fileName[16];

    //Increment the filenumber until no file with that datalog name is found.
    sprintf(fileName, FILENAME_PATTERN, i);
    while (SD.exists(fileName)){
      i++;   
      sprintf(fileName, FILENAME_PATTERN, i);
    }

    
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println(CSV_HEADER);
      writeToFile = true;
    } else {
      SerialPrintLn("No file could be opened");
    }
  } else {
     SerialPrintLn("Card failed, or not present");
  }
  
  lcd.setCursor(0, 1);
  if (writeToFile) {
    lcd.print("SD Card OK");
  } else {
    lcd.print("No Datalogging");
  }
}

//The saveCSV method assumes that other functions will not change the
//contents of the buffer during a write
void saveCSV(){  
  for(int i = 0; i < CSV_LEN - 1; i++) {
    dataFile.print(csv_buffer[i]);
    dataFile.print(",");
  }
  dataFile.println(csv_buffer[CSV_LEN - 1]);
}

void flushCSV(){
  dataFile.flush();
}


void initImu(Adafruit_BNO055 imu) {

   if(!imu.begin()) {
    FATAL("No BNO055 detected Check your wiring or I2C ADDR!");
    delay(200);
  }

  imu.setSpeed(400000);
  
  sensor_t sensor;
  imu.getSensor(&sensor);


  int eeAddress = sensor.sensor_id * IMU_CALIB_SIZE;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  if (bnoID != sensor.sensor_id) {
      SerialPrintLn("\nNo Calibration Data for this sensor exists in EEPROM");
  } else {
      SerialPrintLn("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);
      SerialPrintLn("\n\nRestoring Calibration data to the BNO055...");
      imu.setSensorOffsets(calibrationData);
  }
    
    delay(200);
   /* Crystal must be configured AFTER loading calibration data into BNO055. */
    imu.setExtCrystalUse(true);
}

long offset_t = 0;
void setup() {
  #ifdef DEBUG_SERIAL
    Serial.begin(9600);
  #endif

  pinMode(POTENTIOMETER, INPUT);

  lcd.begin(16, 2);
  lcd.clear();
  // Print a message to the LCD.
  lcd.print("Welcome to GLIDE");

  
  #ifdef DEBUG_SD
    initDataLog();
  #endif
  
  initImu(baseImu);
  initImu(imuOne);
  initImu(imuTwo);

  //TODO set orientation of all 3 IMUs


  motor1Gains.err_deadzone = 0;
  motor2Gains.err_deadzone = 0;
  motor1.setBrake(false);
  motor2.setBrake(false);

  float average1 = 0;
  float average2 = 0;

  for (int i = 0; i < 5; i++){
    imu::Vector<3> euler1 = imuOne.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> euler2 = baseImu.getVector(Adafruit_BNO055::VECTOR_EULER);
    average1 += (euler1.y() - euler2.y())/100;
    average2 += (euler1.z() - euler2.z())/100;
    delay(10);
  }
  
  imu::Vector<3> eulerBase = baseImu.getVector(Adafruit_BNO055::VECTOR_EULER);

  enc1.write((long)(eulerBase.y()*SPOOL_PLATE_RATIO));
  enc2.write((long)(eulerBase.z()*SPOOL_PLATE_RATIO));

  SerialPrintLn(average1);
  SerialPrintLn(average2);


  if (!motorControlTimer.begin(motorLoop, 1000000/F_MC_LOOP_HZ)) {
    FATAL("Hardware timers unavailable");
  }
  offset_t = millis();
}

void motorLoop() {
  if (mode == CAL_MODE) {
    motor1.setBrake(false);
    motor1.setPower(0);
    
    motor2.setBrake(false);
    motor2.setPower(0);
    return;
  }
  
  long pos1 = enc1.read();
  double power1 = motor1Lead.update(pos1);
  
  if (pos1 > SPOOL_PLATE_RATIO*ANGLE_LIMIT && power1 < 0) {
    motor1.setBrake(true);
    motor1.setPower(0);
  } else if (pos1 < -SPOOL_PLATE_RATIO*ANGLE_LIMIT && power1 > 0) {
    motor1.setBrake(true);
    motor1.setPower(0);
  } else {
    motor1.setBrake(false);
    motor1.setPower(power1);
  }

  long pos2 = enc2.read();
  double power2 = motor2Gains.update(pos2);
  
  if (pos2 > SPOOL_PLATE_RATIO*ANGLE_LIMIT && power2 < 0) {
    motor2.setBrake(true);
    motor2.setPower(0);
  } else if (pos2 < -SPOOL_PLATE_RATIO*ANGLE_LIMIT && power2 > 0) {
    motor2.setBrake(true);
    motor2.setPower(0);
  }else {
    motor2.setBrake(false);
    motor2.setPower(power2);
  }
}

void positionLoop() {
    imu::Vector<3> euler1 = imuOne.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> euler2 = baseImu.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> euler3 = imuTwo.getVector(Adafruit_BNO055::VECTOR_EULER);


    float cmd1 = euler2.y();
    float cmd2 = euler2.z();
    motor1Gains.target = (long)(SPOOL_PLATE_RATIO*cmd1);
    motor1Lead.target = (long)(SPOOL_PLATE_RATIO*cmd1);
    motor2Gains.target = (long)(SPOOL_PLATE_RATIO*cmd2); 
    
    csv_buffer[CSV_Z1] = euler1.z();
    csv_buffer[CSV_Z2] = euler2.z();
    csv_buffer[CSV_Z3] = euler3.z();
    csv_buffer[CSV_Y1] = euler1.y();
    csv_buffer[CSV_Y2] = euler2.y();
    csv_buffer[CSV_Y3] = euler3.y();
    csv_buffer[CSV_COMMAND_1] = cmd1;
    csv_buffer[CSV_COMMAND_2] = cmd2;
}

void loop() {
    switch (mode) {
      case STABLE_MODE:
            positionControlTimer.executeIfTime();
      break;
      case HOLD_MODE:
      break;
      case CAL_MODE:
          imu::Vector<3> eulerBase = baseImu.getVector(Adafruit_BNO055::VECTOR_EULER);

          enc1.write((long)(eulerBase.y()*SPOOL_PLATE_RATIO));
          enc2.write((long)(eulerBase.z()*SPOOL_PLATE_RATIO));
          delay(50);
      break;
    }
    modeCheckTimer.executeIfTime();

    #ifdef DEBUG_SD
    if (writeToFile) {
      csv_buffer[CSV_TIME] = millis();
      csv_buffer[CSV_ENC1_POS] = (double)enc1.read();
      csv_buffer[CSV_ENC2_POS] = (double)enc2.read();
      saveCSVTimer.executeIfTime();
      flushCSVTimer.executeIfTime();
    }
    #endif
}


void checkMode() {
  int potVal = analogRead(POTENTIOMETER);
  short lastMode = mode;

  //Map the potentiometer to the desired mode.
  //Hold has been made the largest range and the default.
  if (potVal > 735) {
    mode = STABLE_MODE;
  } else if (potVal > 325) {
    mode = HOLD_MODE;
  } else if (potVal > 210){
    mode = CAL_MODE;
  } else {
    //Unplugged
    mode = HOLD_MODE;
  }

  //One time actions to take when the mode changes.
  //Switch on the new mode
  if (mode != lastMode){
    lcd.clear();
    lcd.setCursor(0, 0);
    switch(mode){
      case HOLD_MODE: lcd.print("Mode: Hold");
      motor1Gains.target = enc1.read();
      motor2Gains.target = enc2.read(); 
      break;
      case STABLE_MODE: lcd.print("Mode: Level");
      break;
      case CAL_MODE: lcd.print("Mode: Calibrate");
      lcd.setCursor(0, 1);
      lcd.print("Manual Level");
      break;
    }
  }
  
}
