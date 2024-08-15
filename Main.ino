//7/16/2024



#include <adcread2024.h>
//LIBRARIES
#include <Preferences.h>

Preferences preferences;
#include <Arduino.h>

#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>
#include "bms_temp.h"
#include "slaver.h"





#include "BluetoothSerial.h"
#include "nvs_flash.h"
#include <can_bms.h>       //DALY CANBUS LIBRARY
#include "can_server.h"   // ATES INVERTER LIBRARY



char receivedChar;
boolean newData = false;

bool RUNRT = false;
bool RUNSTRING = false;
uint16_t temp1 = 0;
uint16_t temp2 = 0;
float prechargeVolt = 0;
float prechargeCounter = 0;
bool prechargeStatus = true;
bool prechargeTrigger = false;
bool FORCE = false;
bool disconnection = false;
int BREAKER_PULSE = 0;
int ANALOG_PULSE = 0;
int CANBUS_PULSE = 0;
int MODBUS_PULSE = 0;
int disconnect_counter = 0;
int AlarmCode = 0;
int DischargeCounter = 0;
bool DischargeOvertake = false;
int BoardVersion = 1;
bool SOCLockEnable = false;
int SOCLockValue = 0;
int CanSim = 0;

//PINS
#define GEN_RE  0


uint16_t MODBUSARRAY[800];

//OBJECTS
uint8_t *BMSAR;


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//TASK HANDLES
TaskHandle_t CANBUSTASK;
TaskHandle_t INVERTERCANBUSTASK;
TaskHandle_t BREAKERCONTROLTASK;
TaskHandle_t MODBUSTASK;
TaskHandle_t BT;
TaskHandle_t ANALOGTASK;
TaskHandle_t SERIALMONITORTASK;


//VARIABLES
double TerminalVoltageArray[40];
double TerminalCurrentArray[40];
double TerminalTempArray[40];
double TerminalSOCArray[40];
double RemainingCapacity[40];
int ChargeStatusArray[40];
int DischargeStatusArray[40];
int AlarmStatusArray[40];
double MaxCellArray[40];
double MinCellArray[40];
float String1Current = 0;
float String1Voltage = 0;
float String1SOC = 0;
float String1CurrentCal = 0;
float String1VoltageCal = 0;
float String1SOCCal = 0;
float String1TempCal = 0;
float String1Temp = 0;
float String2Current = 0;
float String2Voltage = 0;
float String2SOC = 0;
float String2CurrentCal = 0;
float String2VoltageCal = 0;
float String2SOCCal = 0;
String SystemAlarm1 = "NoAlarm";
String SystemAlarm2 = "NoAlarm";
String SystemAlarm3 = "NoAlarm";
String SystemAlarm4 = "NoAlarm";
String SystemAlarm5 = "NoAlarm";
String SystemAlarm6 = "NoAlarm";
String SystemAlarm7 = "NoAlarm";
String SystemAlarm8 = "NoAlarm";
String SystemWarning1 = "NoWarning";
String SystemWarning2 = "NoWarning";
String SystemWarning3 = "NoWarning";
String SystemWarning4 = "NoWarning";
String SystemWarning5 = "NoWarning";
String SystemWarning6 = "NoWarning";
String SystemWarning7 = "NoWarning";
String SystemWarning8 = "NoWarning";


bool TaskArray[7];
String TaskStringArray[7] = {"INVERTER-CANBUS", "MODULE-CANBUS", "MODBUS-HOST", "ANALOG-READING", "BREAKER-CONTROL", "BLUETOOTH-CONTROL", "SERIAL-MONITOR"};


uint8_t datart[8];

float HighVoltageAlarmStart = 538;
float HighVoltageAlarmStop = 530;
float LowVoltageAlarmStart = 485;
float LowVoltageAlarmStop = 490;
float HighTempAlarmStart = 40;
float HighTempAlarmStop = 35;
bool Rack1ChargeRelay = true;
bool Rack1DischargeRelay = true;
bool Bypass1Relay = true;
bool Rack2ChargeRelay = true;
bool Rack2DischargeRelay = true;
bool Bypass2Relay = true;
bool Fan1 = true;
bool OutputDry = true;
bool Rack1ChargeRelayCal = true;
bool Rack1DischargeRelayCal = true;
bool Bypass1RelayCal = true;
bool Fan1Cal = true;
bool OutputDryCal = true;
bool Rack2ChargeRelayCal = true;
bool Rack2DischargeRelayCal = true;
bool Bypass2RelayCal = true;
bool Fan2Cal = true;
bool OutputDryCa2 = true;
bool EnableForce = false;
bool ForcedRack1ChargeRelay = false;
bool ForcedRack1DischargeRelay = false;
bool ForcedBypass1Relay = false;
bool ForcedFan1 = false;
bool ForcedPrecharge = false;
String FirmwareVer = "1.0.2";
String SerialProcessor = "";
int SubID = 1;
int ModuleSize = 10;
//how many module communicates subcontroller
int InverterType = 0; //
//0: DEYE
//1: ATESS
int MaximumModuleVolt = 54;
int MaximumStringVolt = 1000;
bool ChargeRelay = true;
bool DischargeRelay = true;
int Heartbeat = 0;
float String1MaxCell = 0;
float String1MinCell = 500;
int MaxCurrent = 100;
int CanbusBaudRate = 250;
int ModbusBaudRate = 9600;

//adc read default
uint16_t SDA_0 = 33;
uint16_t SCL_0 = 32;
String receivedString;
String SerialNumber = "C"; // ENC-DATE-NUMBER
String HardwareSerial = "XXX-XXX";

void setup() {
  RXS = 16;
  TXS = 17;
  en_rs485 = 25;

  Serial.begin(115200);


  //




  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);


  HardwareSerial = getHardwareSerial();

  preferences.begin("my-app", false);
  HighVoltageAlarmStart = preferences.getInt("HVST", 538);
  HighVoltageAlarmStop = preferences.getInt("HVSP", 532);
  LowVoltageAlarmStart = preferences.getInt("LVST", 485);
  LowVoltageAlarmStop = preferences.getInt("LVSP", 490);
  HighTempAlarmStart = preferences.getInt("HTST", 40);
  HighTempAlarmStop = preferences.getInt("HTSP", 35);
  MaxCurrent = preferences.getInt("MaxCurrent", 100);
  SerialNumber = preferences.getString("SN", "UNIFIED-CONTROLLER");
  SubID = preferences.getInt("SubID", 1);
  ModuleSize = preferences.getInt("MDS", 10);
  InverterType = preferences.getInt("IT", 0);
  MaximumModuleVolt = preferences.getInt("MMV", 54);
  MaximumStringVolt = preferences.getInt("MSTV", 1000);
  CanbusBaudRate = preferences.getInt("CBR", 250);
  ModbusBaudRate = preferences.getInt("MBR", 9600);
  BoardVersion = preferences.getInt("BV", 0);
  SOCLockEnable = preferences.getBool("SLE", false);
  SOCLockValue = preferences.getInt("SLV", 0);
  preferences.end();

  if (BoardVersion) {
    SDA_0 = 33;
    SCL_0 = 32;
    pinMode(19, OUTPUT); // Charge1
    pinMode(18, OUTPUT);//Discharge1
    pinMode(21, OUTPUT);//Bypass1
    pinMode(22, OUTPUT);//Fan1
    pinMode(5, OUTPUT);//Precharge
    digitalWrite(26, false);
  }
  else {
    SDA_0 = 21;
    SCL_0 = 22;
    pinMode(5, OUTPUT); // Charge1
    pinMode(19, OUTPUT);//Discharge1
    pinMode(15, OUTPUT);//Bypass1
    pinMode(26, OUTPUT);//Fan1
    pinMode(18, OUTPUT);//Precharge
  }

  //CREATING TASK FOR CANBUS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    INVERTERCANBUSTASK_CODE,   /* Task function. */
    "INVERTERCANBUSTASK",     /* name of task. */
    6000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    8,           /* priority of the task */
    &INVERTERCANBUSTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);
  //
  //
  //CREATING TASK FOR CANBUS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    CANBUSTASK_CODE,   /* Task function. */
    "CANBUSTASK",     /* name of task. */
    9000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    8,           /* priority of the task */
    &CANBUSTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);
  //
  //
  //CREATING TASK FOR RELAYS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    BREAKERCONTROLTASK_CODE,   /* Task function. */
    "BREAKERCONTROLTASK",     /* name of task. */
    6000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    3,           /* priority of the task */
    &BREAKERCONTROLTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);
  //
  //
  //CREATING TASK FOR MODBUS
  xTaskCreatePinnedToCore(
    MODBUSTASK_CODE,   /* Task function. */
    "MODBUSTASK",     /* name of task. */
    60000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    5,           /* priority of the task */
    &MODBUSTASK,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(50);

  //
  //CREATING TASK FOR BLUETOOTH_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    BT_CODE,   /* Task function. */
    "BT",     /* name of task. */
    9000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &BT,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);

  //
  //CREATING TASK FOR ANALOG READING_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    ANALOGTASK_CODE,   /* Task function. */
    "ANALOGTASK",     /* name of task. */
    6000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &ANALOGTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);

  //   CREATING TASK FOR ANALOG READING_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    SERIALMONITOR_CODE,   /* Task function. */
    "SERIALMONITORTASK",     /* name of task. */
    6000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &SERIALMONITORTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);
}

void INVERTERCANBUSTASK_CODE( void * pvParameters ) {

  TaskArray[0] = true;
  can_start(CanbusBaudRate);

  float String1MaxCellModbus = 500;
  float String1MinCellModbus = 500;

  for (;;) {


    //      Serial.println("String1MaxCellCANBUS:" + String(String1MaxCell));
    //      Serial.println("String1MinCellCANBUS:" + String(String1MinCell));


    if (String1MaxCell < 4.1 && String1MaxCell > 2.5) {
      String1MaxCellModbus = String1MaxCell;
    }

    if (String1MinCell < 4.1 && String1MinCell > 2.5) {
      String1MinCellModbus = String1MinCell;
    }



    if (String1MaxCellModbus  < 4.8  && String1MinCellModbus  < 4.8  && String1SOC < 105  && abs(String1Current) < 500 && String1MaxCellModbus  > 2.2 && String1MinCellModbus  > 2.2 && String1Voltage < 1000 && String1Voltage > 90 ) {

      //
      //      Serial.println("String1MaxCellMODBUS:" + String(String1MaxCellModbus));
      //      Serial.println("String1MinCellMODBUS:" + String(String1MinCellModbus));
      //      Serial.println("CANBUS Valid");

      if (InverterType == 1) {
        set_maxvoltage(String1MaxCellModbus, String1MinCellModbus, String1SOC, String1SOC, 0, 0x180150F1);

        if (String1Voltage < MaximumStringVolt * 0.9) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent, MaxCurrent, 0x180250F1);
        }


        else if (String1Voltage >= MaximumStringVolt * 0.9 && String1Voltage < MaximumStringVolt * 0.91) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.9, MaxCurrent, 0x180250F1);
        }

        else if (String1Voltage >= MaximumStringVolt * 0.91 && String1Voltage < MaximumStringVolt * 0.92 ) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.8, MaxCurrent, 0x180250F1);
        }

        else if (String1Voltage >= MaximumStringVolt * 0.92 && String1Voltage < MaximumStringVolt * 0.93 ) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.7, MaxCurrent, 0x180250F1);
        }

        else if (String1Voltage >= MaximumStringVolt * 0.93 && String1Voltage < MaximumStringVolt * 0.94 ) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.6, MaxCurrent, 0x180250F1);
        }

        else if (String1Voltage >= MaximumStringVolt * 0.94 && String1Voltage < MaximumStringVolt * 0.95 ) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float disc7arge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.5, MaxCurrent, 0x180250F1);
        }

        else if (String1Voltage >= MaximumStringVolt * 0.95 && String1Voltage < MaximumStringVolt * 0.96 ) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.4, MaxCurrent, 0x180250F1);
        }
        else if (String1Voltage >= MaximumStringVolt * 0.96 && String1Voltage < MaximumStringVolt * 0.97 ) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.3, MaxCurrent, 0x180250F1);
        }
        else if (String1Voltage >= MaximumStringVolt * 0.97 && String1Voltage < MaximumStringVolt * 0.98) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.2, MaxCurrent, 0x180250F1);
        }
        else if (String1Voltage >= MaximumStringVolt * 0.985 && String1Voltage < MaximumStringVolt * 0.99 ) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.1, MaxCurrent, 0x180250F1);
        }
        else if (String1Voltage >= MaximumStringVolt ) {
          //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
          set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0, MaxCurrent, 0x180250F1);
        }
        delay(25);
        //set_groupnumber(uint8_t maxvolt_grpnm, uint8_t maxvolt_packnm, uint8_t maxvolt_boxnm, uint8_t maxtemp_grpnm, uint8_t maxtemp_packnm, uint8_t maxtemp, 0x180350F1);
        set_groupnumber(1, 3, 4, 1, 3, 25, 0x180350F1);
        delay(25);
        set_groupnumbermin(2, 4, 6, 2, 3, 25,  0x180450F1);

        set_warnings(4, 0, 0, 0, 0 , 0x180650F1);
        delay(25);
        set_warnings2(0, 0, 0, 0, 0, 0x180750F1);
        delay(25);
      }
      if (InverterType == 0) {  //DEYE AND CANBUS BOARD
        send_inverter(String1SOC, String1Voltage, String1Current,  12);
        delay(25);
      }
      SystemAlarm4 = "NoAlarm";
    }
    else {
      SystemAlarm4 = "Inverter canbus did not executed due to improper reading";
      //
      //            Serial.println("String1MaxCellMODBUS:" + String(String1MaxCellModbus));
      //            Serial.println("String1MinCellMODBUS:" + String(String1MinCellModbus));
      //            Serial.println("CANBUS Error!!!!!!!!!!!!!!!!!!!!!!!");
      delay(200);
    }

  }
}

void CANBUSTASK_CODE( void * pvParameters ) {
  TaskArray[1] = true;


  for (;;) {
    CANBUS_PULSE++;
    for (int i = 0; i < ModuleSize + 1; i++) {
      BMS_recieve(0x90, i + 1);
      BMS_recieve(0x92, i + 1);
      BMS_recieve(0x91, i + 1);
      BMS_recieve(0x93, i + 1);
      BMS_recieve(0x95, i + 1);
      BMS_recieve(0x98, i + 1);


      if (CanSim == 0) {
        if (( abs(BMS_can.current_can - 30000) * 0.1 < 400)  && (abs(BMS_can.discharge_can) < 2) && (abs(BMS_can.discharge_can) < 2) &&
            ((BMS_can.max_cell_temp_can - 40) < 80) && ((BMS_can.max_cell_temp_can - 40) > -5) && (abs(BMS_can.sum_voltage_can * 0.1) < 70) &&
            ((BMS_can.max_cell_volt_can * 0.001) < 5) && ((BMS_can.min_cell_volt_can * 0.001) < 5) && ((BMS_can.sum_voltage_can * 0.1) > 8)) {
          if (Heartbeat < 256) {
            Heartbeat++;
          }
          else {
            Heartbeat = 0;
          }
          TerminalVoltageArray[i] = BMS_can.sum_voltage_can * 0.1;
          TerminalCurrentArray[i] = (BMS_can.current_can - 30000) * 0.1;
          TerminalTempArray[i] = BMS_can.max_cell_temp_can - 40;
          TerminalSOCArray[i] = BMS_can.SOC_can * 0.1;
          DischargeStatusArray[i] = BMS_can.discharge_can;
          ChargeStatusArray[i] = BMS_can.charge_can;
          AlarmStatusArray[i] = BMS_can.AlarmStatus_can;
          RemainingCapacity[i] = BMS_can.rem_cap_can * 0.001 * 48 * 0.001;
          MaxCellArray[i] = BMS_can.max_cell_volt_can * 0.001;
          MinCellArray[i] = BMS_can.min_cell_volt_can * 0.001;
          AlarmCode = 0;
        }
        else {
          if (i < ModuleSize) {
            Heartbeat = 0;
            AlarmStatusArray[i] = 63;
            AlarmCode = 63;
          }
        }
      }

      else {// SIMULATION
        if (Heartbeat < 256) {
          Heartbeat++;
        }
        else {
          Heartbeat = 0;
        }
        TerminalVoltageArray[i] = random(5000, 5400) * 0.01;
        TerminalCurrentArray[i] = random(1, 10);
        TerminalTempArray[i] = random(200, 300) * 0.1;
        TerminalSOCArray[i] = random(1, 100);
        DischargeStatusArray[i] = random(0, 1);
        ChargeStatusArray[i] = random(0, 1);
        AlarmStatusArray[i] = random(0, 1);
        RemainingCapacity[i] = random(0, 200);
        MaxCellArray[i] = random(300, 420) * 0.01;
        MinCellArray[i] = random(270, 300) * 0.01;
        AlarmCode = 0;
      }
      delay(100);
    }

    /////Rack #1////////////////////////

    String1VoltageCal = 0;
    for (int i = 0; i < ModuleSize; i++) {
      String1VoltageCal = TerminalVoltageArray[i] + String1VoltageCal;
    }
    String1Voltage = String1VoltageCal;

    String1CurrentCal = 0;
    for (int i = 0; i < ModuleSize; i++) {
      String1CurrentCal = TerminalCurrentArray[i] + String1CurrentCal;
    }
    String1CurrentCal = String1CurrentCal / ModuleSize;
    String1Current = String1CurrentCal;

    String1SOCCal = 0;
    for (int i = 0; i < ModuleSize; i++) {
      String1SOCCal = TerminalSOCArray[i] + String1SOCCal;
    }
    String1SOCCal = String1SOCCal / ModuleSize;
    String1SOC = String1SOCCal;


    String1TempCal = 0;
    for (int i = 0; i < ModuleSize; i++) {
      String1TempCal = TerminalTempArray[i] + String1TempCal;
    }
    String1TempCal = String1TempCal / ModuleSize;
    String1Temp = String1TempCal;

    String1MaxCell = 0;
    for (int i = 0; i < ModuleSize; i++) {
      if (MaxCellArray[i] > String1MaxCell) {
        String1MaxCell = MaxCellArray[i];
      }
    }
    String1MinCell = 500;
    for (int i = 0; i < ModuleSize; i++) {
      if (MinCellArray[i] < String1MinCell) {
        String1MinCell = MinCellArray[i];
      }
    }
  }
}


void BREAKERCONTROLTASK_CODE( void * pvParameters ) {
  TaskArray[4] = true;
  float CurrentMillisBreaker = 0;
  float PreviousMillisBreaker = 0;
  bool TakeOver1 = false;
  bool TakeOver2 = false;
  float DischargeCounter2 = 0;
  float CurrentMillisBreaker2 = 0;
  float PreviousMillisBreaker2 = 0;
  bool TakeOver12 = false;
  bool TakeOver22 = false;
  float DischargeCounter22 = 0;

  for (;;) {
    BREAKER_PULSE++;
    //Charge Control#1
    Rack1ChargeRelayCal = true;
    if (String1Voltage < HighVoltageAlarmStart) {
      for (int i = 0; i < ModuleSize; i++) {
        Rack1ChargeRelayCal = Rack1ChargeRelayCal && ChargeStatusArray[i];
      }
      Rack1ChargeRelay = Rack1ChargeRelayCal;
      SystemAlarm1 = "NoAlarm";
    }
    else {
      SystemAlarm1 = "charge relay#1 is off due to high voltage:" + String(String1Voltage) + ">" + String(HighVoltageAlarmStart);
      Rack1ChargeRelay = false;
    }
    if (SOCLockEnable)
    {
      if (String1SOC > SOCLockValue)
      {
        //Discharge Control#1
        if (String1Voltage > LowVoltageAlarmStart) {
          Rack1DischargeRelayCal = true;
          for (int i = 0; i < ModuleSize; i++) {
            Rack1DischargeRelayCal = Rack1DischargeRelayCal && DischargeStatusArray[i];
          }
          Rack1DischargeRelay = Rack1DischargeRelayCal;
          SystemAlarm2 = "NoAlarm";
        }
        else {
          SystemAlarm2 = "discharge relay#1 is off due to low voltage:" + String(String1Voltage) + "<" + String(LowVoltageAlarmStart);
          //Serial.println(String1Voltage);
          Rack1DischargeRelay = false;
        }
        SystemAlarm2 = "NoAlarm";
      }
      else {
        Serial.println("SOC Lock activated!");
        Rack1DischargeRelay = false;
        SystemAlarm2 = "SOC Lock activated!";
      }
    }
    else {
      SystemAlarm2 = "NoAlarm";
      //Discharge Control#1
      if (String1Voltage > LowVoltageAlarmStart) {
        Rack1DischargeRelayCal = true;
        for (int i = 0; i < ModuleSize; i++) {
          Rack1DischargeRelayCal = Rack1DischargeRelayCal && DischargeStatusArray[i];
        }
        Rack1DischargeRelay = Rack1DischargeRelayCal;
        SystemAlarm2 = "NoAlarm";
      }
      else {
        SystemAlarm2 = "discharge relay#1 is off due to low voltage:" + String(String1Voltage) + "<" + String(LowVoltageAlarmStart);
        //Serial.println(String1Voltage);
        Rack1DischargeRelay = false;
      }

    }

    //Take over conditions are defined for Discharge Control
    TakeOver1 = !Rack1DischargeRelay;
    TakeOver2 = String1Voltage < LowVoltageAlarmStart;

    if ((TakeOver1 || TakeOver2)) {
      CurrentMillisBreaker = millis();
      DischargeCounter2 = DischargeCounter2 + (CurrentMillisBreaker - PreviousMillisBreaker) * 0.001;

      SystemWarning1 = "Discharge takeover counter running:" + String(DischargeCounter2);
    }
    else {
      SystemWarning1 = "NoWarning";
    }
    if (DischargeCounter2 > 600 && DischargeCounter2 < 690) {
      if (String1Current >= -2) {
        Rack1DischargeRelay = true;
      }
      else {
        SystemWarning1 = "Discharge#1 take over is finished due to high load";
        Rack1DischargeRelay = false;
        DischargeCounter2 = 0;
      }
    }
    if (DischargeCounter2 > 690) {
      SystemWarning1 = "Discharge#1 take over is finished due to timeout";
      DischargeCounter2 = 0;
      Rack1DischargeRelay = !(TakeOver1 || TakeOver2);
    }
    if (DischargeCounter2 <= 600) {
      Rack1DischargeRelay = !(TakeOver1 || TakeOver2);
    }
    //Bypass#1 Control
    if (String1Current < 0 && Rack1DischargeRelay && (abs(String1Current) > 20)) {
      Bypass1Relay = true;
      SystemWarning2 = "bypass relay is on due to high current:" + String(String1Current) + ">" + String(20);
    }
    else if (String1Current > 0 && Rack1ChargeRelay && (abs(String1Current) > 20)) {
      Bypass1Relay = true;
      SystemWarning2 = "NoWarning";
    }
    else {
      Bypass1Relay = false;
      SystemWarning2 = "NoWarning";
    }
    if (BoardVersion) {
      if (!FORCE) {
        digitalWrite(18, Rack1ChargeRelay);
        digitalWrite(19, Rack1DischargeRelay);
        digitalWrite(21, Bypass1Relay);
        digitalWrite(22, Fan1);
        digitalWrite(5, prechargeStatus);
      }
      else {
        //        SystemWarning = "CAUTION !!Breakers are forced!!!!!!!!!!!";
        digitalWrite(18, ForcedRack1ChargeRelay);
        digitalWrite(19, ForcedRack1DischargeRelay);
        digitalWrite(21, ForcedBypass1Relay);
        digitalWrite(22, ForcedFan1);
        digitalWrite(5, ForcedPrecharge);
      }
    }
    else {
      if (!FORCE) {
        SystemWarning3 = "NoWarning";

        digitalWrite(5, Rack1ChargeRelay);
        digitalWrite(19, Rack1DischargeRelay);
        digitalWrite(15, Bypass1Relay);
        digitalWrite(26, Fan1);
        digitalWrite(18, prechargeStatus);
      }
      else {
        SystemWarning3 = "CAUTION !!Breakers are forced!!!!!!!!!!!";
        digitalWrite(5, ForcedRack1ChargeRelay);
        digitalWrite(19, ForcedRack1DischargeRelay);
        digitalWrite(15, ForcedBypass1Relay);
        digitalWrite(26, ForcedFan1);
        digitalWrite(18, ForcedPrecharge);
      }
    }
    PreviousMillisBreaker = millis();
    delay(250);
  }
}


void ANALOGTASK_CODE( void * pvParameters ) {
  TaskArray[3] = true;
  int PrechargeCounter2024 = 0;
  int CurrentMillis = 0;
  int PreviousMillis = 0;

  I2C_0 .begin(SDA_0, SCL_0);
  delay(500);

  for (;;) {
    ANALOG_PULSE++;
    result = i2c_search(0x48);

    delay(100);

    if (result == 1)
    {
      if (BoardVersion == 0) {
        SystemAlarm3 = "NoAlarm";
        send_config(1);
        delay(250);
        prechargeVolt = (adc_value);
        // Serial.println("Config#1:" + String(adc_value));
        send_config(2);
        delay(250);
        temp1 = (adc_value) * 100;
        //     Serial.println("Config#2:" + String(adc_value));
        send_config(3);
        delay(250);
        temp2 = adc_value;
        //    Serial.println("Config#3:" + String(adc_value));
      }

      else {
        SystemAlarm3 = "NoAlarm";
        send_config(1);
        delay(250);
        //  Serial.println("Config#1:" + String(adc_value));
        temp1 = (adc_value) * 100;
        send_config(2);
        delay(250);
        //  Serial.println("Config#2:" + String(adc_value));
        prechargeVolt = (adc_value) * 100;
        send_config(3);
        delay(250);
        //   Serial.println("Config#3:" + String(adc_value));
        temp2 = adc_value * 100;
      }


      if (prechargeVolt > 0.2) {
        // Serial.println("Precharge voltage is good!");
        PrechargeCounter2024++;
        //Serial.println("Precharge counter:"+String(PrechargeCounter2024));
        if (PrechargeCounter2024 > 20)
        {
          prechargeStatus = false;
        }
      }
      else {
        // Serial.println("Precharge voltage is low:" + String(prechargeVolt));
      }
    }

    else {
      SystemAlarm3 = "I2C is failed";
    }
  }

}


void BT_CODE( void * pvParameters ) {

  String message = "";
  int param_start = 0;
  int param_end = 0;
  String BTProcessor;
  int param_start2 = 0;
  int param_end2 = 0;
  String BTProcessor2;

  String BT_STRING;
  String BT_REMAINING;
  int ModuleNumber = 0;



  String BT_STATION = "EN-SUB-" + SerialNumber + "-#" + String(SubID);

  BluetoothSerial SerialBT;
  SerialBT.begin(BT_STATION.c_str()); //Bluetooth device name
  TaskArray[5] = true;

  for (;;) {

    if (SerialBT.available()) {
      char incomingChar = SerialBT.read();
      if (incomingChar != '\n') {
        message += String(incomingChar);
      }
      else {
        message = "";
      }


      if (message != "") {

      }

      //SET ID NUMBER ////////////////////////////////
      param_start2 = message.indexOf("SETID");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        SubID = BTProcessor2.toInt();
        SerialBT.println("ID:" + String(SubID));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("SubID", SubID);
        preferences.end();
        SerialBT.println("System will restart");
        delay(1000);

        ESP.restart();
      }


      //GET ID NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETID");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("ID:" + String(SubID));
        message = "";
      }


      //GET ModuleSize NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETMDS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("ModuleSize:" + String(ModuleSize));
        message = "";
      }

      //SET ModuleSize NUMBER ////////////////////////////////
      param_start2 = message.indexOf("SETMDS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        ModuleSize = BTProcessor2.toInt();
        SerialBT.println("ModuleSize:" + String(ModuleSize));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("MDS", ModuleSize);
        preferences.end();
      }



      //SET MaximumStringVolt  ////////////////////////////////
      param_start2 = message.indexOf("SETMSTV");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 7, param_end2);
        MaximumStringVolt = BTProcessor2.toInt();
        SerialBT.println("MaximumStringVolt:" + String(MaximumStringVolt));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("MSTV", MaximumStringVolt);
        preferences.end();
      }


      //GET MaximumStringVolt NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETMSTV");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("MaximumStringVolt:" + String(MaximumStringVolt));
        message = "";
      }




      //SET InverterType  ////////////////////////////////
      param_start2 = message.indexOf("SETIT");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        InverterType = BTProcessor2.toInt();
        SerialBT.println("InverterType:" + String(InverterType));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("IT", InverterType);
        preferences.end();

      }

      //GET InverterType  ////////////////////////////////
      param_start2 = message.indexOf("GETIT");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("InverterType:" + String(InverterType));
        message = "";
      }

      //GET fIRMWARE NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETFW");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("FW:" + FirmwareVer);
        message = "";
      }

      //SET SERIAL NUMBER ////////////////////////////////
      param_start2 = message.indexOf("SETSN");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        SerialNumber = BTProcessor2;
        SerialBT.println("SerialNumber:" + String(SerialNumber));
        message = "";

        preferences.begin("my-app", false);
        preferences.putString("SN", SerialNumber);
        preferences.end();
      }

      //get SERIAL ////////////////////////////////
      param_start2 = message.indexOf("GETSN");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println("SerialNumber:" + SerialNumber);
        message = "";
      }

      //get PULSES ////////////////////////////////
      param_start2 = message.indexOf("GETPS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println("ANLG:" + String(ANALOG_PULSE) + "/BRKR:" + String(BREAKER_PULSE) + "/CNBS:" + String(CANBUS_PULSE) + "/MDBS:" + String(MODBUS_PULSE));
        message = "";
      }

      //get voltage setttings ////////////////////////////////
      param_start2 = message.indexOf("GETVS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";
      }

      //SET High Temp Start ////////////////////////////////
      param_start2 = message.indexOf("HTST");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        HighTempAlarmStart = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("HTST", HighTempAlarmStart);
        preferences.end();
      }

      //SET High Temp Stop ////////////////////////////////
      param_start2 = message.indexOf("HTSP");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        HighTempAlarmStop = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("HTSP", HighTempAlarmStop);
        preferences.end();
      }

      //SET High Voltage Start ////////////////////////////////
      param_start2 = message.indexOf("HVST");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        HighVoltageAlarmStart = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("HVST", HighVoltageAlarmStart);
        preferences.end();
      }

      //SET High Voltage Stop ////////////////////////////////
      param_start2 = message.indexOf("HVSP");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        HighVoltageAlarmStop = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("HVSP", HighVoltageAlarmStop);
        preferences.end();
      }


      //SET Low Voltage Start ////////////////////////////////
      param_start2 = message.indexOf("LVST");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        LowVoltageAlarmStart = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("LVST", LowVoltageAlarmStart);
        preferences.end();
      }


      //SET Low Voltage Stop ////////////////////////////////
      param_start2 = message.indexOf("LVSP");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        LowVoltageAlarmStop = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("LVSP", LowVoltageAlarmStop);
        preferences.end();
      }

      //SET FORCE ////////////////////////////////
      param_start2 = message.indexOf("FC");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        FORCE = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";

        ForcedRack1DischargeRelay = false;
        ForcedRack1ChargeRelay = false;
        ForcedBypass1Relay = false;
        ForcedFan1 = false;
        ForcedPrecharge   = false;
      }

      //SET CHARGE ////////////////////////////////
      param_start2 = message.indexOf("CH");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedRack1ChargeRelay = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }


      //SET DISCHARGE ////////////////////////////////
      param_start2 = message.indexOf("DC");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedRack1DischargeRelay = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }

      //SET BYPASS
      ////////////////////////////////
      param_start2 = message.indexOf("BP");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedBypass1Relay = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }


      //SET FAN
      ////////////////////////////////
      param_start2 = message.indexOf("FN");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedFan1 = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }



      //SET PRECHARGE
      ////////////////////////////////
      param_start2 = message.indexOf("PC");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedPrecharge = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }

      //SET ALL OPEN
      ////////////////////////////////
      param_start2 = message.indexOf("ALLON");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        ForcedRack1ChargeRelay = true;
        ForcedRack1DischargeRelay = true;
        ForcedBypass1Relay = true;
        ForcedFan1 = true;
        ForcedPrecharge = true;
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }



      //SET ALL close
      ////////////////////////////////
      param_start2 = message.indexOf("ALLOFF");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        ForcedRack1ChargeRelay = false;
        ForcedRack1DischargeRelay = false;
        ForcedBypass1Relay = false;
        ForcedFan1 = false;
        ForcedPrecharge = false;
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }

      //SET MAX CURRENT//////////

      param_start2 = message.indexOf("SETMAXCURRENT");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 13, param_end2);
        MaxCurrent = BTProcessor2.toInt();
        SerialBT.println("MaxCurrent:" + String(MaxCurrent));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("MaxCurrent", MaxCurrent);
        preferences.end();
      }


      //GET MAXCURRENT ////////////////////////////////
      param_start2 = message.indexOf("GETMAXCURRENT");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("MAX CURRENT:" + String(MaxCurrent));
        message = "";
      }




      //set cansim ////////////////////////////////
      param_start2 = message.indexOf("SETCANSIM");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 9, param_end2);
        CanSim = BTProcessor2.toInt();
        SerialBT.println("CanSim:" + String(CanSim));
        message = "";
      }

      //GET cansim ////////////////////////////////
      param_start2 = message.indexOf("GETCANSIM");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("CanSim:" + String(CanSim));
        message = "";
      }

      //RESET////////////////////////////////
      param_start2 = message.indexOf("RESET");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println("System will reset:" + String(CanSim));
        delay(500);
        message = "";
        ESP.restart();
      }




      //GET MODULE DATA//////////

      param_start2 = message.indexOf("GETMOD");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        ModuleNumber = BTProcessor2.toInt();
        SerialBT.println("ID" + String(ModuleNumber) + "/" + "Voltage:" + String(TerminalVoltageArray[ModuleNumber - 1]) + "/" +
                         "Current:" + String(TerminalCurrentArray[ModuleNumber - 1]) + "/" +
                         "Temperature:" + String(TerminalTempArray[ModuleNumber - 1]) + "/" +
                         "SOC:" + String(TerminalSOCArray[ModuleNumber - 1]) + "/" +
                         "Charge:" + String(ChargeStatusArray[ModuleNumber - 1]) + "/" +
                         "Discharge:" + String(DischargeStatusArray[ModuleNumber - 1]) + "/" +
                         "Alarm:" + String(AlarmStatusArray[ModuleNumber - 1]) + "/" +
                         "Heartbeat:" + String(Heartbeat) + "#");
        message = "";
      }


      //GET string DATA//////////
      param_start2 = message.indexOf("GETSTRING");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println("ChargeRack#:" + String(Rack1ChargeRelay) + "/" +
                         "DischargeRack#:" + String(Rack1DischargeRelay) + "/" +
                         "BypassRack#:" + String(Bypass1Relay) + "/" +
                         "Fan#:" + String(Fan1) + "/" +
                         "Precharge#:" + String(prechargeStatus) + "/" +
                         "String1MaxCell#:" + String(String1MaxCell) + "/" +
                         "String1Temperature#:" + String(temp1) + "/" +
                         "String1MinCell#:" + String(String1MinCell) + "/" +
                         "String1SOC#:" + String(String1SOC) + "/" +
                         "String1Voltage#:" + String(String1Voltage) + "/" +
                         "String1Current#:" + String(String1Current) + "/" +
                         "I2C Working Status#:" + String(result) + "/" +
                         "I2C (Temp#1):" + String(temp1) + "/" +
                         "I2C (Temp#2):" + String(temp2) + "/" +
                         "I2C (PrechargeVolt):" + String(prechargeVolt) + "/" +
                         "AH#:" + String(ANALOG_PULSE) + "#" );
        message = "";
      }


      else {

        //SerialBT.println("Unidentified Command Received");

      }

    }
  }
}

void MODBUSTASK_CODE( void * pvParameters ) {
  TaskArray[2] = true;
  Modbus slave(SubID, Serial2, GEN_RE);
  baudbaud = ModbusBaudRate;
  slave.start();

  for (;;) {

    MODBUS_PULSE++;

    if (temp1 > HighTempAlarmStart) {
      Fan1 = true;
    }
    if (temp1 < HighTempAlarmStop) {
      Fan1 = false;
    }

    for (int i = 0; i < ModuleSize; i++) {
      MODBUSARRAY[i + ModuleSize * 0] = TerminalVoltageArray[i] * 10;
      MODBUSARRAY[i + ModuleSize * 1] = TerminalCurrentArray[i] * 10;
      MODBUSARRAY[i + ModuleSize * 2] = TerminalTempArray[i] * 10;
      MODBUSARRAY[i + ModuleSize * 3] = MaxCellArray[i] * 100;
      MODBUSARRAY[i + ModuleSize * 4] = MinCellArray[i] * 100;
      MODBUSARRAY[i + ModuleSize * 5] = TerminalSOCArray[i] * 10;
      MODBUSARRAY[i + ModuleSize * 6] = ChargeStatusArray[i];
      MODBUSARRAY[i + ModuleSize * 7] = DischargeStatusArray[i];
    }

    MODBUSARRAY[ModuleSize * 8 + 0] = Rack1ChargeRelay;
    MODBUSARRAY[ModuleSize * 8 + 1] = Rack1DischargeRelay;
    MODBUSARRAY[ModuleSize * 8 + 2] = Bypass1Relay;
    MODBUSARRAY[ModuleSize * 8 + 3] = prechargeStatus;
    MODBUSARRAY[ModuleSize * 8 + 4] = Fan1;
    MODBUSARRAY[ModuleSize * 8 + 5] = temp1;
    MODBUSARRAY[ModuleSize * 8 + 6] = Heartbeat;
    MODBUSARRAY[ModuleSize * 8 + 7] = String1Voltage * 10;
    MODBUSARRAY[ModuleSize * 8 + 8] = String1Current * 10;
    MODBUSARRAY[ModuleSize * 8 + 9] = String1SOC * 10;
    MODBUSARRAY[ModuleSize * 9 + 0] = String1Temp * 10;
    MODBUSARRAY[ModuleSize * 9 + 1] = (String1MaxCell - String1MinCell) * 100;




    MODBUSARRAY[ModuleSize * 9 + 2] = TerminalVoltageArray[ModuleSize] * 10;
    MODBUSARRAY[ModuleSize * 9 + 3] = TerminalCurrentArray[ModuleSize] * 10;
    MODBUSARRAY[ModuleSize * 9 + 4] = TerminalTempArray[ModuleSize] * 10;
    MODBUSARRAY[ModuleSize * 9 + 5] = TerminalSOCArray[ModuleSize] * 10;

    slave.poll( MODBUSARRAY, ModuleSize * 10);
    delay(100);
  }
}



void SERIALMONITOR_CODE( void * pvParameters ) {
  TaskArray[6] = true;


  for (;;) {
    recvOneChar();
    showNewData();
    ContinuousOperations();
    // Settings();
    delay(1000);
  }
}


void loop() {

}


void recvOneChar() {
  if (Serial.available() > 0) {
    receivedString = Serial.readStringUntil('\n');
    newData = true;
    delay(100);
  }
}

void showNewData() {
  if (newData == true) {

    Serial.println(receivedString);
    analyseNewData();
    newData = false;
    delay(100);
  }
}
void analyseNewData() {


  int Processor1, Processor2;
  int ModuleNumber;


  if (receivedString == "-SHOWSUMMARY") {

    Serial.println();
    Serial.println("   ---   SYSTEM SUMMARY START   ---   ");
    Serial.println();
    Serial.println("         HardwareSerial:" + String(HardwareSerial));
    Serial.println("         BoardVersion:" + String(BoardVersion));
    Serial.println("         SubID:" + String(SubID) );
    Serial.println("         ModuleSize:" + String(ModuleSize));
    Serial.println("         SerialNumber:" + String(SerialNumber));
    Serial.println("         FirmwareVersion:" + String(FirmwareVer));
    Serial.println("         Canbus BaudRate:" + String(CanbusBaudRate) + "kb");
    Serial.println("         Modbus BaudRate:" + String(ModbusBaudRate) + "kb");
    Serial.println("         Inverter Type:" + String(InverterType));
    Serial.println("         Simulation Status:" + String(CanSim));
    Serial.println("         Maximum Module Volt:" + String(MaximumModuleVolt) + "V");
    Serial.println("         Maximum String Volt:" + String(MaximumStringVolt) + "V");
    Serial.println();
    Serial.println("         HighVoltageAlarmStart:" + String(HighVoltageAlarmStart) + "V");
    Serial.println("         HighVoltageAlarmStop:" + String(HighVoltageAlarmStop) + "V");
    Serial.println("         LowVoltageAlarmStart:" + String(LowVoltageAlarmStart) + "V");
    Serial.println("         LowVoltageAlarmStop:" + String(LowVoltageAlarmStop) + "V");
    Serial.println("         HighTempAlarmStart:" + String(HighTempAlarmStart) + "C");
    Serial.println("         HighTempAlarmStop:" + String(HighTempAlarmStop) + "C");

    Serial.println("         SOC Lock Status:" + String(SOCLockEnable));
    Serial.println("         SOC Lock Value:" + String(SOCLockValue));
    Serial.println();

    Serial.println("   ---   SYSTEM SUMMARY END  ---   ");
    Serial.println();

  }

  else if (receivedString == "-SHOWRT") {
    Serial.println();
    Serial.println("   ---   REALTIME SYSTEM DATA START  ---   ");
    Serial.println();

    for (int i = 0; i < ( ModuleSize + 1); i++) {
      Serial.println("");
      Serial.println("ID:" + String(i + 1) + "Voltage:" + String(TerminalVoltageArray[i]));
      Serial.println("ID:" + String(i + 1) + "Current:" + String(TerminalCurrentArray[i]));
      Serial.println("ID:" + String(i + 1) + "Temperature:" + String(TerminalTempArray[i]));
      Serial.println("ID:" + String(i + 1) + "SOC:" + String(TerminalSOCArray[i]));
      Serial.println("ID:" + String(i + 1) + "Charge:" + String(ChargeStatusArray[i]));
      Serial.println("ID:" + String(i + 1) + "Discharge:" + String(DischargeStatusArray[i]));
      Serial.println("ID:" + String(i + 1) + "Alarm:" + String(AlarmStatusArray[i]));
      Serial.println("HeartBeat:" + String(Heartbeat));
      Serial.println("----------");
    }
    Serial.println("   ---   REALTIME SYSTEM DATA  END  ---   ");
    Serial.println();
  }

  else if (receivedString == "-SHOWTASK") {

    Serial.println();
    Serial.println("   ---   TASK SUMMARIES START  ---   ");
    Serial.println();

    for (int i = 0; i < 7; i++) {
      Serial.println("       TaskName:" + TaskStringArray[i] + " Status:" + String(TaskArray[i]));
    }
    Serial.println();
    Serial.println("   ---   TASK SUMMARIES END  ---   ");
    Serial.println();
  }

  else if (receivedString == "-SHOWSTRING") {

    Serial.println();
    Serial.println("   ---   SUBSTRING LIST START  ---   ");
    Serial.println();
    Serial.println("         ChargeRack#1:" + String(Rack1ChargeRelay) );
    Serial.println("         DischargeRack#1:" + String(Rack1DischargeRelay));
    Serial.println("         BypassRack#1:" + String(Bypass1Relay));
    Serial.println("         Fan#1:" + String(Fan1));
    Serial.println("         Precharge#1:" + String(prechargeStatus));
    Serial.println("         Inverter String SOC:" + String(String1SOC));
    Serial.println("         Inverter String Voltage:" + String(String1Voltage));
    Serial.println("         Inverter String Current:" + String(String1Current));


    Serial.println("         String1MaxCell:" + String(String1MaxCell));
    Serial.println("         String1Temperature:" + String(temp1));
    Serial.println("         String1MinCell:" + String(String1MinCell));
    Serial.println("         String1SOC:" + String(String1SOC));
    Serial.println("         String1Voltage:" + String(String1Voltage));
    Serial.println("         String1Current:" + String(String1Current));
    Serial.println("         MaxCurrent:" + String(MaxCurrent));

    Serial.println("         SystemAlarm #1:" + String(SystemAlarm1));
    Serial.println("         SystemAlarm #2:" + String(SystemAlarm2));
    Serial.println("         SystemAlarm #3:" + String(SystemAlarm3));
    Serial.println("         SystemAlarm #4:" + String(SystemAlarm4));
    Serial.println("         SystemWarning #1:" + String(SystemWarning1));
    Serial.println("         SystemWarning #2:" + String(SystemWarning2));
    Serial.println("         SystemWarning #3:" + String(SystemWarning3));






    Serial.println("         I2C Working Status:" + String(result));
    Serial.println("         I2C1#:" + String(temp1));
    Serial.println("         I2C2#:" + String(temp2));
    Serial.println("         I2C3#:" + String(prechargeVolt));
    Serial.println("         ANALOG_HEARTBEAT#:" + String(ANALOG_PULSE));
    Serial.println("         FORCE Status#:" + String(FORCE));

    Serial.println();
    Serial.println("   ---   SUBSTRING LIST END  ---   ");
  }

  else if (receivedString == "-RUNRT") {
    Serial.println("   ---   RUNRT ACK  ---   ");
    RUNRT = true;
  }

  else if (receivedString == "-STOPRT") {
    Serial.println("   ---   STOPRT ACK  ---   ");
    RUNRT = false;
  }

  else if (receivedString == "- ") {
    Serial.println("   ---   RUNSTRING ACK  ---   ");
    RUNSTRING = true;
  }

  else if (receivedString == "-STOPSTRING") {
    Serial.println("   ---   STOPSTRING ACK  ---   ");
    RUNSTRING = false;
  }



  else if (receivedString == "-STOPSTRING") {
    Serial.println("   ---   STOPSTRING ACK  ---   ");
    RUNSTRING = false;
  }

  else if (receivedString.indexOf("GETMOD") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("GETMOD") + 6, receivedString.indexOf("#"));
    ModuleNumber = SerialProcessor.toInt();
    Serial.println("ID" + String(ModuleNumber) + "/" + "Voltage:" + String(TerminalVoltageArray[ModuleNumber - 1]) + "/" +
                   "Current:" + String(TerminalCurrentArray[ModuleNumber - 1]) + "/" +
                   "Temperature:" + String(TerminalTempArray[ModuleNumber - 1]) + "/" +
                   "SOC:" + String(TerminalSOCArray[ModuleNumber - 1]) + "/" +
                   "Charge:" + String(ChargeStatusArray[ModuleNumber - 1]) + "/" +
                   "Discharge:" + String(DischargeStatusArray[ModuleNumber - 1]) + "/" +
                   "Alarm:" + String(AlarmStatusArray[ModuleNumber - 1]) + "/" +
                   "Heartbeat:" + String(Heartbeat) + "#");
    receivedString = "";

  }

  else if (receivedString.indexOf("SETID") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETID") + 5, receivedString.indexOf("#"));
    SubID = SerialProcessor.toInt();
    Serial.println("ID:" + String(SubID));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("SubID", SubID);
    preferences.end();
    Serial.println("System will restart");
    delay(1000);
    ESP.restart();
  }

  else if (receivedString.indexOf("SETMDS") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETMDS") + 6, receivedString.indexOf("#"));
    ModuleSize = SerialProcessor.toInt();
    Serial.println("ModuleSize:" + String(ModuleSize));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("MDS", ModuleSize);
    preferences.end();
  }

  else if (receivedString.indexOf("SETMSTV") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETMSTV") + 7, receivedString.indexOf("#"));
    MaximumStringVolt = SerialProcessor.toInt();
    Serial.println("MaximumStringVolt:" + String(MaximumStringVolt));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("MSTV", MaximumStringVolt);
    preferences.end();
  }

  else if (receivedString.indexOf("SETIT") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETIT") + 5, receivedString.indexOf("#"));
    InverterType = SerialProcessor.toInt();
    Serial.println("InverterType:" + String(InverterType));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("IT", InverterType);
    preferences.end();
  }

  else if (receivedString.indexOf("SETSN") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETSN") + 5, receivedString.indexOf("#"));
    SerialNumber = SerialProcessor.toInt();
    Serial.println("SerialNumber:" + String(SerialNumber));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putString("SN", SerialNumber);
    preferences.end();
  }


  else if (receivedString.indexOf("HTST") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("HTST") + 4, receivedString.indexOf("#"));
    HighTempAlarmStart = SerialProcessor.toInt();
    Serial.println("HighTempAlarmStart:" + String(HighTempAlarmStart));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("HTST", HighTempAlarmStart);
    preferences.end();
  }

  else if (receivedString.indexOf("HTSP") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("HTSP") + 4, receivedString.indexOf("#"));
    HighTempAlarmStop = SerialProcessor.toInt();
    Serial.println("HighTempAlarmStop:" + String(HighTempAlarmStop));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("HTSP", HighTempAlarmStop);
    preferences.end();
  }


  else if (receivedString.indexOf("HVST") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("HVST") + 4, receivedString.indexOf("#"));
    HighVoltageAlarmStart = SerialProcessor.toInt();
    Serial.println("HighVoltageAlarmStart:" + String(HighVoltageAlarmStart));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("HVST", HighVoltageAlarmStart);
    preferences.end();
  }

  else if (receivedString.indexOf("HVSP") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("HVSP") + 4, receivedString.indexOf("#"));
    HighVoltageAlarmStop = SerialProcessor.toInt();
    Serial.println("HighVoltageAlarmStop:" + String(HighVoltageAlarmStop));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("HVSP", HighVoltageAlarmStop);
    preferences.end();
  }

  else if (receivedString.indexOf("LVST") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("LVST") + 4, receivedString.indexOf("#"));
    LowVoltageAlarmStart = SerialProcessor.toInt();
    Serial.println("LowVoltageAlarmStart:" + String(LowVoltageAlarmStart));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("LVST", LowVoltageAlarmStart);
    preferences.end();
  }

  else if (receivedString.indexOf("LVSP") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("LVSP") + 4, receivedString.indexOf("#"));
    LowVoltageAlarmStop = SerialProcessor.toInt();
    Serial.println("LowVoltageAlarmStop:" + String(LowVoltageAlarmStop));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("LVSP", LowVoltageAlarmStop);
    preferences.end();
  }

  else if (receivedString.indexOf("SETMAXCURRENT") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETMAXCURRENT") + 13, receivedString.indexOf("#"));
    MaxCurrent = SerialProcessor.toInt();
    Serial.println("MaxCurrent:" + String(MaxCurrent));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("MaxCurrent", MaxCurrent);
    preferences.end();
  }

  else if (receivedString.indexOf("SETCBR") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETCBR") + 6, receivedString.indexOf("#"));
    CanbusBaudRate = SerialProcessor.toInt();
    Serial.println("CanbusBaudRate:" + String(CanbusBaudRate));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("CBR", CanbusBaudRate);
    preferences.end();
  }

  else if (receivedString.indexOf("SETMBR") != -1 && receivedString.indexOf("#") != -1) {

    SerialProcessor = receivedString.substring(receivedString.indexOf("SETMBR") + 6, receivedString.indexOf("#"));
    ModbusBaudRate = SerialProcessor.toInt();
    Serial.println("ModbusBaudRate:" + String(ModbusBaudRate));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("MBR", ModbusBaudRate);
    preferences.end();
  }


  else if (receivedString.indexOf("CANSIM") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("CANSIM") + 6, receivedString.indexOf("#"));
    CanSim = SerialProcessor.toInt();
    Serial.println("CanSim:" + String(CanSim));
    receivedString = "";
  }

  else if (receivedString.indexOf("FC") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("FC") + 2, receivedString.indexOf("#"));
    FORCE = SerialProcessor.toInt();
    Serial.println("FORCE:" + String(FORCE));
    receivedString = "";
    ForcedRack1DischargeRelay = false;
    ForcedRack1ChargeRelay = false;
    ForcedBypass1Relay = false;
    ForcedFan1 = false;
    ForcedPrecharge   = false;
  }


  else if (receivedString.indexOf("CH") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("CH") + 2, receivedString.indexOf("#"));
    ForcedRack1ChargeRelay = SerialProcessor.toInt();
    Serial.println("ForcedRack1ChargeRelay:" + String(ForcedRack1ChargeRelay));
    receivedString = "";
  }


  else if (receivedString.indexOf("DS") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("DS") + 2, receivedString.indexOf("#"));
    ForcedRack1DischargeRelay = SerialProcessor.toInt();
    Serial.println("ForcedRack1DischargeRelay:" + String(ForcedRack1DischargeRelay));
    receivedString = "";
  }


  else if (receivedString.indexOf("BP") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("BP") + 2, receivedString.indexOf("#"));
    ForcedBypass1Relay = SerialProcessor.toInt();
    Serial.println("ForcedBypass1Relay:" + String(ForcedBypass1Relay));
    receivedString = "";
  }


  else if (receivedString.indexOf("FN") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("FN") + 2, receivedString.indexOf("#"));
    ForcedFan1 = SerialProcessor.toInt();
    Serial.println("ForcedFan1:" + String(ForcedFan1));
    receivedString = "";
  }


  else if (receivedString.indexOf("PC") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("PC") + 2, receivedString.indexOf("#"));
    ForcedPrecharge = SerialProcessor.toInt();
    Serial.println("ForcedPrecharge:" + String(ForcedPrecharge));
    receivedString = "";
  }

  else if (receivedString.indexOf("ALLON") != -1 && receivedString.indexOf("#") != -1) {
    Serial.println("ALLON:" + String(1));
    receivedString = "";
    ForcedRack1ChargeRelay = true;
    ForcedRack1DischargeRelay = true;
    ForcedBypass1Relay = true;
    ForcedFan1 = true;
    ForcedPrecharge = true;
  }


  else if (receivedString.indexOf("ALLOFF") != -1 && receivedString.indexOf("#") != -1) {
    Serial.println("ALLOFF:" + String(1));
    receivedString = "";
    ForcedRack1ChargeRelay = false;
    ForcedRack1DischargeRelay = false;
    ForcedBypass1Relay = false;
    ForcedFan1 = false;
    ForcedPrecharge = false;
  }



  else if (receivedString.indexOf("SETSLE") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETSLE") + 6, receivedString.indexOf("#"));
    SOCLockEnable = SerialProcessor.toInt();
    Serial.println("SOCLockEnable:" + String(SOCLockEnable));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("SLE", SOCLockEnable);
    preferences.end();
  }


  else if (receivedString.indexOf("SETSLV") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETSLV") + 6, receivedString.indexOf("#"));
    SOCLockValue = SerialProcessor.toInt();
    Serial.println("SOCLockValue:" + String(SOCLockValue));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("SLV", SOCLockValue);
    preferences.end();
  }

  else if (receivedString.indexOf("SETBV") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("SETBV") + 5, receivedString.indexOf("#"));
    BoardVersion = SerialProcessor.toInt();
    Serial.println("BoardVersion:" + String(BoardVersion));
    receivedString = "";
    preferences.begin("my-app", false);
    preferences.putInt("BV", BoardVersion);
    preferences.end();

    Serial.println("SYSTEM WILL RESET!");
    delay(1000);
    ESP.restart();
  }
  else if (receivedString.indexOf("GETBV") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("GETBV") + 5, receivedString.indexOf("#"));
    Serial.println("BoardVersion:" + String(BoardVersion));
    receivedString = "";
  }
  else if (receivedString.indexOf("RESET") != -1 && receivedString.indexOf("#") != -1) {
    Serial.println("SYSTEM WILL RESET!");
    receivedString = "";
    delay(1000);
    ESP.restart();
  }
  else if (receivedString.indexOf("GETMOD") != -1 && receivedString.indexOf("#") != -1) {
    SerialProcessor = receivedString.substring(receivedString.indexOf("GETMOD") + 6, receivedString.indexOf("#"));
    ModuleNumber = SerialProcessor.toInt();

    Serial.println("ID" + String(ModuleNumber) + "/" + "Voltage:" + String(TerminalVoltageArray[ModuleNumber - 1]) + "/" +
                   "Current:" + String(TerminalCurrentArray[ModuleNumber - 1]) + "/" +
                   "Temperature:" + String(TerminalTempArray[ModuleNumber - 1]) + "/" +
                   "SOC:" + String(TerminalSOCArray[ModuleNumber - 1]) + "/" +
                   "Charge:" + String(ChargeStatusArray[ModuleNumber - 1]) + "/" +
                   "Discharge:" + String(DischargeStatusArray[ModuleNumber - 1]) + "/" +
                   "Alarm:" + String(AlarmStatusArray[ModuleNumber - 1]) + "/" +
                   "Heartbeat:" + String(Heartbeat) + "#");
  }
  else if (receivedString.indexOf("GETSTRING") != -1 && receivedString.indexOf("#") != -1) {
    Serial.println("ChargeRack#:" + String(Rack1ChargeRelay) + "/" +
                   "DischargeRack#:" + String(Rack1DischargeRelay) + "/" +
                   "BypassRack#:" + String(Bypass1Relay) + "/" +
                   "Fan#:" + String(Fan1) + "/" +
                   "Precharge#:" + String(prechargeStatus) + "/" +
                   "String1MaxCell#:" + String(String1MaxCell) + "/" +
                   "String1Temperature#:" + String(temp1) + "/" +
                   "String1MinCell#:" + String(String1MinCell) + "/" +
                   "String1SOC#:" + String(String1SOC) + "/" +
                   "String1Voltage#:" + String(String1Voltage) + "/" +
                   "String1Current#:" + String(String1Current) + "/" +
                   "I2C Working Status#:" + String(result) + "/" +
                   "I2C1#:" + String(temp1) + "/" +
                   "I2C2#:" + String(temp2) + "/" +
                   "I2C3#:" + String(prechargeVolt) + "/" +
                   "AH#:" + String(ANALOG_PULSE) + "#" );
  }

  else if  (receivedString.indexOf("-SHOWSERIALNO") != -1) {
    Serial.println("HardwareSerialNumber:" + HardwareSerial);
  }









  else
  {
    Serial.println("Invalid Command Received!, please try commands below:");
    Serial.println("1.   -SHOWSTRING");
    Serial.println("2.   -SHOWSUMMARY");
    Serial.println("3.   -SHOWTASK");
    Serial.println("4.   -SHOWRT");
    Serial.println("5.   -RUNRT");
    Serial.println("6.   -STOPRT");
    Serial.println("7.   -RUNSTRING");
    Serial.println("8.   -STOPSTRING");
    Serial.println("9.   SETID___#");
    Serial.println("10.  SETMDS___#");
    Serial.println("11.  SETMSTV___#");
    Serial.println("12.  SETIT___#");
    Serial.println("13.  SETSN___#");
    Serial.println("14.  HTST___#");
    Serial.println("15.  HTSP___#");
    Serial.println("16.  HVST___#");
    Serial.println("17.  HVSP___");
    Serial.println("18.  LVST___");
    Serial.println("19.  LVSP___");
    Serial.println("20.  GETCURRENT___#");
    Serial.println("21.  FC___#");
    Serial.println("22.  CH___#");
    Serial.println("23.  DS___#");
    Serial.println("24.  BP___#");
    Serial.println("25.  FN___#");
    Serial.println("26.  PC___#");
    Serial.println("27.  ALLON#");
    Serial.println("28.  ALLOFF#");
    Serial.println("29.  CBR___#");
    Serial.println("30.  MBR___#");
    Serial.println("31.  CANSIM___#");
  }

}
void ContinuousOperations() {
  if (RUNRT) {
    Serial.println();
    Serial.println("   ---   REALTIME SYSTEM DATA START  ---   ");
    Serial.println();

    for (int i = 0; i < ( ModuleSize + 1); i++) {
      Serial.println("");
      Serial.println("ID:" + String(i + 1) + "Voltage:" + String(TerminalVoltageArray[i]));
      Serial.println("ID:" + String(i + 1) + "Current:" + String(TerminalCurrentArray[i]));
      Serial.println("ID:" + String(i + 1) + "Temperature:" + String(TerminalTempArray[i]));
      Serial.println("ID:" + String(i + 1) + "SOC:" + String(TerminalSOCArray[i]));
      Serial.println("ID:" + String(i + 1) + "Charge:" + String(ChargeStatusArray[i]));
      Serial.println("ID:" + String(i + 1) + "Discharge:" + String(DischargeStatusArray[i]));
      Serial.println("ID:" + String(i + 1) + "Alarm:" + String(AlarmStatusArray[i]));
      Serial.println("HeartBeat:" + String(Heartbeat));
      Serial.println("----------");
    }
    Serial.println("   ---   REALTIME SYSTEM DATA  END  ---   ");
    Serial.println();
  }
  if (RUNSTRING) {
    Serial.println();
    Serial.println("   ---   SUBSTRING LIST START-TEST ---   ");
    Serial.println();
    Serial.println("         ChargeRack#1:" + String(Rack1ChargeRelay) );
    Serial.println("         DischargeRack#1:" + String(Rack1DischargeRelay));
    Serial.println("         BypassRack#1:" + String(Bypass1Relay));
    Serial.println("         Fan#1:" + String(Fan1));
    Serial.println("         Precharge#1:" + String(prechargeStatus));
    Serial.println("         Inverter String SOC:" + String(String1SOC));
    Serial.println("         Inverter String Voltage:" + String(String1Voltage));
    Serial.println("         Inverter String Current:" + String(String1Current));
    Serial.println("         String1MaxCell:" + String(String1MaxCell));
    Serial.println("         String1MinCell:" + String(String1MinCell));
    Serial.println("         String1SOC:" + String(String1SOC));
    Serial.println("         String1Voltage:" + String(String1Voltage));
    Serial.println("         String1Current:" + String(String1Current));
    Serial.println("         MaxCurrent:" + String(MaxCurrent));
    Serial.println("         I2C Working Status:" + String(result));
    Serial.println("         I2C1#:" + String(temp1));
    Serial.println("         I2C2#:" + String(temp2));
    Serial.println("         I2C3#:" + String(prechargeVolt));
    Serial.println("         ANALOG_HEARTBEAT#:" + String(ANALOG_PULSE));
    Serial.println("         FORCE Status#:" + String(FORCE));
    Serial.println();
    Serial.println("   ---   SUBSTRING LIST END  ---   ");
  }
}



String getHardwareSerial() {
  //:ESP32s3-0CA8D4

  String MCUtype = "ESP32s3" ;
  char ssid[14];
  snprintf(ssid, 14, "%llX", ESP.getEfuseMac());
  // Serial.println(ssid); //
  String ssid1 = ssid ; // convert from char to String
  // Serial.println(ssid1);
  int StrLen = ssid1.length();
  // Serial.printf("Length of the string %i characters \r\n",StrLen);
  String MAC = "";
  int i = 0; while ( i < 6) {
    MAC = MAC + ssid1.substring(StrLen - 2 - 2 * i, StrLen - 2 * i) + ":";
    i++ ;
  }
  MAC = MAC.substring(0, 17); // remove last :
  //Serial.println(MAC);
  String Serialnumber1 = MCUtype + "-" ;
  int j = 3; while ( j < 6) {
    Serialnumber1 = Serialnumber1 + ssid1.substring(StrLen - 2 - 2 * j, StrLen - 2 * j);
    j++ ;
  }
  Serial.println(Serialnumber1);
  return Serialnumber1;
}
