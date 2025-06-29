#include <SPI.h>
#include <SD.h>
#include <FlexCAN_T4.h>
#include "EasyNextionLibrary.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

EasyNex myNex(Serial3);

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;

File myFile;
char fileName[20];

// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
const int chipSelect = BUILTIN_SDCARD; 

int log_counter = 0;
unsigned long startTime = 0;

// VARIABLE DEFINITIONS:
// BMS signals
//0x680
int relay_state_1 = 0;
int relay_state_2 = 0;
int fail_s_status_1 = 0;
int fail_s_status_2 = 0;
int pack_inst_vol_1 = 0;
int pack_inst_vol_2 = 0;
int pack_SOC = 0;

int pack_inst_vol = 0;

//0x681
int input_supply_vol = 0; // Voltage at the 12V input
int pack_current_1 = 0;
int pack_current_2 = 0;

int pack_current = 0;

//0x682
int pack_amphours_1 = 0;
int pack_amphours_2 = 0;
int internal_temp = 0;

int pack_amphours = 0;

//0x683
int high_cell_vol_1 = 0;
int high_cell_vol_2 = 0;
int low_cell_vol_1 = 0;
int low_cell_vol_2 = 0;
int avg_cell_vol_1 = 0;
int avg_cell_vol_2 = 0;

int avg_cell_vol = 0;
int high_cell_vol = 0;
int low_cell_vol = 0;

// Thermistor expansion module
//0x1838F380
// Thermistor buffer...
int thermistor_temps[29] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int avg_therm_temp = 0;
int high_therm_temp = 0;

// MC signals...
//0x051
int ERPM_1 = 0;
int ERPM_2 = 0;
int ERPM_3 = 0;
int ERPM_4 = 0;
int duty_cycle_1 = 0;
int duty_cycle_2 = 0;
int MC_input_voltage_1 = 0;
int MC_input_voltage_2 = 0;

int ERPM = 0;
int duty_cycle = 0;
int MC_input_voltage = 0;

//0x151
int AC_current_1 = 0;
int AC_current_2 = 0;
int DC_current_1 = 0;
int DC_current_2 = 0;

int AC_current = 0;
int DC_current = 0;

//0x251
int controller_temp_1 = 0;
int controller_temp_2 = 0;
int motor_temp_1 = 0;
int motor_temp_2 = 0;
int MC_fault_code = 0;

int controller_temp = 0;
int motor_temp = 0;

void canSniff(const CAN_message_t &msg);

int hexToDec(char hex[]);

void nextionSend();

void createLogFile()
{
  int fileNumber = 1;
  do
  {
    sprintf(fileName, "datalog%d.csv", fileNumber++);
  } while (SD.exists(fileName));
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile)
  {
    Serial.print("Log file created: ");
    Serial.println(fileName);
    //myFile.println("Timestamp(ms), ID, Length, Data[hex]");
  } else
    {
    Serial.println("Error creating log file.");
    }
}


void setup() {
 
  myNex.begin(9600);
  Serial.begin(115200);

  // Initializing SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect))  {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");
  createLogFile();


  // Starting the CANbus
  can2.begin();
  can2.setBaudRate(500000);
  Serial.println("CANbus started...");

  startTime = millis(); 

}


void loop() {
  // Saving the log file every 100 messages and writing to the display
  if (log_counter == 100) {
    myFile.flush();
    log_counter = 0;
    Serial.println("Message checkpoint...");

    // UPDATING THE DISPLAY WITH CURRENT VALUES...
    myNex.writeStr("MCTemp.txt", controller_temp/10);
    myNex.writeStr("motorTemp.txt", motor_temp/10);
    Serial.println(controller_temp/10);
    Serial.println(motor_temp/10);
    
    if (ERPM == 0) {
      myNex.writeStr("speed.txt", 0);
    }
    else {
      myNex.writeStr("speed.txt", (65535 - ERPM)/503);
    }

    myNex.writeNum("battery.val", pack_SOC/2);
    myNex.writeStr("soc.txt", pack_SOC/2);


    avg_therm_temp = 0;
    high_therm_temp = 0;
    for (int i = 0; i < 29; i++) {
      if (thermistor_temps[i] > high_therm_temp) {
        high_therm_temp = thermistor_temps[i];
      }
      avg_therm_temp += thermistor_temps[i];
    }
    avg_therm_temp = avg_therm_temp/27;

    myNex.writeStr("cellTemp.txt", avg_therm_temp);
    myNex.writeStr("highCellTemp.txt", high_therm_temp);

    myFile.print("0x1839F380, "); myFile.print("Time, "); myFile.print(millis() - startTime); myFile.print(", ");
    myFile.print("Highest Cell Temp, "); myFile.print(high_therm_temp); myFile.print(", ");
    myFile.print("Average Cell Temp, "); myFile.print(avg_therm_temp); myFile.println();

    myNex.writeStr("maxTorque.txt", DC_current/2);

  }

  if ( can2.read(msg) ) {
    log_counter += 1;

    // Address 0x051
    if (msg.id == 81) {
      ERPM_1 = msg.buf[0];
      ERPM_2 = msg.buf[1];
      ERPM_3 = msg.buf[2];
      ERPM_4 = msg.buf[3];
      duty_cycle_1 = msg.buf[4];
      duty_cycle_2 = msg.buf[5];
      MC_input_voltage_1 = msg.buf[6];
      MC_input_voltage_2 = msg.buf[7];

      ERPM = concat(ERPM_3, ERPM_4);
      duty_cycle = concat(duty_cycle_1, duty_cycle_2);
      MC_input_voltage = concat(MC_input_voltage_1, MC_input_voltage_2);

      myFile.print("0x051, "); myFile.print("Time, "); myFile.print(millis() - startTime); myFile.print(", ");
      myFile.print("ERPM, "); myFile.print(ERPM); myFile.print(", ");
      myFile.print("MC Input Voltage, "); myFile.print(MC_input_voltage); myFile.println();

    }

    // Address 0x151
    else if (msg.id == 337) {
      AC_current_1 = msg.buf[0];
      AC_current_2 = msg.buf[1];
      DC_current_1 = msg.buf[2];
      DC_current_2 = msg.buf[3];

      AC_current = concat(AC_current_1, AC_current_2);
      DC_current = concat(DC_current_1, DC_current_2);

      myFile.print("0x151, "); myFile.print("Time, "); myFile.print(millis() - startTime); myFile.print(", ");
      myFile.print("AC Current, "); myFile.print(AC_current); myFile.print(", ");
      myFile.print("DC Current, "); myFile.print(DC_current); myFile.println();
      
    }

    // Address 0x251
    else if (msg.id == 593) {
      controller_temp_1 = msg.buf[0];
      controller_temp_2 = msg.buf[1];
      motor_temp_1 = msg.buf[2];
      motor_temp_2 = msg.buf[3];
      MC_fault_code = msg.buf[4];

      controller_temp = concat(controller_temp_1, controller_temp_2);
      motor_temp = concat(motor_temp_1, motor_temp_2);

      myFile.print("0x251, "); myFile.print("Time, "); myFile.print(millis() - startTime); myFile.print(", ");
      myFile.print("Controller Temp, "); myFile.print(controller_temp); myFile.print(", ");
      myFile.print("Motor Temp, "); myFile.print(motor_temp); myFile.print(", ");
      myFile.print("Fault Code, "); myFile.print(MC_fault_code); myFile.println();

    }

    // Address 0x680
    else if (msg.id == 1664) {
      relay_state_1 = msg.buf[0];
      relay_state_2 = msg.buf[1];
      fail_s_status_1 = msg.buf[2];
      fail_s_status_2 = msg.buf[3];
      pack_inst_vol_1 = msg.buf[4];
      pack_inst_vol_2 = msg.buf[5];
      pack_SOC = msg.buf[6];

      pack_inst_vol = concat(pack_inst_vol_1, pack_inst_vol_2);

      myFile.print("0x680, "); myFile.print("Time, "); myFile.print(millis() - startTime); myFile.print(", ");
      myFile.print("Relay State 1, 0x"); myFile.print(relay_state_1, HEX); myFile.print(", ");      // Verify that the "HEX" bit actually works
      myFile.print("Relay State 2, 0x"); myFile.print(relay_state_2, HEX); myFile.print(", ");
      myFile.print("Failsafe Status 1, 0x"); myFile.print(fail_s_status_1, HEX); myFile.print(", ");
      myFile.print("Failsafe Status 2, 0x"); myFile.print(fail_s_status_2, HEX); myFile.print(", ");
      myFile.print("Pack Instant Voltage, "); myFile.print(pack_inst_vol); myFile.print(", ");
      myFile.print("Pack SOC, "); myFile.print(pack_SOC/2); myFile.println();

    }

    // Address 0x681
    else if (msg.id == 1665) {
      input_supply_vol = msg.buf[6];
      pack_current_1 = msg.buf[4];
      pack_current_2 = msg.buf[5];

      pack_current = concat(pack_current_1, pack_current_2);

      myFile.print("0x681, "); myFile.print("Time, "); myFile.print(millis() - startTime); myFile.print(", ");
      myFile.print("Input Supply Voltage, "); myFile.print(input_supply_vol); myFile.print(", ");
      myFile.print("Pack Current, "); myFile.print(pack_current); myFile.println();

    }

    // Address 0x682
    else if (msg.id == 1666) {
      pack_amphours_1 = msg.buf[0];
      pack_amphours_2 = msg.buf[1];
      internal_temp = msg.buf[2];

      pack_amphours = concat(pack_amphours_1, pack_amphours_2);

      myFile.print("0x682, "); myFile.print("Time, "); myFile.print(millis() - startTime); myFile.print(", ");
      myFile.print("Pack Amphours, "); myFile.print(pack_amphours); myFile.print(", ");
      myFile.print("Internal Temp, "); myFile.print(internal_temp); myFile.println();
    }

    // Address 0x683
    else if (msg.id == 1667) {

      int high_cell_vol_1 = 0;
      int high_cell_vol_2 = 0;
      int low_cell_vol_1 = 0;
      int low_cell_vol_2 = 0;
      int avg_cell_vol_1 = 0;
      int avg_cell_vol_2 = 0;

      int avg_cell_vol = 0;
      int high_cell_vol = 0;
      int low_cell_vol = 0;

      high_cell_vol_1 = msg.buf[0];
      high_cell_vol_2 = msg.buf[1];
      low_cell_vol_1 = msg.buf[2];
      low_cell_vol_2 = msg.buf[3];
      avg_cell_vol_1 = msg.buf[5];
      avg_cell_vol_2 = msg.buf[6];

      high_cell_vol = concat(high_cell_vol_1, high_cell_vol_2);
      low_cell_vol = concat(low_cell_vol_1, low_cell_vol_2);
      avg_cell_vol = concat(avg_cell_vol_1, avg_cell_vol_2);

      myFile.print("0x683, "); myFile.print("Time, "); myFile.print(millis() - startTime); myFile.print(", ");
      myFile.print("High Cell Voltage, "); myFile.print(high_cell_vol); myFile.print(", ");
      myFile.print("Low Cell Voltage, "); myFile.print(low_cell_vol); myFile.print(", ");
      myFile.print("Average Cell Voltage "); myFile.print(avg_cell_vol); myFile.println();
    }

    // Reading individual thermistor temps
    // 0x1838F380
    else if (msg.id == 406385536) {
      if (msg.buf[2] < 85) {
        thermistor_temps[msg.buf[1]] = msg.buf[2];
      }
    }
  }
}


// ===================== SUPPORTING FUNCTIONS ============================
// hex to decimal conversion function
int hexToDec(char hex[]){
     long long decimal;  
    int i = 0, val = 0, len;                  // variables declaration
    decimal = 0; 
    len = strlen(hex);  
    len--;  
  
    /* 
     * Iterate over each hex digit 
     */  
    for(i=0; hex[i]!='\0'; i++) {  
   
         /* Find the decimal representation of hex[i] */
        if(hex[i]>='0' && hex[i]<='9')
        {
            val = hex[i] - 48;
        }
        else if(hex[i]>='a' && hex[i]<='f')
        {
            val = hex[i] - 97 + 10;
        }
        else if(hex[i]>='A' && hex[i]<='F')
        {
            val = hex[i] - 65 + 10;
        }

        decimal += val * pow(16, len);
        len--; 
    } 
    return decimal;
}

// given two ints that were converted from HEX, concating the equivalent int value of their concated HEX
long long concat(int a, int b){
  //int isOneDigitA = 0;
  int isOneDigitB = 0;
  long long dec;  
  char s1[20];
  char s2[20];
//char zeroA[20] = "0";
  char zeroB[20] = "0";
//onvert both the integers to hex
  sprintf(s1, "%X", a);
  sprintf(s2, "%X", b);
//  if(a/10 <= 0){
//     Serial.println("First");
//     isOneDigitA = 1;
//   }
   if(b/10 <= 0){
     //Serial.println("Second");
     isOneDigitB = 1;
     strcat(zeroB, s2);
   }
//     if(isOneDigitA==1){
//       strcat(zeroA, s1);
//     }
//     if(isOneDigitB == 1){
//       strcat(zeroB, s2);
//     }

//     if(isOneDigitA==1 && isOneDigitB == 1){
//       strcat(zeroA, zeroB);
//       dec = hexToDec(zeroA);
//       Serial.print("Pack part final: "); Serial.print(dec);
//       return dec;
//     }else if(isOneDigitA==1 && isOneDigitB == 0){
//       strcat(zeroA, s2);
//       dec = hexToDec(zeroA);
//       Serial.print("Pack part final: "); Serial.print(dec);
//       return dec;
//     }else 
      if(isOneDigitB == 1){
        strcat(s1, zeroB);
        //Serial.println(s1);
        dec = hexToDec(s1);
        //Serial.print("Pack part final: "); Serial.print(dec);
        return dec;
    }else{
      strcat(s1, s2);
    //change hex(s1) to decimal
    dec = hexToDec(s1);
    //Serial.print("Pack part final: "); Serial.print(dec);
    return dec;
    }
    // Serial.println(zeroA);
    // Serial.println(zeroB);
    //sprintf(s1, "%d", a);
    //sprintf(s2, "%d", b);
  
    // Concatenate both hex and store in s1
    //strcat(s1, s2);
    //change hex(s1) to decimal
    // long long dec = hexToDec(s1);
     //Serial.print("Pack part final: "); Serial.print(dec);
  
    // Convert the concatenated string
    // to integer
  
    // return the formed integer
   // return dec;
}