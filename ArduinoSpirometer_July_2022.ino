/**
 * File         texasSpirometer_1.ino
 * Date         May, 2022
 * Modified from Rabbit Creek Githup repository https://github.com/rabbitcreek/TexasSpiromoter/blob/main/texasSpirometerCSV.ino
 * 
 * Authors      Ian Murray  (imurray@tamu.edu)
 *              Brendan D'Souza(b11dsouza@exchange.tamu.edu)
 *              Evan George (eeg150030@exchange.tamu.edu)
 *      
 * Materials: 
 *            3D printed spirometer- print the tube.stl file only https://www.instructables.com/3D-Printed-Spirometer/
 *            Differential pressure sensor MPXV7002DP on breadboard  https://www.digikey.com/en/products/detail/nxp-usa-inc/MPXV7002DP/1168436
 *            
 * Pin connections to arduino
* GND- GND, 5V - 5V, Analog- A0
* 
 * Instructions: This program has 4 different ouput modes, an  you will need to change the int mode within the code to switch between each.  
 *    1 = Spirogram (Serial Plotter)
 *    2 = Plot Total Volume for each test (Serial Plotter)
 *    3 = Text Output (Serial Monitor)
 *    4 = Data for Flow-Volume Loop (Transfer to Excel to Plot)
 * 
Mode 1: The Spirogram is poutput via the Serial Plotter in the Arduino IDE (in the tab click on Tools --> Serial Plotter)

Mode 2: The volume per time plot is output and viewed via the Serial Plotter in the Arduino IDE (in the tab click on Tools --> Serial Plotter)
    This mode is ready to collect data ONLY when the data is scrolling across the page. If the data is not scrolling across the page, then the pressure sensor calibration step is being peformed. This caliraton step is needed, as the sensor is very sensitive 
    and can record data just by moving tghe sensor. The data is plotted as flow rate (red) and total volume expired (blue). It does not output the entire inhalation and exhalation flow volume loop.

Mode 3: Text data will outputin the Serial Monitor (in the tab click on Tools --> Serial Monitor).Starting calibration,and then indicating that you ca start the test. You then put air into the device and the FEV1 and FVC are output. See example below. It will continue giving values after the test, as the sensor is a bit buggy.
  
   Starting calibration
   Calibration Complete - Ready to Test.
   **********Test Started**********
   FEV1:
   2149.06
   FVC:
   3483.77
   **********Done**********

   Mode 4: This will output data for Flow-Volume Loop (Transfer to Excel to Plot)
*

* 
* License: 
* Pressure sensor code: Hrisko, J. (2020). MPS20N0040D Pressure Sensor Calibration with Arduino. Maker Portal. https://makersportal.com/blog/2020/6/4/mps20n0040d-pressure-sensor-calibration-with-arduino. Modified July 2021
**/

#include <Arduino.h>
#include <Wire.h>

//*****MODE***** YOU WILL NEET TO CHANGE THIS TO GET DIFFERENT OUTPUTS ###############
int mode = 3;
//  1 = Spirogram (Serial Plotter)
//  2 = Plot Total Volume for each test (Serial Plotter)
//  3 = Text Output (Serial Monitor)
//  4 = Data for Flow-Volume Loop (Transfer to Excel to Plot)

//SENSOR
const int analogInPin = 0;
float voltage = 0.0;
float sensorValue = 0; 

//PRESSURE CALIBRATION AND DATA
bool pressureCorrection = false;
float pressureCorrecter = 0.0;

float pressure = 0.0;
float recentPressure [20];
float recentPressureSum = 0.0;
float rollingPressure;

//SPIROMETER MOUTHPIECE MEASUREMENTS
float area_1 = 0.000531; //these are the areas taken carefully from the 3D printed venturi 2M before constriction
float area_2 = 0.000201; // this is area within the venturi
float rho = 1.225; //Demsity of air in kg/m3;

//TEXT CONTROLLER
bool testing = 0;
float TimerLast = 0.0;
float TestStartTime = 0.0;
float TestTimer = 0.0;
float testVolumeTotal = 0.0;
float testVolume = 0.0;
float spiroVol = 0.0;

//TEXT CONTROLLER - MEASURES
float FEV1 = 0.0;
bool FEV1needed = true;
float FVC = 0.0;
bool FVCneeded = true;
float maxFlow = 0.0;

//SPIROMETRY CALCULATIONS
float massFlow = 0;
float volFlow = 0.0;;

void setup() {
    Serial.begin(115200);
    Wire.begin();
}

void loop() {
    //******PRESSURE CALIBRATION*****
    
    //Determine and set pressureCorrecter during first 100 pressure readings
    //Calibrates baseline pressure at the beginning AND immediately after each test completes
    if(pressureCorrection==false) {
      if(mode==3) //  3 = Text Output (Serial Monitor)
      {
        Serial.println("Starting calibration");
      }
      int pressureCounts = 0;
      float totalPressure = 0.0;
      while(pressureCounts<100) {
        //Get voltage reading
        sensorValue= analogRead(analogInPin);
        voltage =(5*(sensorValue)/1023); //convert sensor value to voltage
              
        //Convert voltage reading to pressure  WHEN CONVERTING FROM VOLTAGE TO PRESSURE, BE SURE TO MULTIPLE BY 1000 TO GO FROM THE UNITS OF THE PRESSURE DIFFERENTIAL (KILOPASCAL) TO S.I. UNITS (PASCALS) https://makersportal.com/blog/2019/02/06/arduino-pitot-tube-wind-speed-theory-and-experiment
        pressure = (((voltage-2.5)*1000));  

        totalPressure = totalPressure + pressure;
        pressureCounts++;

        delay(20);
      }
      pressureCorrecter = totalPressure/100; 
      pressureCorrection = true;
      if(mode==3)  //  3 = Text Output (Serial Monitor)
      {
        Serial.println("Calibration Complete - Ready to Test.");   
      }
    }

    //*****PRESSURE DATA*****

    //Get voltage reading
    sensorValue= analogRead(analogInPin);if(abs(pressure) < 0.5) pressure = 0;
    voltage =(5*(sensorValue)/1023);
        
    //Convert voltage reading to pressure
    pressure = (((voltage-2.5)*1000)-pressureCorrecter);

    //Update list of recent pressures and calculate the rolling average
    recentPressure[19] = pressure;
    recentPressureSum = pressure;
    for(int i=0; i<19; i++) {
      recentPressure[i] = recentPressure[i+1];
      recentPressureSum += recentPressure[i];
    }  
    rollingPressure = recentPressureSum/20;

    //Flatlines the pressure if the pressure is minimal
    if(abs(rollingPressure)<2 && mode!=1) {
      rollingPressure = 0;
    }

    //*****SPIROMETRY CALCULATIONS*****
    
    //Calculates flow using the Bernoulli equation
    massFlow = 1000*sqrt((abs(rollingPressure)*2*rho)/((1/(pow(area_2,2)))-(1/(pow(area_1,2))))); //Bernoulli equation converting pressure to massflow
    if(rollingPressure<0)
    {
      massFlow = massFlow*(-1);
    }
    volFlow = massFlow/rho; //volumetric flow of air
    
    //Calculates total volume for the test
    if (testing){
      testVolume = volFlow*(millis()-TimerLast)+testVolume; //integrates volumes over units of time to get total volume
      if (volFlow>0)
      {
        testVolumeTotal = volFlow*(millis()-TimerLast)+testVolumeTotal; //only adds expiration
      }
    }
    
    //Calculates cumulative volume for the spirometer
    spiroVol = volFlow*(millis()-TimerLast)+spiroVol;

    //Updates maxFlow rate, if needed
    if(volFlow > maxFlow) maxFlow = volFlow;
    
    //*****TEST CONTROLLER*****
    
    //Collects FEV1 after 1 second, Ends test after 3 seconds
    //Starts a new test if there is positive flow
    if(testing){
      TestTimer = TimerLast-TestStartTime; // this timer keeps track of time to allow for calculation of FEV1
      if((TestTimer>1000)&&(FEV1needed)) {
        FEV1 = testVolumeTotal;
        if(mode==3) // this will output the text data
        {
          Serial.println("FEV1:");
          Serial.println(FEV1);
        }
        FEV1needed = false;
      }
      if(TestTimer>3000) {
        FVC = testVolumeTotal;
        if(mode==3)
        {
          Serial.println("FVC:");
          Serial.println(FVC);
          Serial.println("FEV1/FVC: ");
          Serial.println(FEV1/FVC);
          Serial.println ("PEF: "); //Peak expiratory flow
          Serial.println(maxFlow);
        }
        testing=false;
        pressureCorrection=false;
        if(mode==3)
        {
          Serial.println("**********Done**********");
        }
        TestTimer = 0.0;
        testVolumeTotal = 0.0;
        testVolume = 0.0;
        FEV1 = 0.0;
        FEV1needed = true;
      }
    }
    else if(volFlow>0 && mode!=1) { // Mode 1 = spirogram
      testing=true;
      if(mode==3) 
      {
        Serial.println("**********Test Started**********");
      }
      TestStartTime = TimerLast;
    }

    //*****SPIROMETRY GRAPHING***** (Mode 1)
    
    //Graphs flow and test volume expired
    if(mode==1) // this will output the volume passed through the spirometer
    {
      Serial.println(spiroVol);
    }
 
    //*****TOTAL VOLUME GRAPHING***** (Mode 2)
    
    //Graphs flow and test volume expired
    if(mode==2) // this will output the flow and volume per time
    {
      Serial.print(testVolumeTotal);
      Serial.print(",");
      if(volFlow<0){volFlow=0;} //Prevents inhalation from being plotted
      Serial.println(volFlow*300); //300 is an arbitrary scaling factor
    }
    
    //*****FLOW-VOLUME LOOP DATA***** (Mode 4)
    
    //Graphs flow and test volume expired
    if(mode==4) // this will output the flow and volume per time
    {
      Serial.print("  ");
      Serial.print(testVolume);      
      Serial.print(",");
      Serial.println(volFlow*300); //300 is an arbitrary scaling factor
    }
    
    //*****LOOP RESET*****
    
    TimerLast = millis();  
    delay(20);
}
