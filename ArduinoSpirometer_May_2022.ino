// MPX7002DP Test Code  
 // A.Lang - 2017  
 // This code exercises the MPX7002DP  
 // Pressure sensor connected to A0  
 //langster1980.blogspot.com/2017/04/create-spirometer-using-msp7002dp.html

#define WINDOW_SIZE 4 // change this to aler the filtering of the ECG https://maker.pro/arduino/tutorial/how-to-clean-up-noisy-sensor-data-with-a-moving-average-filter
 
 int sensorPin = A0;  // select the input pin for the Pressure Sensor  
 int sensorValue = 0;  // variable to store the Raw Data value coming from the sensor 
  
  //Variables to change 
 float volt = 0; // Voltage (converted from 0-255 to 0-5)
 float outputValue = 0; // variable to store converted kPa value   
 float pressurekpa=0;
 float pressurecmh2o=0;
int INDEX = 0; 
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;
 
 void setup() {  
  // start serial port at 9600 bps and wait for port to open:  
  Serial.begin(9600);  
  while (!Serial) {  
   ; // wait for serial port to connect. Needed for native USB port only  
  }  
  pinMode(sensorPin, INPUT);  // Pressure sensor is on Analogue pin 0  
 }  
 void loop() {  
  // read the value from the sensor:  
  sensorValue= analogRead(analogInPin);
  
  
// WHEN CONVERTING FROM VOLTAGE TO PRESSURE, BE SURE TO MULTIPLE BY 1000 TO GO FROM THE UNITS OF THE PRESSURE DIFFERENTIAL (KILOPASCAL) TO S.I. UNITS (PASCALS) https://makersportal.com/blog/2019/02/06/arduino-pitot-tube-wind-speed-theory-and-experiment
 //convert voltage to pressure
   voltage =(5*(sensorValue)/1023); //convert sensor value to voltage

 //Convert voltage reading to pressure  WHEN CONVERTING FROM VOLTAGE TO PRESSURE, BE SURE TO MULTIPLE BY 1000 TO GO FROM THE UNITS OF THE PRESSURE DIFFERENTIAL (KILOPASCAL) TO S.I. UNITS (PASCALS) https://makersportal.com/blog/2019/02/06/arduino-pitot-tube-wind-speed-theory-and-experiment
    pressure = (((voltage-2.5)*1000));  
  
//  print the results to the serial monitor:  
//  Serial.print("sensor = " );  
// Serial.println(sensorValue-526);  
//  Serial.print("pressure = " ); 
 Serial.println(pressure); 
//  Serial.print("pressurecmh2o = " );  
//  Serial.print(pressurecmh2o); 
//  Serial.print("voltage = ");  
//  Serial.println(voltage); 


  // wait 100 milliseconds before the next loop  
  // for the analog-to-digital converter and  
  // pressure sensor to settle after the last reading:  
  delay(10);

  
  }  
