/* 
Based off of MMA8452Q Basic Example Code by Nathan Seidle
Shape Detection by George Melcer
 
 License: All Right Reserved 2014. Message george@boxunfolded.com for licensing information for use with commercial/for-profit purposes. 
 
 Hardware setup:
 MMA8452 Breakout ------------ Arduino
 3.3V --------------------- 3.3V
 SDA -------^^(330)^^------- A4
 SCL -------^^(330)^^------- A5
 GND ---------------------- GND
 
 The MMA8452 is 3.3V so we recommend using 330 or 1k resistors between a 5V Arduino and the MMA8452 breakout.
 
 The MMA8452 has built in pull-up resistors for I2C so you do not need additional pull-ups.
 */

#include <Wire.h> // Used for I2C

// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low

//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D
#define CTRL_REG1  0x2A
#define CTRL_REG2 0x2B
#define CTRL_REG3 0x2C
#define CTRL_REG4 0x2D
#define CTRL_REG5 0x2E

#define TRANSIENT_CFG 0x1D   //R+W
#define TRANSIENT_SRC 0x1E   //R
#define TRANSIENT_THS 0x1F   //R+W
#define TRANSIENT_COUNT 0x20 //R+W
#define INT_SOURCE 0x0C   //R

#define UART_BAUD 57600

#define  BUFFER_LEN 200
float accelMagBuffer[BUFFER_LEN];

#define AXISES   3

#define INT1_PIN 2
#define INT2_PIN 3

#define X_AXIS      0
#define Y_AXIS      1
#define Z_AXIS      2

#define SHAKE_INTENSITY_THRESHOLD      40


#define ENABLE_DEBUG  0

#define GSCALE 8 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

void setup()
{
  Serial.begin(UART_BAUD);
  Serial.println("MMA8452 Basic Example");
  pinMode(INT1_PIN, INPUT);
  pinMode(INT2_PIN, INPUT);
  digitalWrite(INT1_PIN, LOW);
  digitalWrite(INT2_PIN, LOW);
  Wire.begin(); //Join the bus as a master

  initMMA8452(); //Test and intialize the MMA8452
  
  
  
}



void loop()
{  
    int accelCount[3];  // Stores the 12-bit signed value
    static int bufferPosition = 0;
    static int bufWrapAround= 0;
    int transientStatus =0;
    float accelG[AXISES];  // Stores the real accel value in g's
    int movementAverage=0;
  
    if(ENABLE_DEBUG)
    {
      Serial.println("INT1: " + String(digitalRead(INT1_PIN)));
      Serial.println("INT2: " + String(digitalRead(INT2_PIN)));
    }
    
    transientStatus = readRegister(INT_SOURCE); 
    readRegister(TRANSIENT_SRC); 

  if(ENABLE_DEBUG)
    Serial.println("IRQ SRC: " + String(transientStatus));
    



    
  readAccelData(accelCount);  // Read the x/y/z adc values

    

  for (int i = 0 ; i < AXISES ; i++)  //convert register values into g's
  {
    accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
  }
  

  accelMagBuffer[bufferPosition++] =sqrt(pow(accelG[X_AXIS],2) +  pow(accelG[Y_AXIS],2) +pow(accelG[Z_AXIS],2)); //magitude of the acceleration
  if(bufferPosition > BUFFER_LEN)
    bufferPosition = 0;



  if(bufferPosition> 50)
 for(int i=bufferPosition; i>bufferPosition-50; i--)
   movementAverage +=accelMagBuffer[i];
   
  if(movementAverage > SHAKE_INTENSITY_THRESHOLD)
    Serial.println("Shaking Detected");

  

  delay(50);  // Delay here for visibility

}

void readAccelData(int *destination)
{
  byte rawData[6];  // x/y/z accel register data stored here

  readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array

  // Loop to calculate 12-bit ADC and g value for each axis
  for(int i = 0; i < 3 ; i++)
  {
    int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  //Combine the two 8 bit registers into one 12-bit number
    gCount >>= 4; //The registers are left align, here we right align the 12-bit integer

    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F)
    {  
      gCount = ~gCount + 1;
      gCount *= -1;  // Transform into negative 2's complement #
    }

    destination[i] = gCount; //Record this gCount into the 3 int array
  }
}

void initThresholdDetection(int enX, int enY, int enZ)
{
 writeRegister(TRANSIENT_CFG, 0x16); //transient CFG reg
 writeRegister(TRANSIENT_THS, 0x8); //Transient THS reg
 writeRegister(TRANSIENT_COUNT, 0x5); //Transient Count reg
   writeRegister(CTRL_REG3, 0x02);  // Active high, push-pull interrupts
  writeRegister(CTRL_REG4, 0x20); //COnfiguring Transient detection in IQRQ pin
  writeRegister(CTRL_REG5, 0x20);

  int temp = 0;
temp =  readRegister(TRANSIENT_CFG);
 Serial.println("Transient Config Reg: " + String(temp));

temp =  readRegister(TRANSIENT_THS);
 Serial.println("Transient Threshold reg: " + String(temp));


temp =  readRegister(TRANSIENT_COUNT);
 Serial.println("Transient Count Reg: " + String(temp)); 
 
    temp =  readRegister(TRANSIENT_SRC);
 Serial.println("Transient Source Reg: " + String(temp)); 
 
 temp =  readRegister(CTRL_REG4);
 Serial.println("CTRL REG 4: " + String(temp)); 
 
 temp =  readRegister(CTRL_REG5);
 Serial.println("CTRL Reg 5:" + String(temp)); 
 
  temp =  readRegister(INT_SOURCE);
 Serial.println("INT SOURCE Reg: " + String(temp)); 
 

}


// Initialize the MMA8452 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
void initMMA8452()
{
  byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {  
    Serial.println("MMA8452Q is online...");
  }
  else
  {
    Serial.print("Could not connect to MMA8452Q: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  MMA8452Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  byte fsr = GSCALE;
  if(fsr > 8) fsr = 8; //Easy error check
  fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
  writeRegister(XYZ_DATA_CFG, fsr);
  initThresholdDetection(1,1,1);
  initLowPowerSetup();
  MMA8452Active();  // Set to active to start reading
}


void initLowPowerSetup(void)
{
 
  int reg =  readRegister(CTRL_REG1);
  Serial.println("Reg 1:" + String(reg)); 
  reg = 0x88;
  writeRegister(CTRL_REG1, reg);
}





// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}





// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}




// Read bytesToRead sequentially, starting at addressToRead into the dest byte array
void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default

  while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();    
}



// Read a single byte from addressToRead and return it as a byte
byte readRegister(byte addressToRead)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default

  while(!Wire.available()) ; //Wait for the data to come back
  return Wire.read(); //Return this one byte
}



// Writes a single byte (dataToWrite) into addressToWrite
void writeRegister(byte addressToWrite, byte dataToWrite)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToWrite);
  Wire.write(dataToWrite);
  Wire.endTransmission(); //Stop transmitting
}
