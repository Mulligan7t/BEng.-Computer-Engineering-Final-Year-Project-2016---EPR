/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
/*End of auto generated code by Atmel studio */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
//Beginning of Auto generated function prototypes by Atmel Studio
void displaySensorDetails(void );

void wheeldir(int wheelnum);
void parseData();
void showParsedData();
void sensorloop();
void delaysensor(int t);
//End of Auto generated function prototypes by Atmel Studio



/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void MotorsInit();
void AccelerometerInit();
void AccelerometerRead();
void GyroInit();
void getGyroscopeData(int result);
void GyroRead();
void writeTo(int DEVICE, byte address, byte val);
void readFrom(int DEVICE, byte address, int num, byte buff[]);

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
// I2C library, Gyroscope ITG3200
#define GYRO 0x68 //  when AD0 is connected to GND ,gyro address is 0x68. #define GYRO 0x69   when AD0 is connected to VCC ,gyro address is 0x69
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define G_TO_READ 8 // 2 bytes for each axis x, y, z
// offsets are chip specific.
int g_offx = 120;
int g_offy = 20;
int g_offz = 93;
int truetemp;
uint32_t timer;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
float headingDegrees;

int d = 100;
int speedall = 255;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String newlinechar = "\n\r";

float speedFromPC[4] = {0.0,0.0,0.0,0.0};
char recvChar;
char endMarker = '>';
boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data

// Motor 1
int dir1PinLF = 12;
int dir2PinLF = A2;
int speedPinLF = 10; // Needs to be a PWM pin to be able to control motor speed
// Motor 2
int dir1PinRF = A0;
int dir2PinRF = A1;
int speedPinRF = 9; // Needs to be a PWM pin to be able to control motor speed
// Motor 3
int dir1PinLB = 4;
int dir2PinLB = 2;
int speedPinLB = 3; // Needs to be a PWM pin to be able to control motor speed
// Motor 4
int dir1PinRB = 5;
int dir2PinRB = 6;
int speedPinRB = 11; // Needs to be a PWM pin to be able to control motor speed

int speedPin[4] = {10,9,3,11};
int dir1[4] = {12,A0,4,5};
int dir2[4] = {A2,A1,2,6};


//initializes the gyroscope
void setup()
{
	#if 1
	newlinechar = "\n";
	#endif

		
	Serial.begin(115200);
	Wire.begin();

	MotorsInit();
	AccelerometerInit();
	GyroInit();
	inputString.reserve(200); // reserve 200 bytes for the inputString:

	Serial.println("start");
	timer = micros();
}

void displaySensorDetails(void)
{
	sensor_t sensor;
	mag.getSensor(&sensor);
	Serial.println("------------------------------------");
	Serial.print  ("Sensor:       "); Serial.println(sensor.name);
	Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
	Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
	Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
	Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
	Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
	Serial.println("------------------------------------");
	Serial.println("");
	delay(500);
}

void MotorsInit(){
	//Define L298N Dual H-Bridge Motor Controller Pins

	// pin 3 = d3
	// pin 10 = d10
	// pin 9 = d9
	//pin 11 = d11
	// pin 2 = d2 = EN4
	// pin 4 = d4 = EN3
	// pin 5 = d5 = EN2
	// pin 6 = d6 = EN1
	//THR = PIN 2
	//ROL = PIN 4
	//PIT = PIN 5
	//YAW = PIN 6
	//A0 = 23
	//A1 = 24
	//A2 = 25
	//D12 = 12

	pinMode(dir1PinLF, OUTPUT);
	pinMode(dir2PinLF, OUTPUT);
	pinMode(speedPinLF, OUTPUT);

	pinMode(dir1PinRF, OUTPUT);
	pinMode(dir2PinRF, OUTPUT);
	pinMode(speedPinRF, OUTPUT);

	pinMode(dir1PinLB, OUTPUT);
	pinMode(dir2PinLB, OUTPUT);
	pinMode(speedPinLB, OUTPUT);

	pinMode(dir1PinRB, OUTPUT);
	pinMode(dir2PinRB, OUTPUT);
	pinMode(speedPinRB, OUTPUT);
}

void AccelerometerInit()
{
	Wire.beginTransmission(0x40); // address of the accelerometer
	// reset the accelerometer
	Wire.write(0x10);
	Wire.write(0xB6);
	Wire.endTransmission();
	delay(10);
	
	Wire.beginTransmission(0x40); // address of the accelerometer
	// low pass filter, range settings
	Wire.write(0x0D);
	Wire.write(0x10);
	Wire.endTransmission();
	
	Wire.beginTransmission(0x40); // address of the accelerometer
	Wire.write(0x20); // read from here
	Wire.endTransmission();
	Wire.requestFrom(0x40, 1);
	byte data = Wire.read();

	Wire.beginTransmission(0x40); // address of the accelerometer
	Wire.write(0x20);
	//	Wire.write(data & 0x0F); // low pass filter to 10 Hz
	Wire.write(data & 0x0F); // low pass filter to 10 Hz
	Wire.endTransmission();
	
	Wire.beginTransmission(0x40); // address of the accelerometer
	Wire.write(0x35); // read from here
	Wire.endTransmission();
	Wire.requestFrom(0x40, 1);
	data = Wire.read();
	Wire.beginTransmission(0x40); // address of the accelerometer
	Wire.write(0x35);
	Wire.write((data & 0xF1) | 0x04); // range +/- 2g
	Wire.endTransmission();

	Wire.beginTransmission(0x40); // address of the accelerometer
	Wire.write(0x30);
	//Wire.write(0x); //
	Wire.endTransmission();

	//Serial.println(data, BIN);

}

void AccelerometerRead()
{
	Wire.beginTransmission(0x40); // address of the accelerometer
	Wire.write(0x02); // set read pointer to data
	Wire.endTransmission();
	Wire.requestFrom(0x40, 6);
	
	// read in the 3 axis data, each one is 16 bits
	// print the data to terminal
	
	short data = Wire.read();
	data += Wire.read() << 8;
	accX = data;
	data = Wire.read();
	data += Wire.read() << 8;
	accY = data;
	data = Wire.read();
	data += Wire.read() << 8;
	accZ = data;
	#if 0
	Serial.print("Accelerometer: X = ");
	Serial.print(accX);
	Serial.print(" , Y = ");
	Serial.print(accY);
	Serial.print(" , Z = ");
	Serial.print(accZ);
	Serial.println();
	#endif

}

void GyroInit()
{
	/*****************************************
	* ITG 3200
	* power management set to:
	* clock select = internal oscillator
	* no reset, no sleep mode
	* no standby mode
	* sample rate to = 125Hz
	* parameter to +/- 2000 degrees/sec
	* low pass filter = 5Hz
	* no interrupt
	******************************************/
	writeTo(GYRO, G_PWR_MGM, 0x00);
	writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
	writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
	writeTo(GYRO, G_INT_CFG, 0x00);
}

void getGyroscopeData(int * result)
{
	/**************************************
	Gyro ITG-3200 I2C
	registers:
	temp MSB = 1B, temp LSB = 1C
	x axis MSB = 1D, x axis LSB = 1E
	y axis MSB = 1F, y axis LSB = 20
	z axis MSB = 21, z axis LSB = 22
	*************************************/
	int regAddress = 0x1B;
	int temp, x, y, z;
	byte buff[G_TO_READ];
	readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
	result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
	result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
	result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
	result[3] = (buff[0] << 8) | buff[1]; // temperature
}

void GyroRead (){
	byte addr;
	int gyro[4];
	getGyroscopeData(gyro);
	gyroX = gyro[0] / 14.375; //convert to degrees per second
	gyroY = gyro[1] / 14.375;
	gyroZ = gyro[2] / 14.375;
	truetemp = 35+ ((double) (gyro[3] + 13200)) / 280; // temperature
	#if 0
	Serial.print(" X=");
	Serial.print(gyroX);
	Serial.print(" Y=");
	Serial.print(gyroY);
	Serial.print(" Z=");
	Serial.print(gyroZ);
	Serial.print(" F=");
	Serial.print(truetemp);
	Serial.print("'");
	Serial.println("C");
	//delay(500);
	#endif

	
}

//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) {
	Wire.beginTransmission(DEVICE); //start transmission to ACC
	Wire.write(address);        // send register address
	Wire.write(val);        // send value to write
	Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on ACC in to buff array
void readFrom(int DEVICE, byte address, int num, byte buff[]) {
	Wire.beginTransmission(DEVICE); //start transmission to ACC
	Wire.write(address);        //sends address to read from
	Wire.endTransmission(); //end transmission
	
	Wire.beginTransmission(DEVICE); //start transmission to ACC
	Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC
	
	int i = 0;
	while(Wire.available())    //ACC may send less than requested (abnormal)
	{
		buff[i] = Wire.read(); // receive a byte
		i++;
	}
	Wire.endTransmission(); //end transmission
}



void wheeldir (int wheelnum) {
	if (speedFromPC[wheelnum]>0)
	{
		digitalWrite(dir1[wheelnum],HIGH);
		digitalWrite(dir2[wheelnum],LOW);
	}else if (speedFromPC[wheelnum]<0)
	{
		digitalWrite(dir1[wheelnum],LOW);
		digitalWrite(dir2[wheelnum],HIGH);
	} 
	else
	{
		digitalWrite(dir1[wheelnum],HIGH);
		digitalWrite(dir2[wheelnum],HIGH);
	}
	analogWrite(speedPin[wheelnum], int(abs(speedFromPC[wheelnum])));

}


void recvWithEndMarker() {
	static byte ndx = 0;
	char endMarker = '\r';
	char rc;
	
	// if (Serial.available() > 0) {
	while (Serial.available() > 0 && newData == false) {
		rc = Serial.read();

		if (rc != endMarker) {
			receivedChars[ndx] = rc;
			ndx++;
			if (ndx >= numChars) {
				ndx = numChars - 1;
			}
		}
		else {
			receivedChars[ndx] = '\0'; // terminate the string
			ndx = 0;
			newData = true;
		}
	}
}

void showNewData() {
	if (newData == true) {
		parseData();
		showParsedData();
		newData = false;
	}
}


void parseData() {
	// split the data into its parts
	char * strtokIndx; // this is used by strtok() as an index
	
	strtokIndx = strtok(receivedChars," ");      // get the first part - the string
	speedFromPC[0] = atof(strtokIndx);     // convert this part to an float
	
	strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
	speedFromPC[1] = atof(strtokIndx);     // convert this part to an float
	
	strtokIndx = strtok(NULL, " ");
	speedFromPC[2] = atof(strtokIndx);     // convert this part to a float

	strtokIndx = strtok(NULL, " ");
	speedFromPC[3] = atof(strtokIndx);     // convert this part to a float

}


void showParsedData() {
	Serial.print("LF ");
	Serial.print(speedFromPC[0]);
	Serial.print(" RF ");
	Serial.print(speedFromPC[1]);
	Serial.print(" LB ");
	Serial.print(speedFromPC[2]);
	Serial.print(" RB ");
	Serial.println(speedFromPC[3]);
}

void sensorloop(int printout)
{
	
	AccelerometerRead();
	GyroRead();
	
	double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
	timer = micros();

	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
	#ifdef RESTRICT_PITCH // Eq. 25 and 26
	double roll  = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	#else // Eq. 28 and 29
	double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	#endif

	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	double gyroYrate = gyroY / 131.0; // Convert to deg/s

	#ifdef RESTRICT_PITCH
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll);
		compAngleX = roll;
		kalAngleX = roll;
		gyroXangle = roll;
	} else
	kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleX) > 90)
	gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
	#else
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
		kalmanY.setAngle(pitch);
		compAngleY = pitch;
		kalAngleY = pitch;
		gyroYangle = pitch;
	} else
	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleY) > 90)
	gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	#endif

	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	gyroYangle += gyroYrate * dt;
	//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
	//gyroYangle += kalmanY.getRate() * dt;

	compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
	compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

	// Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
	gyroXangle = kalAngleX;
	if (gyroYangle < -180 || gyroYangle > 180)
	gyroYangle = kalAngleY;



	/* Get a new sensor event */
	sensors_event_t event;
	mag.getEvent(&event);
	

	// Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	float heading = atan2(event.magnetic.y, event.magnetic.x);
	
	// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
	// Find yours here: http://www.magnetic-declination.com/
	// Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
	// If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
	float declinationAngle = 0.22;
	heading += declinationAngle;
	
	// Correct for when signs are reversed.
	if(heading < 0)
	heading += 2*PI;
	
	// Check for wrap due to addition of declination.
	if(heading > 2*PI)
	heading -= 2*PI;
	
	// Convert radians to degrees for readability.
	headingDegrees = heading * 180/M_PI;
	
	/* Display the results (magnetic vector values are in micro-Tesla (uT)) */
	#if 0
	Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
	Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
	Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
	Serial.print("Heading (degrees):                                        "); Serial.println(headingDegrees);
	
	#endif
	
	/* Print Data */
	
	if (printout)
	{
	Serial.print(accX/16384*9.80665); Serial.print(" ");
	Serial.print(accY/16384*9.80665); Serial.print(" ");
	Serial.print(accZ/16384*9.80665); Serial.print(" ");

	Serial.print(gyroX); Serial.print(" ");
	Serial.print(gyroY); Serial.print(" ");
	Serial.print(gyroZ); Serial.print(" ");

	Serial.print(event.magnetic.x); Serial.print(" ");
	Serial.print(event.magnetic.y); Serial.print(" ");
	Serial.print(headingDegrees); Serial.print(" ");
	
	Serial.print(dt);
	Serial.print(newlinechar);
	}

	#if 0
	Serial.print(roll); Serial.print("\t");
	Serial.print(gyroXangle); Serial.print("\t");
	Serial.print(compAngleX); Serial.print("\t");
	Serial.print(kalAngleX); Serial.print("\t");

	Serial.print("\t");

	Serial.print(pitch); Serial.print("\t");
	Serial.print(gyroYangle); Serial.print("\t");
	Serial.print(compAngleY); Serial.print("\t");
	Serial.print(kalAngleY); Serial.print("\t");

	#endif


	#if 0 // Set to 1 to print the temperature
	Serial.print("\t");

	double temperature = (double)tempRaw / 340.0 + 36.53;
	Serial.print(temperature); Serial.print("\t");
	#endif

	
	#if 0
	
	//Serial.print("\r\n");
	
	Serial.print(pitch*10);
	Serial.print(" ");
	Serial.print(gyroXangle*10);
	Serial.print(" ");
	Serial.print(compAngleX*10);
	Serial.print(" ");
	Serial.print(kalAngleX*10);
	Serial.print(" ");
	Serial.print(headingDegrees);
	Serial.println();
	#endif
	
	delay(d); // slow down output
}

void delaysensor(int t){
	for (int x = 0; x < t/d; x++)
	{
		sensorloop(0);
	}
}

void loop() {
	delaysensor(100);
	d = 100;
	recvWithEndMarker();
	showNewData();
	
	for (int x = 0; x < 4; x++)
	{
		wheeldir(x);
	}


	
	#if 0
	
	analogWrite(speedPinLF, speedall);//Sets speed variable via PWM
	analogWrite(speedPinRF, speedall);//Sets speed variable via PWM
	analogWrite(speedPinLB, speedall);//Sets speed variable via PWM
	analogWrite(speedPinRB, speedall);//Sets speed variable via PWM
	
	//
	//speedall = 255;
	//turnleft();
	//forward();
	//delaysensor(1000);
	if (headingDegrees >0  && headingDegrees < 180)
	{
		speedall = 255;
		turnleft();
		delaysensor(10);
		}else if(headingDegrees>200 && headingDegrees <350){
		speedall = 255;
		turnright();
		delaysensor(10);
		}else {
		speedall = 0;
		stop();
		delaysensor(10);
	}
	#endif

}


