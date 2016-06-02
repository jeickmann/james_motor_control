#include "Arduino.h"
#include <PID_v1.h>

int inApin[2] = { 7, 4 };  // INA: Clockwise input
int inBpin[2] = { 8, 9 }; // INB: Counter-clockwise input
int pwmpin[2] = { 5, 6 }; // PWM input
int cspin[2] = { 2, 3 }; // CS: Current sense ANALOG input
int enpin[2] = { 0, 1 }; // EN: Status of switches output (Analog pin)

double cmds[2] = { 0, 0 };
double speeds[2] = { 0, 0 };
double powers[2] = { 0, 0 };

double kp[2] = { 0.5, 0.5 };
double ki[2] = { 0, 0 };
double kd[2] = { 0, 0 };

PID pid_0(&(speeds[0]), &(powers[0]), &(cmds[0]), kp[0], ki[0], kd[0], DIRECT);
PID pid_1(&(speeds[1]), &(powers[1]), &(cmds[1]), kp[1], ki[1], kd[1], DIRECT);

void initMotor(int number) {
	pinMode(inApin[number], OUTPUT);
	pinMode(inBpin[number], OUTPUT);
	pinMode(pwmpin[number], OUTPUT);

	pinMode(cspin[number], INPUT);

	//brake initially
	digitalWrite(inApin[number], LOW);
	digitalWrite(inBpin[number], LOW);
}

void setMotorPower(int number, double power) {
	int pwmPower = power * 255;
	if (abs(power) < 0.1) {
		digitalWrite(inApin[number], LOW);
		digitalWrite(inBpin[number], LOW);

		analogWrite(pwmpin[number], 0);
	} else if (power > 0) {
		digitalWrite(inApin[number], HIGH);
		digitalWrite(inBpin[number], LOW);

		analogWrite(pwmpin[number], pwmPower);
	} else {
		digitalWrite(inApin[number], LOW);
		digitalWrite(inBpin[number], HIGH);
	}
}

//The setup function is called once at startup of the sketch
void setup() {
	//Serial.begin(115200);
	initMotor(0);
	initMotor(1);
	/*pid_0.SetOutputLimits(-1, 1);
	pid_1.SetOutputLimits(-1, 1);
	pid_0.SetMode(AUTOMATIC);
	pid_1.SetMode(AUTOMATIC);*/
}

void sendSpeeds() {
	Serial.write((char*) speeds, sizeof(float) * 2);
}

void updateSpeeds() {
	//TODO: implement speed encoders
}

// The loop function is called in an endless loop
void loop() {
	/*
	updateSpeeds();

	if (Serial.available()) {
		char c = Serial.read();
		if (c == '?') {
			sendSpeeds();
		} else if (c == '#') {
			Serial.readBytes((char*) cmds, sizeof(float) * 2);
		}
	}

	//pid_0.Compute();
	//pid_1.Compute();

	setMotorPower(0, powers[0]);
	setMotorPower(1, powers[1]);
	*/
	setMotorPower(0, 0.5);
}
