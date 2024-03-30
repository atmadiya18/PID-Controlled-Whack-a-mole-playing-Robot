//driver mechanism
//I/O pins
//Motor 1
//PID constants

#define echoPin 50
#define trigPin 4
int turn = 0;
double kp = 2;
double ki = 5;
double kd = 1;

//First H Bridge

int en1 = 9; //Motor 1
int in1 = 7;
int in2 = 8;

int en2 = 3; //Motor 2
int in4 = 22;
int in3 = 24;

//Second H Bridge 
int en3 = 5; //Motor 3
int in5 = 12; // in1 of second H bridge
int in6 = 13; // in2 of second H bridge

int en4 = 6; //Motor 4 second H bridge
int in8 = 10; // in3 of second H Bridge
int in7 = 11; //in4 of second H Bridge 

// defines variables for Ultrasonic Sensor
long int duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
int measured_distance;

//Line Follower Output Check
int S2_check;
int S3_check;
int S4_check;
int S5_check;

int Linefollowerarray[5] = {0, 0, 0, 0, 0};

int linePassed = 0;

//for (int i = 0; i <= 3; i = i + 1) {
//  Linefollowerarray[i] = 0;
//}

//int in8 = ;//Define states
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

const int S2 = A2;
const int S3 = A3;
const int S4 = A4;
const int S5 = A5;

int S2_output;
int S3_output;
int S4_output;
int S5_output;

//const int STAGE_stop = 100;
//const int STAGE_forward = 101;
//const int STAGE_leftturn = 102;
//const int STAGE_rightturn = 103;
//int state = STAGE_stop;

void setup() {
  // put your setup code here, to run once:
  setPoint = 0;
  //set point at zero degrees; check from calibration of rotary encoder

  //Motor1
  pinMode(en1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  //Motor2
  pinMode(en2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Motor3
  pinMode(en3, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);

  //Motor4
  pinMode(en4, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);

  //Ultrasonic Sensor Pin Declarations
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");

  //Line Follower Declaration

  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  // delay(3000);

}

void LineFollower(int Linefollowerarray[]) {

  S2_output = digitalRead(S2);
  Linefollowerarray[1] = S2_output;
  Serial.print(S2_output);
  Serial.print("\t");
  S3_output = digitalRead(S3);
  Linefollowerarray[2] = S3_output;
  Serial.print(S3_output);
  Serial.print("\t");
  S4_output = digitalRead(S4);
  Linefollowerarray[3] = S4_output;
  Serial.print(S4_output);
  Serial.print("\t");
  S5_output = digitalRead(S5);
  Linefollowerarray[4] = S5_output;
  Serial.print(S5_output);
  Serial.print("\t");
  Serial.println("\t");
}

double UltrasonicSensor()
{
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor

  //  Serial.print("Distance: ");
  //  Serial.print(distance);
  //  Serial.println(" cm");

  return distance;
}

void loop() {
  // put your main code for direction here, to run repeatedly; this is for direction

  int measured_distance;
  measured_distance = UltrasonicSensor();
  LineFollower(Linefollowerarray);

//  if (measured_distance > 30 && Linefollowerarray[2] == 1 && Linefollowerarray[3] == 0 && Linefollowerarray[4] == 1) {
//
//    //S2 is 1; S3 is 2; S4 is 3; S5 is 4;
//    //Serial.println(Linefollowerarray[3]);
//    Serial.println("Going Straight");
//    digitalWrite(in1, LOW); //Clockwise for Motor 1
//    digitalWrite(in2, HIGH);
//    analogWrite(en1, 160);
//
//    digitalWrite(in3, HIGH); //clockwise for Motor 2
//    digitalWrite(in4, LOW);
//    analogWrite(en2, 160);
//
//    digitalWrite(in5, LOW); // clockwise for Motor3
//    digitalWrite(in6, HIGH);
//    analogWrite(en3, 160);
//
//    digitalWrite(in7, HIGH); // Clockwise for Motor4
//    digitalWrite(in8, LOW);
//    analogWrite(en4, 160);
//  }

//  else if (measured_distance > 30 && Linefollowerarray[2] == 1 && Linefollowerarray[3] == 1 && Linefollowerarray[4] == 0) {
//
//    //S2 is 1; S3 is 2; S4 is 3; S5 is 4;
//    //Serial.println(Linefollowerarray[4]);
//    Serial.println("Going Left");
//
//    digitalWrite(in1, LOW); //Clockwise for Motor 1
//    digitalWrite(in2, HIGH);
//    analogWrite(en1, 96);
//
//    digitalWrite(in3, HIGH); //clockwise for Motor 2
//    digitalWrite(in4, LOW);
//    analogWrite(en2, 96);
//
//    digitalWrite(in5, LOW); // clockwise for Motor3
//    digitalWrite(in6, HIGH);
//    analogWrite(en3, 160);
//
//    digitalWrite(in7, HIGH); // Connections for Motor4
//    digitalWrite(in8, LOW);
//    analogWrite(en4, 160);
//  }

//  else if (measured_distance > 30 && Linefollowerarray[2] == 0 && Linefollowerarray[3] == 1 && Linefollowerarray[4] == 1) {
//
//    //S2 is 1; S3 is 2; S4 is 3; S5 is 4;
//    //Serial.println(Linefollowerarray[2]);
//    Serial.println("Going Right");
//
//    digitalWrite(in1, LOW); //Clockwise for Motor 1
//    digitalWrite(in2, HIGH);
//    analogWrite(en1, 160);
//
//    digitalWrite(in3, HIGH); //clockwise for Motor 2
//    digitalWrite(in4, LOW);
//    analogWrite(en2, 160);
//
//    digitalWrite(in5, LOW); // clockwise for Motor3
//    digitalWrite(in6, HIGH);
//    analogWrite(en3, 96);
//
//    digitalWrite(in7, HIGH); // Connections for Motor4
//    digitalWrite(in8, LOW);
//    analogWrite(en4, 96);
//  }

//  else if (measured_distance > 30 && Linefollowerarray[2] == 0 && Linefollowerarray[3] == 0 && Linefollowerarray[4] == 0) {
//    digitalWrite(in1, LOW); //Stop for Motor 1
//    digitalWrite(in2, LOW);
//
//    digitalWrite(in3, LOW); //Stop for Motor 2
//    digitalWrite(in4, LOW);
//
//
//    digitalWrite(in5, LOW); // Stop for Motor3
//    digitalWrite(in6, LOW);
//
//
//    digitalWrite(in7, LOW); // Stop for Motor4
//    digitalWrite(in8, LOW);
//
//    linePassed = linePassed + 1;
//
//    delay(1000);
//    if (linePassed == 3) {
//      while (Linefollowerarray[4] == 1 && Linefollowerarray[3] == 1 && Linefollowerarray[1] == 1)
//      {
//        digitalWrite(in1, LOW); //Clockwise for Motor 1
//        digitalWrite(in2, HIGH);
//        analogWrite(en1, 224);
//
//        digitalWrite(in3, LOW); //Anticlockwise for Motor 2
//        digitalWrite(in4, HIGH);
//        analogWrite(en2, 224);
//
//        digitalWrite(in5, LOW); // clockwise for Motor3
//        digitalWrite(in6, HIGH);
//        analogWrite(en3, 224);
//
//        digitalWrite(in7, LOW); // Anticlockwise for Motor4
//        digitalWrite(in8, HIGH);
//        analogWrite(en4, 224);
//
//        delay(50);
//      }
//    }
//    digitalWrite(in1, LOW); //Clockwise for Motor 1
//    digitalWrite(in2,HIGH);
//    analogWrite(en1, 160);
//
//    digitalWrite(in3, HIGH); //clockwise for Motor 2
//    digitalWrite(in4, LOW);
//    analogWrite(en2, 160);
//
//    digitalWrite(in5, LOW); // clockwise for Motor3
//    digitalWrite(in6, HIGH);
//    analogWrite(en3, 160);
//
//    digitalWrite(in7, HIGH); // Clockwise for Motor4
//    digitalWrite(in8, LOW);
//    analogWrite(en4, 160);
//  }
//  
//  else if (measured_distance < 30) {

/*    digitalWrite(in1, LOW); //Stop for Motor 1
    digitalWrite(in2, LOW);


    digitalWrite(in3, LOW); //Stop for Motor 2
    digitalWrite(in4, LOW);


    digitalWrite(in5, LOW); // Stop for Motor3
    digitalWrite(in6, LOW);


    digitalWrite(in7, LOW); // Stop for Motor4
    digitalWrite(in8, LOW);

    //additions for turing left

    delay(500);*/

    //while (Linefollowerarray[4] == 1 && turn == 0)
    //{
      digitalWrite(in1, LOW); //Clockwise for Motor 1
      digitalWrite(in2, HIGH);
      analogWrite(en1, 128);

      digitalWrite(in3, LOW); //Anticlockwise for Motor 2
      digitalWrite(in4, HIGH);
      analogWrite(en2, 0);

      digitalWrite(in5, LOW); // clockwise for Motor3
      digitalWrite(in6, HIGH);
      analogWrite(en3, 192);

      digitalWrite(in7, HIGH); // Anticlockwise for Motor4
      digitalWrite(in8, LOW);
      analogWrite(en4, 192);

      delay(1000);
    //}
    turn = 1;
    digitalWrite(in1, LOW); //Stop for Motor 1
    digitalWrite(in2, LOW);


    digitalWrite(in3, LOW); //Stop for Motor 2
    digitalWrite(in4, LOW);


    digitalWrite(in5, LOW); // Stop for Motor3
    digitalWrite(in6, LOW);


    digitalWrite(in7, LOW); // Stop for Motor4
    digitalWrite(in8, LOW); 
    
}

//delay (1000);
//}
// // put your main code here for speed control using PID
//
//  input = analogRead(A0);                //read from rotary encoder connected to A0
//  output = computePID(input);
//  delay(100);
//  analogWrite(en1, output); // PID Tuning is Going Here; var = 200 is default; writing
//double computePID(double inp){ //Only for PID Control
//        currentTime = millis();                //get current time
//        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
//
//        error = Setpoint - inp;                                // determine error
//        cumError += error * elapsedTime;                // compute integral
//        rateError = (error - lastError)/elapsedTime;   // compute derivative
//
//        double out = kp*error + ki*cumError + kd*rateError;                //PID output
//
//        lastError = error;                                //remember current error
//        previousTime = currentTime;                        //remember current time
//
//        return out;                                        //have function return the PID output
//}
