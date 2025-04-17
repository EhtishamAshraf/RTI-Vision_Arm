/*
Test 2: 
     Control the xy platform using Arduino, the code also works when controlling it with ROS

Command Action:
              xl10  = Move left 10 steps
              xr10  = Move right 10 steps
              yu10  = Move up 10 steps
              yd10  = Move down 10 steps
              ii  = Move to home position
              ix  = Initialize X-axis
              iy  = Initialize Y-axis
              cx100 = Set X calibration to 100
              cy100 = Set Y calibration to 100
              mx5 = Move to X = 5
              my5 = Move to Y = 5
*/

// Stepper motor pins:
int x1 = 5; //PUL1
int x2 = 6; //DIR1
int x3 = 9; //PUL2
int x4 = 8; //DIR2

double delayTime = 1;  // delay

String com;
String line1;
String line2;

// Sensor pins:
int sensorY = 4;
int sensorX = 3;
int sensor_y_val = analogRead(sensorY);
int sensor_x_val = analogRead(sensorX);

double x_calibration = 83.333333;
double y_calibration = 83.333333;

double x_current_position = 0.0;
double y_current_position = 0.0;

String feedback = "";

//dict - i stands for initialize, c stands for calibration (length to cycles), u - up, d - down, l - left and r- right

void setup() 
{
  pinMode(x1, OUTPUT);
  pinMode(x2, OUTPUT);
  pinMode(x3, OUTPUT);
  pinMode(x4, OUTPUT);

  Serial.begin(9600);
  
}
// Functions to control the platform based on the input steps or based on sensor's value

void move_left(int steps)
{
  digitalWrite(x2, LOW); // changed from HIGH to LOW

  for(int i=0; i<steps; i++){
    digitalWrite(x1, HIGH);
    delay(delayTime);
    digitalWrite(x1, LOW);
    delay(delayTime);
  }
  
}

void move_right(int steps)
{
  digitalWrite(x2, HIGH); // changed from LOW to HIGH
  
  for(int i=0; i<steps; i++){
    digitalWrite(x1, HIGH);
    delay(delayTime);
    digitalWrite(x1, LOW);
    delay(delayTime);
  }
  
}

void move_up(int steps)
{
  digitalWrite(x4, LOW); // changed from HIGH to LOW
  
  for(int i=0; i<steps; i++){
    digitalWrite(x3, HIGH);
    delay(delayTime);
    digitalWrite(x3, LOW);
    delay(delayTime);
  }
  
}

void move_down(int steps)
{
  digitalWrite(x4, HIGH); // changed from LOW to HIGH
  
  for(int i=0; i<steps; i++){
    digitalWrite(x3, HIGH);
    delay(delayTime);
    digitalWrite(x3, LOW);
    delay(delayTime);
  }
  
}

void move_to_home()
{
  sensor_y_val = analogRead(sensorY);
  sensor_x_val = analogRead(sensorX);

  digitalWrite(x2, LOW);  // CHANGED FROM HIGH TO LOW
  
  while(sensor_x_val>10){
    digitalWrite(x1, HIGH);
    delay(delayTime);
    digitalWrite(x1, LOW);
    delay(delayTime);  
    sensor_x_val = analogRead(sensorX);
  }

  digitalWrite(x4, HIGH); // CHANGED FROM LOW TO HIGH

  while(sensor_y_val>10){
    digitalWrite(x3, HIGH);
    delay(delayTime);
    digitalWrite(x3, LOW);
    delay(delayTime);
    sensor_y_val = analogRead(sensorY);
  }

  x_current_position = 0.0;
  y_current_position = 0.0;
  
}

void init_x()
{
  sensor_x_val = analogRead(sensorX);

  
  digitalWrite(x2, LOW); // CHANGED FROM HIGH TO LOW

  while(sensor_x_val>10){
    digitalWrite(x1, HIGH);
    delay(delayTime);
    digitalWrite(x1, LOW);
    delay(delayTime);
    sensor_x_val = analogRead(sensorX);
  }

  x_current_position = 0.0;
  
}

void init_y()
{
  sensor_y_val = analogRead(sensorY);
  
  digitalWrite(x4, HIGH);
  
  while(sensor_y_val>10){
    digitalWrite(x3, HIGH);
    delay(delayTime);
    digitalWrite(x3, LOW);
    delay(delayTime);  
    sensor_y_val = analogRead(sensorY);
  }

  y_current_position = 0.0;
  
}

void move_x(double x_target_position)
{
  double x_displacement = x_target_position - x_current_position;
  
  if (x_displacement > 0.0){
    int steps = x_displacement * x_calibration;
    move_right(steps);    
  }
   
  if (x_displacement < 0.0){
    int steps = abs(x_displacement) * x_calibration;
    move_left(steps);
  }
  
  if(x_target_position == 0){
    init_x();
  }

  x_current_position = x_target_position;
  Serial.println("x: " + String(x_current_position));
    
}

void move_y(double y_target_position)
{
  double y_displacement = y_target_position - y_current_position;
  
  if (y_displacement > 0.0){
    int steps = y_displacement * y_calibration;
    move_up(steps);
  }
   
  if (y_displacement < 0.0){
    int steps = abs(y_displacement) * y_calibration;
    move_down(steps);
  }
  
  if(y_target_position == 0){
    init_y();
  }

  y_current_position = y_target_position;
  Serial.println("y: " + String(y_current_position));
    
}


void loop() 
{


//    Serial.print("Sensor X: ");
//    Serial.println(analogRead(3)); // sensorX
//    Serial.print("Sensor Y: ");
//    Serial.println(analogRead(4)); // sensorY
//    delay(100);
  
  if(Serial.available() > 0){
    feedback = "";
    com = Serial.readString();
    line1 = com.substring(0,2);
    line2 = com.substring(2,7);   
    feedback = feedback + "Received command: " + com+ ", ";
    feedback = feedback + "Line1: " + line1+ ", ";
    feedback = feedback + "Line2: " + line2+ ", ";
    //feedback = feedback + "current system status: ";
    //feedback = feedback + "sensor_x:" + String(sensor_x_val)+ ", ";
    //feedback = feedback + "sensor_y:" + String(sensor_y_val)+ ", ";
    //feedback = feedback + "x_position:" + String(x_current_position)+ ", ";
    //feedback = feedback + "y_position:" + String(y_current_position)+ ", ";
    Serial.println(feedback); 
    
  }
  
  

  if(line1 == "xl"){
    move_left(line2.toInt());    
  }

  if(line1 == "xr"){
    move_right(line2.toInt());    
  }

  if(line1 == "yu"){
    move_up(line2.toInt());    
  }

  if(line1 == "yd"){
    move_down(line2.toInt());    
  }

  if(line1 =="ii"){
    move_to_home();
  }

  if(line1 =="ix"){
    init_x();
  }

  if(line1 =="iy"){
    init_y();
  }

  if(line1 =="cx"){
    x_calibration = line2.toDouble();
  }

  if(line1 =="cy"){
    y_calibration = line2.toDouble();
  }

  if(line1 =="mx"){
    move_x(line2.toDouble());
  }

  if(line1 =="my"){
    move_y(line2.toDouble());
  }

  sensor_y_val = analogRead(sensorY);
  sensor_x_val = analogRead(sensorX);
  
  
  com="";
  line1 = "";
  line2 = "";
}
