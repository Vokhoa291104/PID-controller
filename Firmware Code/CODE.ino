#include <SimpleKalmanFilter.h>
#include <stdio.h>
#include<string.h>

SimpleKalmanFilter filter(2, 2, 0.001);
double pulse; //đếm xung
double speed, Setpoint = 0; //tốc độ thực tế , tốc độ đặt
double E, E1, E2;
double alpha, gama, beta;
double Output, LastOutput; //PWM
const int IN1_PIN =7;  //IN1 
const int IN2_PIN =6;  //IN2
const int SPEED_PIN = 8; //ENA
int enco1 = 2; //Chân đọc Encoder A
int enco2 = 3; //Chân đọc Encoder B
int waittime = 100; // Thời gian lấy mẫu
double T = (waittime * 1.0)/1000;
float Kp_temp=0,Ki_temp=0,Kd_temp=0,Setpoint_temp=0;
unsigned long counttime; //đếm thời gian
unsigned long present;
float Kp = 0, Kd = 0, Ki = 0; // Biến chứa thông số Ki Kp Kd nhận từ HMI
String Direction;

void setup() 
{
 pinMode(enco1, INPUT);//chân đọc encoder
 pinMode(enco2, INPUT);
 pinMode(SPEED_PIN, OUTPUT);//chân PWM
 pinMode(IN1_PIN, OUTPUT);//chân DIR1
 pinMode(IN2_PIN, OUTPUT);//chân DIR2
 speed=0;
 E = 0 ; 
 E1 = 0; 
 E2 = 0;
 Output=0; 
 LastOutput=0;
  Serial.begin(9600); 
 
 
  
  attachInterrupt(digitalPinToInterrupt(enco1), PulseCount, RISING); 
  
}

 void PulseCount()
{
  pulse++;
}







void loop(){
String data ;
  while (Serial.available() > 0) {
    char c = Serial.read();
    data += c;
    delay(5);
  }
  data.trim();
if (data == NULL){
  goto after_get_data;
  }
  else{
  const int maxTokens = 10; // Số lượng token tối đa
  char* tokens[maxTokens];
  int numTokens = 0;

  
  char* dataPtr = const_cast<char*>(data.c_str()); // Chuyển đổi kiểu dữ liệu
  char* token = strtok(dataPtr, " ");
  while (token != NULL && numTokens < maxTokens) {
    tokens[numTokens] = token;
    numTokens++;
    token = strtok(NULL, " ");
    }
     Kp=atof(tokens[0]);
    Ki=atof(tokens[1]);
    Kd=atof(tokens[2]);
    Setpoint=atof(tokens[3]);
    Direction= tokens[4];
    Output=0;
    
  analogWrite(SPEED_PIN,0);
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
   
  }
   
 after_get_data:
  counttime = millis();
  if (counttime - present >= waittime)
  {
    present = counttime;
    speed = (pulse/(21.3*11))*(1/T)*60;
  
    Serial.println(String(filter.updateEstimate(speed)));
     /*Serial.println(Setpoint);
     Serial.println(Kp);
     Serial.println(Ki);
     Serial.println(Kd);
      Serial.println(Output);
      Serial.println();*/
    pulse=0;

    // E = Setpoint - speed;
    // alpha = Kp*E;
    // beta += Ki*T/2*E;
    // gama = Kd/T*(E-E1);
    // Output = alpha + beta + gama;
    // E1=E;

    // Tính toán giá trị ngõ ra PID rời rạc
    E = Setpoint - speed;
    alpha = 2*T*Kp + Ki*T*T + 2*Kd;
    beta = T*T*Ki -4*Kd -2*T*Kp;
    gama = 2*Kd;
    Output = (alpha*E + beta*E1 + gama*E2 +2*T*LastOutput)/(2*T);
    if(Output>255){
      Output=255;}
    if(Output<0){
      Output=0;}
    LastOutput = Output;
    E2=E1;
    E1=E;

   


  
  }
  analogWrite(SPEED_PIN, Output);
   if(Direction =="Thuan"){
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
   }
 if(Direction =="Nghich"){
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
   }
}
