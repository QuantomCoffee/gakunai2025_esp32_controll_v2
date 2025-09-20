#include <Arduino.h>
#include <PS4Controller.h>
#include <ESP32Encoder.h>
#include <SMS_STS.h>

#define S2_TX 16
#define S2_RX 17
#define S0_TX 0
#define S0_RX 1

#define AUTO_SWITCH_MS 7500 // 起動後 何ms で自動に切り替えるか

#define WHEEL_RADIUS 0.0 //車輪半径 (mm)
#define WHEEL_SHAFTL 0.0 //車輪間距離 (mm)
#define ENCODER_SIG_PER_ROLL 374 //ロータリエンコーダの1周当たりの信号数
#define MD_PIN0 32
#define MD_PIN1 33
#define MD_PIN2 25
#define MD_PIN3 26
#define MD_PIN_LOCK 27
#define MV_SCALE 0.35

// define global status
int prev_ms;
int sw_time;
bool move_updated;
int delta_sw_time;
bool motor_powered = false;

ESP32Encoder encoder_m0;
ESP32Encoder encoder_m1;
ESP32Encoder encoder_m2;
ESP32Encoder encoder_m3;

// put function declarations here:
int myFunction(int, int);
uint8_t culc_checksum(uint8_t*);
bool test_checksum(uint8_t*);

int move_liner(int speed, int dx, int dy);
int move_rotate(int speedClockwise);

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(115200,134217756U,S2_RX,S2_TX); // to ESP32
  Serial.begin(115200);
  
    // motor encoder
  ESP32Encoder::useInternalWeakPullResistors=puType::none;

  encoder_m0.attachSingleEdge(14,12);
  encoder_m1.attachSingleEdge(23,22);
  encoder_m2.attachSingleEdge(19,18);
  encoder_m3.attachSingleEdge(34,35);

  encoder_m0.clearCount();
  encoder_m1.clearCount();
  encoder_m2.clearCount();
  encoder_m3.clearCount();

  // motor driver
  pinMode(MD_PIN_LOCK,OUTPUT); // 基本ON... なんだけど停止時にまでやる必要はなし。
  pinMode(MD_PIN0,OUTPUT); // 0番モーター
  pinMode(MD_PIN1,OUTPUT); // 1番モーター
  pinMode(MD_PIN2,OUTPUT); // 2番モーター
  pinMode(MD_PIN3,OUTPUT); // 3番モーター

  digitalWrite(MD_PIN_LOCK,LOW);
  ledcSetup(0, 9600, 8); ledcAttachPin(MD_PIN0, 0); ledcWrite(0, 128);
  ledcSetup(1, 9600, 8); ledcAttachPin(MD_PIN1, 1); ledcWrite(1, 128);
  ledcSetup(2, 9600, 8); ledcAttachPin(MD_PIN2, 2); ledcWrite(2, 128);
  ledcSetup(3, 9600, 8); ledcAttachPin(MD_PIN3, 3); ledcWrite(3, 128);


  // for FEETECH Servo (STS3215 12V)
  // Servo.Ping(0); //アームは1～6を使用、ベルトコンベアは知らん。
  prev_ms = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint8_t movement[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
  
  // 行動パターンを決めます。1バイト目は0x58で固定です。8バイト目はチェックサムです。
  // movementは基本的にそのままESP32(下)に転送します。

  // int8_t senddata[8] = {0x58,0xfe,0x0,0x0,0x0,0x0,0x0,0x0};
  // ラズパイ/PCに返却するデータです。
  // 余裕がないので消えました。
  prev_ms = millis();
  move_updated = false;

  if (Serial2.available()) //データ受け取りが可能なら
  {
    uint8_t temp_mov[8];
    Serial2.readBytes(temp_mov,8);
    Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d (%dms) \n",temp_mov[0],temp_mov[1],temp_mov[2],temp_mov[3],temp_mov[4],temp_mov[5],temp_mov[6],temp_mov[7],millis()-sw_time);
    if(test_checksum(temp_mov)){
      for (size_t i = 0; i < 8; i++) {movement[i]=temp_mov[i];}        
    }
    move_updated = true;
    sw_time = prev_ms;
  }else{ //0.05秒ごとに送る指示出してるのに0.2秒も待たせるって何があった?
    move_updated = false;
    if((prev_ms-sw_time)>=200){
      for (size_t i = 0; i < 8; i++) {movement[i]=0x00;}        
    }
  }
  
  
  if(movement[1]==0x20){       // 0x20 平行移動
    int mvdx = movement[2]-0x80;
    int mvdy = movement[3]-0x80;
    move_liner(movement[4],mvdx,mvdy);
    if(move_updated){Serial.printf(" -> Move: liner (x: %d, y: %d, speed: %d)\n",mvdx,mvdy,movement[4]);};
  }else if(movement[1]==0x21){ // 0x21 回転移動
    move_rotate(movement[4]-0x80);
    if(move_updated){Serial.printf(" -> Move: rotate (c.w.speed: %d)\n",movement[4]-0x80);};
  }else if(movement[1]==0x2f || movement[1]==0x00){ // 0x00 不明 // 0x2f 移動停止
    digitalWrite(MD_PIN_LOCK,LOW);
    motor_powered = false;
    if(move_updated){Serial.printf(" -> Move: stop\n");};
  }





  


  // CheckSum
  // senddata[7]=culc_checksum(senddata);
  // if(Serial.available()){Serial.write(senddata,8);}

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

uint8_t culc_checksum(uint8_t* data){
  uint16_t tempval = 0;
  for (size_t i = 0; i < 7; i++){
    tempval+=data[i];
  }
  return (tempval&0xff);  
}

bool test_checksum(uint8_t* data){
  return (data[7]==culc_checksum(data));
};

int move_liner(int speed, int dx, int dy) {
  //Serial.printf("Move Liner : %d to angle %d\n",speed,axis);
  if(speed==0){
    digitalWrite(MD_PIN_LOCK,LOW);
    motor_powered = false;
    return 0;
  }else if (abs(speed)<=128){
    // memo; ledcは8bitにおいて0-255.
    // また、Locked-Antiphase PWM方式を参考にするので、0寄りがcw,256寄りがccw.
    // 誤動作防止のため、モーターのONは最後に切り替える。
    double dxad = (dx*speed)/128.0;
    double dyad = (dy*speed)/128.0;

    /*
        モーター配置
          前(x+)
        #0 v v #1
         >     <
         >     <
        #3 ^ ^ #2
    
    */
    // モーターへ方向を入力               //  前 - 右
    ledcWrite(0, 128+round((-dxad-dyad)*MV_SCALE)); //  cw - cw
    ledcWrite(1, 128+round((+dxad-dyad)*MV_SCALE)); // ccw - cw
    ledcWrite(2, 128+round((+dxad+dyad)*MV_SCALE)); // ccw - ccw
    ledcWrite(3, 128+round((-dxad+dyad)*MV_SCALE)); //  cw - ccw

    if(!motor_powered){ // モーター起動
      digitalWrite(MD_PIN_LOCK,HIGH);
      motor_powered = true; 
    }

    return 0;
  }else{
    return -1;
  }
}

int move_rotate(int speedClockwise) {
  if(speedClockwise==0){
    digitalWrite(MD_PIN_LOCK,LOW);
    motor_powered = false;
    return 0;
  }else if (abs(speedClockwise)<=128){
    // memo; ledcは8bitにおいて0-255.
    // また、Locked-Antiphase PWM方式を参考にするので、0寄りがcw,256寄りがccw.
    // 誤動作防止のため、モーターのONは最後に切り替える。
    
    // モーターへ方向を入力
    ledcWrite(0, 128+speedClockwise);
    ledcWrite(1, 128+speedClockwise);
    //ledcWrite(2, 128+speedClockwise);
    //ledcWrite(3, 128+speedClockwise);

    if(!motor_powered){ // モーター起動
      digitalWrite(MD_PIN_LOCK,HIGH);
      motor_powered = true; 
    }

    return 0;
  }else{
    return -1;
  }
}