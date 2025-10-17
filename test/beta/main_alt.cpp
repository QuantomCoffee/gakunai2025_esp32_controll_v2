#include <Arduino.h> 
#include <PS4Controller.h>
#include <SMS_STS.h> // STServo.zipより

// pin群 
#define S2_TX 16 // to ESP(lowerhalf)
#define S2_RX 17 // to ESP(lowerhalf)
#define S1_TX 18 // to FEETECH Servo
#define S1_RX 19 // to FEETECH Servo
#define S0_TX 1 // to PC or RaspberryPi
#define S0_RX 3 // to PC or RaspberryPi

// そのほか設定 
#define AUTO_SWITCH_MS 7500 // 起動後 何ms で自動に切り替えるか

// アーム用のあれこれ 
#define LEG_2 140.0f // サーボ2<->3 の長さ (mm)
#define LEG_3 190.0f // サーボ3<->4 の長さ (mm)
#define LEG_4 165.0f // サーボ4<->5 の長さ (mm)
#define LEG_s -8875.0f // - サーボ3<->4 の長さの2乗 + サーボ4<->5 の長さの2乗 (mm2)
#define PRG_2 29 // サーボ2 水平位置 (x1/4096回転)
#define PRG_3 2033 // サーボ3 水平位置 (x1/4096回転)
#define PRG_4 2129 // サーボ4 水平位置 (x1/4096回転)
#define PRG_5 3115 // サーボ5 水平位置 (x1/4096回転)
#define PRG_6 716  // サーボ6 水平位置 (x1/4096回転)
#define GER_2 -1.0f // サーボ2 ギア比 (モーター 1:n 駆動)
#define GER_3 -1.0f // サーボ3 ギア比 (モーター 1:n 駆動)
#define GER_4 -1.0f // サーボ4 ギア比 (モーター 1:n 駆動)
#define GER_5 2.0f // サーボ5 ギア比 (モーター 1:n 駆動) 6は負
#define ARM_RESETTING true // trueの場合、LIMの範囲はすべて自動で設定される。
#define DEBUG_MODE true
#define LIM_X_MIN 40.0f   // Xの最小値mm
#define LIM_X_MAX 600.0f  // Xの最大値mm
#define LIM_Y_MIN -100.0f // Yの最小値mm
#define LIM_Y_MAX 250.0f  // Yの最大値mm
#define TG_OPEN 2000
#define TG_CLOS 2600

/*
  モーター反映状況
  M1: Ok
  M2: NG ギア比: -1
  M3: Ok ギア比: -1
  M4: Ok ギア比: -1
  M5: NG ギア比: 2
  M6: NG ギア比: -2
*/


// define global status
bool automationEnable = false; // falseの場合は自動化しない (ラズパイは虚空に送り続ける)
SMS_STS Servo;
int prev_ms;  // PS4コントローラー通信タイムアウト
int loop_delta; // コントロールの時間差分
int sw_time;  // コントローラーが最後につながってた時間
int delta_sw_time;

// define arm values
float arm_pos_x; // アーム先端のX座標 (mm)
float arm_pos_y; // アーム先端のY座標 (mm)
bool is_arm_opened; // アーム開閉状況

// put function declarations here:
int myFunction(int, int);
uint8_t culc_checksum(uint8_t*);
bool test_checksum(uint8_t*);

//2乗
float csq(float x){return x*x;};
void registering_pos(uint8_t id,float radian_arg);

// Githubより。
#include <nvs.h>
#include <nvs_flash.h>
void clearNVS() {
    int err;
    err=nvs_flash_init();
    Serial.println("nvs_flash_init: " + err);
    err=nvs_flash_erase();
    Serial.println("nvs_flash_erase: " + err);
 }
// https://github.com/espressif/arduino-esp32/issues/1941


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // to PC, or RaspberryPi
  Serial1.begin(1000000,134217756U,S1_RX,S1_TX);
  Serial2.begin(115200,134217756U,S2_RX,S2_TX); // to ESP32  
  clearNVS();
  delay(50);
  
  PS4.begin("90:15:06:7c:3e:26"); // PS4 Controller

  // for FEETECH Servo (STS3215 12V)
  Servo.pSerial=&Serial1;
  Servo.WritePosEx(1,0,0);
  Servo.WritePosEx(2,0,0);

  // アーム角度計算 (1/4096回転単位)
  float DEG_3 = (Servo.Ping(3)==-1) ? 0 : (Servo.ReadPos(3)-PRG_3)/GER_3;
  float DEG_4 = (Servo.Ping(4)==-1) ? 0 : (Servo.ReadPos(4)-PRG_4)/GER_4;
  float DEG_5 = (Servo.Ping(5)==-1) ? 0 : (Servo.ReadPos(5)-PRG_5)/GER_5;

  // アーム現在地取得
  // cos,sinはラジアンを引数に取るので、PI/2048をかけて-4096~4096を-2pi～2piへ変換

  #define ROTPI 0.0015340 //=PI/2048

  if(DEG_5==0||DEG_4==0||DEG_3==0){
    arm_pos_x = 150;
    arm_pos_y = 0;
  }else{
    arm_pos_x = 
      LEG_4*cosf((DEG_5)*ROTPI) + 
      LEG_3*cosf((DEG_5+DEG_4)*ROTPI) +
      LEG_2*cosf((DEG_5+DEG_4+DEG_3)*ROTPI);
    arm_pos_y = 
      LEG_4*sinf((DEG_5)*ROTPI) + 
      LEG_3*sinf((DEG_5+DEG_4)*ROTPI) +
      LEG_2*sinf((DEG_5+DEG_4+DEG_3)*ROTPI);
  }

  registering_pos(1,TG_CLOS);
  is_arm_opened = false;

  // Servo.Ping(1); //アームは1～6を使用、ベルトコンベアは知らん。
  prev_ms = millis(); 
  loop_delta = millis();
  sw_time = millis();
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:

  uint8_t movement[9] = {0x58,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
  // 行動パターンを決めます。1バイト目は0x58で固定です。9バイト目はチェックサムです。
  // movementは基本的にそのままESP32(下)に転送します。

  // uint8_t senddata[9] = {0x58,0xfe,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
  // ラズパイ/PCに返却するデータです。
  // 余裕がないので消えました。

  // コントローラーはいきてますかー
  if (PS4.isConnected())
  {
    // 死活タイマーリセット
    sw_time = millis();
    delta_sw_time = sw_time + 0;
    // senddata[6]=0x10;

    // 動きを指定(というか直接動かす)

    // MODE 1 | 運転メイン用 キーコンフィグ
    #define GUAGE_DRIVE_ACCEL PS4.R2Value() //加速度を決定します。L2またはR2またはスティックが設定できます。
    #define STICK_DRIVE_FRONT PS4.LStickY() //平行移動時、前に進む勢いを示します。スティックが設定できます。
    #define STICK_DRIVE_RIGHT PS4.LStickX() //平行移動時、右に進む勢いを示します。スティックが設定できます。
    #define KEY_ROTATE_CW PS4.Right() //右(上から見て時計回り)へ回転移動します。ボタンが設定できます。
    #define KEY_ROTATE_CCW PS4.Left() //左(上から見て反時計回り)へ回転移動します。ボタンが設定できます。
    #define KEY_DRIVE_FRONT PS4.Up() //前方へ進む。ボタンが設定できます。
    #define KEY_DRIVE_BACK PS4.Down() //後方へ進む。ボタンが設定できます。
    
    // MODE 1+| キャリブレーション
    #define KEY_ADJ_STRT PS4.Share() // 押している間に、[STICK_ROTATE]の調整を行います。

    // MODE 2 | アームメイン用 キーコンフィグ
    #define KEY_ARM_HOLD PS4.Circle() //アームでつかむ・はなすを切り替えます。はなす場合は後述の解放キーを同時に押す必要があります。
    #define KEY_ARM_REL_LOCK PS4.R1() //アームで話す場合の安全ロックです。ボタンまたはTrueが設定できます。Trueを代入すると無効になります。
    #define STICK_ARM_FRONT PS4.LStickY() //アーム先端を操縦時、前に進む勢いを示します。スティックが設定できます。
    #define STICK_ROTATE PS4.LStickX() //機体を回転させます。スティックが設定できます。
    #define KEY_ARM_UP PS4.Up() //アームの根元を前へ進めたり、アーム先端を上げたりします。
    #define KEY_ARM_DW PS4.Down() //アームの根元を後へ進めたり、アーム先端を下げたりします。
    #define KEY_ARM_SW PS4.Share() //押されてる間、アームの根元を移動させます。ボタンが設定できます。

    // MODE 2a| 応急処置
    #define KEY_ARM_PRESET_1 PS4.L1() // プリセット1へ移動させます。
    #define KEY_ARM_PRESET_2 PS4.L2() // プリセット2へ移動させます。
    #define KEY_ARM_PRESET_3 PS4.PSButton() // プリセット3へ移動させます。

    const float ARMPRESET1[2] = {50.0f,320.0f};  /*X, Y(mm)*/ 
    const float ARMPRESET2[2] = {50.0f,130.0f};  /*X, Y(mm)*/ 
    const float ARMPRESET3[2] = {80.0f,-200.0f}; /*X, Y(mm)*/ 


    // MODE 2+| 予備
    // #define KEY_RAIL_ON PS4.Square() //レールを起動します
    // #define KEY_RAIL_OFF PS4.Triangle() //レールをオフにします

    // MODE 切り替え
    #define KEY_MODE_SW PS4.Options() //単押しで運転モードとアームモードを切り替えます。長押しすると自動化を起動します。
    #define KEY_MODE_SW_TM 2000 // ↑の長押しの基準


    static int mode_ = 1;
    static int mode_sw_bt = 0; // 押されたらその時のmillis()に書き換える

    static int deltasumstick = 0;
    static int deltasumcount = 0;
    static int adjusting_stick_100x = 0;

    // mode check
    if(KEY_MODE_SW){
      if(mode_sw_bt==0){
        mode_sw_bt = millis();
      }
    }else{
      if(mode_sw_bt!=0){
        if((millis()-mode_sw_bt)>=KEY_MODE_SW_TM){
          automationEnable = !automationEnable;
        }else{
          if(mode_==2){
            mode_=1;
          }else{
            mode_+=1;
          }
        }
        mode_sw_bt=0;
      }
    }
    
    if(mode_==2){
      // やだああああああああああああああああああああ
      PS4.setLed(0x30,0xff,0x30);
      if(KEY_ROTATE_CCW&&KEY_ROTATE_CW){ // 回転移動指定が同時に押されたとき
        Serial.println("WHAT???");
      }else if(KEY_ROTATE_CW){ // 時計回りに回転
        movement[1]=0x21;
        movement[4]=0x80+(GUAGE_DRIVE_ACCEL>>1);
      }else if(KEY_ROTATE_CCW){ // 反時計回りに回転
        movement[1]=0x21;
        movement[4]=0x80-(GUAGE_DRIVE_ACCEL>>1);
      }else {
        movement[4]=0x80+round(GUAGE_DRIVE_ACCEL*(STICK_ROTATE*100-adjusting_stick_100x)/12800);
      }

      // put アームの処理 here.

      

      static bool key_arm_holding = false;
      if(KEY_ARM_HOLD){
        PS4.setLed(0xff,0x00,0xff);          
        PS4.setFlashRate(200,200);
        if(KEY_ARM_REL_LOCK&&!is_arm_opened&&!key_arm_holding){ // しまってるんだったら...
          is_arm_opened=true;
          key_arm_holding=true;
        }else if(is_arm_opened&&!key_arm_holding){
          is_arm_opened=false;
          key_arm_holding=true;
        }
      }else{
        PS4.setFlashRate(0,0);
        key_arm_holding=false;
      }

      if(is_arm_opened){registering_pos(1,TG_OPEN);}
      else{registering_pos(1,TG_CLOS);}

    }else if(mode_==1){
      PS4.setLed(0xff,0xb7,0x00); // 橙色

      // 加速する。優先度は回転>微調整>スティック
      // movement[1]には操作の種類、[2],[3]はX,Y成分、[4]には速度(場合によっては時計回りのみ)が入力される
      if(KEY_ROTATE_CCW&&KEY_ROTATE_CW){ // 左右回転が同時に押されたとき
        Serial.println("WHAT?");
      }else if(KEY_DRIVE_BACK&&KEY_DRIVE_FRONT){ // 前後移動が同時に押されたとき
        Serial.println("WHAT??");
      }else if((KEY_ROTATE_CCW||KEY_ROTATE_CW)&&(KEY_DRIVE_BACK||KEY_DRIVE_FRONT)){ // 回転移動指定と前後移動指定が同時に押されたとき
        Serial.println("WHAT???");
      }else if(KEY_ROTATE_CW){ // 時計回りに回転
        movement[1]=0x21;
        movement[4]=0x80+(GUAGE_DRIVE_ACCEL>>1);
      }else if(KEY_ROTATE_CCW){ // 反時計回りに回転
        movement[1]=0x21;
        movement[4]=0x80-(GUAGE_DRIVE_ACCEL>>1);
      }else if(KEY_DRIVE_FRONT){ // 前方へ全力前進
        movement[1]=0x20;
        movement[2]=0xff; // 127
        movement[3]=0x80; //  0
        movement[4]=(GUAGE_DRIVE_ACCEL>>1);
      }else if(KEY_DRIVE_BACK){ // 後方へ全力前進
        movement[1]=0x20;
        movement[2]=0x01; //-127
        movement[3]=0x80; //   0
        movement[4]=(GUAGE_DRIVE_ACCEL>>1);
      }else if (GUAGE_DRIVE_ACCEL) { //方向指定がなければ
        if((abs(STICK_DRIVE_FRONT)>=16)||(abs(STICK_DRIVE_RIGHT)>=16)){ // とりあえず動かす
          movement[1]=0x20;
          movement[2]=STICK_DRIVE_FRONT+0x80;
          movement[3]=STICK_DRIVE_RIGHT+0x80;
          movement[4]=(GUAGE_DRIVE_ACCEL>>1);
        }
      }else { // なんもないなら止めようぜ
        movement[1]=0x2f;
      }

      // スティックのキャリブレーション
      if(KEY_ADJ_STRT){
        PS4.setFlashRate(200,200);
        deltasumcount+=1;
        deltasumstick+=STICK_ROTATE;
      }else{
        PS4.setFlashRate(0,0);
        if(deltasumcount>0){
          adjusting_stick_100x = (deltasumstick*100)/deltasumcount;
          deltasumcount = 0;
          deltasumstick = 0;
        }
      }
    }else{
      mode_=1;
    }

    // PS4に状態カラーセンサーの色などを反映
    if((millis()-prev_ms)>50){
      PS4.sendToController();
      prev_ms=millis();
    }
    
    // CheckSum
    movement[8]=culc_checksum(movement);

    // ESPにぶん投げる
    Serial2.write(movement,9);
    if ((loop_delta-millis())<25){
      delay(25);
    }
    loop_delta = millis();
  }
  if (Serial.available()&&automationEnable&&movement[0]==0x0) //何も書き込まれてなくて、自動化がOKなら。
  {
    delta_sw_time = millis() - sw_time; //さすがにノータイムで自動運転始めるとまずいので...
    if (delta_sw_time >= AUTO_SWITCH_MS) {
      // senddata[6]=0x40;
      uint8_t temp_mov[9];
      if(Serial.readBytes(temp_mov,9)==9){
        if(test_checksum(temp_mov)){
          for (size_t i = 0; i < 9; i++) {movement[i]=temp_mov[i];}        
        }
      }
      // ESPにぶん投げる
      Serial2.write(movement,8);
      if ((loop_delta-millis())<25){
        delay(25);
      }
      loop_delta = millis();

      // put アームの処理 here, again.

      
    } else {
      // senddata[6]=0x60;
    }
  }
  
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

void registering_pos(uint8_t id,float arg){
  int nxtpos = arg + 0; /*roundf(radian_arg*651.899);*/
  int nowpos = Servo.ReadPos(id);
  if(isfinite(nxtpos)){Servo.WritePosEx(id, nxtpos, 300, 150);}
}
