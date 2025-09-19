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
#define LEG_2 0.0 // サーボ2<->3 の長さ (mm)
#define LEG_3 0.0 // サーボ3<->4 の長さ (mm)
#define LEG_4 0.0 // サーボ4<->5 の長さ (mm)
#define LEG_s 0.0 // サーボ3<->4 の長さの2乗 + サーボ4<->5 の長さの2乗 (mm2)
#define PRG_2 -0 // サーボ2 水平位置 (x1/4096回転)
#define PRG_3 -0 // サーボ3 水平位置 (x1/4096回転)
#define PRG_4 -0 // サーボ4 水平位置 (x1/4096回転)
#define PRG_5 -0 // サーボ5 水平位置 (x1/4096回転)
#define GER_2 1.0 // サーボ2 ギア比 (モーター 1:n 駆動)
#define GER_3 1.0 // サーボ3 ギア比 (モーター 1:n 駆動)
#define GER_4 1.0 // サーボ4 ギア比 (モーター 1:n 駆動)
#define GER_5 1.0 // サーボ5 ギア比 (モーター 1:n 駆動)
#define LIM_X_MIN 0.0 // 40.0 // Xの最小値mm
#define LIM_X_MAX 0.0 // 200.0 // Xの最大値mm
#define LIM_Y_MIN 0.0 // -100.0 // Yの最小値mm
#define LIM_Y_MAX 0.0 // 450.0 // Xの最大値mm


// define global status
bool automationEnable = false; // falseの場合は自動化しない (ラズパイは虚空に送り続ける)
bool arm_status; // これは後で位置を確認するので未定義。
SMS_STS Servo;
int prev_ms;  // PS4コントローラー通信タイムアウト
int loop_delta; // コントロールの時間差分
int sw_time;  // コントローラーが最後につながってた時間
int delta_sw_time;

// put function declarations here:
int myFunction(int, int);
uint8_t culc_checksum(uint8_t*);
bool test_checksum(uint8_t*);

//2乗
float csq(float x){return x*x;};
void registering_pos(uint8_t id,float radian_arg);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // to PC, or RaspberryPi
  Serial1.begin(1000000,134217756U,S1_RX,S1_TX);
  Serial2.begin(115200,134217756U,S2_RX,S2_TX); // to ESP32
  PS4.begin("90:15:06:7c:3e:26"); // PS4 Controller

  // for FEETECH Servo (STS3215 12V)
  Servo.pSerial=&Serial1;
  Servo.WritePosEx(1,0,0);
  Servo.WritePosEx(2,0,0);

  // Servo.Ping(1); //アームは1～6を使用、ベルトコンベアは知らん。
  prev_ms = millis(); 
  loop_delta = millis();
  sw_time = millis();
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  uint8_t movement[8] = {0x58,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
  // 行動パターンを決めます。1バイト目は0x58で固定です。8バイト目はチェックサムです。
  // movementは基本的にそのままESP32(下)に転送します。

  // uint8_t senddata[8] = {0x58,0xfe,0x0,0x0,0x0,0x0,0x0,0x0};
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
    #define KEY_ADJ_STRT PS4.Share() // 押している間に、下記STICK_ROTATEの調整を行います。

    // MODE 2 | アームメイン用 キーコンフィグ
    #define KEY_ARM_HOLD PS4.Circle() //アームでつかむ・はなすを切り替えます。はなす場合は後述の解放キーを同時に押す必要があります。
    #define KEY_ARM_REL_LOCK PS4.R1() //アームで話す場合の安全ロックです。ボタンまたはTrueが設定できます。Trueを代入すると無効になります。
    #define STICK_ARM_FRONT PS4.LStickY() //アーム先端を操縦時、前に進む勢いを示します。スティックが設定できます。
    //#define STICK_ROTATE PS4.LStickX() //機体を回転させます。スティックが設定できます。
    #define KEY_ARM_UP PS4.Up() //アームの根元を前へ進めたり、アーム先端を上げたりします。
    #define KEY_ARM_DW PS4.Down() //アームの根元を後へ進めたり、アーム先端を下げたりします。
    #define KEY_ARM_SW PS4.Share() //押されてる間、アームの根元を移動させます。ボタンが設定できます。

    // MODE 2+| 予備
    #define KEY_RAIL_ON PS4.Square() //レールを起動します
    #define KEY_RAIL_OFF PS4.Triangle() //レールをオフにします

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
      if(KEY_ROTATE_CCW&&KEY_ROTATE_CW){ // 回転移動指定と前後移動指定が同時に押されたとき
        Serial.println("WHAT???");
      }else if(KEY_ROTATE_CW){ // 時計回りに回転
        movement[1]=0x21;
        movement[4]=0x80+(GUAGE_DRIVE_ACCEL>>1);
      }else if(KEY_ROTATE_CCW){ // 反時計回りに回転
        movement[1]=0x21;
        movement[4]=0x80-(GUAGE_DRIVE_ACCEL>>1);
      }else { // なんもないなら止めようぜ
        movement[1]=0x2f;
      }

      // put アームの処理 here.

      /*
        注意：PRG_3,4,5について
        これらは「水平時の角度データ」を示す。
        角度を出す際はこの値を引く。
        例えばPRG_3=1024であれば、サーボ3が0であった場合-90°(-1024)を示す。
        もし図面とサーボ回転方向が逆であればギア比を負にすること。
      */

      // 角度計算 (1/4096回転単位)

      float DEG_3 = (Servo.ReadPos(3)-PRG_3)/GER_3;
      float DEG_4 = (Servo.ReadPos(4)-PRG_4)/GER_4;
      float DEG_5 = (Servo.ReadPos(5)-PRG_5)/GER_5;


      // 現在地取得
      // cos,sinはラジアンを引数に取るので、PI/2048をかけて-4096~4096を-2pi～2piへ変換

      #define ROTPI 0.0015340 //=PI/2048

      static float arm_pos_x = 
        LEG_2*cosf((DEG_3)*ROTPI) + 
        LEG_3*cosf((DEG_3+DEG_4)*ROTPI) +
        LEG_4*cosf((DEG_3+DEG_4+DEG_5)*ROTPI);
      static float arm_pos_y = 
        LEG_2*sinf((DEG_3)*ROTPI) + 
        LEG_3*sinf((DEG_3+DEG_4)*ROTPI) +
        LEG_4*sinf((DEG_3+DEG_4+DEG_5)*ROTPI);

      // 差分計算
      if(abs(STICK_ARM_FRONT)>=24){
        arm_pos_x += STICK_ARM_FRONT * 0.10 /*(m/s)*/ * (millis()-loop_delta); 
      }
      if(KEY_ARM_UP&&KEY_ARM_DW){
        // what?
      }else if(KEY_ARM_UP){
        arm_pos_y += 0.10 /*(m/s)*/ * (millis()-loop_delta); 
      }else if(KEY_ARM_DW){
        arm_pos_y -= 0.10 /*(m/s)*/ * (millis()-loop_delta); 
      }

      // サイズ制限上の問題
      if (arm_pos_x>LIM_X_MAX) {arm_pos_x=LIM_X_MAX;}
      else if (arm_pos_x<LIM_X_MIN) {arm_pos_x=LIM_X_MIN;}
      else if (arm_pos_y>LIM_Y_MAX) {arm_pos_y=LIM_Y_MAX;}
      else if (arm_pos_y<LIM_Y_MIN) {arm_pos_y=LIM_Y_MIN;}
      
      // 姿勢角 alpha
      float arm_arg = atanf(arm_pos_x/arm_pos_y)-(PI/2);

      float T_ARG_5,T_ARG_4,T_ARG_3,T_ARG_2;
      float CALC_A = arm_pos_x-(LEG_2*cosf(arm_arg));
      float CALC_B = arm_pos_y-(LEG_2*sinf(arm_arg));
      float C_A2_B2 = csq(CALC_A)+csq(CALC_B);
      float CALC_G = atanf(CALC_B/CALC_A);

      // 計算フェーズ
      T_ARG_5 =
        CALC_G +
        acosf(
          (C_A2_B2+LEG_s)/
          (2*LEG_4*sqrtf(C_A2_B2))
        );
      
      float CALC_H = atanf(
          (CALC_B-(LEG_4*sinf(T_ARG_5)))/
          (CALC_A-(LEG_4*cosf(T_ARG_5)))
        );
      
      T_ARG_4 = 
        -T_ARG_5 + CALC_H;
      T_ARG_3 = 
        arm_arg - CALC_H;
      T_ARG_2 = 
        -PI-arm_arg;

      // 動かす。


    }else{
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
      /*
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
      }*/
    }

    // PS4に状態カラーセンサーの色などを反映
    if((millis()-prev_ms)>50){
      PS4.sendToController();
      prev_ms=millis();
    }
    
    // CheckSum
    movement[7]=culc_checksum(movement);

    // ESPにぶん投げる
    Serial2.write(movement,8);
    if ((loop_delta-millis())<25){
      delay(25);
    }
    loop_delta = millis();
  }
  /*
  if (Serial.available()&&automationEnable&&movement[0]==0x0) //何も書き込まれてなくて、自動化がOKなら。
  {
    delta_sw_time = millis() - sw_time; //さすがにノータイムで自動運転始めるとまずいので...
    if (delta_sw_time >= AUTO_SWITCH_MS) {
      // senddata[6]=0x40;
      int8_t temp_mov[8];
      if(Serial.readBytes(temp_mov,8);==8){
        if(test_checksum(temp_mov)){
          for (size_t i = 0; i < 8; i++) {movement[i]=temp_mov[i];}        
        }
      }
    } else {
      // senddata[6]=0x60;
    }
  } */
  
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

void registering_pos(uint8_t id,float radian_arg){
  int nxtpos = roundf(radian_arg*651.899);
  int nowpos = Servo.ReadPos(id);
  while (nxtpos<0) {
    nxtpos+=4096;
  }
  while (nxtpos>4096) {
    nxtpos-=4096;
  }
  Servo.WritePosEx(id, nxtpos, 0);
}
