#include <Arduino.h> 
#include <PS4Controller.h>
#include <SMS_STS.h> // STServo.zipより
#include <Adafruit_TSL2591.h> //カラーセンサー

#define S2_TX 16
#define S2_RX 17
#define S0_TX 0
#define S0_RX 1

#define AUTO_SWITCH_MS 7500 // 起動後 何ms で自動に切り替えるか

// define global status

bool automationEnable = false; // falseの場合は自動化しない (ラズパイは虚空に送り続ける)
bool arm_status; // これは後で位置を確認するので未定義。
SMS_STS Servo;
int prev_ms;
int sw_time;
int delta_sw_time;

// put function declarations here:
int myFunction(int, int);
int8_t culc_checksum(int8_t*);
bool test_checksum(int8_t*);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // to PC, or RaspberryPi
  Serial2.begin(115200,134217756U,S2_RX,S2_TX); // to ESP32
  PS4.begin("90:15:06:7c:3e:26"); // PS4 Controller

  // for FEETECH Servo (STS3215 12V)
  // Servo.Ping(0); //アームは1～6を使用、ベルトコンベアは知らん。
  prev_ms = millis();
  sw_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  uint8_t movement[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
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

    // MODE 2 | アームメイン用 キーコンフィグ
    #define KEY_ARM_HOLD PS4.Circle() //アームでつかむ・はなすを切り替えます。はなす場合は後述の解放キーを同時に押す必要があります。
    #define KEY_ARM_REL_LOCK PS4.R1() //アームで話す場合の安全ロックです。ボタンまたはTrueが設定できます。Trueを代入すると無効になります。
    #define STICK_ARM_FRONT PS4.LStickY() //アーム先端を操縦時、前に進む勢いを示します。スティックが設定できます。
    #define STICK_ARM_RIGHT PS4.LStickX() //アーム先端を操縦時、右に進む勢いを示します。スティックが設定できます。
    #define KEY_ARM_UP PS4.Up() //アームの根元を前へ進めたり、アーム先端を上げたりします。
    #define KEY_ARM_DW PS4.Uown() //アームの根元を後へ進めたり、アーム先端を下げたりします。
    #define KEY_ARM_SW PS4.Share() //押されてる間、アームの根元を移動させます。ボタンが設定できます。

    // MODE 2+| 予備
    #define KEY_RAIL_ON PS4.Square() //レールを起動します
    #define KEY_RAIL_OFF PS4.Triangle() //レールをオフにします

    #define KEY_MODE_SW PS4.Options() //単押しで運転モードとアームモードを切り替えます。長押しすると自動化を起動します。
    #define KEY_MODE_SW_TM 2000 // ↑の長押しの基準

    static int mode_ = 1;
    static int mode_sw_bt = 0; // 押されたらその時のmillis()に書き換える

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
            mode_==1;
          }else{
            mode_+=1;
          }
        }
        mode_sw_bt=0;
      }
    }
    
    if(mode_==2){
      // やだああああああああああああああああああああ
    }else{
      PS4.setLed(0xff,0xb7,0x00);
      if (GUAGE_DRIVE_ACCEL)
      {
        // 加速する。優先度は回転>微調整>スティック
        // movement[1]には操作の種類、[2],[3]はX,Y成分、[4]には速度(場合によっては時計回りのみ)が入力される
        if(KEY_ROTATE_CCW&&KEY_ROTATE_CW){
          // 左右回転が同時に押されたとき
          Serial.println("WHAT?");
        }else if(KEY_DRIVE_BACK&&KEY_DRIVE_FRONT){
          // 前後移動が同時に押されたとき
          Serial.println("WHAT??");
        }else if((KEY_ROTATE_CCW||KEY_ROTATE_CW)&&(KEY_DRIVE_BACK||KEY_DRIVE_FRONT)){
          // 回転移動指定と前後移動指定が同時に押されたとき
          Serial.println("WHAT???");
        }else if(KEY_ROTATE_CW){
          // 時計回りに回転
          movement[1]=0x21;
          movement[4]=(GUAGE_DRIVE_ACCEL>>1);
        }else if(KEY_ROTATE_CCW){
          movement[1]=0x21;
          movement[4]=(0x100-(GUAGE_DRIVE_ACCEL>>1))&0xff; // 0xff(=-1)
        }else if(KEY_DRIVE_FRONT){
          movement[1]=0x20;
          movement[2]=0x7f;
          movement[3]=0x00;
          movement[4]=(GUAGE_DRIVE_ACCEL>>1);
        }else if(KEY_DRIVE_BACK){
          movement[1]=0x20;
          movement[2]=0x100-0x7f;
          movement[3]=0x00;
          movement[4]=(GUAGE_DRIVE_ACCEL>>1);
        }else if((STICK_DRIVE_FRONT>=16)||(STICK_DRIVE_RIGHT>=16)){
          movement[1]=0x20;
          movement[2]=STICK_DRIVE_FRONT&0xf0;
          movement[3]=STICK_DRIVE_RIGHT&0xf0;
          movement[4]=(GUAGE_DRIVE_ACCEL>>1);
        }else{
          movement[1]=0x01;
        }
      }
    }

    // PS4にカラーセンサーの色などを反映
    PS4.sendToController();
    
    // CheckSum
    movement[7]=culc_checksum(movement);
    Serial2.write(movement,8);
















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