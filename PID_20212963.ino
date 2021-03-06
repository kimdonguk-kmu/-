#include <Servo.h> // [2972] 서보 헤더파일 포함

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 // [1234] LED를 아두이노 GPIO 9번 핀에 연결
                  // [2345] ...하면 개선
#define PIN_SERVO 10 // [2951] servo moter를 아두이노 GPIO 10번 핀에 연결
#define PIN_IR A0      // [2961] 적외선센서를 아두이노 A0핀에 연결

// Framework setting
#define _DIST_TARGET 255 //[2967] 탁구공 위치까지의 거리를 255로 고정
         // [2952] 탁구공의 목표 위치를 25.5cm로 설정
#define _DIST_MIN 100 // [2972] 거리 센서가 인식 가능하게 설정한 최소 거리
#define _DIST_MAX 410 // [2972] 거리 센서가 인식 가능하게 설정한 최대 거리

// Distance sensor
#define _DIST_ALPHA 0.4  // [2959] ema 필터에 적용할 알파값

// Servo range
#define _DUTY_MIN 1000 // [2952] 서보의 최소 각도값
#define _DUTY_NEU 1500 // [2952] 서보의 중간 각도값
#define _DUTY_MAX 2000 // [1691] 서보의 최대 각도

// Servo speed control
#define _SERVO_ANGLE 30  //[2967] 서보 각도 설정
#define _SERVO_SPEED 50  //[2959] 서보의 속도 설정

// Event periods
#define _INTERVAL_DIST 20  //[2959] 센서의 거리측정 인터벌값
#define _INTERVAL_SERVO 20 //[2967] 서보 INTERVAL값 설정
#define _INTERVAL_SERIAL 100  //[2959] 시리얼 모니터/플로터의 인터벌값 설정

// PID parameters
#define _KP 0.3

// 실제 거리가 100mm, 400mm일 때 센서가 읽는 값(각 a, b)
#define a 69.42
#define b 190.42

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  // [2972] 서보 정의

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, dist_cali;    // [2961] dist_raw : 적외선센서로 얻은 거리를 저장하는 변수
                             // [2961] dist_ema : 거리를 ema필터링을 한 값을 저장하는 변수
float alpha;    // [2959] ema의 알파값을 저장할 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; // [2952] 한 주기당 변화할 서보 활동량을 정의
int duty_target, duty_curr; // [2961] 목표위치, 서보에 입력할 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED, OUTPUT); // [2952] LED를 GPIO 9번 포트에 연결
myservo.attach(PIN_SERVO); // [2952] 서보 모터를 GPIO 10번 포트에 연결

duty_target = 1600;
  duty_curr = 1475;

// initialize global variables
alpha = _DIST_ALPHA;   // [2959] ema의 알파값 초기화
dist_ema = 0;          // [2959] dist_ema 초기화

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU); // [2952] 서보 모터를 중간 위치에 지정

// initialize serial port
Serial.begin(115200); // [2952] 시리얼 포트를 115200의 속도로 연결

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);                // [2959] 한 주기마다 이동할 양(180.0, 1000.0은 실수타입이기 때문에 나눗셈의 결과가 실수타입으로 리턴)
// [2974] INTERVAL -> _INTERVAL_SERVO 로 수정

// [2974] 이벤트 변수 초기화
  last_sampling_time_dist = 0; // [2974] 마지막 거리 측정 시간 초기화
  last_sampling_time_servo = 0; // [2974] 마지막 서보 업데이트 시간 초기화
  last_sampling_time_serial = 0; // [2974] 마지막 출력 시간 초기화
  event_dist = event_servo = event_serial = false; // [2974] 각 이벤트 변수 false로 초기화
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
unsigned long time_curr = millis();  // [2964] event 발생 조건 설정
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true; // [2957] 거리 측정 주기가 됐다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true; // [2957] 서보모터 제어 주기가 됐다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true; // [2957] 출력주기가 됐다는 이벤트 발생
  }

////////////////////
// Event handlers //
////////////////////

  if(event_dist) { 
     event_dist = false;
  // get a distance reading from the distance sensor
      dist_raw = ir_distance_filtered();// [2959] dist_raw에 필터링된 측정값 저장
      dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
      if (dist_ema == 0){            // [2959] 맨 처음
        dist_ema = dist_cali;              // [2959] 맨 처음 ema값 = 필터링된 측정값
      }
      else{
        dist_ema = alpha * dist_cali + (1-alpha) * dist_ema;   // [2959] ema 구현
      }
  // PID control logic
    error_curr = dist_ema - _DIST_TARGET;
    pterm = error_curr;
    control = _KP*error_curr;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU - control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  if (duty_target > _DUTY_MAX) {duty_target = _DUTY_MIN;}
  else if (duty_target < _DUTY_MIN) {duty_target = _DUTY_MAX;}

  }
  
  if(event_servo) {
    event_servo = false; // [2974] 서보 이벤트 실행 후, 다음 주기를 기다리기 위해 이벤트 종료
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {  // [2964] 현재 서보 각도 읽기
      duty_curr += duty_chg_per_interval+1; // [2961] duty_curr은 주기마다 duty_chg_per_interval만큼 증가
      if(duty_curr > duty_target) {duty_curr = duty_target;} // [29
    }
    else {
      duty_curr -= duty_chg_per_interval-1;  // [2961] duty_curr은 주기마다 duty_chg_per_interval만큼 감소
      if (duty_curr < duty_target) {duty_curr = duty_target;}
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);  // [2964] 서보 움직임 조절

  }
  
  if(event_serial) {
    event_serial = false; // [2974] 출력 이벤트 실행 후, 다음 주기까지 이벤트 종료
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_ema); // [2957] 적외선 센서로부터 받은 값 출력
    Serial.print(",pterm:");
    Serial.print(pterm);
    Serial.print(",duty_target:"); // [2957] 목표로 하는 거리 출력
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.println(",High:310,Max:1600");
  }

}
float ir_distance(void){ // return value unit: mm
                         // [2959] 센서가 측정한 거리를 리턴해주는 함수
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0; // [2961] *10 : cm -> mm로 변환
  return val;            // [2959] 거리측정 구현
  return 100 + 300.0 / (b - a) * (val - a);  // [2964] 센서가 읽은 거리를 이용한 실제
}

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}
