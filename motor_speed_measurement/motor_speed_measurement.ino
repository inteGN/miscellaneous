//************************************************
//  FILE        :motor_speed_measurement.ino.ino
//  DATE        :2026/04/07
//  DESCRIPTION :motor speed measurement by timer GPT capture
//  BOARD TYPE  :NANO R4
//  AUTHER      :inteGN
//************************************************
/*
このプログラムは、Arduino NANO R4 (RA4M1)でGPT4タイマーを使用してモータ回転数を測定します。
D1/GPT4_Aピンの立ち下がりエッジ毎にタイムスタンプが付けられ、連続するエッジ間の間隔がモータ回転数に変換されます。

This program measures motor rotational speed on the Arduino NANO R4 (RA4M1) using the GPT4 timer.
Each falling edge on the pin D1/GPT4_A is timestamped, and the interval between consecutive edges is converted into motor speed.
*/

//// Pin connection
//  - D1ピンにモータ回転の信号を入力
//  - input motor revolution signal to pin D1


//// Includes
#include <Arduino.h>
#include "FspTimer.h"
#include "pwm.h"

//// Definitions
#define   minPulse      6000                  //2 msec: minimum valid pulse interval, 0.333 microsecond/LSB
#define   timeOver      7500000               //2.5 sec: timeout threshold to indicate wheel stop, 0.333 microsecond/LSB

//// Grobals
FspTimer  timer4;
volatile  uint32_t capt_time = 0;             //captured time
volatile  uint32_t capt_width = 0xFFFFFFFF;   //captured pulse width, set 0xFFFFFFFF as specific marker of timeout
volatile  bool     capt_flag  = false;        //flag indicating new capture data is available
uint32_t           daout_rpm = 0;             //DAC output value

//// ISR tasks to capture pluse timing and count of timer roll over
void onCallback(timer_callback_args_t *args) {
  static uint32_t extCount   = 0;             //extended counter for 32-bit emulation
  static uint32_t extCountup = 0;             //additional increment when overflow and capture occur simultaneously
  static uint32_t captValue_new;
  static uint32_t captValue_old;
  static uint32_t timeCheck = 0;              //timeover base point of previous pulse signal or timeout
  static bool capt_valid = false;
  if (args->event == TIMER_EVENT_CAPTURE_A) {
    if (R_GPT4->GTST_b.TCFPO == 1) {      //**This part handles the edge case that capture occurs just after overflow.
      if (args->capture < 0x8000) {
        extCountup = 0x00010000;              //overflow occurred just before capture
      }
      else {
        extCountup = 0;
      }
    }
    if ((uint32_t)args->capture + extCount + extCountup - captValue_new > minPulse) {   //reject pulses shorter than the minimum valid
      captValue_old = captValue_new;
      captValue_new = (uint32_t)args->capture + extCount + extCountup;
      if (capt_valid == true) {
        capt_time  = captValue_new;
        capt_width = captValue_new - captValue_old;
        capt_flag  = true;
      }
      timeCheck = captValue_new;              //set next timeover base point
      capt_valid = true;
    }
  }
  else if (args->event == TIMER_EVENT_CYCLE_END) {
    R_GPT4->GTST_b.TCFPO = 0;
    extCount += 0x00010000;                   //extend counter by 16-bit overflow
    if (extCount - timeCheck > timeOver) {
      capt_time  = extCount;
      capt_width = 0xFFFFFFFF;                //set spesific value of timeout
      capt_flag  = true;
      timeCheck = extCount;                   //set next timeover base point
      capt_valid = false;
    }
  }
}

//// Setup function
void setup() {
//Setup general port
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  analogWrite(DAC, 0);
  Serial.begin(115200);
  delay(2000);
//timer configurate: use Arduino pin D1 / RA4M1 P302 GPT4_A as capture
  noInterrupts();
  bool rv = timer4.begin(TIMER_MODE_PWM, GPT_TIMER, 4, 0, 0, TIMER_SOURCE_DIV_16, onCallback);
  if (rv) {
    auto cfg = timer4.get_cfg();
    auto ext = (gpt_extended_cfg_t*)cfg->p_extend;
    ext->capture_filter_gtioca = GPT_CAPTURE_FILTER_PCLKD_DIV_4;     //(4 x 3 + 1) clocks of delay to sample
    timer4.set_source_capture_a((gpt_source_t)(GPT_SOURCE_GTIOCA_FALLING_WHILE_GTIOCB_LOW | GPT_SOURCE_GTIOCA_FALLING_WHILE_GTIOCB_HIGH));
    timer4.setup_capture_a_irq(4);
    timer4.setup_overflow_irq(4);
    timer4.open();
    timer4.start();
    R_GPT4->GTST_b.TCFPO = 0;
    pinPeripheral(D1, (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));     //GPT4_A input
  }
  interrupts();
  if (rv == false) {
    Serial.println("Initialization failed.");
    while (1) {}
  }
//dump GPT registers
  Serial.print("R_GPT4->GTPR: "); Serial.println(R_GPT4->GTPR, HEX);
  Serial.print("R_GPT4->GTIOR: "); Serial.println(R_GPT4->GTIOR, HEX);
  Serial.print("R_GPT4->GTICASR: "); Serial.println(R_GPT4->GTICASR, HEX);
  Serial.println();
  
}

//// Loop function
void loop() {
//show captured value and revolution speed in rpm
  if (capt_flag) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("capt_time: "); Serial.print(capt_time); Serial.print(" clock   ");
    Serial.print("capt_width: "); Serial.print(capt_width); Serial.print(" clock   ");
    Serial.print("revolution: "); Serial.print(3000000.0f / (float)capt_width * 60.0f); Serial.println(" rpm");
    daout_rpm = (uint32_t)(3000000.0f / (float)capt_width * 60.0f / 50000.0f * 4096.0f + 0.5f);
    if (daout_rpm > 0x0FFF) {daout_rpm = 0x0FFF;}
    R_DAC->DADR[0] = (uint16_t)daout_rpm;
    digitalWrite(LED_BUILTIN, LOW);
    capt_flag = false;
  }
  delayMicroseconds(10);
}


