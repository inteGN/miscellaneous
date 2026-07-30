//************************************************
//  FILE    :lcddisp2.c
//  DATE    :2026/07/23
//  DESCRIPTION :lcd dispray main
//  BOARD TYPE  :RL78
//  AUTHER      :inteGN
//************************************************


//Includes
#include <string.h>
#include "r_cg_macrodriver.h"
#include "Config_TAU0_2.h"
#include "Config_TAU0_7.h"
#include "Config_UART0.h"
#include "lcddisp2.h"
#include "lcdhandler.h"


//Constants
const             uint32_t        NProgramVersion = 0x00010000;         //program version 1.00
const             uint8_t         ProgramSpec[8]  = "test01";


//Globals
                  uint32_t        NProgramData;           //バージョン番号
                  uint8_t         ProgSpec[8];            //プログラム仕様
                  uint8_t         activation_code;
                  uint8_t         count_init;
                  uint8_t         flag_active;            // 0=inactive, 1=active
                  uint8_t         flag_wait;              // 0=normal, 1=waiting
                  uint8_t         led_perc;
                  uint16_t        TCR07_old;
                  MD_STATUS        md_status;
                  uint8_t         rx_buf[1];
                  uint8_t         CHRGET;
                  uint8_t         CHRPTR = 0;
                  uint8_t         CHRBUF[160];
volatile          uint8_t         TXPTR0 = 0;
volatile          uint8_t         TXPTR1 = 0;
volatile          uint8_t         RXPTR0 = 0;
volatile          uint8_t         RXPTR1 = 0;
volatile          uint8_t         TXBUF_LOCAL[64];
volatile          uint8_t         RXBUF_LOCAL[256];       //volatile不要かも


//Setup function
void  setup(void) {
  NProgramData = NProgramVersion;
  memcpy(ProgSpec, ProgramSpec, 8);
  P0 = 0x00;
  LCD_E  = 0;
  LCD_E2 = 0;
  if (DCSET == 0) {
    activation_code = 0x11;
  }
  else {
    activation_code = 0x12;
  }
  count_init = 0;
  flag_active = 1;
  flag_wait   = 1;
  led_perc    = 100;
  lcd_led(led_perc);
  R_Config_UART0_Start();
  md_status = R_Config_UART0_Receive(rx_buf, 1);
  R_Config_TAU0_2_Start();
  R_Config_TAU0_7_Start();
  TCR07_old = TCR07;
  EI();  
}


//loop function
void  loop(void) {
  if (flag_wait != 0) {
    if ((TCR07_old - TCR07) > STARTUP_WAIT)  {
      flag_wait = 0;
#ifdef  STARTUP_INIT
      lcd_init();
      CHRPTR = 0;
      for (int i=0;i<160;i++) {
        CHRBUF[i] = ' ';
      }
#endif
    }
  }
  else {
    if (RXPTR0 != RXPTR1) {
      CHRGET = RXBUF_LOCAL[RXPTR1];
      RXPTR1++;
      if ((CHRGET >= ctrl_dcmin) && (CHRGET <= ctrl_dcmax)) {
        if (CHRGET == activation_code) {
          flag_active = 1;
        }
        else {
          flag_active = 0;
        }
        count_init = 0;
      }
      else { 
        if (flag_active != 0) {
          if (CHRGET == ctrl_init) {
            if (++count_init >= 2) {
              count_init = 0;
              lcd_init();
              CHRPTR = 0;
              for (int i=0;i<160;i++) {
                CHRBUF[i] = ' ';
              }
            }
          }
          else {
            count_init = 0;
            set_char();
          }
        }
      }
    }
  }
}


//set one charactor to buffer
void  set_char(void) {
  if (CHRGET == ctrl_home) {
    //バッファ先頭
    CHRPTR = 0;
  }
  else if (CHRGET == ctrl_flush) {
    //バッファ描画
    lcd_flush();
    CHRPTR = 0;
  }
  else if (CHRGET < 0x20) {
    //reserved
  }
  else if (CHRGET == 0x7F) {
    //reserved  
  }
  else if ((CHRGET >= 0x88) && (CHRGET <= 0x8F)) {
    //pwm
    lcd_led((CHRGET - 0x88) * 20U);
  }
  else if ((CHRGET >= 0x90) && (CHRGET <= 0x9F)) {
    //reserved  
  }
  else {
    if ((CHRGET >= 0x80) && (CHRGET <= 0x87)) {
      //cgram
      CHRGET = CHRGET - 0x80;
    }
    if ((CHRGET >= 0x20) && (CHRPTR < 160)) {
      CHRBUF[CHRPTR] = CHRGET;
      CHRPTR++;
    }
  }
}


//set led pwm in percent
void  lcd_led(uint8_t perc) {
  if (perc >= 100) {
    TDR03 = _0063_TAU_TDR02_VALUE + 1;
  }
  else {
    TDR03 = (uint16_t)perc;
  }
}


//receive callback function 
void  getrxdata(void) {
  if ((RXPTR0 + 1) != RXPTR1) {
    RXBUF_LOCAL[RXPTR0] = rx_buf[0];
    RXPTR0++;
  }
  md_status = R_Config_UART0_Receive(rx_buf, 1);
}




