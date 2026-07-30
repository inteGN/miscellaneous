//************************************************
//  FILE    :lcdhandler.c
//  DATE    :2026/07/23
//  DESCRIPTION :lcd handler
//  BOARD TYPE  :RL78
//  AUTHER      :inteGN
//************************************************


//Includes
#include "r_cg_macrodriver.h"
#include "Config_TAU0_2.h"
#include "lcddisp2.h"
#include "lcdhandler.h"


//struct of lcd port
                  union {
                    uint8_t   BYTE;
                    struct {
                      uint8_t   b0      :1;       //  not use
                      uint8_t   b1      :1;       //  not use
                      uint8_t   b2      :1;       //  not use
                      uint8_t   RS      :1;       //  RS bit 0=instruction, 1=data
                      uint8_t   DATA    :4;       //  b4-b7 data bit
                    } ;
                  } LCDPORT;


//
void  lcd_init(void) {
  delay(INITIAL_WAIT);
  wrtlcd4(0, 0x30);         //function set 8bit
  delay(6150);
  wrtlcd4(0, 0x30);         //function set 8bit
  delay(150);
  wrtlcd4(0, 0x30);         //function set 8bit
  wrtlcd4(0, 0x20);         //function set 4bit
  wrtlcd8(0, 0x28);         //function set 4bit 1/16
  wrtlcd8(0, 0x08);         //display off
  wrtlcd8(0, 0x06);         //entry mode set
  wrtlcd8(0, 0x01);         //display clear
  delay(2280);
  wrtlcd8(0, 0x0C);         //display on
  wrtlcd8(1, 0xEB);         //write first charactor
}


//
void  lcd_flush(void) {
  
#ifdef SC1602
  flush_1602();
#endif

#ifdef SC2004
  flush_2004();
#endif

#ifdef SC4004
  flush_4004();
#endif

}


//
void  flush_1602(void) {
  uint8_t i;
  dadset(0);
  for (i=0;i<16;i++) {
    wrtlcd8(1, CHRBUF[i]);
#ifdef CLEAR_AFTER_WRITE
    CHRBUF[i] = ' ';
#endif
  }
  dadset(64);
  for (i=16;i<32;i++) {
    wrtlcd8(1, CHRBUF[i]);
#ifdef CLEAR_AFTER_WRITE
    CHRBUF[i] = ' ';
#endif
  }
}


//
void  flush_2004(void) {
  uint8_t i;
  dadset(0);
  for (i=0;i<20;i++) {
    wrtlcd8(1, CHRBUF[i]);
#ifdef CLEAR_AFTER_WRITE
    CHRBUF[i] = ' ';
#endif
  }
  dadset(64);
  for (i=20;i<40;i++) {
    wrtlcd8(1, CHRBUF[i]);
#ifdef CLEAR_AFTER_WRITE
    CHRBUF[i] = ' ';
#endif
  }
  dadset(20);
  for (i=40;i<60;i++) {
    wrtlcd8(1, CHRBUF[i]);
#ifdef CLEAR_AFTER_WRITE
    CHRBUF[i] = ' ';
#endif
  }
  dadset(84);
  for (i=60;i<80;i++) {
    wrtlcd8(1, CHRBUF[i]);
#ifdef CLEAR_AFTER_WRITE
    CHRBUF[i] = ' ';
#endif
  }
}


//
void  flush_4004(void) {
//is not implemented
}


//write 4 bit data to lcd
void  wrtlcd4(uint8_t rs, uint8_t data) {
  LCDPORT.RS = rs;
  LCDPORT.DATA = (data >> 4);
  P0 = LCDPORT.BYTE;
  LCD_E = 1;
  delay(1);
  LCD_E = 0;
  delay(60);
}


//write 8 bit data to lcd
void  wrtlcd8(uint8_t rs, uint8_t data) {
  LCDPORT.RS = rs;
  LCDPORT.DATA = (data >> 4);
  P0 = LCDPORT.BYTE;
  LCD_E = 1;
  delay(1);
  LCD_E = 0;
  delay(1);
  LCDPORT.DATA = (data & 0x0F);
  P0 = LCDPORT.BYTE;
  LCD_E = 1;
  delay(1);  
  LCD_E = 0;
  delay(60);
}


//set CGRAM address
void  cadset(uint8_t addr) {
  wrtlcd8(0, (addr & 0x3F) + 0x40);
  delay(55);
}


//set DDRAM address
void  dadset(uint8_t addr) {
  wrtlcd8(0, (addr & 0x7F) + 0x80);
  delay(55);
}


//delay(usec)
void  delay(uint16_t time) {
  if (time < 65534) { 
    time++;
  }
  uint16_t i = TCR07;
  while ((i - TCR07) < time)  {}
}








