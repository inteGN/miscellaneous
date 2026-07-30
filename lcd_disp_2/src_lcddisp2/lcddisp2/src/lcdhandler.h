//************************************************
//  FILE    :lcdhandler.h
//  DATE    :2026/07/23
//  DESCRIPTION :header of lcd handler
//  BOARD TYPE  :RL78
//  AUTHER      :inteGN
//************************************************


#ifndef LCDHNDL_H
#define LCDHNDL_H


//Includes
#include "r_cg_macrodriver.h"


//Definitions
#define           LCD_E           P12_bit.no2
#define           LCD_E2          P12_bit.no1
#define           DCSET           P13_bit.no7


//Prototypes
void  lcd_init(void);
void  lcd_flush(void);
void  flush_1602(void);
void  flush_2004(void);
void  flush_4004(void);
void  wrtlcd4(uint8_t rs, uint8_t wrtdata);
void  wrtlcd8(uint8_t rs, uint8_t wrtdata);
void  cadset(uint8_t addr);
void  dadset(uint8_t addr);
void  delay(uint16_t time);




#endif


