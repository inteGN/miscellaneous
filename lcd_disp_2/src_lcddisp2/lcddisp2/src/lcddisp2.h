//************************************************
//  FILE    :lcddisp2.h
//  DATE    :2026/07/23
//  DESCRIPTION :header of lcd dispray main
//  BOARD TYPE  :RL78
//  AUTHER      :inteGN
//************************************************


#ifndef __LCDDISP2_H__
#define __LCDDISP2_H__


//Includes
#include "r_cg_macrodriver.h"


//Definitions
//#define    SC1602
#define    SC2004
//#define    SC4004                          //is not implemented

#define   STARTUP_WAIT            37500     //start-up wait time in microseconds
#define   INITIAL_WAIT            22500     //initialize wait time in microseconds
#define   STARTUP_INIT                      //allow start-up initialize if this defined
#define   CLEAR_AFTER_WRITE                 //allow clear charactor buffer after write

#define           ctrl_home       0x0B      //cursor home
#define           ctrl_flush      0x0D      //flushing
#define           ctrl_dcmin      0x11      //DC1
#define           ctrl_dcmax      0x14      //DC4
#define           ctrl_init       0x16      //initialize


//Prototypes
void  setup(void);
void  loop(void);
void  set_char(void);
void  lcd_led(uint8_t perc);
void  getrxdata(void);


//externs
extern            uint8_t         CHRPTR;
extern            uint8_t         CHRBUF[160];



#endif

