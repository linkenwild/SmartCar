/*******************************2012-2013, NJUT, Edu.************************** 
FileName: lcd_font.h 
Author:  孙冬梅       Version :  1.0        Date: 2014.11.30
Description:    lcd字体头文件    
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    13/12/30     1.0     文件创建   
 ******************************************************************************/ 

#ifndef LCD_FONT_H
#define LCD_FONT_H 	

#ifndef _ASCII_H
#define _ASCII_H

//常用ASCII表
//纵向取模
//偏移量:32
//大小:16*8

const unsigned char ASCII_1608[95][16]={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//" ",0
{0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x33,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00},//"!",1
{0x00,0x00,0x10,0x00,0x0C,0x00,0x06,0x00,0x10,0x00,0x0C,0x00,0x06,0x00,0x00,0x00},//""",2
{0x40,0x04,0xC0,0x3F,0x78,0x04,0x40,0x04,0xC0,0x3F,0x78,0x04,0x40,0x04,0x00,0x00},//"#",3
{0x00,0x00,0x70,0x18,0x88,0x20,0xFC,0xFF,0x08,0x21,0x30,0x1E,0x00,0x00,0x00,0x00},//"$",4
{0xF0,0x00,0x08,0x21,0xF0,0x1C,0x00,0x03,0xE0,0x1E,0x18,0x21,0x00,0x1E,0x00,0x00},//"%",5
{0x00,0x1E,0xF0,0x21,0x08,0x23,0x88,0x24,0x70,0x19,0x00,0x27,0x00,0x21,0x00,0x10},//"&",6
{0x10,0x00,0x16,0x00,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//"'",7
{0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x07,0x18,0x18,0x04,0x20,0x02,0x40,0x00,0x00},//"(",8
{0x00,0x00,0x02,0x40,0x04,0x20,0x18,0x18,0xE0,0x07,0x00,0x00,0x00,0x00,0x00,0x00},//")",9
{0x40,0x02,0x40,0x02,0x80,0x01,0xF0,0x0F,0x80,0x01,0x40,0x02,0x40,0x02,0x00,0x00},//"*",10
{0x00,0x01,0x00,0x01,0x00,0x01,0xF0,0x1F,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x00},//"+",11
{0x00,0x80,0x00,0xB0,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//",",12
{0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01},//"-",13
{0x00,0x00,0x00,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//".",14
{0x00,0x00,0x00,0x60,0x00,0x18,0x00,0x06,0x80,0x01,0x60,0x00,0x18,0x00,0x04,0x00},//"/",15
{0x00,0x00,0xE0,0x0F,0x10,0x10,0x08,0x20,0x08,0x20,0x10,0x10,0xE0,0x0F,0x00,0x00},//"0",16
{0x00,0x00,0x10,0x20,0x10,0x20,0xF8,0x3F,0x00,0x20,0x00,0x20,0x00,0x00,0x00,0x00},//"1",17
{0x00,0x00,0x70,0x30,0x08,0x28,0x08,0x24,0x08,0x22,0x88,0x21,0x70,0x30,0x00,0x00},//"2",18
{0x00,0x00,0x30,0x18,0x08,0x20,0x88,0x20,0x88,0x20,0x48,0x11,0x30,0x0E,0x00,0x00},//"3",19
{0x00,0x00,0x00,0x07,0xC0,0x04,0x20,0x24,0x10,0x24,0xF8,0x3F,0x00,0x24,0x00,0x00},//"4",20
{0x00,0x00,0xF8,0x19,0x08,0x21,0x88,0x20,0x88,0x20,0x08,0x11,0x08,0x0E,0x00,0x00},//"5",21
{0x00,0x00,0xE0,0x0F,0x10,0x11,0x88,0x20,0x88,0x20,0x18,0x11,0x00,0x0E,0x00,0x00},//"6",22
{0x00,0x00,0x38,0x00,0x08,0x00,0x08,0x3F,0xC8,0x00,0x38,0x00,0x08,0x00,0x00,0x00},//"7",23
{0x00,0x00,0x70,0x1C,0x88,0x22,0x08,0x21,0x08,0x21,0x88,0x22,0x70,0x1C,0x00,0x00},//"8",24
{0x00,0x00,0xE0,0x00,0x10,0x31,0x08,0x22,0x08,0x22,0x10,0x11,0xE0,0x0F,0x00,0x00},//"9",25
{0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x30,0xC0,0x30,0x00,0x00,0x00,0x00,0x00,0x00},//":",26
{0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//";",27
{0x00,0x00,0x00,0x01,0x80,0x02,0x40,0x04,0x20,0x08,0x10,0x10,0x08,0x20,0x00,0x00},//"<",28
{0x40,0x04,0x40,0x04,0x40,0x04,0x40,0x04,0x40,0x04,0x40,0x04,0x40,0x04,0x00,0x00},//"=",29
{0x00,0x00,0x08,0x20,0x10,0x10,0x20,0x08,0x40,0x04,0x80,0x02,0x00,0x01,0x00,0x00},//">",30
{0x00,0x00,0x70,0x00,0x48,0x00,0x08,0x30,0x08,0x36,0x08,0x01,0xF0,0x00,0x00,0x00},//"?",31
{0xC0,0x07,0x30,0x18,0xC8,0x27,0x28,0x24,0xE8,0x23,0x10,0x14,0xE0,0x0B,0x00,0x00},//"@",32
{0x00,0x20,0x00,0x3C,0xC0,0x23,0x38,0x02,0xE0,0x02,0x00,0x27,0x00,0x38,0x00,0x20},//"A",33
{0x08,0x20,0xF8,0x3F,0x88,0x20,0x88,0x20,0x88,0x20,0x70,0x11,0x00,0x0E,0x00,0x00},//"B",34
{0xC0,0x07,0x30,0x18,0x08,0x20,0x08,0x20,0x08,0x20,0x08,0x10,0x38,0x08,0x00,0x00},//"C",35
{0x08,0x20,0xF8,0x3F,0x08,0x20,0x08,0x20,0x08,0x20,0x10,0x10,0xE0,0x0F,0x00,0x00},//"D",36
{0x08,0x20,0xF8,0x3F,0x88,0x20,0x88,0x20,0xE8,0x23,0x08,0x20,0x10,0x18,0x00,0x00},//"E",37
{0x08,0x20,0xF8,0x3F,0x88,0x20,0x88,0x00,0xE8,0x03,0x08,0x00,0x10,0x00,0x00,0x00},//"F",38
{0xC0,0x07,0x30,0x18,0x08,0x20,0x08,0x20,0x08,0x22,0x38,0x1E,0x00,0x02,0x00,0x00},//"G",39
{0x08,0x20,0xF8,0x3F,0x08,0x21,0x00,0x01,0x00,0x01,0x08,0x21,0xF8,0x3F,0x08,0x20},//"H",40
{0x00,0x00,0x08,0x20,0x08,0x20,0xF8,0x3F,0x08,0x20,0x08,0x20,0x00,0x00,0x00,0x00},//"I",41
{0x00,0xC0,0x00,0x80,0x08,0x80,0x08,0x80,0xF8,0x7F,0x08,0x00,0x08,0x00,0x00,0x00},//"J",42
{0x08,0x20,0xF8,0x3F,0x88,0x20,0xC0,0x01,0x28,0x26,0x18,0x38,0x08,0x20,0x00,0x00},//"K",43
{0x08,0x20,0xF8,0x3F,0x08,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x30,0x00,0x00},//"L",44
{0x08,0x20,0xF8,0x3F,0xF8,0x00,0x00,0x3F,0xF8,0x00,0xF8,0x3F,0x08,0x20,0x00,0x00},//"M",45
{0x08,0x20,0xF8,0x3F,0x30,0x20,0xC0,0x00,0x00,0x07,0x08,0x18,0xF8,0x3F,0x08,0x00},//"N",46
{0xE0,0x0F,0x10,0x10,0x08,0x20,0x08,0x20,0x08,0x20,0x10,0x10,0xE0,0x0F,0x00,0x00},//"O",47
{0x08,0x20,0xF8,0x3F,0x08,0x21,0x08,0x01,0x08,0x01,0x08,0x01,0xF0,0x00,0x00,0x00},//"P",48
{0xE0,0x0F,0x10,0x18,0x08,0x24,0x08,0x24,0x08,0x38,0x10,0x50,0xE0,0x4F,0x00,0x00},//"Q",49
{0x08,0x20,0xF8,0x3F,0x88,0x20,0x88,0x00,0x88,0x03,0x88,0x0C,0x70,0x30,0x00,0x20},//"R",50
{0x00,0x00,0x70,0x38,0x88,0x20,0x08,0x21,0x08,0x21,0x08,0x22,0x38,0x1C,0x00,0x00},//"S",51
{0x18,0x00,0x08,0x00,0x08,0x20,0xF8,0x3F,0x08,0x20,0x08,0x00,0x18,0x00,0x00,0x00},//"T",52
{0x08,0x00,0xF8,0x1F,0x08,0x20,0x00,0x20,0x00,0x20,0x08,0x20,0xF8,0x1F,0x08,0x00},//"U",53
{0x08,0x00,0x78,0x00,0x88,0x07,0x00,0x38,0x00,0x0E,0xC8,0x01,0x38,0x00,0x08,0x00},//"V",54
{0xF8,0x03,0x08,0x3C,0x00,0x07,0xF8,0x00,0x00,0x07,0x08,0x3C,0xF8,0x03,0x00,0x00},//"W",55
{0x08,0x20,0x18,0x30,0x68,0x2C,0x80,0x03,0x80,0x03,0x68,0x2C,0x18,0x30,0x08,0x20},//"X",56
{0x08,0x00,0x38,0x00,0xC8,0x20,0x00,0x3F,0xC8,0x20,0x38,0x00,0x08,0x00,0x00,0x00},//"Y",57
{0x10,0x20,0x08,0x38,0x08,0x26,0x08,0x21,0xC8,0x20,0x38,0x20,0x08,0x18,0x00,0x00},//"Z",58
{0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x7F,0x02,0x40,0x02,0x40,0x02,0x40,0x00,0x00},//"[",59
{0x00,0x00,0x0C,0x00,0x30,0x00,0xC0,0x01,0x00,0x06,0x00,0x38,0x00,0xC0,0x00,0x00},//"\",60
{0x00,0x00,0x02,0x40,0x02,0x40,0x02,0x40,0xFE,0x7F,0x00,0x00,0x00,0x00,0x00,0x00},//"]",61
{0x00,0x00,0x00,0x00,0x04,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x04,0x00,0x00,0x00},//"^",62
{0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80},//"_",63
{0x00,0x00,0x02,0x00,0x02,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//"`",64
{0x00,0x00,0x00,0x19,0x80,0x24,0x80,0x22,0x80,0x22,0x80,0x22,0x00,0x3F,0x00,0x20},//"a",65
{0x08,0x00,0xF8,0x3F,0x00,0x11,0x80,0x20,0x80,0x20,0x00,0x11,0x00,0x0E,0x00,0x00},//"b",66
{0x00,0x00,0x00,0x0E,0x00,0x11,0x80,0x20,0x80,0x20,0x80,0x20,0x00,0x11,0x00,0x00},//"c",67
{0x00,0x00,0x00,0x0E,0x00,0x11,0x80,0x20,0x80,0x20,0x88,0x10,0xF8,0x3F,0x00,0x20},//"d",68
{0x00,0x00,0x00,0x1F,0x80,0x22,0x80,0x22,0x80,0x22,0x80,0x22,0x00,0x13,0x00,0x00},//"e",69
{0x00,0x00,0x80,0x20,0x80,0x20,0xF0,0x3F,0x88,0x20,0x88,0x20,0x88,0x00,0x18,0x00},//"f",70
{0x00,0x00,0x00,0x6B,0x80,0x94,0x80,0x94,0x80,0x94,0x80,0x93,0x80,0x60,0x00,0x00},//"g",71
{0x08,0x20,0xF8,0x3F,0x00,0x21,0x80,0x00,0x80,0x00,0x80,0x20,0x00,0x3F,0x00,0x20},//"h",72
{0x00,0x00,0x80,0x20,0x98,0x20,0x98,0x3F,0x00,0x20,0x00,0x20,0x00,0x00,0x00,0x00},//"i",73
{0x00,0x00,0x00,0xC0,0x00,0x80,0x80,0x80,0x98,0x80,0x98,0x7F,0x00,0x00,0x00,0x00},//"j",74
{0x08,0x20,0xF8,0x3F,0x00,0x24,0x00,0x02,0x80,0x2D,0x80,0x30,0x80,0x20,0x00,0x00},//"k",75
{0x00,0x00,0x08,0x20,0x08,0x20,0xF8,0x3F,0x00,0x20,0x00,0x20,0x00,0x00,0x00,0x00},//"l",76
{0x80,0x20,0x80,0x3F,0x80,0x20,0x80,0x00,0x80,0x3F,0x80,0x20,0x80,0x00,0x00,0x3F},//"m",77
{0x80,0x20,0x80,0x3F,0x00,0x21,0x80,0x00,0x80,0x00,0x80,0x20,0x00,0x3F,0x00,0x20},//"n",78
{0x00,0x00,0x00,0x1F,0x80,0x20,0x80,0x20,0x80,0x20,0x80,0x20,0x00,0x1F,0x00,0x00},//"o",79
{0x80,0x80,0x80,0xFF,0x00,0xA1,0x80,0x20,0x80,0x20,0x00,0x11,0x00,0x0E,0x00,0x00},//"p",80
{0x00,0x00,0x00,0x0E,0x00,0x11,0x80,0x20,0x80,0x20,0x80,0xA0,0x80,0xFF,0x00,0x80},//"q",81
{0x80,0x20,0x80,0x20,0x80,0x3F,0x00,0x21,0x80,0x20,0x80,0x00,0x80,0x01,0x00,0x00},//"r",82
{0x00,0x00,0x00,0x33,0x80,0x24,0x80,0x24,0x80,0x24,0x80,0x24,0x80,0x19,0x00,0x00},//"s",83
{0x00,0x00,0x80,0x00,0x80,0x00,0xE0,0x1F,0x80,0x20,0x80,0x20,0x00,0x00,0x00,0x00},//"t",84
{0x80,0x00,0x80,0x1F,0x00,0x20,0x00,0x20,0x00,0x20,0x80,0x10,0x80,0x3F,0x00,0x20},//"u",85
{0x80,0x00,0x80,0x01,0x80,0x0E,0x00,0x30,0x00,0x08,0x80,0x06,0x80,0x01,0x80,0x00},//"v",86
{0x80,0x0F,0x80,0x30,0x00,0x0C,0x80,0x03,0x00,0x0C,0x80,0x30,0x80,0x0F,0x80,0x00},//"w",87
{0x00,0x00,0x80,0x20,0x80,0x31,0x00,0x2E,0x80,0x0E,0x80,0x31,0x80,0x20,0x00,0x00},//"x",88
{0x80,0x80,0x80,0x81,0x80,0x8E,0x00,0x70,0x00,0x18,0x80,0x06,0x80,0x01,0x80,0x00},//"y",89
{0x00,0x00,0x80,0x21,0x80,0x30,0x80,0x2C,0x80,0x22,0x80,0x21,0x80,0x30,0x00,0x00},//"z",90
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x7C,0x3F,0x02,0x40,0x02,0x40},//"{",91
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00},//"|",92
{0x00,0x00,0x02,0x40,0x02,0x40,0x7C,0x3F,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//"}",93
{0x00,0x00,0x06,0x00,0x01,0x00,0x01,0x00,0x02,0x00,0x02,0x00,0x04,0x00,0x04,0x00},//"~",94
};




//常用ASCII表
//纵向取模
//偏移量:32
//大小:8*6
const unsigned char ASCII_0806[95][6]={
{0x00,0x00,0x00,0x00,0x00,0x00},//- -   //space
{0x00,0x00,0x4f,0x00,0x00,0x00},//-!-
{0x00,0x07,0x00,0x07,0x00,0x00},//-"-
{0x14,0x7f,0x14,0x7f,0x14,0x00},//-#-
{0x24,0x2a,0x7f,0x2a,0x12,0x00},//-$-
{0x23,0x13,0x08,0x64,0x62,0x00},//-%-
{0x36,0x49,0x55,0x22,0x50,0x00},//-&-
{0x00,0x05,0x07,0x00,0x00,0x00},//-'-
{0x00,0x1c,0x22,0x41,0x00,0x00},//-(-
{0x00,0x41,0x22,0x1c,0x00,0x00},//-)-
{0x14,0x08,0x3e,0x08,0x14,0x00},//-*-
{0x08,0x08,0x3e,0x08,0x08,0x00},//-+-
{0x00,0x50,0x30,0x00,0x00,0x00},//-,-
{0x08,0x08,0x08,0x08,0x08,0x00},//---
{0x00,0x60,0x60,0x00,0x00,0x00},//-.-
{0x20,0x10,0x08,0x04,0x02,0x00},//-/-
{0x3e,0x51,0x49,0x45,0x3e,0x00},//-0-
{0x00,0x42,0x7f,0x40,0x00,0x00},//-1-
{0x42,0x61,0x51,0x49,0x46,0x00},//-2-
{0x21,0x41,0x45,0x4b,0x31,0x00},//-3-
{0x18,0x14,0x12,0x7f,0x10,0x00},//-4-
{0x27,0x45,0x45,0x45,0x39,0x00},//-5-
{0x3c,0x4a,0x49,0x49,0x30,0x00},//-6-
{0x01,0x71,0x09,0x05,0x03,0x00},//-7-
{0x36,0x49,0x49,0x49,0x36,0x00},//-8-
{0x06,0x49,0x49,0x29,0x1e,0x00},//-9-
{0x00,0x36,0x36,0x00,0x00,0x00},//-:-
{0x00,0x56,0x36,0x00,0x00,0x00},//-;-
{0x08,0x14,0x22,0x41,0x00,0x00},//-<-
{0x14,0x14,0x14,0x14,0x14,0x00},//-=-
{0x00,0x41,0x22,0x14,0x08,0x00},//->-
{0x02,0x01,0x51,0x09,0x06,0x00},//-?-
{0x32,0x49,0x79,0x41,0x3e,0x00},//-@-
{0x7e,0x11,0x11,0x11,0x7e,0x00},//-A-
{0x7f,0x49,0x49,0x49,0x36,0x00},//-B-
{0x3e,0x41,0x41,0x41,0x22,0x00},//-C-
{0x7f,0x41,0x41,0x22,0x1c,0x00},//-D-
{0x7f,0x49,0x49,0x49,0x41,0x00},//-E-
{0x7f,0x09,0x09,0x09,0x01,0x00},//-F-
{0x3e,0x41,0x49,0x49,0x7a,0x00},//-G-
{0x7f,0x08,0x08,0x08,0x7f,0x00},//-H-
{0x00,0x41,0x7f,0x41,0x00,0x00},//-I-
{0x20,0x40,0x41,0x3f,0x01,0x00},//-J-
{0x7f,0x08,0x14,0x22,0x41,0x00},//-K-
{0x7f,0x40,0x40,0x40,0x40,0x00},//-L-
{0x7f,0x02,0x0c,0x02,0x7f,0x00},//-M-
{0x7f,0x04,0x08,0x10,0x7f,0x00},//-N-
{0x3e,0x41,0x41,0x41,0x3e,0x00},//-O-
{0x7f,0x09,0x09,0x09,0x06,0x00},//-P-
{0x3e,0x41,0x51,0x21,0x5e,0x00},//-Q-
{0x7f,0x09,0x19,0x29,0x46,0x00},//-R-
{0x46,0x49,0x49,0x49,0x31,0x00},//-S-
{0x01,0x01,0x7f,0x01,0x01,0x00},//-T-
{0x3f,0x40,0x40,0x40,0x3f,0x00},//-U-
{0x1f,0x20,0x40,0x20,0x1f,0x00},//-V-
{0x3f,0x40,0x38,0x40,0x3f,0x00},//-W-
{0x63,0x14,0x08,0x14,0x63,0x00},//-X-
{0x07,0x08,0x70,0x08,0x07,0x00},//-Y-
{0x61,0x51,0x49,0x45,0x43,0x00},//-Z-
{0x00,0x7f,0x41,0x41,0x00,0x00},//-[-
{0x02,0x04,0x08,0x10,0x20,0x00},//-\-
{0x00,0x41,0x41,0x7f,0x00,0x00},//-]-
{0x04,0x02,0x01,0x02,0x04,0x00},//-^-
{0x40,0x40,0x40,0x40,0x40,0x00},//-_-
{0x01,0x02,0x04,0x00,0x00,0x00},//-`-
{0x20,0x54,0x54,0x54,0x78,0x00},//-a-
{0x7f,0x48,0x48,0x48,0x30,0x00},//-b-
{0x38,0x44,0x44,0x44,0x44,0x00},//-c-
{0x30,0x48,0x48,0x48,0x7f,0x00},//-d-
{0x38,0x54,0x54,0x54,0x58,0x00},//-e-
{0x00,0x08,0x7e,0x09,0x02,0x00},//-f-
{0x48,0x54,0x54,0x54,0x3c,0x00},//-g-
{0x7f,0x08,0x08,0x08,0x70,0x00},//-h-
{0x00,0x00,0x7a,0x00,0x00,0x00},//-i-
{0x20,0x40,0x40,0x3d,0x00,0x00},//-j-
{0x7f,0x20,0x28,0x44,0x00,0x00},//-k-
{0x00,0x41,0x7f,0x40,0x00,0x00},//-l-
{0x7c,0x04,0x38,0x04,0x7c,0x00},//-m-
{0x7c,0x08,0x04,0x04,0x78,0x00},//-n-
{0x38,0x44,0x44,0x44,0x38,0x00},//-o-
{0x7c,0x14,0x14,0x14,0x08,0x00},//-p-
{0x08,0x14,0x14,0x14,0x7c,0x00},//-q-
{0x7c,0x08,0x04,0x04,0x08,0x00},//-r-
{0x48,0x54,0x54,0x54,0x24,0x00},//-s-
{0x04,0x04,0x3f,0x44,0x24,0x00},//-t-
{0x3c,0x40,0x40,0x40,0x3c,0x00},//-u-
{0x1c,0x20,0x40,0x20,0x1c,0x00},//-v-
{0x3c,0x40,0x30,0x40,0x3c,0x00},//-w-
{0x44,0x28,0x10,0x28,0x44,0x00},//-x-
{0x04,0x48,0x30,0x08,0x04,0x00},//-y-
{0x44,0x64,0x54,0x4c,0x44,0x00},//-z-
{0x08,0x36,0x41,0x41,0x00,0x00},//-{-
{0x00,0x00,0x77,0x00,0x00,0x00},//-|-
{0x00,0x41,0x41,0x36,0x08,0x00},//-}-
{0x04,0x02,0x02,0x02,0x01,0x00},//-~-
};


#endif


#endif
 
