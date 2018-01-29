#ifndef __MALLOC_H
#define __MALLOC_H
#include "stm32f10x.h"
#include "string.h"


void mymemset(void *s,u8 c,u32 count);	//设置内存
void mymemcpy(void *des,void *src,u32 n);//复制内存     
extern int mymemcmp(void *str1,void *str2,int len);//比较内存

#endif



