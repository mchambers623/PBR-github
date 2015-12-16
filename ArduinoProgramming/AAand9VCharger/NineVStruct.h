/* 
* File: NineVStruct.h
*
* Authors: Matthew E. Chambers and Alexander D. Gregoire
*
* Date: 12/16/15
*
* Brief: This header file is used to define a structure made to
* hold the charging stage, timing, and duty cycle of the 9V charging.
 */
 
 
#ifndef NineVStruct_H
#define NineVStruct_H

struct NineVStruct{
  int DutyCycle;
  int Counter;
  int Stage;
};

#endif
