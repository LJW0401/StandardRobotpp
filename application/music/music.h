#ifndef MUSIC_H
#define MUSIC_H

#include "struct_typedef.h"
#define MUSIC_TASK_INIT_TIME 10
#define MUSIC_TASK_TIME_MS 1

// clang-format off
#define note_A   220
#define note_3A  110  
#define note_5A  440  
#define note_sA  233  //233.082
#define note_B   247  //246.942
#define note_3B  123  //123.471
#define note_5B  494  //493.883
#define note_C   262  //261.626
#define note_5C  523  //523.251
#define note_sC  277  //277.183
#define note_D   294  //293.665
#define note_sD  311  //311.127
#define note_5D  587  //587.33
#define note_3sD 156  //155.563
#define note_E   330  //329.629
#define note_3E  165  //164.814
#define note_F   349  //349.228
#define note_3F  175  //174.614
#define note_sF  370  //369.994
#define note_3sF 185  //184.997
#define note_G   392  //391.995
#define note_sG  415  //415.305
#define note_3G  196  //195.998
#define note_5sG 831  //830.609

//定义低音  
#define A1  131
#define A2  147
#define A3  165
#define A4  175
#define A5  196
#define A6  220
#define A7  247
  
//定义中音  
#define B1  262
#define B2  296
#define B3  330
#define B4  349
#define B5  392
#define B6  440
#define B7  494
  
//定义高音  
#define C1  523
#define C2  587
#define C3  659
#define C4  698
#define C4p 741
#define C5  784
#define C6  880
#define C7  988
  
//定义高二度  
#define D1  1047
#define D2  1175
#define D3  1319
#define D4  1397
#define D5  1568
#define D6  1760
#define D7  1976
  
//定义节拍  
#define OneBeat   200//一拍子两个1beat 
#define HalfBeat  100
// clang-format on

typedef struct __Note
{
    int note;
    float Long;
    uint32_t end;
} Note;

#endif  // MUSIC_H
