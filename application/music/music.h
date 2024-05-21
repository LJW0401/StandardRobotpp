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

//定义音符
#define dolll   66
#define sdolll  70
#define relll   74
#define srelll  78
#define milll   83
#define falll   88
#define sfalll  93
#define solll   98
#define ssolll  104
#define lalll   110
#define slalll  117
#define silll   124

#define doll   131
#define sdoll  139
#define rell   147
#define srell  156
#define mill   165
#define fall   175
#define sfall  185
#define soll   196
#define ssoll  208
#define lall   220
#define slall  233
#define sill   247

#define dol   262
#define sdol  277
#define rel   294
#define srel  311
#define mil   330
#define fal   349
#define sfal  370
#define sol   392
#define ssol  415
#define lal   440
#define slal  466
#define sil   494

#define don   523
#define sdon  554
#define ren   578
#define sren  622
#define min   659
#define fan   698
#define sfan  740
#define son   784
#define sson  831
#define lan   880
#define slan  932
#define sin   988

#define doh   1046
#define sdoh  1046
#define reh   1175
#define sreh  1245
#define mih   1318
#define fah   1397
#define sfah  1480
#define soh   1568
#define ssoh  1661
#define lah   1760
#define slah  1865
#define sih   1976

#define dohh   2092
#define sdohh  2092
#define rehh   2350
#define srehh  2490
#define mihh   2636
#define fahh   2794
#define sfahh  2960
#define sohh   3136
#define ssohh  3322
#define lahh   3520
#define slahh  3730
#define sihh   3952

#define dohhh   4184
#define sdohhh  4184
#define rehhh   4700
#define srehhh  4980
#define mihhh   5272
#define fahhh   5588
#define sfahhh  5920
#define sohhh   6272
#define ssohhh  6644
#define lahhh   7040
#define slahhh  7460
#define sihhh   7904
// clang-format on

typedef struct __Note
{
    int note;
    float Long;
    uint32_t end;
} Note;

#endif  // MUSIC_H
