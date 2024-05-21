#ifndef MUSIC_H
#define MUSIC_H

#include "struct_typedef.h"
#define MUSIC_TASK_INIT_TIME 10
#define MUSIC_TASK_TIME_MS 1

// clang-format off
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
