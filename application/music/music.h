#ifndef MUSIC_H
#define MUSIC_H

#include "struct_typedef.h"
#define MUSIC_TASK_INIT_TIME 10
#define MUSIC_TASK_TIME_MS 1

typedef struct __Note
{
    int note;
    float Long;
    uint32_t end;
} Note;

#endif  // MUSIC_H
