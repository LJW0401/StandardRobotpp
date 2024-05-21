#include "music_unity.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

// clang-format off
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

#define NOTE_NUM 300
static Note Notes[NOTE_NUM];  // Array of notes

static uint32_t last_note_id = 0;  // Index of the last note
static uint32_t write_id = 1;      // Index of the note to be written
static uint32_t play_id = 1;       // Index of the note to be played

static uint32_t start_time = 0;  // Start time of the music
static uint32_t now = 0;

/*-------------------- Private functions --------------------*/
static void WriteNote(int note, float Long)
{
    Notes[write_id].note = note;
    Notes[write_id].Long = Long;
    Notes[write_id].end = Notes[write_id - 1].end + Long;
    write_id++;
}

/*-------------------- User functions --------------------*/
void MusicUnityPlay(void)
{
    now = HAL_GetTick();
    if (now - start_time >= Notes[play_id].end) {
        play_id++;
        if (play_id > last_note_id) {
            play_id = 1;
            start_time = now;
        }

        buzzer_note(Notes[play_id].note, 0.05);
    }
}

void MusicUnityInit(void)
{
    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 4);

    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 4);

    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 2 + HalfBeat);
    WriteNote(D1, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);
    WriteNote(0, OneBeat + HalfBeat);

    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 4);

    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 4);

    WriteNote(B6, HalfBeat / 2);
    WriteNote(C1, HalfBeat * 3);
    WriteNote(C2, HalfBeat / 2);
    WriteNote(C3, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 2 + HalfBeat);
    WriteNote(C5, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(D1, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);
    WriteNote(0, OneBeat + HalfBeat);

    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 4);

    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 4);

    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 2 + HalfBeat);
    WriteNote(D1, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);
    WriteNote(0, OneBeat + HalfBeat);

    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 4);

    WriteNote(D3, HalfBeat / 2);
    WriteNote(D2, HalfBeat * 3);
    WriteNote(D1, HalfBeat / 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 4);

    WriteNote(B6, HalfBeat / 2);
    WriteNote(C1, HalfBeat * 3);
    WriteNote(C2, HalfBeat / 2);
    WriteNote(C3, HalfBeat * 3);
    WriteNote(C5, HalfBeat / 2);
    WriteNote(C6, OneBeat * 2 + HalfBeat);
    WriteNote(C5, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(D1, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);
    WriteNote(0,OneBeat + 150);
    WriteNote(C3, OneBeat * 2);
    WriteNote(0,OneBeat + 150);

    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(D1, HalfBeat);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C5, OneBeat * 2);
    WriteNote(C3, OneBeat * 2);

    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(D6, HalfBeat);
    WriteNote(D5, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);

    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, HalfBeat * 3);
    WriteNote(D2, HalfBeat);
    WriteNote(D3, HalfBeat * 3);
    WriteNote(D5, HalfBeat);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);
    WriteNote(D1, OneBeat * 2);

    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, HalfBeat * 3);
    WriteNote(D2, HalfBeat);
    WriteNote(D3, HalfBeat * 3);
    WriteNote(D5, HalfBeat);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);
    WriteNote(D1, OneBeat * 2);

    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(D1, HalfBeat);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C5, OneBeat * 2);
    WriteNote(C3, OneBeat * 2);

    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, OneBeat * 2);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(C5, HalfBeat);
    WriteNote(C6, HalfBeat * 3);
    WriteNote(D6, HalfBeat);
    WriteNote(D5, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D2, OneBeat * 2);

    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, HalfBeat * 3);
    WriteNote(D2, HalfBeat);
    WriteNote(D1, HalfBeat * 3);
    WriteNote(D2, HalfBeat);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, HalfBeat * 3);
    WriteNote(D2, HalfBeat);
    WriteNote(D1, HalfBeat * 3);
    WriteNote(D2, HalfBeat);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);
    WriteNote(D3, OneBeat * 2);

    last_note_id = write_id - 1;
    write_id = 1;
}
