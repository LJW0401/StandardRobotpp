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

#define NOTE_NUM 570
static Note Notes[NOTE_NUM];  // Array of notes

static uint32_t last_note_id = 0;  // Index of the last note
static uint32_t write_id = 1;      // Index of the note to be written
static uint32_t play_id = 1;       // Index of the note to be played

static uint32_t start_time = 0;  // Start time of the music
static uint32_t now = 0;

static void WriteNote(int note, float Long)
{
    Notes[write_id].note = note;
    Notes[write_id].Long = Long;
    Notes[write_id].end = Notes[write_id - 1].end + Long * 200;
    write_id++;
}

void MusicUnityPlay(void)
{
    now = HAL_GetTick();
    if (now - start_time >= Notes[play_id].end) {
        play_id++;
        if (play_id > last_note_id) {
            play_id = 1;
            start_time = now;
        }

        buzzer_note(Notes[play_id].note,0.1);
    }
}

void MusicUnityInit(void)
{
    Notes[0].end = 0;

    float t = 0.1;

    //1 1 7 1 0 5 0 5
    WriteNote(note_C, 1);
    WriteNote(0, t);
    WriteNote(note_C, 1);
    WriteNote(0, t);
    WriteNote(note_B, 1);
    WriteNote(0, t);
    WriteNote(note_C, 1);
    WriteNote(0, 1);
    WriteNote(note_A, 1);
    WriteNote(0, 1);
    WriteNote(note_A, 1);

    //1 4 3 1 0 0
    WriteNote(0, t);
    WriteNote(note_C, 1);
    WriteNote(0, t);
    WriteNote(note_E, 1);
    WriteNote(0, t);
    WriteNote(note_D, 1);
    WriteNote(0, t);
    WriteNote(note_C, 1);
    WriteNote(0, 1);
    WriteNote(0, 1);

    //1 1 7 1 0 5 0 5
    WriteNote(note_C, 1);
    WriteNote(0, t);
    WriteNote(note_C, 1);
    WriteNote(0, t);
    WriteNote(note_B, 1);
    WriteNote(0, t);
    WriteNote(note_C, 1);
    WriteNote(0, 1);
    WriteNote(note_A, 1);
    WriteNote(0, 1);
    WriteNote(note_A, 1);
    
    //1 4 3 1 0 1
    WriteNote(0, t);
    WriteNote(note_C, 1);
    WriteNote(0, t);
    WriteNote(note_E, 1);
    WriteNote(0, t);
    WriteNote(note_D, 1);
    WriteNote(0, t);
    WriteNote(note_C, 1);
    WriteNote(0, 1);
    WriteNote(note_C, 1);

    last_note_id = write_id - 1;
    write_id = 1;
}
