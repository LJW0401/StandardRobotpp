#include "music_1.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

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

void Music1Play(void)
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

void Music1Init(void)
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
