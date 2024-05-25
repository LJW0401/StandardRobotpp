#include "music_castle_in_the_sky.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

// clang-format off
#define	qdo    262 
#define qre    294
#define qmi    330		//q前缀为低音，1后缀为高音，s前缀为半音阶 
#define qfa    349
#define qso    392
#define qla    440
#define qsi    494
#define do     523
#define re     578
#define mi     659
#define fa     698
#define so     784
#define la     880
#define si     988
#define do1    1046
#define re1    1175
#define mi1    1318
#define fa1    1480
#define so1    1568
#define la1    1760
#define si1    1976
#define sqdo   277
#define sqre   311
#define sqfa   370
#define sqso   415
#define sqla   466
#define sdo    554
#define sre    622
#define sfa    740
#define sso    831
#define sla    932
#define sdo1   1046
#define sre1   1245
#define sfa1   1480
#define sso1   1661
#define sla1   1865
// clang-format on

#define NOTE_NUM 400
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

static void SleepNote(float Long) { WriteNote(0, Long); }

/*-------------------- User functions --------------------*/
void MusicCastleInTheSkyPlay(void)
{
    now = HAL_GetTick();
    if (now - start_time >= Notes[play_id].end) {
        play_id++;
        if (play_id > last_note_id) {
            play_id = 1;
            start_time = now;
        }

        buzzer_note(Notes[play_id].note, 0.07);
    }
}

void MusicCastleInTheSkyInit(void)
{
    int pai = 400, ban = 200;
    int ting = 128;

    SleepNote(1000);

    WriteNote(la, ban);
    WriteNote(si, ban);
    SleepNote(ting);

    WriteNote(do1, pai + ban);
    WriteNote(si, ban);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);
    WriteNote(mi1, pai);
    SleepNote(ting);

    WriteNote(si, 3 * pai);
    SleepNote(ting);
    WriteNote(mi, ban);
    WriteNote(mi, ban);

    WriteNote(la, ban + pai);
    WriteNote(so, ban);
    SleepNote(ting);
    WriteNote(la, pai);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);

    WriteNote(so, 2 * pai);
    SleepNote(ting);
    SleepNote(pai);
    WriteNote(mi, ban);
    SleepNote(ting / 2);
    WriteNote(mi, ban);
    SleepNote(ting / 2);

    WriteNote(fa, pai + ban);
    WriteNote(mi, ban);
    SleepNote(ting);
    WriteNote(fa, ban);
    WriteNote(do1, ban + pai);
    SleepNote(ting);

    WriteNote(mi, 2 * pai);
    SleepNote(ting);
    SleepNote(ban);
    WriteNote(do1, ban);
    SleepNote(ting / 2);
    WriteNote(do1, ban);
    SleepNote(ting / 2);
    WriteNote(do1, ban);
    SleepNote(ting / 2);

    WriteNote(si, ban + pai);
    WriteNote(sfa, ban);
    SleepNote(ting);
    WriteNote(sfa, pai);
    WriteNote(si, pai);
    SleepNote(ting);

    WriteNote(si, 2 * pai);
    SleepNote(ting);
    SleepNote(pai);
    WriteNote(la, ban);
    WriteNote(si, ban);
    SleepNote(ting);

    WriteNote(do1, pai + ban);
    WriteNote(si, ban);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);
    WriteNote(mi1, pai);
    SleepNote(ting);

    WriteNote(si, 2 * pai);
    SleepNote(ting);
    SleepNote(pai);
    WriteNote(mi, ban);
    SleepNote(20);
    WriteNote(mi, ban);
    SleepNote(ting);

    WriteNote(la, pai + ban);
    WriteNote(so, ban);
    SleepNote(ting);
    WriteNote(la, pai);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);

    WriteNote(so, 3 * pai);
    SleepNote(ting + ban);
    WriteNote(mi, ban);
    SleepNote(ting / 2);

    WriteNote(fa, pai);
    SleepNote(ting);
    WriteNote(do1, ban);
    WriteNote(si, ban);
    SleepNote(20);
    WriteNote(si, pai);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);

    WriteNote(re1, ban);
    SleepNote(20);
    WriteNote(re1, ban);
    SleepNote(20);
    WriteNote(mi1, ban);
    SleepNote(ting / 2);
    WriteNote(do1, pai);
    SleepNote(ting + pai);

    WriteNote(do1, pai);
    WriteNote(si, ban);
    SleepNote(ting);
    WriteNote(la, ban);
    SleepNote(20);
    WriteNote(la, ban);
    SleepNote(ting);
    WriteNote(si, pai);
    SleepNote(ting);
    WriteNote(sso, pai);
    SleepNote(ting);

    WriteNote(sso, 2 * pai);
    SleepNote(ting + pai);
    WriteNote(do1, ban);
    WriteNote(re1, ban);
    SleepNote(ting);

    WriteNote(mi1, pai + ban);
    WriteNote(re1, ban);
    SleepNote(ting);
    WriteNote(mi1, pai);
    SleepNote(ting);
    WriteNote(fa1, pai);
    SleepNote(ting);

    WriteNote(re1, 2 * pai);
    SleepNote(pai + ting);
    WriteNote(so, ban);
    SleepNote(20);
    WriteNote(so, ban);
    SleepNote(ting);

    WriteNote(do1, ban);
    WriteNote(si, ban);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);
    WriteNote(mi1, pai);
    SleepNote(ting);

    WriteNote(mi1, 2 * pai);
    SleepNote(ting + 2 * pai);

    WriteNote(la, ban);
    WriteNote(si, ban);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);
    WriteNote(si, pai);
    SleepNote(ting);
    WriteNote(re1, ban);
    SleepNote(20);
    WriteNote(re1, ban);
    SleepNote(ting);

    WriteNote(do1, pai + ban);
    WriteNote(so, ban);
    SleepNote(20);
    WriteNote(so, pai);
    SleepNote(pai + ting);

    WriteNote(fa1, pai);
    SleepNote(ting);
    WriteNote(mi1, pai);
    SleepNote(ting);
    WriteNote(re1, pai);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);

    WriteNote(mi1, 4 * pai);

    WriteNote(mi1, pai * 2);
    SleepNote(pai + ting);
    WriteNote(mi1, pai);
    SleepNote(ting);

    WriteNote(la1, 2 * pai);
    SleepNote(ting);
    WriteNote(so1, pai);
    SleepNote(ting);
    WriteNote(so1, pai);
    SleepNote(ting);

    WriteNote(mi1, ban);
    SleepNote(ting / 2);
    WriteNote(re1, ban);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting + ban);
    WriteNote(do1, ban);
    SleepNote(ting);

    WriteNote(re1, pai);
    SleepNote(ting);
    WriteNote(do1, ban);
    WriteNote(re1, ban);
    SleepNote(20);
    WriteNote(re1, ban);
    SleepNote(ting);
    WriteNote(so1, pai);
    SleepNote(ting);

    WriteNote(mi1, 2 * pai);
    SleepNote(ting + pai);
    WriteNote(mi, pai);
    SleepNote(ting);

    WriteNote(la1, 2 * pai);
    SleepNote(ting);
    WriteNote(so1, 2 * pai);
    SleepNote(ting);

    WriteNote(mi1, ban);
    WriteNote(re1, ban);
    SleepNote(ting);
    WriteNote(do1, 2 * pai);
    SleepNote(ting + ban);
    WriteNote(do1, ban);
    SleepNote(ting);

    WriteNote(re1, pai);
    SleepNote(ting);
    WriteNote(do1, ban);
    WriteNote(re1, ban);
    SleepNote(20);
    WriteNote(re1, ban);
    SleepNote(ting);
    WriteNote(si, pai);
    SleepNote(ting);

    WriteNote(la, 2 * pai);
    SleepNote(ting);
    WriteNote(la, ban);
    WriteNote(si, ban);

    WriteNote(do1, pai + ban);
    WriteNote(si, ban);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);
    WriteNote(mi1, pai);
    SleepNote(ting);

    WriteNote(si, 3 * pai);
    SleepNote(ting);
    WriteNote(mi, ban);
    WriteNote(mi, ban);

    WriteNote(la, ban + pai);
    WriteNote(so, ban);
    SleepNote(ting);
    WriteNote(la, pai);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);

    WriteNote(so, 2 * pai);
    SleepNote(ting);
    SleepNote(pai);
    WriteNote(mi, ban);
    SleepNote(ting / 2);
    WriteNote(mi, ban);
    SleepNote(ting / 2);

    WriteNote(fa, pai + ban);
    WriteNote(mi, ban);
    SleepNote(ting);
    WriteNote(fa, ban);
    WriteNote(do1, ban + pai);
    SleepNote(ting);

    WriteNote(mi, 2 * pai);
    SleepNote(ting);
    SleepNote(ban);
    WriteNote(do1, ban);
    SleepNote(ting / 2);
    WriteNote(do1, ban);
    SleepNote(ting / 2);
    WriteNote(do1, ban);
    SleepNote(ting / 2);

    WriteNote(si, ban + pai);
    WriteNote(sfa, ban);
    SleepNote(ting);
    WriteNote(sfa, pai);
    WriteNote(si, pai);
    SleepNote(ting);

    WriteNote(si, 2 * pai);
    SleepNote(ting);
    SleepNote(pai);
    WriteNote(la, ban);
    WriteNote(si, ban);
    SleepNote(ting);

    WriteNote(do1, pai + ban);
    WriteNote(si, ban);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);
    WriteNote(mi1, pai);
    SleepNote(ting);

    WriteNote(si, 2 * pai);
    SleepNote(ting);
    SleepNote(pai);
    WriteNote(mi, ban);
    SleepNote(20);
    WriteNote(mi, ban);
    SleepNote(ting);

    WriteNote(la, pai + ban);
    WriteNote(so, ban);
    SleepNote(ting);
    WriteNote(la, pai);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);

    WriteNote(so, 3 * pai);
    SleepNote(ting + ban);
    WriteNote(mi, ban);
    SleepNote(ting / 2);

    WriteNote(fa, pai);
    SleepNote(ting);
    WriteNote(do1, ban);
    WriteNote(si, ban);
    SleepNote(20);
    WriteNote(si, pai);
    SleepNote(ting);
    WriteNote(do1, pai);
    SleepNote(ting);

    WriteNote(re1, ban);
    SleepNote(20);
    WriteNote(re1, ban);
    SleepNote(20);
    WriteNote(mi1, ban);
    SleepNote(ting / 2);
    WriteNote(do1, pai);
    SleepNote(ting + pai);

    WriteNote(la, 4 * pai);

    SleepNote(1000);

    last_note_id = write_id - 1;
    write_id = 1;
}
