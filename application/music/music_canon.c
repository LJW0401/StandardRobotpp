#include "music_canon.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

// clang-format off
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

#define NOTE_NUM 1030
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
void MusicCanonPlay(void)
{
    now = HAL_GetTick();
    if (now - start_time >= Notes[play_id].end) {
        play_id++;
        if (play_id > last_note_id) {
            play_id = 1;
            start_time = now;
        }

        buzzer_note(Notes[play_id].note, 0.3);
    }
}

void MusicCanonInit(void)
{
    SleepNote(800);

    //第一行
    // WriteNote(min*1.1225,800);
    // SleepNote(800);
    // WriteNote(ren*1.1225,800);
    // SleepNote(800);

    // WriteNote(don*1.1225,800);
    // SleepNote(800);
    // WriteNote(sil*1.1225,800);
    // SleepNote(800);

    // WriteNote(lal*1.1225,800);
    // SleepNote(800);
    // WriteNote(sol*1.1225,800);
    // SleepNote(800);

    // WriteNote(lal*1.1225,800);
    // SleepNote(800);
    // WriteNote(sil*1.1225,800);
    // SleepNote(800);

    //第二行
    // WriteNote(mih*1.1225,800);
    // SleepNote(800);
    // WriteNote(reh*1.1225,800);
    // WriteNote(fan*1.1225,800);

    // WriteNote(doh*1.1225,800);
    // SleepNote(800);
    // WriteNote(sin*1.1225,800);
    // WriteNote(son*1.1225,800);

    // WriteNote(lan*1.1225,800);
    // SleepNote(800);
    // WriteNote(son*1.1225,800);
    // WriteNote(min*1.1225,800);

    // WriteNote(fan*1.1225,800);
    // SleepNote(800);
    // WriteNote(son*1.1225,800);
    // WriteNote(sin*1.1225,800);

    //第三行
    // WriteNote(doh*1.1225,400);
    // WriteNote(sin*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // WriteNote(min*1.1225,400);
    // WriteNote(son*1.1225,800);
    // WriteNote(sin*1.1225,800);

    // WriteNote(doh*1.1225,800);
    // WriteNote(mih*1.1225,800);
    // WriteNote(soh*1.1225,400);
    // WriteNote(mih*1.1225,400);
    // WriteNote(soh*1.1225,400);
    // WriteNote(lah*1.1225,400);

    // WriteNote(fah*1.1225,400);
    // WriteNote(mih*1.1225,400);
    // WriteNote(reh*1.1225,400);
    // WriteNote(fah*1.1225,400);
    // WriteNote(mih*1.1225,400);
    // WriteNote(reh*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // WriteNote(sin*1.1225,400);

    // WriteNote(lan*1.1225,400);
    // WriteNote(fan*1.1225,400);
    // WriteNote(doh*1.1225,800);
    // WriteNote(doh*1.1225,400);
    // WriteNote(fan*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // WriteNote(sin*1.1225,400);

    //第四行
    // WriteNote(doh*1.1225,400);
    // WriteNote(sin*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // WriteNote(fan*1.1225,400);
    // WriteNote(son*1.1225,400);
    // WriteNote(sil*1.1225,400);
    // WriteNote(sin*1.1225,400);
    // WriteNote(fan*1.1225,400);

    // WriteNote(doh*1.1225,800);
    // WriteNote(mih*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // WriteNote(soh*1.1225,400);
    // WriteNote(mih*1.1225,400);
    // WriteNote(soh*1.1225,400);
    // WriteNote(lah*1.1225,400);

    // WriteNote(fah*1.1225,400);
    // WriteNote(mih*1.1225,400);
    // WriteNote(reh*1.1225,400);
    // WriteNote(fah*1.1225,400);
    // WriteNote(mih*1.1225,400);
    // WriteNote(reh*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // WriteNote(sin*1.1225,400);

    // WriteNote(lan*1.1225,400);
    // WriteNote(son*1.1225,400);
    // WriteNote(fan*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // WriteNote(doh*1.1225,600);
    // WriteNote(ren*1.1225,200);
    // WriteNote(sin*1.1225,400);
    // WriteNote(reh*1.1225,200);
    // WriteNote(son*1.1225,200);

    //第五行
    // WriteNote(mih*1.1225,600);
    // WriteNote(son*1.1225,200);
    // WriteNote(mih*1.1225,400);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,800);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(min*1.1225,200);

    // WriteNote(doh*1.1225,600);
    // WriteNote(doh*1.1225,400);
    // WriteNote(min*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,400);
    // WriteNote(son*1.1225,200);
    // WriteNote(min*1.1225,400);
    // WriteNote(son*1.1225,400);
    // WriteNote(don*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(don*1.1225,400);
    // WriteNote(sin*1.1225,400);
    // WriteNote(don*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(don*1.1225,200);
    // WriteNote(son*1.1225,400);
    // WriteNote(min*1.1225,400);
    // WriteNote(son*1.1225,400);
    // WriteNote(don*1.1225,200);
    // WriteNote(don*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(don*1.1225,400);
    // WriteNote(don*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(ren*1.1225,400);
    // WriteNote(sin*1.1225,400);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(son*1.1225,200);

    //第六行
    // WriteNote(mih*1.1225,800);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,400);
    // WriteNote(son*1.1225,400);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(min*1.1225,200);

    // WriteNote(reh*1.1225,600);
    // WriteNote(doh*1.1225,400);
    // WriteNote(min*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,400);
    // WriteNote(son*1.1225,200);
    // WriteNote(min*1.1225,400);
    // WriteNote(son*1.1225,400);
    // WriteNote(don*1.1225,200);

    // WriteNote(lan*1.1225,600);
    // WriteNote(doh*1.1225,400);
    // WriteNote(lan*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(doh*1.1225,600);
    // WriteNote(don*1.1225,200);
    // WriteNote(don*1.1225,200);

    WriteNote(lan * 1.1225, 200);
    WriteNote(don * 1.1225, 400);
    WriteNote(don * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(sin * 1.1225, 600);
    WriteNote(doh * 1.1225, 600);
    WriteNote(reh * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);

    //第七行
    WriteNote(soh * 1.1225, 400);
    WriteNote(mih * 1.1225, 200);
    WriteNote(fah * 1.1225, 200);
    WriteNote(soh * 1.1225, 400);
    WriteNote(mih * 1.1225, 200);
    WriteNote(fah * 1.1225, 200);
    WriteNote(soh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(mih * 1.1225, 200);
    WriteNote(fah * 1.1225, 200);

    WriteNote(mih * 1.1225, 400);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(mih * 1.1225, 400);
    WriteNote(min * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);

    WriteNote(lan * 1.1225, 400);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(lan * 1.1225, 400);
    WriteNote(son * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(min * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);

    WriteNote(lan * 1.1225, 400);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 400);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(mih * 1.1225, 200);
    WriteNote(fah * 1.1225, 200);
    WriteNote(soh * 1.1225, 200);
    //第八行
    WriteNote(soh * 1.1225, 400);
    WriteNote(mih * 1.1225, 200);
    WriteNote(fah * 1.1225, 200);
    WriteNote(soh * 1.1225, 400);
    WriteNote(mih * 1.1225, 200);
    WriteNote(fah * 1.1225, 200);
    WriteNote(soh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(mih * 1.1225, 200);
    WriteNote(fah * 1.1225, 200);

    WriteNote(mih * 1.1225, 400);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(mih * 1.1225, 400);
    WriteNote(min * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);

    WriteNote(lan * 1.1225, 400);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(lan * 1.1225, 400);
    WriteNote(son * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(min * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);

    WriteNote(lan * 1.1225, 400);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 400);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(mih * 1.1225, 200);
    WriteNote(fah * 1.1225, 200);
    WriteNote(soh * 1.1225, 200);
    //第九行
    WriteNote(soh * 1.1225, 400);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(mih * 1.1225, 400);
    WriteNote(reh * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(mih * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);

    WriteNote(doh * 1.1225, 400);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 400);
    WriteNote(don * 1.1225, 200);
    WriteNote(ren * 1.1225, 200);
    WriteNote(min * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(min * 1.1225, 200);
    WriteNote(ren * 1.1225, 200);
    WriteNote(min * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);

    WriteNote(lan * 1.1225, 400);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(lan * 1.1225, 400);
    WriteNote(son * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(min * 1.1225, 200);
    WriteNote(fan * 1.1225, 200);
    WriteNote(son * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);

    WriteNote(lan * 1.1225, 400);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 400);
    WriteNote(sin * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(reh * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    WriteNote(doh * 1.1225, 200);
    WriteNote(lan * 1.1225, 200);
    WriteNote(sin * 1.1225, 200);
    //第十行
    WriteNote(doh * 1.1225, 200);
    WriteNote(dol * 1.1225, 200);
    WriteNote(sol * 1.1225, 400);
    WriteNote(don * 1.1225, 400);
    WriteNote(min * 1.1225, 400);
    WriteNote(sol * 1.1225, 400);
    WriteNote(ren * 1.1225, 400);
    WriteNote(son * 1.1225, 400);
    WriteNote(sin * 1.1225, 400);

    WriteNote(lal * 1.1225, 400);
    WriteNote(min * 1.1225, 400);
    WriteNote(lan * 1.1225, 400);
    WriteNote(doh * 1.1225, 400);
    WriteNote(mil * 1.1225, 400);
    WriteNote(son * 1.1225, 400);
    WriteNote(sin * 1.1225, 400);
    WriteNote(mih * 1.1225, 400);

    WriteNote(fal * 1.1225, 400);
    WriteNote(don * 1.1225, 400);
    WriteNote(fan * 1.1225, 400);
    WriteNote(lan * 1.1225, 400);
    WriteNote(dol * 1.1225, 400);
    WriteNote(min * 1.1225, 400);
    WriteNote(son * 1.1225, 400);
    WriteNote(doh * 1.1225, 400);

    WriteNote(fal * 1.1225, 400);
    WriteNote(fan * 1.1225, 400);
    WriteNote(lan * 1.1225, 400);
    WriteNote(doh * 1.1225, 400);
    WriteNote(sol * 1.1225, 400);
    WriteNote(ren * 1.1225, 400);
    WriteNote(son * 1.1225, 400);
    WriteNote(sin * 1.1225, 400);

    //第十一行
    // 	WriteNote(don*1.1225,400);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(soh*1.1225,400);
    // WriteNote(mih*1.1225,400);
    // 	WriteNote(sol*1.1225,400);
    // WriteNote(reh*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fah*1.1225,400);
    // WriteNote(reh*1.1225,400);

    // 	WriteNote(lal*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(mih*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // 	WriteNote(mil*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(doh*1.1225,400);
    // WriteNote(sin*1.1225,400);

    // 	WriteNote(fal*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,400);
    // WriteNote(lan*1.1225,400);
    // 	WriteNote(dol*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,400);
    // WriteNote(son*1.1225,400);

    // 	WriteNote(fal*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,400);
    // WriteNote(lan*1.1225,200);
    // WriteNote(son*1.1225,400);
    // WriteNote(son*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,400);
    // WriteNote(sin*1.1225,400);
    // //第十二行
    // 	WriteNote(don*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(soh*1.1225,400);
    // WriteNote(mih*1.1225,400);
    // 	WriteNote(sol*1.1225,400);
    // WriteNote(reh*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fah*1.1225,400);
    // WriteNote(reh*1.1225,400);

    // 	WriteNote(lal*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(mih*1.1225,400);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,400);
    // WriteNote(sin*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(mih*1.1225,400);
    // WriteNote(soh*1.1225,400);

    // WriteNote(lah*1.1225,400);
    // WriteNote(lah*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(fah*1.1225,400);
    // WriteNote(lah*1.1225,400);
    // WriteNote(soh*1.1225,400);
    // WriteNote(soh*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(mih*1.1225,400);
    // WriteNote(soh*1.1225,400);

    // WriteNote(lah*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(lah*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(lah*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(sih*1.1225,200);
    // WriteNote(lah*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(sih*1.1225,200);
    // WriteNote(lah*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // //第十三行
    // WriteNote(mih*1.1225,200);
    // WriteNote(son*1.1225,400);
    // WriteNote(son*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,400);
    // WriteNote(son*1.1225,400);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(min*1.1225,200);

    // WriteNote(doh*1.1225,600);
    // WriteNote(doh*1.1225,400);
    // WriteNote(min*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,400);
    // WriteNote(son*1.1225,200);
    // WriteNote(min*1.1225,400);
    // WriteNote(son*1.1225,400);
    // WriteNote(don*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(don*1.1225,400);
    // WriteNote(sin*1.1225,400);
    // WriteNote(don*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(don*1.1225,200);
    // WriteNote(son*1.1225,600);
    // WriteNote(doh*1.1225,400);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(don*1.1225,400);
    // WriteNote(don*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(ren*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // WriteNote(ren*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // //第十四行
    // WriteNote(mih*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,400);
    // WriteNote(son*1.1225,400);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(min*1.1225,200);

    // WriteNote(reh*1.1225,600);
    // WriteNote(doh*1.1225,400);
    // WriteNote(min*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(sih*1.1225,200);
    // WriteNote(dohh*1.1225,200);

    // WriteNote(sih*1.1225,200);
    // WriteNote(lah*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(son*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // //第十五行
    // WriteNote(mih*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lah*1.1225,400);
    // WriteNote(soh*1.1225,400);
    // WriteNote(fah*1.1225,200);
    // WriteNote(min*1.1225,200);

    // WriteNote(mih*1.1225,200);
    // WriteNote(min*1.1225,400);
    // WriteNote(min*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(ren*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(don*1.1225,200);

    // WriteNote(doh*1.1225,200);
    // WriteNote(don*1.1225,400);
    // WriteNote(don*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,400);
    // WriteNote(doh*1.1225,400);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(don*1.1225,200);
    // WriteNote(doh*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(don*1.1225,400);
    // WriteNote(don*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(ren*1.1225,400);
    // WriteNote(sin*1.1225,400);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // //第十六行
    // WriteNote(mih*1.1225,200);
    // WriteNote(son*1.1225,400);
    // WriteNote(son*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lah*1.1225,400);
    // WriteNote(soh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(min*1.1225,200);

    // WriteNote(mih*1.1225,200);
    // WriteNote(min*1.1225,400);
    // WriteNote(min*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(ren*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(don*1.1225,200);

    // WriteNote(doh*1.1225,200);
    // WriteNote(don*1.1225,400);
    // WriteNote(don*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,400);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(ren*1.1225,200);
    // WriteNote(doh*1.1225,400);
    // WriteNote(ren*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // //第十七行
    // WriteNote(mih*1.1225,200);
    // WriteNote(son*1.1225,400);
    // WriteNote(son*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lah*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(min*1.1225,200);

    // WriteNote(mih*1.1225,200);
    // WriteNote(min*1.1225,400);
    // WriteNote(min*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(fan*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(fan*1.1225,200);

    // WriteNote(doh*1.1225,200);
    // WriteNote(fan*1.1225,400);
    // WriteNote(fan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,400);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(ren*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(ren*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // //第十八行
    // WriteNote(mih*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lah*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(soh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(min*1.1225,200);

    // WriteNote(mih*1.1225,200);
    // WriteNote(min*1.1225,400);
    // WriteNote(min*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(fah*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(min*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(fan*1.1225,200);

    // WriteNote(doh*1.1225,200);
    // WriteNote(fan*1.1225,400);
    // WriteNote(fan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,400);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(ren*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(ren*1.1225,200);
    // WriteNote(reh*1.1225,400);
    // //第十九行
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);

    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // //第二十行
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(mih*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);
    // WriteNote(reh*1.1225,200);

    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(doh*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);
    // WriteNote(son*1.1225,200);

    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(lan*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,200);
    // WriteNote(sin*1.1225,133);
    // WriteNote(sin*1.1225,133);
    // WriteNote(sin*1.1225,134);
    // //结尾
    // WriteNote(sin*1.1225,400);
    // WriteNote(doh*1.1225,800);
    // SleepNote(2000);

    last_note_id = write_id - 1;
    write_id = 1;
}
