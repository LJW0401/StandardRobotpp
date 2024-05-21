
#include "music_you.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

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
// clang-format on

#define NOTE_NUM 570
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
    Notes[write_id].end = Notes[write_id - 1].end + Long * 200;
    write_id++;
}


/*-------------------- User functions --------------------*/
void MusicYouPlay(void)
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

void MusicYouInit(void)
{
    Notes[0].end = 0;

    float t = 0.2;

    WriteNote(note_5B, 1);  //前奏
    WriteNote(note_G, 1);
    WriteNote(note_5B, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5D, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5C, 1);
    WriteNote(note_5C, 1);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 1);
    WriteNote(note_5C, 1);

    WriteNote(note_5B, 1);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5D, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5C, 1);
    WriteNote(note_5C, 1);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 1);
    WriteNote(note_5C, 1);

    WriteNote(note_5B, 1);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5D, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5C, 1);
    WriteNote(note_5C, 1);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 1);
    WriteNote(note_5C, 1);

    WriteNote(note_5B, 1);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5B, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5D, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5C, 1);
    WriteNote(note_5C, 1);
    WriteNote(note_G, 1);
    WriteNote(note_D, 2);

    WriteNote(note_E, 6);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 4);

    WriteNote(note_5C, 4);
    WriteNote(note_5B, 4);
    WriteNote(note_E, 4);
    WriteNote(note_D, 2);

    WriteNote(note_E, 6);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_5C, 8);

    WriteNote(note_5B, 2);
    WriteNote(note_5D, 4);
    WriteNote(note_E, 10);

    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_5B, 4);

    WriteNote(note_5C, 4);
    WriteNote(note_5B, 4);
    WriteNote(note_E, 4);
    WriteNote(note_D, 2);

    WriteNote(note_E, 6);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_5B, 4);

    WriteNote(note_5C, 4);
    WriteNote(note_5D, 10);

    WriteNote(0, 4);  //我一直追寻着你
    WriteNote(note_E, 2);
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);

    WriteNote(0, 2);  //你好像不远也不近
    WriteNote(note_E, 2);
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 4);
    WriteNote(note_C, 6);

    WriteNote(0, 2);  //却总保持着距离
    WriteNote(note_E, 2);
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    //	WriteNote( note_D , 2 );
    WriteNote(note_D, 2);
    //	WriteNote( note_C , 1 );
    WriteNote(note_C, 2);
    WriteNote(note_D, 2);

    WriteNote(0, 2);
    WriteNote(note_E, 2);  //我一直幻想着你
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);

    WriteNote(0, 2);  //在我身边在我怀里
    WriteNote(note_E, 2);
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_5C, 4);
    WriteNote(note_C, 6);

    WriteNote(0, 2);  //让我欢笑让我哭泣
    WriteNote(note_E, 2);
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_B, 2);
    WriteNote(note_A, 1);
    WriteNote(note_3G, 5);

    WriteNote(0, 1);  //你是我灵魂的旋律
    WriteNote(note_3G, 1);
    WriteNote(0, t);
    WriteNote(note_3G, 1);
    WriteNote(0, t);
    WriteNote(note_3G, 1);
    WriteNote(note_G, 4);
    WriteNote(note_E, 3);
    WriteNote(note_D, 1);
    WriteNote(note_C, 2);
    WriteNote(0, t);
    WriteNote(note_C, 4);

    WriteNote(0, 1);
    WriteNote(note_C, 2);  //春日的细雨
    WriteNote(0, 0.05);
    WriteNote(note_C, 2);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_A, 6);

    WriteNote(0, 2);  //墓碑的雏菊
    WriteNote(note_A, 2);
    WriteNote(note_E, 2);
    WriteNote(note_D, 2);
    WriteNote(note_C, 2);
    WriteNote(note_D, 6);
    WriteNote(0, 2);

    WriteNote(note_E, 4);  //我从来不会计算代价
    WriteNote(note_F, 4);
    WriteNote(note_G, 6);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);
    WriteNote(note_E, 1);
    WriteNote(note_G, 3);
    WriteNote(note_5B, 4);
    WriteNote(note_5C, 6);

    WriteNote(note_C, 2);  //为了你可以纵身无底悬崖
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 4);
    WriteNote(note_5A, 4);
    WriteNote(note_G, 1);
    WriteNote(note_5A, 3);
    WriteNote(note_G, 1);
    WriteNote(0, 0.1);
    WriteNote(note_G, 3);
    WriteNote(0, 0.1);
    WriteNote(note_G, 4);
    WriteNote(note_D, 8);

    WriteNote(note_E, 4);  //像条狗更像一个笑话
    WriteNote(note_F, 4);
    WriteNote(note_G, 6);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);
    WriteNote(note_E, 1);
    WriteNote(note_G, 3);
    WriteNote(note_5B, 4);
    WriteNote(note_5C, 6);

    WriteNote(note_C, 2);  //也许我很傻但我不会怕
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_5B, 4);
    WriteNote(note_5A, 6);
    WriteNote(0, t);
    WriteNote(note_5A, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5A, 3);
    WriteNote(note_5C, 4);
    WriteNote(note_5D, 6);

    WriteNote(0, 2);
    WriteNote(note_G, 2);  //我愿意呀
    WriteNote(note_5C, 2);
    WriteNote(note_5B, 1);
    WriteNote(note_5C, 12);

    WriteNote(0, 4);

    WriteNote(0, 2);  //人们都追寻着你
    WriteNote(note_E, 2);
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);

    WriteNote(0, 2);  //都曾把你当作唯一
    WriteNote(note_E, 2);
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 4);
    WriteNote(note_C, 6);

    //	WriteNote( 0 , 4 );

    WriteNote(0, 2);  //最后却无能为力
    WriteNote(note_E, 2);
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    //	WriteNote( note_D , 2 );
    WriteNote(note_D, 2);
    //	WriteNote( note_C , 1 );
    WriteNote(note_C, 2);
    WriteNote(note_D, 2);

    WriteNote(0, 2);  //人们都幻想着你
    WriteNote(note_E, 2);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);

    WriteNote(0, 2);  //幻想你依偎他怀里
    WriteNote(note_E, 2);
    WriteNote(0, t);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_5C, 4);
    WriteNote(note_C, 6);

    //	WriteNote( 0 , 4 );

    WriteNote(0, 2);  //一朝拥有一劳永逸
    WriteNote(note_E, 2);
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_E, 3);
    WriteNote(note_D, 2);
    WriteNote(note_B, 2);
    WriteNote(note_A, 1);
    WriteNote(note_3G, 5);

    WriteNote(0, 1);  //可是你不为谁守候
    WriteNote(note_3G, 1);
    WriteNote(0, t);
    WriteNote(note_3G, 1);
    WriteNote(0, t);
    WriteNote(note_3G, 1);
    WriteNote(note_G, 4);
    WriteNote(note_E, 3);
    WriteNote(note_D, 1);
    WriteNote(note_C, 2);
    WriteNote(0, t);
    WriteNote(note_C, 4);

    WriteNote(0, 1);
    WriteNote(note_C, 2);  //不承诺永久
    WriteNote(0, t);
    WriteNote(note_C, 2);
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_A, 6);

    WriteNote(0, 2);  //不轻易停留
    WriteNote(note_A, 2);
    WriteNote(note_E, 2);
    WriteNote(note_D, 2);
    WriteNote(note_C, 2);
    WriteNote(note_D, 10);

    WriteNote(0, 4);
    WriteNote(note_E, 4);  //我知道只有不断出发
    WriteNote(note_F, 4);
    WriteNote(note_G, 6);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);
    WriteNote(note_E, 1);
    WriteNote(note_G, 3);
    WriteNote(note_5B, 4);
    WriteNote(note_5C, 6);

    WriteNote(note_C, 2);  //才能够紧随你纵情的步伐
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 4);
    WriteNote(note_5A, 5);
    WriteNote(note_G, 1);
    WriteNote(note_5A, 2);
    WriteNote(note_G, 1);
    WriteNote(0, t);
    WriteNote(note_G, 3);
    WriteNote(0, 0.1);
    WriteNote(note_D, 2);
    WriteNote(0, 0.1);
    WriteNote(note_D, 8);

    WriteNote(note_E, 4);  //就算是海角至天涯
    WriteNote(note_F, 4);
    WriteNote(note_G, 4);
    WriteNote(0, 0.5);
    WriteNote(note_G, 2);
    WriteNote(note_E, 1);
    WriteNote(note_G, 3);
    WriteNote(note_5D, 4);
    WriteNote(note_5C, 6);

    WriteNote(note_C, 2);  //青丝变白发
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_5C, 4);
    WriteNote(note_5A, 5);

    WriteNote(note_G, 1);  //只等着你回答
    WriteNote(note_5A, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5A, 3);
    WriteNote(note_5C, 4);
    WriteNote(note_5D, 6);

    WriteNote(0, 2);
    WriteNote(note_D, 2);  //我愿意呀
    WriteNote(note_E, 2);
    WriteNote(note_D, 1);
    WriteNote(note_C, 12);

    //间奏略

    WriteNote(0, 4);
    WriteNote(note_E, 4);  //我从来不会计算代价
    WriteNote(note_F, 4);
    WriteNote(note_G, 6);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);
    WriteNote(note_E, 1);
    WriteNote(note_G, 3);
    WriteNote(note_5B, 4);
    WriteNote(note_5C, 6);

    WriteNote(note_C, 2);  //为了你可以纵身无底悬崖
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 4);
    WriteNote(note_5A, 5);
    WriteNote(note_G, 1);
    WriteNote(note_5A, 2);
    WriteNote(note_G, 1);
    WriteNote(0, 0.1);
    WriteNote(note_G, 3);
    WriteNote(0, 0.1);
    WriteNote(note_G, 4);
    WriteNote(note_D, 8);

    WriteNote(note_E, 4);  //像条狗更像一个笑话
    WriteNote(note_F, 4);
    WriteNote(note_G, 6);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);
    WriteNote(note_E, 1);
    WriteNote(note_G, 3);
    WriteNote(note_5B, 4);
    WriteNote(note_5C, 6);

    WriteNote(note_C, 2);  //也许我很傻但我不会怕
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_5C, 4);
    WriteNote(note_5A, 6);
    WriteNote(0, t);
    WriteNote(note_5A, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5A, 3);
    WriteNote(note_5C, 4);
    WriteNote(note_5D, 6);

    WriteNote(note_E, 4);  //我知道只有不断出发
    WriteNote(note_F, 4);
    WriteNote(note_G, 6);
    WriteNote(note_E, 2);
    WriteNote(note_G, 2);
    WriteNote(note_E, 1);
    WriteNote(note_G, 3);
    WriteNote(note_5B, 4);
    WriteNote(note_5C, 6);

    WriteNote(note_C, 2);  //才能够紧随你纵情的步伐
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_G, 4);
    WriteNote(note_5A, 5);
    WriteNote(note_G, 1);
    WriteNote(note_5A, 2);
    WriteNote(note_G, 1);
    WriteNote(0, 0.1);
    WriteNote(note_G, 3);
    WriteNote(0, 0.1);
    WriteNote(note_G, 4);
    //	WriteNote( note_D , 2 );
    //	WriteNote( 0 , 0.1 );
    WriteNote(note_D, 8);

    WriteNote(note_E, 4);  //就算是海角至天涯
    WriteNote(note_F, 4);
    WriteNote(note_G, 4);
    WriteNote(0, 1);
    WriteNote(note_G, 2);
    WriteNote(note_E, 1);
    WriteNote(note_G, 3);
    WriteNote(note_5D, 4);
    WriteNote(note_5C, 6);

    WriteNote(note_C, 2);  //青丝变白发
    WriteNote(note_D, 2);
    WriteNote(note_E, 2);
    WriteNote(note_5C, 4);
    WriteNote(note_5A, 5);

    WriteNote(note_G, 1);  //只等着你回答
    WriteNote(note_5A, 2);
    WriteNote(note_G, 1);
    WriteNote(note_5A, 3);
    WriteNote(note_5C, 4);
    WriteNote(note_5D, 6);

    WriteNote(0, 2);
    WriteNote(note_G, 2);  //我愿意呀
    WriteNote(note_5C, 2);
    WriteNote(note_5B, 1);
    WriteNote(note_5C, 12);

    //尾
    WriteNote(0, 4);
    WriteNote(note_E, 4);
    WriteNote(note_D, 4);
    WriteNote(note_C, 4);
    WriteNote(note_G, 4);
    WriteNote(note_C, 4);
    WriteNote(note_D, 4);
    WriteNote(note_E, 4);
    WriteNote(note_F, 4);
    WriteNote(note_G, 4);
    WriteNote(note_F, 4);

    WriteNote(note_E, 4);
    WriteNote(note_D, 4);
    WriteNote(note_C, 4);
    WriteNote(note_D, 4);
    WriteNote(note_E, 4);
    WriteNote(note_F, 4);
    WriteNote(note_E, 4);
    WriteNote(note_D, 4);
    WriteNote(note_C, 4);
    WriteNote(note_G, 4);

    WriteNote(note_E, 4);
    WriteNote(note_D, 4);
    WriteNote(note_E, 4);
    WriteNote(note_F, 4);
    WriteNote(note_G, 4);
    WriteNote(note_F, 4);
    WriteNote(note_E, 4);
    WriteNote(note_D, 4);

    WriteNote(note_E, 4);
    WriteNote(note_D, 4);
    WriteNote(note_E, 4);
    WriteNote(note_F, 4);

    WriteNote(note_E, 4);
    WriteNote(note_D, 4);
    WriteNote(note_C, 4);
    WriteNote(note_G, 4);
    WriteNote(note_C, 4);
    WriteNote(note_D, 4);
    WriteNote(note_E, 4);
    WriteNote(note_F, 4);
    WriteNote(note_G, 4);
    WriteNote(note_F, 4);
    WriteNote(note_E, 4);
    WriteNote(note_D, 4);
    last_note_id = write_id - 1;
    write_id = 1;
}
