#ifndef __WAVE__
#define __WAVE__

#include <stm32f10x.h>
#include "ff.h"


typedef struct wavfile_header_s
{
    char    ChunkID[4];     /*  4   */
    uint32_t ChunkSize;      /*  4   */
    char    Format[4];      /*  4   */

    char    Subchunk1ID[4]; /*  4   */
    uint32_t Subchunk1Size;  /*  4   */
    uint16_t AudioFormat;    /*  2   */
    uint16_t NumChannels;    /*  2   */
    uint32_t SampleRate;     /*  4   */
    uint32_t ByteRate;       /*  4   */
    uint16_t BlockAlign;     /*  2   */
    uint16_t BitsPerSample;  /*  2   */

    char    Subchunk2ID[4];
    uint32_t Subchunk2Size;
} wavfile_header_t;

typedef struct PCM16_stereo_s
{
    uint16_t left;
    uint16_t right;
} PCM16_stereo_t;

#define SUBCHUNK1SIZE   (16)
#define AUDIO_FORMAT    (1) /*For PCM*/
#define NUM_CHANNELS    (2)
#define SAMPLE_RATE     (8000)//(44100)

#define BITS_PER_SAMPLE (16)
#define BYTE_RATE       (SAMPLE_RATE * NUM_CHANNELS * BITS_PER_SAMPLE/8)
#define BLOCK_ALIGN     (NUM_CHANNELS * BITS_PER_SAMPLE/8)

int wave_write_header(FIL*   file_p,uint32_t SampleRate,uint32_t FrameCount);

#endif