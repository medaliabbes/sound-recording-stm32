#include "wave.h"


/**
 * @brief function to write the wave file header 
 * 
 * @param file_p      : file pointer 
 * @param SampleRate  : sampling rate or the frequency of the recording 
 * @param FrameCount  : number of audio simples captured during the recording 
 *                      framecount = SampleRate * duration 
 * @return int 
 */
int wave_write_header(FIL*   file_p,uint32_t SampleRate,uint32_t FrameCount)
{
    

    wavfile_header_t wav_header;
    uint32_t subchunk2_size;
    uint32_t chunk_size;


    subchunk2_size  = FrameCount * NUM_CHANNELS * BITS_PER_SAMPLE/8;
    chunk_size      = 4 + (8 + SUBCHUNK1SIZE) + (8 + subchunk2_size);

    wav_header.ChunkID[0] = 'R';
    wav_header.ChunkID[1] = 'I';
    wav_header.ChunkID[2] = 'F';
    wav_header.ChunkID[3] = 'F';

    wav_header.ChunkSize = chunk_size;

    wav_header.Format[0] = 'W';
    wav_header.Format[1] = 'A';
    wav_header.Format[2] = 'V';
    wav_header.Format[3] = 'E';
     wav_header.Subchunk1ID[0] = 'f';
    wav_header.Subchunk1ID[1] = 'm';
    wav_header.Subchunk1ID[2] = 't';
    wav_header.Subchunk1ID[3] = ' ';

    wav_header.Subchunk1Size = SUBCHUNK1SIZE;
    wav_header.AudioFormat = AUDIO_FORMAT;
    wav_header.NumChannels = NUM_CHANNELS;
    wav_header.SampleRate = SampleRate;
    wav_header.ByteRate = BYTE_RATE;
    wav_header.BlockAlign = BLOCK_ALIGN;
    wav_header.BitsPerSample = BITS_PER_SAMPLE;

    wav_header.Subchunk2ID[0] = 'd';
    wav_header.Subchunk2ID[1] = 'a';
    wav_header.Subchunk2ID[2] = 't';
    wav_header.Subchunk2ID[3] = 'a';
    wav_header.Subchunk2Size = subchunk2_size;

    UINT size = sizeof(wavfile_header_t);
    UINT b;
    f_write(file_p,&wav_header,size,&b);
    
    return 1;
}
