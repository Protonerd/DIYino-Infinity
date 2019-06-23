/*
 * SDWavFile.h
 *
 *  Created on: May 30, 2019
 *      Author: JakeSoft
 */

#ifndef SDWAVFILE_H_
#define SDWAVFILE_H_

#include <Arduino.h>
#include <SD.h>

struct tWavFileHeader
{
    char mChunkID[4];       //"RIFF" = 0x46464952
	uint32_t mChunkSize;    //28 [+ sizeof(wExtraFormatBytes) + wExtraFormatBytes] + sum(sizeof(chunk.id) + sizeof(chunk.size) + chunk.size)
	char mFormat[4];        //"WAVE" = 0x45564157
	char mSubchunk1ID[4];   //"fmt " = 0x20746D66
	uint32_t subchunk1Size; //16 [+ sizeof(wExtraFormatBytes) + wExtraFormatBytes]
	uint16_t audioFormat;
	uint16_t numChannels;
	uint32_t sampleRate;
	uint32_t byteRate;
	uint16_t blockAlign;
	uint16_t bitsPerSample;
};

struct tWavDataHeader
{
    char mID[4]; //"data" = 0x61746164
    uint32_t mSize;  //Chunk data bytes
};

class SDWavFile
{
public:
	SDWavFile(const char* aFilePath);

	virtual ~SDWavFile();

	File& GetFileHandle();

	const tWavFileHeader& GetHeader();

	const tWavDataHeader& GetDataHeader();

	void Close();

	bool SeekStartOfData();

	int Available();

	/**
	 * Fetch the sound data bytes
	 * Args:
	 *   apBuffer - Pointer to buffer to fill with data
	 *   aSize - How many samples to read
	 * Returns: Number of bytes filled
	 * Note: Assumes 16 bits per sample
	 */
	int FetchDataBytes(int16_t* apBuffer, int aSize);

	/**
	 * Fetch the sound data as 16-bit samples
	 * Args:
	 *   apBuffer - Pointer to buffer to fill with data
	 *   aNumSamples - How many samples to read
	 * Returns: Number of samples filled
	 */
	int Fetch16BitSamples(int16_t* apBuffer, int aNumSamples);

	/**
	 * Fetch the sound data as 16-bit samples
	 * Args:
	 *   apBuffer - Pointer to buffer to fill with data
	 *   aNumSamples - How many samples to read
	 * Returns: Number of samples filled
	 */
//	int Fetch16BitSSamplesScaledTo32Bit(int32_t* apBuffer, int aNumSamples);

	/**
	 * Fetch sound data as paired 16-bit I2C samples.
	 * Args:
	 * 	 apBuffer - Pointer to buffer to fill with data
	 *   aNumSamples - How many samples to read
	 *   aForceStereo - Forces all 32-bits of L/R channels to be populated
	 *                  even if file is mono.
	 *   aByteSwap - Byte swap the 16-bit samples
	 * Returns: Number of samples filled
	 */
	int Fetch16BitI2SSamples(int32_t* apBuffer,
			                 int aNumSamples,
			                 bool aForceStereo = false,
							 bool aByteSwap = false, char sampleVolume = 31);

	void VolumeShiftI2SSample(int32_t* apSample, char apVolume = 31);

protected:
	void ReadHeader();

	void ReadDataHeader();

	/**
	 * Byte swap the 16-bit words in an I2S sample
	 *
	 */
	void ByteSwapI2SSample(int32_t* aSample);


	const char* mpFilePath;
	tWavFileHeader mHeader;
	tWavDataHeader mDataHeader;
	File mFileHandle;

	int mBytesPerSample;
};



#endif /* SDWAVFILE_H_ */
