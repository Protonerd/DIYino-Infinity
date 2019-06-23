/*
 * SDWavFile.cpp
 *
 *  Created on: May 30, 2019
 *      Author: JakeSoft
 */


#include "SDWavFile.h"
#include <SD.h>
#include <Arduino.h>

#define DATA_START_OFFSET 44 //Byte offset where the data always starts

#define SCALE_16BIT_TO_32BIT ((uint32_t)65535)

SDWavFile::SDWavFile(const char* apFilePath)
{
	//Store the file path
	mpFilePath = apFilePath;
	//
	mFileHandle = SD.open(apFilePath, FILE_READ);
	ReadHeader();
	ReadDataHeader();
	SeekStartOfData();
}

SDWavFile::~SDWavFile()
{
	mFileHandle.close();
}

File& SDWavFile::GetFileHandle()
{
	return mFileHandle;
}

const tWavFileHeader& SDWavFile::GetHeader()
{
	return mHeader;
}

const tWavDataHeader& SDWavFile::GetDataHeader()
{
	return mDataHeader;
}

void SDWavFile::Close()
{
	mFileHandle.close();
}

bool SDWavFile::SeekStartOfData()
{
	bool lSuccess = false;
	if(mFileHandle.size() >= DATA_START_OFFSET+1)
	{
		lSuccess = mFileHandle.seek(DATA_START_OFFSET);
	}
	return lSuccess;
}

int SDWavFile::Available()
{
	return mFileHandle.available();
}

int SDWavFile::FetchDataBytes(int16_t* apBuffer, int aSize)
{
	char* lpBufferBytePtr = (char*)apBuffer;

	//Read until requested size is met or we run out of data
	int mBytesRead = 0;
	for(int lIdx = 0;
		lIdx < aSize && mFileHandle.available();
		lIdx++)
	{
		//Read the byte from the file
		mBytesRead += mFileHandle.readBytes(lpBufferBytePtr, 1);
		//Move the buffer pointer to the next byte
		lpBufferBytePtr++;
	}

	return mBytesRead;
}


int SDWavFile::Fetch16BitSamples(int16_t* apBuffer, int aNumSamples)
{

	//Read the bytes into the buffer
	int lBytesRead = FetchDataBytes(apBuffer, aNumSamples*2);

	//Divide bytes read by 2 because 2 bytes per 16-bit sample
	return lBytesRead/2;
}

//int SDWavFile::Fetch16BitSSamplesScaledTo32Bit(uint32_t* apBuffer, int aNumSamples)
//{
//	uint16_t lSample = 0;
//	char* lpSamplePtr = (char*)&lSample;
//
//	//Read until requested size is met or we run out of data
//	int lSamplesRead = 0;
//	for(int lIdx = 0;
//		lIdx < aNumSamples && mFileHandle.available() >= sizeof(uint16_t);
//		lIdx++)
//	{
//		//Read the 16-bit sample from the file
//
//		mFileHandle.readBytes(lpSamplePtr, 2);
//
//		//Now lSample contains the 16-bit sample. Scale it to 32 bits.
//		uint32_t lSample32Raw = (uint32_t)lSample;
//		uint32_t lSample32Scaled = lSample32Raw * SCALE_16BIT_TO_32BIT;
//
//		//Store the scaled value in the output buffer
//		apBuffer[lSamplesRead] = lSample32Scaled;
//
//		lSamplesRead++;
//	}
//
//	return lSamplesRead;
//}

int SDWavFile::Fetch16BitI2SSamples(int32_t* apBuffer, int aNumSamples, bool aForceStereo, bool aByteSwap, char sampleVolume)
{
	if(mHeader.bitsPerSample != 16)
	{
		Serial.println("WARNING: File is not 16-bits per sample!");
	}

	//Read until requested size is met or we run out of data
	int lSampleIndex = 0;
	for(lSampleIndex = 0;
		lSampleIndex < aNumSamples && mFileHandle.available() >= 2;
		lSampleIndex++)
	{
		//Clear current sample in the output buffer
		apBuffer[lSampleIndex] = 0;

		//Pointer so we can add one byte at a time to the output buffer
		char* lpOutSampleBytePtr = (char*)&apBuffer[lSampleIndex];

		//Read two bytes (16 bits) from the file into the output buffer
		mFileHandle.readBytes(lpOutSampleBytePtr, 2);

		//If the file is stereo, read the next 16-bit sample to fill in the other I2S channel
		if(2 == mHeader.numChannels)
		{
			//Advance the buffer byte pointer 2 bytes (next channel)
			lpOutSampleBytePtr += 2;
			//Read the next two bytes (16 bits) from the file into the output buffer
			mFileHandle.readBytes(lpOutSampleBytePtr, 2);
		}
		//This is mono, and we want to force stereo output
		else if(true == aForceStereo)
		{
			//Copy the first 2 bytes read for channel 1 into channel 2
			//so left and right IS2 channels will be identical
			memcpy(lpOutSampleBytePtr+2, lpOutSampleBytePtr, 2);
		}
		else  //  true mono
		{
			//Do nothing, leave the other channel blank
		}

		//Do byte swaping
		if(true == aByteSwap)
		{
			ByteSwapI2SSample(&apBuffer[lSampleIndex]);
		}
		// scale the sample according to the volume setting
		if (sampleVolume!=31) { // if not default volume (31)
			VolumeShiftI2SSample(&apBuffer[lSampleIndex],sampleVolume);
		}
	}

	return lSampleIndex;

}

void SDWavFile::ReadHeader()
{
	if(mFileHandle.available())
	{
		char* lpReadByte = (char*)&mHeader;
		mFileHandle.readBytes(lpReadByte, sizeof(tWavFileHeader));
	}

	mBytesPerSample = (mHeader.bitsPerSample / 8);
}

void SDWavFile::ReadDataHeader()
{
	if(mFileHandle.available())
	{
		char* lpReadByte = (char*)&mDataHeader;
		mFileHandle.readBytes(lpReadByte, sizeof(tWavDataHeader));
	}
}

void SDWavFile::ByteSwapI2SSample(int32_t* apSample)
{
	//4-byte temporary buffer to hold the raw 4 byte (32 bits) I2S sample
	uint8_t lpTempBytes[4];
	//1-byte pointer so we can manipulate the 4 byte input sample one byte at a time
	uint8_t* lpSampleBytePtr = (uint8_t*)apSample;

	//Store raw sample bytes in temporary byte buffer
	memcpy(&lpTempBytes[0], &apSample[0], sizeof(int32_t));

	//Swap bytes 0 and 1 (Channel 1)
	memcpy(&lpSampleBytePtr[1], &lpTempBytes[0], 1);
	memcpy(&lpSampleBytePtr[0], &lpTempBytes[1], 1);
	//Swap bytes 2 and 3 (Channel 2)
	memcpy(&lpSampleBytePtr[2], &lpTempBytes[3], 1);
	memcpy(&lpSampleBytePtr[3], &lpTempBytes[2], 1);
}

void SDWavFile::VolumeShiftI2SSample(int32_t* apSample, char apVolume)
{
	//2x16bit temporary buffer to hold the raw 4 byte (32 bits) I2S sample
	int16_t lpTempBytes[2];
	uint16_t* lpSamplePtr = (uint16_t*)apSample;
	//Store raw 16-bit samples in temporary buffer
	memcpy(&lpTempBytes[0], &apSample[0], sizeof(int32_t));
	// source: https://www.microchip.com/forums/m932509.aspx
if (apVolume&0x01)
{
    apVolume++;
    //for odd volume step - add back in half as much again!
	lpTempBytes[0]=lpTempBytes[0]>>(31-apVolume);
	lpTempBytes[0]=lpTempBytes[0]+(lpTempBytes[0]>>1);
	lpTempBytes[1]=lpTempBytes[1]>>(31-apVolume);
	lpTempBytes[1]=lpTempBytes[1]+(lpTempBytes[0]>>1);
}
else
{
    //For even volume step - same as normal shift
	lpTempBytes[0]=lpTempBytes[0]>>(31-apVolume);
	lpTempBytes[1]=lpTempBytes[1]>>(31-apVolume);
}
memcpy(&lpSamplePtr[0], &lpTempBytes[0], sizeof(int16_t));
memcpy(&lpSamplePtr[1], &lpTempBytes[1], sizeof(int16_t));

}
