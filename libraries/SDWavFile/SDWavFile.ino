#include "Arduino.h"

#include "SDWavFile.h"

SDWavFile* gpWavFile;

//SPI Cable select pin
#define PIN_SPI_CS  11

//Uncomment this to dump the file data to serial
//#define DUMP_DATA_BLOCK

//Uncomment this to compare I2S output formats
#define COMPAIR_I2S_SAMPLE_FORMATS

//Uncomment this to dump I2S formatted data to serial
#define DUMP_I2S_DATA

void DumpWavFileHeader(SDWavFile& arWavFile)
{
    Serial.println( "*****************");
    Serial.println( "   FILE HEADER   ");
    Serial.println( "*****************");
    //char mChunkID[4]; //"RIFF" = 0x46464952
    Serial.print( "ChunkID       = ");
	for(int lIdx = 0; lIdx < 4; lIdx++)
    {
        Serial.print(arWavFile.GetHeader().mChunkID[lIdx]);
    }
    Serial.println("");

	//uint32_t mChunkSize; //28 [+ sizeof(wExtraFormatBytes) + wExtraFormatBytes] + sum(sizeof(chunk.id) + sizeof(chunk.size) + chunk.size)
	Serial.print( "ChunkSize     = ");
	Serial.println(arWavFile.GetHeader().mChunkSize);

	//char mFormat[4]; //"WAVE" = 0x45564157
	Serial.print( "Format        = ");
	for(int lIdx = 0; lIdx < 4; lIdx++)
    {
        Serial.print( arWavFile.GetHeader().mFormat[lIdx]);
    }
    Serial.println("");

	//char mSubchunk1ID[4]; //"fmt " = 0x20746D66
    Serial.print( "SubchunkID    = ");
	for(int lIdx = 0; lIdx < 4; lIdx++)
    {
        Serial.print(arWavFile.GetHeader().mSubchunk1ID[lIdx]);
    }
	Serial.println("");


	//uint32_t subchunk1Size; //16 [+ sizeof(wExtraFormatBytes) + wExtraFormatBytes]
	Serial.print( "SubChunkSize  = ");
	Serial.println(arWavFile.GetHeader().subchunk1Size);

	//uint16_t audioFormat;
	Serial.print( "AudioFormat   = ");
	Serial.println(arWavFile.GetHeader().audioFormat);

	//uint16_t numChannels;
	Serial.print( "NumChannels   = ");
	Serial.println(arWavFile.GetHeader().numChannels);

	//uint32_t sampleRate;
	Serial.print( "SampleRate    = ");
	Serial.println(arWavFile.GetHeader().sampleRate);

	//uint32_t byteRate;
	Serial.print( "ByteRate      = ");
	Serial.println(arWavFile.GetHeader().byteRate);

	//uint16_t blockAlign;
	Serial.print( "BlockAlign    = ");
	Serial.println(arWavFile.GetHeader().blockAlign);

	//uint16_t bitsPerSample;
	Serial.print( "BitsPerSample = ");
	Serial.println(arWavFile.GetHeader().bitsPerSample);

}

void DumpWavDataChunkHeader(const tWavDataHeader& arDataHeader)
{
    Serial.println( "*****************");
    Serial.println( "   DATA HEADER   ");
    Serial.println( "*****************");
     //char mID[4]; //"data" = 0x61746164
    Serial.print( "ID        = ");
    for(int lIdx = 0; lIdx < 4; lIdx++)
    {
        Serial.print(arDataHeader.mID[lIdx]);
    }
    Serial.println("");

    //uint32_t mSize;  //Chunk data bytes
    Serial.print("Data Size  = ");
    Serial.println(arDataHeader.mSize);

}

void DumpWavSoundData(SDWavFile& arWavFile)
{
	int16_t lWavSamplesBuffer[1]; //Array of size one, but could be any size

	Serial.println("**************");
	Serial.println("  DATA BLOCK  ");
	Serial.println("**************");

	//Make sure file pointer is at start of data block
	arWavFile.SeekStartOfData();

	//Keep looping until no more data is available
	int lSamplesRead = 0;
	while(arWavFile.Available() >= sizeof(int16_t))
	{
		//Fetch one 16-bit sample
		//Note: Could have a larger buffer if we wanted to, but just
		// doing one sample at a time for this example. To fetch
		//more than one sample at a time, increase the second argument
		arWavFile.Fetch16BitSamples(lWavSamplesBuffer, 1);

		//The function commented out is an alternative way to do
		//the same thing, but units are bytes-based instead of
		//based on samples
		//arWavFile.FetchDataBytes(lWavSamplesBuffer, sizeof(int16_t));

		Serial.print(lWavSamplesBuffer[0]);
		Serial.print(",");
		lSamplesRead++;

		if(lSamplesRead % 8 == 0)
		{
			Serial.println("");
		}
	}

	Serial.println("");
	Serial.print(lSamplesRead);
	Serial.print(" samples read (");
	Serial.print(lSamplesRead*2);
	Serial.println(" bytes)");
}

void DumpI2SSoundData(SDWavFile& arWavFile)
{
	uint32_t lWavSamplesBuffer[1]; //Array of size one, but could be any size
	uint16_t* lChannelPtr = (uint16_t*)lWavSamplesBuffer; //Pointer so we can see on 16-bit channel at a time

	Serial.println("**************");
	Serial.println("  DATA BLOCK  ");
	Serial.println("**************");

	//Make sure file pointer is at start of data block
	arWavFile.SeekStartOfData();

	//Keep looping until no more data is available
	int lSamplesRead = 0;
	while(arWavFile.Available() >= sizeof(int16_t))
	{
		//Fetch 32-bit I2S sample frame (16-bit right and 16-bit left)
		//Note: Could have a larger buffer if we wanted to, but just
		// doing one sample at a time for this example. To fetch
		//more than one sample at a time, increase the second argument
		arWavFile.Fetch16BitI2SSamples(lWavSamplesBuffer, 1);

		Serial.print(lChannelPtr[0]); //Right channel 16 bits
		Serial.print("|");
		Serial.print(lChannelPtr[1]); //Left channel 16 bits
		Serial.print(", ");
		lSamplesRead++;

		if(lSamplesRead % 8 == 0)
		{
			Serial.println("");
		}
	}

	Serial.println("");
	Serial.print(lSamplesRead);
	Serial.print(" samples read (");
	Serial.print(lSamplesRead*2);
	Serial.println(" bytes)");
}

void CompareWavI2SSoundDataFormats(SDWavFile& arWavFile)
{
	//Array of size one, but could be any size
	uint32_t lWavSamplesBuffer[1];
	//Pointer so we can examine the sample read byte-by-byte
	uint8_t* lBytePtr = (uint8_t*) &lWavSamplesBuffer;

	Serial.println("********************");
	Serial.println("  I2S DATA COMPARE  ");
	Serial.println("********************");

	//Make sure file pointer is at start of data block
	if(false == arWavFile.SeekStartOfData())
	{
		Serial.println("Seek to start of data failed!");
		return;
	}

	//Note that I2S samples are formatted into a right and left channel
	//with 16 bits allocated to each channel and then packed into the 32-bit word.
	//Optional arguments to Fetch16BitI2SSamples() call can affect how
	//the data is formatted within the fetched 32-bit word

	//Fetch one 32-bit I2S sample, no forced stereo and no byte swapping
	arWavFile.Fetch16BitI2SSamples(lWavSamplesBuffer, 1);

	//Let's examine the sample that was read by dumping the byte-by-byte data
	//to serial
	Serial.println("First sample, basic read:");
	for(int lByteIdx = 0; lByteIdx < 4; lByteIdx++)
	{
		Serial.print((unsigned int)lBytePtr[lByteIdx]);
		Serial.print(" ");
	}
	Serial.println("");

	//Reset the file pointer so we can read the same sample again
	arWavFile.SeekStartOfData();
	//Fetch one 32-bit I2S sample, with forced stereo and no byte swapping
	arWavFile.Fetch16BitI2SSamples(lWavSamplesBuffer, 1, true, false);

	//Let's examine the sample that was read by dumping the byte-by-byte data
	//to serial
	Serial.println("First sample, forced stereo:");
	for(int lByteIdx = 0; lByteIdx < 4; lByteIdx++)
	{
		Serial.print((unsigned int)lBytePtr[lByteIdx]);
		Serial.print(" ");
	}
	Serial.println("");

	//Reset the file pointer so we can read the same sample again
	arWavFile.SeekStartOfData();
	//Fetch one 32-bit I2S sample, no forced stereo, byte swap the output
	arWavFile.Fetch16BitI2SSamples(lWavSamplesBuffer, 1, false, true);

	//Again, let's examine the sample that was read by dumping the byte-by-byte data
	//to serial
	Serial.println("First sample, byte swapped:");
	for(int lByteIdx = 0; lByteIdx < 4; lByteIdx++)
	{
		Serial.print((unsigned int)lBytePtr[lByteIdx]);
		Serial.print(" ");
	}
	Serial.println("");


}

//The setup function is called once at startup of the sketch
void setup()
{
	delay(1000);
	Serial.begin(115200);

	//Initialize SD card
	if(!SD.begin(PIN_SPI_CS))
	{
		Serial.println("SD init failed.");
		return; //Punt. We can't work without SD card
	}

	//Open a WAV file on the SD card
	gpWavFile = new SDWavFile("poweron.wav");

	//Dump the file's header data to serial
	DumpWavFileHeader(*gpWavFile);
	delay(2000);

	//Dump the file's data header to serial
	DumpWavDataChunkHeader(gpWavFile->GetDataHeader());
	delay(2000);

	//Dump the data samples
#ifdef DUMP_DATA_BLOCK
	DumpWavSoundData(*gpWavFile);
	delay(2000);
#endif

#ifdef DUMP_I2S_DATA
	DumpI2SSoundData(*gpWavFile);
	delay(2000);
#endif

#ifdef COMPAIR_I2S_SAMPLE_FORMATS
	CompareWavI2SSoundDataFormats(*gpWavFile);
	delay(2000);
#endif

	//Close the WAV file
	gpWavFile->Close();

	//Deletes the pointer, NOT the file on the SD card
	delete gpWavFile;
}

// The loop function is called in an endless loop
void loop()
{
	//Do nothing
}
