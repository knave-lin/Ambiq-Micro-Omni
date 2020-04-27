
    AREA _CMDData, DATA, READONLY

    EXPORT  u32CMDDataBegin
    EXPORT  u32CMDDataEnd

u32CMDDataBegin
	INCBIN .\Alexa_pack.bin
u32CMDDataEnd

	END