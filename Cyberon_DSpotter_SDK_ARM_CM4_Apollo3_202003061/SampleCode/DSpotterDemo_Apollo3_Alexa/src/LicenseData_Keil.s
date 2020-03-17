
    AREA _LicenseData, DATA, READONLY

    EXPORT  u32LicenseDataBegin
    EXPORT  u32LicenseDataEnd

u32LicenseDataBegin
	INCBIN .\CybLicense.bin
u32LicenseDataEnd

	END