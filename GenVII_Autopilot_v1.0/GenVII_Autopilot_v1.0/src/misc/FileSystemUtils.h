/*                      */
/*                                        */
/*******************************************************************************
 *
 *   Copyright 2007 NetBurner, Inc.  ALL RIGHTS RESERVED
 *
 *   Permission is hereby granted to purchasers of NetBurner hardware to use or
 *   modify this computer program for any use as long as the resultant program
 *   is only executed on NetBurner provided hardware.
 *
 *   No other rights to use this program or its derivatives in part or in whole
 *   are granted.
 *
 *   It may be possible to license this or other NetBurner software for use on
 *   non-NetBurner hardware. Please contact licensing@netburner.com for more
 *   information.
 *
 *   NetBurner makes no representation or warranties with respect to the
 *   performance of this computer program, and specifically disclaims any
 *   responsibility for any damages, special or consequential, connected with
 *   the use of this program.
 *
 *   NetBurner, Inc.
 *   5405 Morehouse Drive
 *   San Diego, CA  92121
 *   http://www.netburner.com
 *
 ******************************************************************************/
#ifndef _FILESYSUTIL_H
#define _FILESYSUTIL_H

#include <effs_fat/fat.h>

#define MAX_EFFS_ERRORCODE   ( 38 )
extern char EffsErrorCode[][80];

#ifdef __cplusplus
extern "C" {
#endif

//FAT Media Types for Format
#define F_FAT12_FORMAT (1)
#define F_FAT16_FORMAT (2)
#define F_FAT32_FORMAT (3)

int OpenOnBoardFlash();
int OpenOffBoardFlash();
int UnmountFlash(int drv);

void DisplayEffsErrorCode(int code);
BYTE FormatExtFlash(int drv, long FATtype = F_FAT32_FORMAT);
BYTE DisplayEffsSpaceStats(int drv);
BYTE DumpDir();
DWORD WriteFile(BYTE* pDataToWrite, char* pFileName, DWORD NumBytes);
DWORD AppendFile(BYTE* pDataToWrite, char* pFileName, DWORD NumBytes);
DWORD ReadFile(BYTE* pReadBuffer, char* pFileName, DWORD NumBytes);
BYTE DeleteFile(char* pFileName);
void ReadWriteTest(char *FileName = "TestFile.txt");
void DisplayTextFile(char *FileName);
void fgets_test(char *FileName);
void fprintf_test();
void fputs_test(char *FileName);

#ifdef __cplusplus
}
#endif

#endif
