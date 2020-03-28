											   
#ifndef __DOFATFS_H
#define	__DOFATFS_H

#include "stm32f10x.h"
#include <stdio.h>

extern FRESULT Clear_Path(char *path);
extern FRESULT Clear_CurrentContent(char *current_content);
extern unsigned int Chang_Number(char *FileName);
extern FRESULT scan_files (char* path);
extern FRESULT clear_files(char* path);
extern unsigned int SD_TotalSize(void);

#endif 
