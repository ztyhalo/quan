#include "ff.h"
//#include "RealTime.h"
#include "do_fatfs.h"
#include <string.h>

static char	Clear_MonthContent(char *yearDir);
/**********************************************************************************
 * 函数名 : Clear_Path
 * 描述   : 删除当前目录下日期最小的文件夹
 * 输入   : - path: 根目录
 * 返回   : -FRESULT
 ***********************************************************************************/
FRESULT Clear_Path(char *path)
{
    FRESULT res;
    static FILINFO finfo;
    static DIR dirs;
	char *fn;
	static char fn_1[10];
	static char s_TempDir[30];
    unsigned int Secount=0, MinSecount=0xffffffff;

    res = f_opendir(&dirs, path);        // 打开根目录
    if (res == FR_OK) 
	{
        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
		    if (finfo.fname[0] == 0)          // 查找到最后了
	        {
			    // 清除日期最小目录中一个月目录
				sprintf(s_TempDir, "%s", fn_1);
				if (Clear_MonthContent(s_TempDir)) // 已经没有文件了
				{
			        res = f_unlink(fn_1);
				}
			    break;
		    }

			if (finfo.fattrib & AM_DIR)  // 是一个目录
	        {
                fn = finfo.fname;

		        Secount = Chang_Number(fn);
		        if (Secount < MinSecount)
			    {
			   	    MinSecount = Secount;
				    sprintf(fn_1, "%s", fn);
			    }
		    }
	    }
    }
    return res;
}

/**********************************************************************************
 * 函数名 : Clear_Path
 * 描述   : 删除年文件夹下日期最小的月文件夹
 * 输入   : - yearDir: 年文件夹
 * 返回   : - char
 ***********************************************************************************/
static char	Clear_MonthContent(char *yearDir)
{
    FRESULT res;
    static FILINFO finfo;
    static DIR dirs;
	char *fn;
   	static char fn_1[10];
	unsigned int Secount=0, MinSecount=0xffffffff;

	res = f_opendir(&dirs, yearDir);
	if (res == FR_OK) 
	{
        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
		    if (finfo.fname[0] == 0)          // 查找到最后了
	        {
				if (MinSecount == 0xffffffff) // 年目录为空
				{
				    return 1;
				}
				strcat(yearDir, "/");
				strcat(yearDir, fn_1);
				sprintf(fn_1, "%s", yearDir);

				Clear_CurrentContent(yearDir); 
			    f_unlink(fn_1);
			    break;
		    }

			if (finfo.fattrib & AM_DIR)      // 是一个目录
	        {
                if (finfo.fname[0] != 0x2E)
				{
					fn = finfo.fname;

		        	Secount = Chang_Number(fn);
		        	if (Secount < MinSecount)
			    	{
			   	    	MinSecount = Secount;
				    	sprintf(fn_1, "%s", fn);
			    	}
				}
		    }
		}
	}
	return 0;
}

/**********************************************************************************
 * 函数名 : Clear_Path
 * 描述   : 删除日期文件夹下所有文件
 * 输入   : - DayDir: 日期文件夹
 * 返回   : - char
 ***********************************************************************************/
static char	Clear_DayContent(char *DayDir)
{
    FRESULT res;
    static FILINFO finfo;
    static DIR dirs;
	char *fn;
   	static char fn_1[30];

    res = f_opendir(&dirs, DayDir);
    if (res == FR_OK) 
	{
        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
	        if (finfo.fname[0] == 0)
	        {
		        break;
		    }

            fn = finfo.fname;

			sprintf(fn_1, "%s", DayDir);
			strcat(fn_1, "/");
			strcat(fn_1, fn);
			f_unlink(fn_1);	             // 删除文件
       }
   }
   return res;
}

/**********************************************************************************
 * 函数名 : Clear_CurrentContent
 * 描述   : 清除当前文件目录下所有文件
 * 输入   : - current_content: 任意文件夹名称
 * 返回   : -FRESULT
 * 注意		: 支持长文件名
 ***********************************************************************************/
FRESULT Clear_CurrentContent(char *current_content)
{
    FRESULT res;
    static FILINFO finfo;
    static DIR dirs;
	char *fn;
   	static char fn_1[20];

	sprintf(fn_1, "%s", current_content);
    res = f_opendir(&dirs, current_content);

    if (res == FR_OK) 
	{
        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
			/* 喂狗 */
			//IWDG_ReloadCounter();
	        if (finfo.fname[0] == 0)
	        {
		        break;
		    }

            fn = finfo.fname;
			if (finfo.fattrib & AM_DIR)      // 是日期文件夹
	        {
                if (finfo.fname[0] != 0x2E)
				{
					strcat(current_content, "/");
				    strcat(current_content, fn);

					Clear_DayContent(current_content);
					f_unlink(current_content);

					sprintf(current_content, "%s", fn_1);
				}
			}
       }
   }
   return res;
}
/**********************************************************************************
 * 函数名 : Chang_Number
 * 描述   : 将文件夹名转换为数字
 * 输入   : 文件夹名称
 * 返回   : 数值
 ***********************************************************************************/
unsigned int Chang_Number(char *FileName)
{
    unsigned int sum = 0;
    static unsigned char i;
    //static char FileName1[8];

    for (i=0; i<8; i++)
    {
	    if (FileName[i] >= 0x30)
		{
		    sum = sum*10 + (FileName[i]-0x30);
		}
		else
		{
		    sum = sum*10;
		}
    }
    return sum;
}

/**********************************************************************************
 * 函数名 : scan_files
 * 描述   : 搜索文件目录下所有文件，并将文件名输出 （只要是根目录下的文件都能连同路径一起输出，不能输出空文件名）
 * 输入   : - path: 根目录
 * 返回   : -FRESULT
 * 注意   : 支持长文件名
 ***********************************************************************************/
FRESULT scan_files ( char* path)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
	  char fname_test1;
    char *fn;
#if _USE_LFN
    static char lfn[_MAX_LFN + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
#endif
    res = f_opendir(&dir, path);                     //打开目录

    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);             //读取目录下文件或者文件夹
			      fname_test1 = fno.fname[0];
            if ((res != FR_OK) || (fname_test1 == 0)) //读取错误或者读取完毕
			      {
				      break;
			      }
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;	//长文件名提取
#else
            fn = fno.fname;
#endif
			if (fno.fattrib & AM_DIR) {                    //目录
                sprintf(&path[i], "/%s", fn);
//                 printf("%s\r\n", path);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {
//                 printf("%s/%s\r\n", path, fn);			  //文件输出
            }
        }
    }
    return res;
}

/**********************************************************************************
 * 函数名 : clear_files
 * 描述   : 删除当前文件目录下所有文件夹和文件
 * 输入   : - path: 根目录
 * 输出   : 无
 * 返回   : -FRESULT
 * 注意		: 支持长文件名
 ***********************************************************************************/
FRESULT clear_files (
    char* path        /* Start node to be scanned (also used as work area) */
)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
	char pathfn[512]="";
	char fname_test1;
    char *fn;   /* This function is assuming non-Unicode cfg. */
#if _USE_LFN
    static char lfn[_MAX_LFN + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
#endif

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
			fname_test1 = fno.fname[0];
            if (res != FR_OK || fname_test1 == 0)
			 {
               	f_unlink (path);
				 break;  /* Break on error or end of dir */
			 }
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                sprintf(&path[i], "/%s", fn);
                res = clear_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {
			    sprintf(&pathfn[0], "%s", path);
			    strcat(pathfn,"/");
				strcat(pathfn,fn);
				f_unlink (pathfn);
            }
        }
    }
    return res;
}

/**********************************************************************************
 * 函数名 : SD_TotalSize
 * 描述   : 查询sd卡还剩多大的存储空间，单位为KB。
 * 输入   : - path: 任意文件夹名称
 * 输出   : 无
 * 返回   : -FRESULT
 * 注意		: 支持长文件名
 ***********************************************************************************/
unsigned int SD_TotalSize(void)
{
    static FRESULT res;
	static FATFS *fs;
    static DWORD fre_clust;

    res = f_getfree(" ", &fre_clust, &fs);  /* 必须是根目录，选择磁盘0 */
    if (res==FR_OK)
    {
//	    printf("\r\n %d KB total drive space.\r\n"
//             " %d KB available.\r\n",
//            ( (fs->n_fatent - 2) * fs->csize ) / 2  , (fre_clust * fs->csize) / 2  );
	    return ((fre_clust * fs->csize) / 2);
	}
	else
	    return 0;
}


