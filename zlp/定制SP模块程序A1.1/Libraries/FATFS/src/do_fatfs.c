#include "ff.h"
//#include "RealTime.h"
#include "do_fatfs.h"
#include <string.h>

static char	Clear_MonthContent(char *yearDir);
/**********************************************************************************
 * ������ : Clear_Path
 * ����   : ɾ����ǰĿ¼��������С���ļ���
 * ����   : - path: ��Ŀ¼
 * ����   : -FRESULT
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

    res = f_opendir(&dirs, path);        // �򿪸�Ŀ¼
    if (res == FR_OK) 
	{
        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
		    if (finfo.fname[0] == 0)          // ���ҵ������
	        {
			    // ���������СĿ¼��һ����Ŀ¼
				sprintf(s_TempDir, "%s", fn_1);
				if (Clear_MonthContent(s_TempDir)) // �Ѿ�û���ļ���
				{
			        res = f_unlink(fn_1);
				}
			    break;
		    }

			if (finfo.fattrib & AM_DIR)  // ��һ��Ŀ¼
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
 * ������ : Clear_Path
 * ����   : ɾ�����ļ�����������С�����ļ���
 * ����   : - yearDir: ���ļ���
 * ����   : - char
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
		    if (finfo.fname[0] == 0)          // ���ҵ������
	        {
				if (MinSecount == 0xffffffff) // ��Ŀ¼Ϊ��
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

			if (finfo.fattrib & AM_DIR)      // ��һ��Ŀ¼
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
 * ������ : Clear_Path
 * ����   : ɾ�������ļ����������ļ�
 * ����   : - DayDir: �����ļ���
 * ����   : - char
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
			f_unlink(fn_1);	             // ɾ���ļ�
       }
   }
   return res;
}

/**********************************************************************************
 * ������ : Clear_CurrentContent
 * ����   : �����ǰ�ļ�Ŀ¼�������ļ�
 * ����   : - current_content: �����ļ�������
 * ����   : -FRESULT
 * ע��		: ֧�ֳ��ļ���
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
			/* ι�� */
			//IWDG_ReloadCounter();
	        if (finfo.fname[0] == 0)
	        {
		        break;
		    }

            fn = finfo.fname;
			if (finfo.fattrib & AM_DIR)      // �������ļ���
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
 * ������ : Chang_Number
 * ����   : ���ļ�����ת��Ϊ����
 * ����   : �ļ�������
 * ����   : ��ֵ
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
 * ������ : scan_files
 * ����   : �����ļ�Ŀ¼�������ļ��������ļ������ ��ֻҪ�Ǹ�Ŀ¼�µ��ļ�������ͬ·��һ�����������������ļ�����
 * ����   : - path: ��Ŀ¼
 * ����   : -FRESULT
 * ע��   : ֧�ֳ��ļ���
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
    res = f_opendir(&dir, path);                     //��Ŀ¼

    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);             //��ȡĿ¼���ļ������ļ���
			      fname_test1 = fno.fname[0];
            if ((res != FR_OK) || (fname_test1 == 0)) //��ȡ������߶�ȡ���
			      {
				      break;
			      }
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;	//���ļ�����ȡ
#else
            fn = fno.fname;
#endif
			if (fno.fattrib & AM_DIR) {                    //Ŀ¼
                sprintf(&path[i], "/%s", fn);
//                 printf("%s\r\n", path);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {
//                 printf("%s/%s\r\n", path, fn);			  //�ļ����
            }
        }
    }
    return res;
}

/**********************************************************************************
 * ������ : clear_files
 * ����   : ɾ����ǰ�ļ�Ŀ¼�������ļ��к��ļ�
 * ����   : - path: ��Ŀ¼
 * ���   : ��
 * ����   : -FRESULT
 * ע��		: ֧�ֳ��ļ���
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
 * ������ : SD_TotalSize
 * ����   : ��ѯsd����ʣ���Ĵ洢�ռ䣬��λΪKB��
 * ����   : - path: �����ļ�������
 * ���   : ��
 * ����   : -FRESULT
 * ע��		: ֧�ֳ��ļ���
 ***********************************************************************************/
unsigned int SD_TotalSize(void)
{
    static FRESULT res;
	static FATFS *fs;
    static DWORD fre_clust;

    res = f_getfree(" ", &fre_clust, &fs);  /* �����Ǹ�Ŀ¼��ѡ�����0 */
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


