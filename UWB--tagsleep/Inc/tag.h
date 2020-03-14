#ifndef __TAG_H
#define __TAG_H

#include "main.h"

typedef enum
{
	ChannelBusy = 0x00,
	ChannelFree = 0x01
}ChannelStatu;

typedef struct
{
	uint64_t ChipID;							//��ǩоƬΨһID
	uint16_t MAC_ID;							//����MAC ID
	uint16_t Reserve;
	unsigned char Name[32];				//����
	uint8_t Sex;									//�Ա�
	uint8_t Age;									//����
	uint16_t JobNum;							//����
	unsigned char Department[32];	//����
	unsigned char Title[32];			//ְ��
	unsigned char WorkSpace[32];	//������
	unsigned char Perm[2];				//Ȩ��
}Tag_Config_s;

//���γ�Ա����
#define CONFIG_CHIPID_LEN		8
#define CONFIG_MACID_LEN		2
#define CONFIG_SEX_LEN			1
#define CONFIG_AGE_LEN			1
#define CONFIG_JOBNUM_LEN		2

//�ַ����ͳ�Ա��󳤶�
#define CONFIG_NAME_LEN_MAX					32
#define CONFIG_DEPT_LEN_MAX					32
#define CONFIG_TITLE_LEN_MAX				32
#define CONFIG_WORKSPACE_LEN_MAX		32
#define CONFIG_PERM_LEN_MAX					2

//#define USART_RECV_FRAME_HEADER_LEN     16

/***********************����--����ͨѶЭ��***********************************/
//����Э��
#define FRAME_64BIT_DEST_ADDR_IDX					5
#define FRAME_64BIT_SOURCE_ADDR_TX_IDX		7
#define FRAME_64BIT_SOURCE_ADDR_RX_IDX		13
#define FRAME_ID_64BIT_TYPE_IDX						15
#define FRAME_64BIT_DATA_IDX							16

//����Э��--����PC���·�����������֡��
#define USART_FRAME_TYPE_IDX				0x01	//֡�������
#define USART_FRAME_LEN_IDX					0x02	//֡����������DATA����+CRC���ȣ�
#define USART_FRAME_DEST_IDX				0x04	//Ŀ���ַ����
#define USART_FRAME_DATA_IDX				0x0C	//֡��������
#define USART_FRAME_CHIPID_IDX			0x0C	//ChipID����
#define USART_FRAME_MACID_IDX				0x14	//MAC ID����
#define USART_FRAME_NAME_IDX				0x16	//Name����

//����Э��--����PC���·�������������֡��
#define USART_FRAME_CRC_IDX					0x02	//CRC����

//����Э��--PC���·�ָ������
#define USART_FRAME_QUERY						0xFF	//��ѯ֡
#define USART_FRAME_SET_ALL					0xFE	//����ȫ����Ϣ
#define USART_FRAME_SET_CHIPID			0x01	//�޸�ChipID
#define USART_FRAME_SET_MACID				0x02	//�޸�MAC ID
#define USART_FRAME_SET_NAME				0x03	//�޸�����
#define USART_FRAME_SET_SEX					0x04	//�޸��Ա�
#define USART_FRAME_SET_AGE					0x05	//�޸�����
#define USART_FRAME_SET_JOBNUM			0x06	//�޸Ĺ���
#define USART_FRAME_SET_DEPT				0x07	//�޸Ĳ���
#define USART_FRAME_SET_TITLE				0x08	//�޸�ְ��
#define USART_FRAME_SET_WORKSPACE		0x09	//�޸Ĺ�����
#define USART_FRAME_SET_PERM				0x0A	//�޸�Ȩ��
#define USART_FRAME_DEINIT					0xFC	//��ǩ�ָ�����״̬
#define USART_FRAME_QUIT						0xAA	//��ǩ�Ƴ�����ģʽ


//����Э��--��ǩ�˻ظ�ָ������
#define USART_FRAME_INFO_CHIPID			0xEF	//��ǩ�ظ�Chip ID(�����ǩ��Ҫ������Ϣ)
#define USART_FRAME_INFO_ALL				0xEE	//��ǩ�ظ�ȫ����Ϣ(�����ǩ����������Ϣ)
#define USART_FRAME_SET_ALL_ACK			0xEC	//��ǩ�ظ�ȫ����Ϣ(�����ǩ����������Ϣ)
#define USART_FRAME_DEINIT_ACK			0xEB	//��ǩ�ָ�����״̬�ظ�
#define USART_FRAME_REPEAT_ACK			0xB1	//��ǩCRCУ�������PC���ط�
#define USART_FRAME_ERR_ACK					0xB2	//��ǩ����3��CRCУ�����

//����Э��--��ǩ�˻ظ�����
#define USART_ACK_TYPE_IDX					0x01
#define USART_ACK_LEN_IDX						0x02
#define USART_ACK_DATA_IDX					0x04
#define USART_ACK_CHIPID_IDX				0x04
#define USART_ACK_MACID_IDX					0x0C
#define USART_ACK_NAME_IDX					0x0E

//����Э��--ͨ��ָ������
#define USART_FRAME_REPEAT					0xEA	//�ط�
#define USART_FRAME_ERR							0xE9	//����

#define USART_FRAME_REPEAT_MAX			3

#define UsartShortFrameLen					30		
/*******************************************************************************/

//FLASH�洢
//sDWT
#define UWB_16BIT_ADDR_IDX					0
#define UWB_16BIT_ADDR_FLAG_IDX			31
#define UWB_MODE_IDX								15

//sTagConfig
#define UWB_PERM_IDX								0
#define UWB_JOBNUM_IDX							16
#define UWB_AGE_IDX									32
#define UWB_SEX_IDX									40
#define UWB_MAC_ID_IDX							48

//FLASH�洢����ַ
#define TAG_INFO_ADDR					ADDR_FLASH_PAGE_63

//FLASH�洢���ݵ�ַƫ����
#define SHORT_ID_ADDR_OFFECT				0
#define CHIPID_ADDR_OFFECT					8
#define MAC_ID_ADDR_OFFECT					16
#define NAME_ADDR_OFFECT						24
#define DEPT_ADDR_OFFECT						56
#define TITLE_ADDR_OFFECT						88
#define WORK_SPACE_ADDR_OFFECT			120

#define IS_USART_FRAME_TYPE(__FRAMETYPE__)		(((__FRAMETYPE__) == USART_FRAME_QUERY) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_ALL) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_CHIPID) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_MACID) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_NAME) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_SEX) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_AGE) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_JOBNUM) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_TITLE) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_WORKSPACE) || \
																							 ((__FRAMETYPE__) == USART_FRAME_SET_PERM) || \
																							 ((__FRAMETYPE__) == USART_FRAME_DEINIT) || \
																							 ((__FRAMETYPE__) == USART_FRAME_REPEAT) || \
																							 ((__FRAMETYPE__) == USART_FRAME_DEINIT))


void UWB_Tag(void *pdata);
void TAG_TOF(void);
void Tag_Reset_Blink(void);
void TAG_ConfigID(uint8_t frametype);
void TAG_ConfigID_ACK(uint8_t frametype);
void TAG_SaveConfigInfoToFlash(void);
void TAG_ReadAllConfigInfo(void);
void ReadConfigInfoFromFlash(void);
int8_t DWT_WakeUp(void);
uint8_t CommDec(void);
ChannelStatu ListenChannel(void);

#endif
