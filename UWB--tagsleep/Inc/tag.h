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
	uint64_t ChipID;							//标签芯片唯一ID
	uint16_t MAC_ID;							//网内MAC ID
	uint16_t Reserve;
	unsigned char Name[32];				//姓名
	uint8_t Sex;									//性别
	uint8_t Age;									//年龄
	uint16_t JobNum;							//工号
	unsigned char Department[32];	//部门
	unsigned char Title[32];			//职务
	unsigned char WorkSpace[32];	//工作区
	unsigned char Perm[2];				//权限
}Tag_Config_s;

//整形成员长度
#define CONFIG_CHIPID_LEN		8
#define CONFIG_MACID_LEN		2
#define CONFIG_SEX_LEN			1
#define CONFIG_AGE_LEN			1
#define CONFIG_JOBNUM_LEN		2

//字符串型成员最大长度
#define CONFIG_NAME_LEN_MAX					32
#define CONFIG_DEPT_LEN_MAX					32
#define CONFIG_TITLE_LEN_MAX				32
#define CONFIG_WORKSPACE_LEN_MAX		32
#define CONFIG_PERM_LEN_MAX					2

//#define USART_RECV_FRAME_HEADER_LEN     16

/***********************无线--串口通讯协议***********************************/
//无线协议
#define FRAME_64BIT_DEST_ADDR_IDX					5
#define FRAME_64BIT_SOURCE_ADDR_TX_IDX		7
#define FRAME_64BIT_SOURCE_ADDR_RX_IDX		13
#define FRAME_ID_64BIT_TYPE_IDX						15
#define FRAME_64BIT_DATA_IDX							16

//串口协议--类型PC端下发索引（配置帧）
#define USART_FRAME_TYPE_IDX				0x01	//帧类别索引
#define USART_FRAME_LEN_IDX					0x02	//帧长度索引（DATA长度+CRC长度）
#define USART_FRAME_DEST_IDX				0x04	//目标地址索引
#define USART_FRAME_DATA_IDX				0x0C	//帧数据索引
#define USART_FRAME_CHIPID_IDX			0x0C	//ChipID索引
#define USART_FRAME_MACID_IDX				0x14	//MAC ID索引
#define USART_FRAME_NAME_IDX				0x16	//Name索引

//串口协议--类型PC端下发索引（非配置帧）
#define USART_FRAME_CRC_IDX					0x02	//CRC索引

//串口协议--PC端下发指令类型
#define USART_FRAME_QUERY						0xFF	//查询帧
#define USART_FRAME_SET_ALL					0xFE	//设置全部信息
#define USART_FRAME_SET_CHIPID			0x01	//修改ChipID
#define USART_FRAME_SET_MACID				0x02	//修改MAC ID
#define USART_FRAME_SET_NAME				0x03	//修改姓名
#define USART_FRAME_SET_SEX					0x04	//修改性别
#define USART_FRAME_SET_AGE					0x05	//修改年龄
#define USART_FRAME_SET_JOBNUM			0x06	//修改工号
#define USART_FRAME_SET_DEPT				0x07	//修改部门
#define USART_FRAME_SET_TITLE				0x08	//修改职务
#define USART_FRAME_SET_WORKSPACE		0x09	//修改工作区
#define USART_FRAME_SET_PERM				0x0A	//修改权限
#define USART_FRAME_DEINIT					0xFC	//标签恢复出厂状态
#define USART_FRAME_QUIT						0xAA	//标签推出配置模式


//串口协议--标签端回复指令类型
#define USART_FRAME_INFO_CHIPID			0xEF	//标签回复Chip ID(代表标签需要配置信息)
#define USART_FRAME_INFO_ALL				0xEE	//标签回复全部信息(代表标签无需配置信息)
#define USART_FRAME_SET_ALL_ACK			0xEC	//标签回复全部信息(代表标签无需配置信息)
#define USART_FRAME_DEINIT_ACK			0xEB	//标签恢复出厂状态回复
#define USART_FRAME_REPEAT_ACK			0xB1	//标签CRC校验错误，需PC端重发
#define USART_FRAME_ERR_ACK					0xB2	//标签连续3次CRC校验错误

//串口协议--标签端回复索引
#define USART_ACK_TYPE_IDX					0x01
#define USART_ACK_LEN_IDX						0x02
#define USART_ACK_DATA_IDX					0x04
#define USART_ACK_CHIPID_IDX				0x04
#define USART_ACK_MACID_IDX					0x0C
#define USART_ACK_NAME_IDX					0x0E

//串口协议--通用指令类型
#define USART_FRAME_REPEAT					0xEA	//重发
#define USART_FRAME_ERR							0xE9	//故障

#define USART_FRAME_REPEAT_MAX			3

#define UsartShortFrameLen					30		
/*******************************************************************************/

//FLASH存储
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

//FLASH存储基地址
#define TAG_INFO_ADDR					ADDR_FLASH_PAGE_63

//FLASH存储数据地址偏移量
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
