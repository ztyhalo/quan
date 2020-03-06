/*********************************************************************************************************************************
** �ļ���:  protocol.h
** �衡��:  ����Э��
** ������: 	����
** �ա���:  2014-12-26
** �޸���:	
** �ա���:	
**
** �桡��:	V1.0.0.0
** ���¼�¼:
** ���¼�¼	��
** 					��    ��      ��    ��                    ��      ��
** 					==========  =============  ========================================
**
**--------------------------------------------------------------------------
**************************Copyright (c) 1998-1999 ��������Ӽ������޹�˾����������*************************************************/

#ifndef __PROTOCOL_H__
#define	__PROTOCOL_H__

#ifndef STM32F10x
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

#endif 

/***************************************ͷ�ļ�����****************************************************************/
//#include "stm32f10x.h"
/***************************************�궨��****************************************************************/
#define TRUE								1
#define FALSE								0
#define ENABLED								1
#define DISABLED							0
//CANͨ��֡ID��֡����-��ʾ������֡��ʲô����
#define FT_TYPE_MAX							31			//֡���͸��������������³������
#define FT_WL_TO_SC_BEAT					0			//���� ���� SC ����������
#define FT_WL_TO_SC_IR_REPORT				1			//���� ���� SC �ĺ����ϱ�����
#define FT_SC_TO_WL_DBUS					2			// SC �������ߵ��������ݣ���ҪӦ��
#define FT_WL_TO_SC_DBUS					3			//���� ���� SC ���������ݣ���ҪӦ��
#define FT_WL_TO_SC_IR_MATCH				4			//���� ���� SC �ĺ������
#define FT_SC_TO_WL_IR_MATCH_RST			5			//SC �������ߵĺ������������ҪӦ��
#define FT_WL_TO_SC_RF_MATCH				6			//���� ���� SC �����߶���
#define FT_SC_TO_WL_RF_MATCH_RST			7			//������������������ SC �����߶���������ҪӦ��
#define FT_SC_TO_WL_MATCH_SC_GROUP_INFO		8			// SC���͸�WL������ܷ��ͳ�����Ϣ
#define FT_SC_TO_WL_UC_SC_GROUP_INFO		9			// SC���͸�WL�����ؼܷ��ͳ�����Ϣ
#define FT_WL_TO_SC_CTL_DATA				10			//���� ���� SC �Ŀ�������
#define FT_SC_TO_WL_CTL_DATA_RECEPT			11			// SC ���͸� WL �����Ƿ���գ���ҪӦ��
#define FT_WL_TO_SC_CTL_DATA_SQN			12			//���� ���� SC �Ŀ�������(������������)
#define FT_WL_TO_SC_CTL_DATA_LIFT			13			//���� ���� SC �Ŀ�������(����̧��)
#define FT_SC_TO_WL_DISCONNECT				14			// SC ���͸� WL �������
#define FT_WL_TO_SC_DISCONNECT				15			//���� ���� SC �������
#define FT_SC_TO_WL_RESET_PAR				16			// SC ���͸� WL �������
#define FT_SC_TO_WL_LIFT_RECEPT				17			// SC ���͸� WL ����̧���Ƿ����
#define FT_SC_TO_WL_RESET_PAR_WL			30			// SC ���͸� WL �������ߺ������ģ��Ĳ���
#define FT_WL_TO_SC_RCV_PROGRAM				0x3FE		// WL ���͸� SC�����ճ����ϱ�֡
#define FT_WL_TO_SC_UPDATE_PROGRAM			0x3FD		// WL ���͸� SC�����³����ϱ�֡

//CAN����֡�ֽڳ���
#define FRM_DLC_ONE_BYTE					1			
#define FRM_DLC_TWO_BYTE					2			
#define FRM_DLC_THREE_BYTE					3			
#define FRM_DLC_FOUR_BYTE					4			
#define FRM_DLC_FIVE_BYTE					5			
#define FRM_DLC_SIX_BYTE					6			
#define FRM_DLC_SENVEN_BYTE					7			
#define FRM_DLC_EIGHT_BYTE					8			

//����ͨ�����ݵ�����
#define MASK_FLAG							(0x80)        // ��ȡ��־λ������
#define MASK_SC_L							0x70
#define MASK_FT								0x0F
#define MASK_RD2							0x80
#define MASK_DIR							0x40
#define MASK_SC_H							0x3F
#define MASK_RD1							0xD0
#define MASK_ID								0x3D
#define MASK_ACK							0x02
#define MASK_RST							0x01
//��ȡ��Ӧ�ֶε���λ��
#define BITS_FLAG							(7)        // ��ȡ��־λ��λ��
#define BITS_SC_L							(4)
#define BITS_FT								(0)
#define BITS_RD2							(7)
#define BITS_DIR							(6)
#define BITS_SC_H							(0)
#define BITS_RD1							(6)
#define BITS_ID								(2)
#define BITS_ACK							(1)
#define BITS_RST							(0)

//��ʱ����غ궨��
#define TIMER_EXPIRED	(u32)0x0				//��ʱ����
#define TIMER_DONE		(u32)0xfffffffe			//��ʱ�����Ѵ���
#define TIMER_CLOSED	(u32)0xffffffff			//��ʱ���ر�

//�������״̬
#define SUCCESS 0
#define ERROR_1 1
#define ERROR_2 2
#define ERROR_3 3
#define ERROR_4 4
#define ERROR_5 5
#define ERROR_6 6
#define ERROR_7 7
#define ERROR_8 8
#define ERROR_9 9
#define ERROR_10 10

#define ERROR_UP_OVERFLOW     ERROR_1
#define ERROR_DOWN_OVERFLOW   ERROR_2
#define ERROR_OUTOFFRAME      ERROR_3
#define ERROR_OVERTIME		  ERROR_4
/***************************************ö�����Ͷ���****************************************************************/
//ʹ�ܻ�ʧ�ܱ�־
enum{
	eDISABLED=0,				//(u16)0		//��ֹ
	eENABLED					//(u16)1		//����
};
//Ӧ���־
enum{
	eNOACK=0,
	eACK
};
// ���ݷ��ͷ���
enum{
	eDirectWL = 0,				// �ֳ��豸���ߺ��ⷢ����->���ߺ�����ն�
	eDirectHS					// ���ߺ�����ն� ->�ֳ��豸���ߺ��ⷢ����
};
/***************************************���ݽṹ����****************************************************************/
////Э��֡ID�ṹ����
//typedef struct{
//	u32 RID:3;						//Ŀ��ID(���շ�)
//	u32 TID:3;						//ԴID(���ͷ�)
//	u32 FT:10;						//֡����
//	u32 SN:4;						//��֡���к�
//	u32 SUM:5;						//��֡��
//	u32 SUB:1;						//��������֡������֡
//	u32 ACK:1;						//Ӧ��λ
//	u32 RD:2;						//����λ			
//} sCanFrameId;
//CANͨ��֡֡��ʽ
typedef struct {
	union {
		struct {
			u32 RID:3;				//Ŀ��ID(���շ�)
			u32 TID:3;				//ԴID(���ͷ�)
			u32 FT:10;				//֡����
			u32 SN:4;				//��֡���к�
			u32 SUM:5;				//��֡��
			u32 SUB:1;				//��������֡������֡
			u32 ACK:1;				//Ӧ��λ
			u32 RD:2;				//����λ
		} ID;
		u32 u32Id;
	} u32ID;
	u8 u8Data[8];
	u16 u16DLC;
}sCanFrame;
//����ͨ��֡��ʽ
typedef struct{
	union{
		struct{
			u32 RST:1;				// ������
			u32 ACK:1;				// Ӧ��λ
			u32 ID:4;				// �豸ID
			u32 RD1:2;				// ����λ
			u32 SC_H:6;				// ���Ƽܺŵĸ�6λ
			u32 DIR:1;				// ���ݷ��ͷ���
			u32 RD2:1;				// ����λ
			u32 FT:4;				// ��������
			u32 SC_L:3;				// ���Ƽܺŵĵ�3λ
			u32 FLAG:1;				// ��־λ���ж�һ֡���ݵ���ʼλ
		}CUSTOM_DATA;
		u32 u32CustomData;
	}u32IRDATA;	
	u8 u8DLC;
}sIrFrame;

#define GET_CRYP_KEY()	(0x00)		// ��ȡ��Կ

#endif

/*********************************������������޹�˾*************************************************************/
