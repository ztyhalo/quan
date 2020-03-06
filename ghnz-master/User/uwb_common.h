#ifndef __UWB_COMMON_H__
#define __UWB_COMMON_H__


#include "stdint.h"
#include "stdbool.h"
#include "can_app.h"
#include "deca_device_api.h"
#include "deca_regs.h"
//#include "filtering.h"

#define BEACON_FRAME_TYPE_IDX											7		//�ű�֡��֡����������

#define FIRST_ADDR_OFF                            5
#define SECOND_ADDR_OFF                           7

#define FRAME_DEC_TAG_ADDR_IDX										5		//ͨѶ����֡����ǩ��ַ����
#define FRAME_16BIT_ADDR_TYPE_IDX									9		//16Bit��ַ����֡��֡��������


/* UWB microsecond (uus) �� device time unit (dtu, 1/(499.2MHz*128)��15.65ps) ����ϵ��.
 * 1 uus = 512 / 499.2 us 
 * 1 us  = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME											65536		//UUS��DWTоƬʱ��ת��ϵ��


#define FRAME_COMM_DEC_RX_TO_RESP_TX_DLY_UUS			300			//ͨѶ����֡������ɵ����ͻظ�֡�ӳ�ʱ��
#define POLL_RX_TO_RESP_TX_DLY_UUS          			300  	  //POLL������ɵ���ʼ����RESP�ӳ�ʱ��
#define RESP_TX_TO_FINAL_RX_DLY_UUS         			150   //RESP������ɵ���ʼ����FINAL�ӳ�ʱ��
#define FINAL_RX_TIMEOUT_UUS                			700  	//FINAL���ճ�ʱʱ��


/*��λ֡��ʽ*/
#define FINAL_MSG_TS_LEN                      		5  //finally��Ϣ�У�ʱ������ȣ�5���ֽ�
#define FINAL_MSG_POLL_TX_TS_IDX              		10  //finally��Ϣ�У�POLL����ʱ�������
#define FINAL_MSG_RESP_RX_TS_IDX              		(FINAL_MSG_POLL_TX_TS_IDX + FINAL_MSG_TS_LEN)  //finally��Ϣ�У�RESP����ʱ�������
#define FINAL_MSG_FINAL_TX_TS_IDX             		(FINAL_MSG_RESP_RX_TS_IDX +  FINAL_MSG_TS_LEN) //finally��Ϣ�У�FINAL����ʱ�������


typedef union 
{
	struct {
		uint16_t frameType:3; //֡����
		uint16_t  security:1;  //����
		uint16_t 	pending:1;   //uwb ��ʹ��
		uint16_t  ack:1;        //�Ƿ���Ҫ ack
		uint16_t  panIdCp:1;  //PAN ID ѹ��ģʽ
		uint16_t  reserve:3;     //����
		uint16_t  destAddrMode:2; //Ŀ�ĵ�ַ֡id����
		uint16_t  frameVer:2;     
		uint16_t  sorcAddrMode:2;  //Դ��ַ֡id����
	}FrameConBit;
	uint16_t  frameControl;
}FrameConField;	


//typedef struct 
//{
//		union {
//		struct {
//			uint16_t frameType:3; //֡����
//			uint16_t  security:1;  //����
//			uint16_t 	pending:1;   //uwb ��ʹ��
//			uint16_t  ack:1;        //�Ƿ���Ҫ ack
//			uint16_t  panIdCp:1;  //PAN ID ѹ��ģʽ
//			uint16_t  reserve:3;     //����
//			uint16_t  destAddrMode:2; //
//			uint16_t  frameVer:2;
//			uint16_t  sorcAddrMode:2;  
//		}FrameConBit;
//		uint16_t  frameControl;
//	}FrameConField;	
//}

/*IEEE 802.15.4֡���Ͷ���*/

typedef enum{
	BEACON_FRAME = 0,
	DATA_FRAME,
	ACK_FRAME,
	MAC_CMD_FRAME,
	RES_FRAME
}IEEE_FRAME_TYPE;
	
/*֡���Ͷ���*/

typedef enum{
	FRAME_POLL=	11,														//Poll֡
  FRAME_RESP,																// 12 Resp֡
  FRAME_FINAL,															//13	//Final֡
	FRAME_DIST_INFO,													//14	//������Ϣ֡
	FRAME_COMM_DEC,														//15	//TagͨѶ����֡
  FRAME_COMM_DEC_RESP												//16	//AncͨѶ�����ظ�֡
}LOCL_FRAME_TYPE;


/*֡id���ȶ���*/
typedef enum{
	ID_LENGTH_NO = 0,
	ID_LENGTH_RES,
	ID_LENGTH_16,
	ID_LENGTH_64
}FRAME_ID_LENGTH;



void SetFirstAddr(uint8_t * dest, uint16_t addr);


void SetSecondAddr(uint8_t * dest, uint16_t addr);

void FinalMsgGetTS_64(const uint8_t *ts_field, uint64_t *ts);

#endif /*__UWB_COMMON_H__*/
