/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*****************************************************************************

 Application Name     - Network terminal

 Application Overview - Network Terminal is a command line interface (cli) based application,
 used to demonstrate the CC32XX/CC31XX networking capabilities.
 It does that by offering a list of commands, divided into four silos:

    Wlan        : Contains link layer functions like scan, connect, etc.
    NetApp      : Demonstrates the usage of networking applications.
    Socket      : Shows variety of socket API and responsible for sending and receiving packets.
    Transceiver : Gives the user a direct interface to the NWP radio for RF tests, raw sockets (L1) and more.

 Application Details  - Refer to 'Network Terminal' README.html

*****************************************************************************/

#include "gettime.h"
#include <stdio.h>


/* Application defines */
#define SIX_BYTES_SIZE_MAC_ADDRESS  (17)

#define UART_BUF_SIZE    4096
#define MAIN_WIFI_VER    1
#define SUB_WIFI_VER     2
pthread_t           gSpawn_thread = (pthread_t)NULL;

extern my_tm tm;
extern TaskHandle_t netWork;

void UDP_TASK();
void TCP_TASK();
void TCP_TASK1();

/****************************************************************************
         all definition in the protocol between simplelinke and APP
                add by yuzongyong
****************************************************************************/
extern int UART_Print_flg;
extern int sl_start_flag;
extern void SPI_FLASH_WORK(void *arg);
extern void TraceMessage(void *arg);
extern void UARTIntHandler();
extern unsigned char recvBuffFromMCU[UART_BUF_SIZE];
extern int writeIndex;
extern int readIndex;
extern long WriteFileToDevice(unsigned long *ulToken);
extern long ReadFileFromDevice(unsigned long ulToken);
extern void getDeviceTime();
extern void setTime();
extern SlDateTime_t dateTime;
extern _u8 pConfigOpt;
extern _u16 pConfigLen;
extern void setSSID();
extern void Get_Set_Time(unsigned char OP_CODE);

#define         main_vesion     1
#define         sub_version     1

//extern void Send_suggested_response();
#define         get_TIME       0
#define         set_TIME        1

// service type
#define NO_SERVICE              0x00
#define SERVICE_FILE            0x01    //文件服务
#define SERVICE_WARNING         0x02    //报警服务
#define SERVICE_SETTING         0x03    //参数配置服务

#define SERVICE_HEARTBEAT       0x05    //心跳服务包
#define SERVICE_CMD             0x06    //命令服务
#define SERVICE_DVR             0x07    //行车记录仪服务
#define SERVICE_DEBUG           0x08    //调试服务

//TLV type
#define TP_FILENAME_ID          0x01
#define TP_FILE_PARA_ID         0x02
#define TP_WARNING_ID           0x03
#define TP_FILE_PACKET_ID       0x04
#define TP_WIFI_PASSWORD_ID     0x05
#define TP_TIME_ID              0x06
#define TP_MH_VER_ID            0x07
#define TP_DEV_VER_ID           0x08
#define TP_DELAY_ID             0x09
#define TP_WORK_TIME_ID         0x0a
#define TP_DVR_KEY_ID           0x0b
#define TP_SWITCH_SCREEN        0x0c
#define TP_FPGA_VER_ID          0x0d
#define TP_ RESULOTION_ID       0x0e
#define TP_WIFI_NAME_ID         0x0f
#define TP_GATE_VER_ID          0x10
#define TP_WARN_DAY_STAT_ID     0x11
#define TP_DATE_INDEX_ID        0x12
#define TP_FRAME_COUNT_ID       0x13
#define TP_DVR_FILE_TYPE_ID     0x14
#define TP_DVR_FILE_LIST_ID     0x15
#define TP_DVR_PLAY_FILE_ID     0x16
#define TP_LOG_SWITCH_ID        0x17
#define TP_LOG_CONTENT_ID       0x18
#define TP_DEBUG_CMD_ID         0x19
#define TP_DVR_RESOLUTION_ID    0x1a
#define TP_CALIBRATE_CMD_ID     0x1b
#define TP_CALIBRATE_RESULT_ID  0x1c
#define TP_DVR_STATUS           0x1d


//1 文件服务
//消息类型定义
#define MSG_FILE_READ_REQ       0x10    //读文件请求
#define MSG_FILE_READ_RESP      0x10    //请求回应
#define MSG_FILE_READ_MORE_RESP 0x11    //读取文件请求回应，更多数据
#define MSG_FILE_WRITE_REQ      0x20    //写文件请求
#define MSG_FILE_WRITE_RESP     0x20    //请求回应
#define MSG_FILE_WRITE_MORE_REQ         0x21
#define MSG_FILE_WRITE_MORE_RESP        0x21
//TLV

//2 报警服务
//消息类型
#define MSG_WARNING_REQ	                0x01	//报警信息
#define MSG_WARNING_DAY_STAT_REQ        0x02    //按天（24小时）报警统计
#define MSG_WARNING_DAY_STAT_RESP	0x02	//按天（24小时）报警统计
#define MSG_WARNING_MONTH_STAT_REQ      0x03    //按月报警统计
#define MSG_WARNING_MONTH_STAT_RESP	0x03	//按月报警统计
#define MSG_WARNING_CLEAR_STAT_REQ      0x04    //清除所有报警统计记录
#define MSG_WARNING_CLEAR_STAT_RESP	0x04	//清除所有报警统计记录
//TLV

//3 参数配置服务
//消息类型
#define MSG_PARA_READ_REQ       0x01    //参数读取请求
#define MSG_PARA_READ_RESP	0x01	//参数读取请求回应
#define MSG_PARA_SET_REQ        0x02    //参数写入请求
#define MSG_PARA_SET_RESP	0x02	//参数写入请求回应
#define MSG_PARA_GET_TIME_REQ   0x03
#define MSG_PARA_GET_TIME_RESP  0x03
//TLV   TP_WIFI_PASSWORD_ID     TP_TIME_ID      TP_MH_VER_ID    TP_DEV_VER_ID   TP_DEV_VER_ID   TP_GATE_VER_ID

//4 固件升级

//5 心跳服务
//消息类型
#define MSG_HEART_BEAT_REQ      0x01    //心跳包
#define MSG_HEART_BEAT_RESP	0x01	//心跳包 任一方发起心跳包，接收方收到后原包返回，但响应标志修改。
#define MSG_HEART_BEAT_DVR	0x02	//参见DVR串口协议，用于Hi3518与DVR间的心跳
#define MSG_HEART_BEAT_GATE	0x03	//ADASGATE定时向mcu发送
//TLV

//6 命令接口服务
#define MSG_CMD_RESET_REQ       0x01    //复位系统命令。设备在收到此命令后，回复一个回应消息，整个开始重新启动。WIFI重启大约需要5秒，Mobileye重启大约需要20秒。0x01    复位整个系统
#define MSG_CMD_RESET_RESP	0x01    //复位系统命令。设备在收到此命令后，回复一个回应消息，整个开始重新启动。WIFI重启大约需要5秒，Mobileye重启大约需要20秒。0x01    复位整个系统
#define MSG_CMD_RESET_ME_REQ    0x02
#define MSG_CMD_RESET_ME_RESP	0x02	//复位Mobileye命令。设备在收到此命令后，回复一个回应消息，在设定时间到达后，开始复位Mobileye。Mobileye重启大约需要20秒。
#define MSG_CMD_RESET_DVR_REQ   0x03
#define MSG_CMD_RESET_DVR_RESP	0x03	//复位DVR命令
#define MSG_CMD_UPDATE_GATE_REQ         0x07
#define MSG_CMD_UPDATE_GATE_RESP	0x07	//更新ADASGate命令。在ADASGate收到ADASGate升级文件后，发送给ADASDaemo，接收到回应后关闭，用于重启ADASGate。
#define MSG_CMD_TEST_REQ                0x08
#define MSG_CMD_TEST_RESP	        0x08	//测试命令。设备在收到此命令后，发送测试数据给显示屏和手机。当速度不为0时，此指令失效。每次接收到此指令后，自动测试3分钟。
#define MSG_CMD_SWITCH_SCREEN	        0x09	//切换视频源 0:查询当前视频源号 1：原车屏幕 2：Mobileye 3：DVR
#define MSG_CMD_POWER_OFF_REQ           0x0A
#define MSG_CMD_POWER_OFF_RESP	        0x0A	//关闭系统，HI3518同时需要发送关机消息给DVR。关机时间为1分钟。用于HI3与MCU或DVR间通信。
#define MSG_CMD_SAVE_FRAMES_REQ         0x0B
#define MSG_CMD_SAVE_FRAMES_RESP	0x0B	//截取3518的VI口输入的视频数据，保存为文件。参数为需要截取的帧数
#define MSG_CMD_STA_MODE_REQ            0x0C
#define MSG_CMD_STA_MODE_RESP	        0x0C	//要求将3518的wifi切换成STA模式
#define MSG_CMD_CALI_REQ                0x0D
#define MSG_CMD_CALI_RESP	        0x0D	//用于校准 0：查询校准状态 1：开始校准/正在校准 2：停止校准
#define MSG_CMD_CALI_RESULT_INFO	0x0E	//校准结果，由3518主动发送
#define MSG_CMD_CAN_DATA_REQ	        0x11	//CAN数据请求


//7  行车记录仪服务
//消息类型
#define MSG_DVR_KEY	        0x01    //按键指令。由App发送，用于控制行车记录的按键操作。
#define MSG_DVR_FILE_LIST_REQ   0x02
#define MSG_DVR_FILE_LIST_RESP	0x02	//获取文件列表，如视频文件列表，照片列表
#define MSG_DVR_PLAY_FILE_REQ   0x03
#define MSG_DVR_PLAY_FILE_RESP	0x03	//播放DVR文件
#define MSG_DVR_RECORD	        0x04	//进入录像界面
#define MSG_DVR_FORMAT	        0x05	//格式化TF卡
#define MSG_DVR_STATUS	        0x06	//行车记录仪当前状态
#define MSG_DVR_DEL_FILE	0x07	//删除文件

//8 调试服务
//消息类型
#define LOG_ADASGATE_SWITCH	0x01	//ADASGATE日志开关
#define LOG_ADASGATE_CONTEXT	0x02	//ADASGATE上报日志
#define LOG_MCU_SWITCH	        0x03	//MCU日志开关
#define LOG_MCU_CONTEXT	        0x04	//MCU上报日志
#define DEBUG_MCU_COMMAND	0x05	//MCU调试用指令
#define DEBUG_FPGA_COMMAND	0x06	//FPGA调试用指令
#define DEBUG_DVR_COMMAND	0x07	//DVR调试用指令


extern unsigned char sendDatabyWIFI[100];//通过WIFI 发送的数据
extern int Length_sendDatabyWIFI;

extern int UART_Print_flg;
extern int sl_start_flag;
extern void SPI_FLASH_WORK(void *arg);
//extern void UART_MCU_CC3220(void *arg);

extern unsigned char *p_WARN_DAY_STAT_TODAY;
extern unsigned char *p_WARN_DAY_STAT_RESP;
extern void dealWarning(char *buf, int len);
extern long FileHandle_Warn_Stat_Today;
extern long FileHandle_Warn_Stat_Index;
extern unsigned char FileName_Warn_Stat_Today[6];//当天报警文件名称
extern unsigned char FileName_Warn_Stat_Index[6];//按index 文件名称
extern unsigned char *pStat24Hour;

typedef struct {
	unsigned char head[16];
	unsigned char TP_FILE_PARA[76];
	unsigned char TP_FILE_PACKET[10];
	unsigned char Data[4096];
} FileStructFirstPackage;
FileStructFirstPackage FilePackage;

typedef struct {
	unsigned char head[16];
	unsigned char TP_FILE_PACKET[10];
	unsigned char Data[4096];
} FileStructFollowPackage;
FileStructFollowPackage FileFollowPackage;

unsigned int Write_File_Resp = 0;
unsigned int Write_File_Resp_More = 0;

static unsigned int crc_table[256] = {
	0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
	0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
	0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
	0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
	0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
	0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
	0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95,
	0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
	0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
	0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
	0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02,
	0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
	0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692,
	0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
	0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
	0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
	0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB,
	0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
	0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B,
	0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
	0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
	0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
	0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B,
	0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
	0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
	0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
	0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
	0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
	0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C,
	0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
	0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C,
	0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
};
unsigned int GetCrc32(unsigned char  *buf, unsigned int len)
{
	unsigned char v3;
	unsigned char v1;
	unsigned int i;
	unsigned int a1 = 0;
	for (i = 0; i < len; ++i) {
		v1 = (*buf++) ^ (i & 0xFF);
		v3 = v1 ^ ((a1 >> 24) & 0xFF);
		a1 = crc_table[v3] ^ (a1 << 8);
	}
	return a1;
}

unsigned int GetCrc32_One(unsigned char  dat, unsigned int a1, unsigned int pos)
{
	unsigned char v3;
	unsigned char v1;
	{
		v1 = dat ^ (pos & 0xFF);
		v3 = v1 ^ ((a1 >> 24) & 0xFF);
		a1 = crc_table[v3] ^ (a1 << 8);
	}
	return a1;
}
unsigned char speed = 0;
bool CheckCRC(unsigned char *buf)
{
	unsigned int crcValue;
	crcValue = GetCrc32(&buf[8], buf[8] + buf[9] * 256 - 8);
	if ((buf[4] == (crcValue & 0xff)) && (buf[5] == ((crcValue >> 8) & 0xff))
	    && (buf[6] == ((crcValue >> 16) & 0xff))
	    && (buf[7] == ((crcValue >> 24) & 0xff))) {
		if ((buf[10] == 0x02) && (buf[11] == 0x01)) {
			speed++;
			//buf[23] = speed;
		}
		return true;
	} else {
		return false;
	}
}

int UART1_STATE = 0; //0-idle   1-busy
int UDP_SEND_RECV_Flag = 0; //0-idle  1-busy
static unsigned char SendBuff2MCU[4096];
static unsigned char SendBuff2APP[4096];
static unsigned char recvBuffFromAPP[4096];

unsigned int Config_timeUpdateFlag = 0;
unsigned int timeUpdate_count = 0;
unsigned int timeUpdate_tmp = 0;
void ChangeWifiData2UartData()
{
	//打包头部
	SendBuff2MCU[0] = SendBuff2MCU[1] = SendBuff2MCU[2] = SendBuff2MCU[3] =
	                                        0xcc; //起始标志

	SendBuff2MCU[8] = recvBuffFromAPP[4];
	SendBuff2MCU[9] = recvBuffFromAPP[5]; //消息长度
	SendBuff2MCU[10] = recvBuffFromAPP[10];
	SendBuff2MCU[11] = recvBuffFromAPP[11]; //服务类型消息类型
	SendBuff2MCU[12] = SendBuff2MCU[13] = SendBuff2MCU[14] = SendBuff2MCU[15] =
	        0x00;//保留

	//打包体
	int length = SendBuff2MCU[8] + SendBuff2MCU[9] * 256;
	for (int i = 16 ; i < length ; i++) {
		SendBuff2MCU[i] =  recvBuffFromAPP[i];
	}
	if (length >= 16) {
		unsigned int crcValue = GetCrc32(&SendBuff2MCU[8],
		                                 8 + recvBuffFromAPP[4] + recvBuffFromAPP[5] * 256 - 16);
		SendBuff2MCU[4] = crcValue & 0xff;
		SendBuff2MCU[5] = (crcValue >> 8) & 0xff;
		SendBuff2MCU[6] = (crcValue >> 16) & 0xff;
		SendBuff2MCU[7] = (crcValue >> 24) & 0xff;
	}
	//else if(length == 16)
	//{
	//SendBuff2MCU[4]=0xbb;
	//SendBuff2MCU[5]=0xbb;
	//SendBuff2MCU[6]=0xbb;
	//SendBuff2MCU[7]=0xbb;
	//}
	else {

	}
}

void ChangeUartData2WifiData()
{
	//打包头部
	SendBuff2APP[0] = SendBuff2APP[1] = SendBuff2APP[2] = SendBuff2APP[3] =
	                                        0xaa; //起始标志
	SendBuff2APP[4] = recvBuffFromMCU[(readIndex + 8) % UART_BUF_SIZE];
	SendBuff2APP[5] = recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE];
	SendBuff2APP[6] = 0; //消息长度
	//响应标志
	//序列号
	SendBuff2APP[10] = recvBuffFromMCU[(readIndex + 10) % UART_BUF_SIZE];
	SendBuff2APP[11] = recvBuffFromMCU[(readIndex + 11) %
	                                   UART_BUF_SIZE]; //服务类型消息类型
	//if(SendBuff2APP[10] == MSG_WARNING_REQ)
	//{
	SendBuff2APP[8] = SendBuff2APP[9] = 0; //序列号
	//}
	//else
	//{
	//SendBuff2APP[7] = 2;//回复
	//SendBuff2APP[8]=SendBuff2APP[9]=0;//序列号
	//}
	//打包体
	int length = SendBuff2APP[4] + SendBuff2APP[5] * 256;
	if (length > 16) {
		for (int i = 16 ; i < length ; i++) {
			SendBuff2APP[i] =  recvBuffFromMCU[(readIndex + i) % UART_BUF_SIZE];
		}
		unsigned int crcValue = GetCrc32(&SendBuff2APP[16], length - 16);
		SendBuff2APP[12] = crcValue & 0xff;
		SendBuff2APP[13] = (crcValue >> 8) & 0xff;
		SendBuff2APP[14] = (crcValue >> 16) & 0xff;
		SendBuff2APP[15] = (crcValue >> 24) & 0xff; //CRC校验
	} else if (length == 16) {
		SendBuff2APP[12] = 0xbb;
		SendBuff2APP[13] = 0xbb;
		SendBuff2APP[14] = 0xbb;
		SendBuff2APP[15] = 0xbb; //CRC校验
	} else {

	}
}
void SendData2MCU()
{
	while (UART1_STATE == 1) {
		Message("\r\n 1");
		vTaskDelay(10);
	}
	UART1_STATE = 1;
	for (int i = 0; i < SendBuff2MCU[8] + SendBuff2MCU[9] * 256 ; i++) {
		UARTCharPut(UARTA1_BASE, SendBuff2MCU[i]);
	}
	UART1_STATE = 0;
}

/*!
    \brief          Main thread

    This routine starts the application:
    It initializes the SPI interface with the NWP.
    Starts the GPIO drivers and UART console.
    Sets the Start time for the realtime clock.
    It starts the driver's internal Spawn thread (to receive asynchronous events).
    and launch the command prompt handler.

    \param          arg

    \return         This function doesn't return.

    \sa             sl_Task, Board_initSPI, InitTerm, cmd_prompt.

*/

typedef struct {
	int FileDataUpLoad_Flag;  //0-TCP上传完，没有数据要上传，1-TCP未上传完，有数据要上传
	int FileDataUpLoad_Length;//TCP上传 长度
	int FileUartReceive_Length;//Uart接收数据长度
	int UpLoad_Over;          //0-上传完毕，1-未上传完毕
	unsigned char FILE_READ_RESP[4096];
} TCP_Upload_Ctrl;
typedef struct {
	unsigned char head[16];
	unsigned char TP_FILE_PARA[72];
} TCP_FileWrite_Resp;
static TCP_Upload_Ctrl File_Upload_Ctrl;
TCP_FileWrite_Resp tcpFileWrite_Resp;

typedef struct {
	unsigned char head[16];
	unsigned char TP_DEV_VER[16];
	unsigned char TP_MH_VER[64];
	unsigned char TP_FPGA_VER[8];
} MCU_PARA_READ;
MCU_PARA_READ MCU_PARA_read;

typedef struct {
	int Realdy;
	unsigned char head[16];
	unsigned char TP_DEV_VER[16];
	unsigned char TP_FPGA_VER[8];
	unsigned char TP_MH_VER[64];
	unsigned char TP_GATE_VER[8];
	unsigned char TP_DVR_VER[8];
} APP_PARA_READ;
static APP_PARA_READ APP_PARA_READ_resp;

typedef struct {
	int Ready;
	unsigned char Head[16];
	unsigned char TP_WORK_TIME[8];
} TEST_CMD_REQ;
static TEST_CMD_REQ TEST_CMD_resp;

typedef struct {
	unsigned char head[16];
	unsigned char TP_TIME[12];
	//unsigned char TP_DVR_RESOLUTION[8];
} GET_TIME;
GET_TIME GET_TIME_REQ; GET_TIME GET_TIME_RESP;


typedef struct {
	unsigned char head[16];
} HeartBeat;
HeartBeat beatHeart;

typedef struct {
	unsigned char Head[16];
	unsigned char TP_DELAY[8];
	int RESET_ME_RESP_rdy;
} RESET_ME_RESP;
static RESET_ME_RESP RESET_ME_resp;

typedef struct {
	unsigned char Head[16];
	unsigned char TP_DEBUG_CMD[24];  //校准数据
	int ready;
} DEBUG_CMD_RESP;
DEBUG_CMD_RESP DEBUG_CMD_resp;


#define device_Connect_num      5
static _i16 Sd_UDP;
static SlSockAddrIn_t Addr_UDP[device_Connect_num];
static int heartbeat_Time_sencond_from_1970[device_Connect_num] = {0, 0, 0, 0, 0};
static unsigned char connect_count = 0;
int UDP_Recv_Flag = 0;


_i16 Status, Role; _i16 Status_UDP;
_i16 ClientSd_UDP;
int UDP_RUN_Flag = 0; int UART_RUN_Flag = 0; int TCP_RUN_Flag = 0;
TaskHandle_t xHandle_UDP_TASK; TaskHandle_t xHandle_TCP_TASK;
TaskHandle_t xHandle_UART_TASK; TaskHandle_t xHandle_RAW_TASK;
unsigned char switch_flag = 0x00;

typedef struct {
	unsigned char head[16];
	unsigned char TP_WARN_DAY_STAT[784];
	int WARNING_DAY_STAT_RESP_RDY;
} WARNING_DAY_STAT_RESP;
WARNING_DAY_STAT_RESP WARNING_DAY_STAT_resp;
typedef struct {
	unsigned char head[16];
	unsigned char TP_DVR_FILE_TYPE[6];
	unsigned char TP_DVR_FILE_LIST[4096];
	int DVR_FILE_LIST_RES_RDY;
} DVR_FILE_LIST_RESP;


typedef struct {
	unsigned char head[18];
	unsigned char Data[9 * 80];
	unsigned char Rdy;  // 1-串口数据有更新 ， 0-未收到新的串口数据
} CAN_DATA;
CAN_DATA can_data;


bool checkUartFrame(unsigned char *frame)
{
	if (frame[10] == 0x01) {
		if (((frame[11] == 0x10) | (frame[11] == 0x11) | (frame[11] == 0x20) | frame[11]
		     == 0x21)) {
			return true;
		} else {
			return false;
		}
	} else if (frame[10] == 0x02) {
		if (frame[11] == 0x01) {
			return true;
		} else {
			return false;
		}
	} else if (frame[10] == 0x03) {
		if ((frame[11] == 0x01) | (frame[11] == 0x02) | (frame[11] == 0x03)) {
			return true;
		} else {
			return false;
		}
	} else if (frame[10] == 0x06) {
		if (((frame[11] >= 0x01) && (frame[11] <= 0x03)) | ((frame[11] >= 0x08)
		        && (frame[11] <= 0x0a)) | (frame[11] == 0x0d) | (frame[11] == 0x0e) |
		    (frame[11] == 0x11)) {
			return true;
		} else {
			return false;
		}
	} else if (frame[10] == 0x07) {
		if (((frame[11] >= 0x05) && (frame[11] <= 0x07)) | (frame[11] == 0x02)) {
			return true;
		} else {
			return false;
		}
	} else if (frame[10] == 0x08) {
		if ((frame[11] >= 0x03) && (frame[11] <= 0x06)) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}


DVR_FILE_LIST_RESP DVR_FILE_LIST_resp;
unsigned portBASE_TYPE uxHighWaterMark_TCP;
unsigned portBASE_TYPE uxHighWaterMark_UDP;
unsigned portBASE_TYPE uxHighWaterMark_UART;
int RestartSimplink()
{
	int32_t             RetVal ;
	pthread_attr_t      pAttrs_spawn;
	struct sched_param  priParam;
	struct timespec     ts = {0};

	/* Initializes the SPI interface to the Network Processor and peripheral SPI (if defined in the board file) */
	Board_initSPI();
	Board_initGPIO();

	/* Init Application variables */
	RetVal = initAppVariables();

	/* initilize the realtime clock */
	clock_settime(CLOCK_REALTIME, &ts);

	/* Create the sl_Task internal spawn thread */
	pthread_attr_init(&pAttrs_spawn);
	priParam.sched_priority = SPAWN_TASK_PRIORITY;
	RetVal = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
	RetVal |= pthread_attr_setstacksize(&pAttrs_spawn, TASK_STACK_SIZE);

	/* The SimpleLink host driver architecture mandate spawn thread to be created prior to calling Sl_start (turning the NWP on). */
	/* The purpose of this thread is to handle asynchronous events sent from the NWP.
	 * Every event is classified and later handled by the Host driver event handlers. */
	RetVal = pthread_create(&gSpawn_thread, &pAttrs_spawn, sl_Task, NULL);
	if (RetVal < 0) {
		/* Handle Error */
		UART_PRINT("Network Terminal - Unable to create spawn thread \n");
		return (0);
	}

	/* Before turning on the NWP on, reset any previously configured parameters */
	/*
	     IMPORTANT NOTE - This is an example reset function, user must update this function
	                      to match the application settings.
	*/
	RetVal = ConfigureSimpleLinkToDefaultState();
	if (RetVal < 0) {
		/* Handle Error */
		UART_PRINT("Network Terminal - Couldn't configure Network Processor\n");
		return (0);
	}

	/* Output device information to the UART terminal */
	//RetVal = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);

	//if(RetVal < 0)
	//{
	/* Handle Error */
	//UART_PRINT("Network Terminal - Unable to retrieve device information \n");
	//return(0);
	//}

	/* Display Network Terminal API commands */
	showAvailableCmd();

	/*
	 * Calling UART handling method which serves as the application main loop.
	 * Note that this function doesn't return.
	 */
	RetVal = cmd_prompt(NULL);

	if (Status) {
		/* error */
		Message("\r\n error , sl_WlanSet! ");

	}
	if (sl_start_flag == 0) {
		sl_start_flag = 1;
	}

	if (RetVal) {
		while (1);
	}
	return (0);
}

unsigned long TokenConfig;
const unsigned char ConfigContent[] = "171024:";
static unsigned char ConfigBuf[256];
long writeConfigFile(unsigned char *buf, unsigned long ulToken)
{
	//记录中最后一日文件名称：配置2：配置3：配置4
	long lRetVal = -1;
	long lFileHandle;
	lFileHandle = sl_FsOpen((unsigned char *)"CONFIG",
	                        SL_FS_CREATE | SL_FS_OVERWRITE | SL_FS_CREATE_MAX_SIZE(1024), &ulToken);
	if (lFileHandle < 0) {
		//创建失败
		return -10;
	}

	lRetVal = sl_FsWrite(lFileHandle, 0, buf, sizeof(ConfigContent));

	if (lRetVal < 0) {
		lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
		//ASSERT_ON_ERROR(FILE_READ_FAILED);
		return -11;
	}

	lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
	if (SL_RET_CODE_OK != lRetVal) {
		//ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
		//关闭失败
		return -12;
	}
	return 0;//success
}
long readConfigFile(unsigned long ulToken)
{
	//记录中最后一日文件名称：配置2：配置3：配置4
	long lRetVal = -1;
	long lFileHandle;
	lFileHandle = sl_FsOpen((unsigned char *)"CONFIG", SL_FS_READ, &ulToken);
	if (lFileHandle < 0) { //配置文件不存在
		return -10;
	} else { //存在配置文件
		lRetVal = sl_FsRead(lFileHandle, 0, ConfigBuf, sizeof(ConfigContent));
		if ((lRetVal < 0) || (lRetVal != sizeof(ConfigContent)))
			//if (lRetVal < 0)
		{
			lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
			//ASSERT_ON_ERROR(FILE_READ_FAILED);
			return -11;
		}

		lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
		if (SL_RET_CODE_OK != lRetVal) {
			//ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
			//关闭失败
			return -12;
		}
		return 0;//success
	}
}


unsigned int Statistics_FileList_Num = 0;
unsigned int Statistics_FileList_Count = 0;
unsigned char Statistics_FileList[100][11];
unsigned char Date_temp_str[7];
typedef struct {
	SlFileAttributes_t attribute;
	char fileName[SL_FS_MAX_FILE_NAME_LENGTH];
} slGetfileList_t;

#define COUNT 5

void PrintFileListProperty(_u16 prop);



int compareFileName(unsigned char *Date1, unsigned char *Date2)
{
	if ((Date2[0] == 0) || (Date2[1] == 0) || (Date2[2] == 0) || (Date2[3] == 0)
	    || (Date2[4] == 0) || (Date2[5] == 0)) {
		return 1;
	}
	unsigned int Date1_Dec = 0; unsigned int Date2_Dec;
	Date1_Dec = (Date1[0] - '0') * 10 * 10 * 10 * 10 * 10 +
	            (Date1[1] - '0') * 10 * 10 * 10 * 10 + (Date1[2] - '0') * 10 * 10 * 10 +
	            (Date1[3] - '0') * 10 * 10 + (Date1[4] - '0') * 10 + (Date1[5] - '0');
	Date2_Dec = (Date2[0] - '0') * 10 * 10 * 10 * 10 * 10 +
	            (Date2[1] - '0') * 10 * 10 * 10 * 10 + (Date2[2] - '0') * 10 * 10 * 10 +
	            (Date2[3] - '0') * 10 * 10 + (Date2[4] - '0') * 10 + (Date2[5] - '0');
	if (Date1_Dec < Date2_Dec) {
		return -1;
	} else if (Date1_Dec == Date2_Dec) {
		return 0;
	} else {
		return 1;
	}
}
int Get_Statistics_FileList()
{
	_i32 NumOfEntriesOrError = 1;
	_i32 Index = -1;
	slGetfileList_t File[COUNT];
	_i32  i;
	_i32 RetVal = 0;

	UART_PRINT("\r\n");
	while (NumOfEntriesOrError > 0) {
		NumOfEntriesOrError = sl_FsGetFileList(&Index, COUNT,
		                                       (_u8)(SL_FS_MAX_FILE_NAME_LENGTH + sizeof(SlFileAttributes_t)),
		                                       (unsigned char *)File, SL_FS_GET_FILE_ATTRIBUTES);
		if (NumOfEntriesOrError < 0) {
			RetVal = NumOfEntriesOrError;//error
			break;
		}
		for (i = 0; i < NumOfEntriesOrError; i++) {
			//UART_PRINT("\n\rUDP TASK 剩余堆栈:%d",uxHighWaterMark_UDP);
			UART_PRINT("Name: %s  ", File[i].fileName);
			UART_PRINT("AllocatedBlocks: %5d ", File[i].attribute.FileAllocatedBlocks);
			UART_PRINT("MaxSize(byte): %5d  ", File[i].attribute.FileMaxSize);
			UART_PRINT("Properties:%d ", (_u16)File[i].attribute.Properties);
			UART_PRINT("\r\n\n");

			if ((File[i].fileName[6] == '.') && (File[i].fileName[7] == 's')
			    && (File[i].fileName[8] == 'a') && (File[i].fileName[9] == 't')) {
				for (int j = 0; j < 10 ; j++) {
					Statistics_FileList[Statistics_FileList_Num][j] = File[i].fileName[j];
				}
				Statistics_FileList[Statistics_FileList_Num][10] = '\0';
				Statistics_FileList_Num++;
			}

		}
	}


	for (int j = 0; j < Statistics_FileList_Num; j++) {
		UART_PRINT("\r\n %s", Statistics_FileList[j]);
	}

	unsigned char temp_str[6];
	//排序
	for (int m = 0; m < Statistics_FileList_Num; m++) {
		for (int n = m; n < Statistics_FileList_Num; n++)
			if (compareFileName(Statistics_FileList[n], Statistics_FileList[n + 1]) < 0) {
				for (i = 0; i < 6; i++) {
					temp_str[i] = Statistics_FileList[n][i];
				}
				for (i = 0; i < 6; i++) {
					Statistics_FileList[n][i] = Statistics_FileList[n + 1][i];
				}
				for (i = 0; i < 6; i++) {
					Statistics_FileList[n + 1][i] = temp_str[i];
				}
			}
	}

	if (Statistics_FileList_Num != 0) {
		for (int j = 0; j < Statistics_FileList_Num; j++) {
			UART_PRINT("\r\n %s", Statistics_FileList[j]);
		}
	}
	//UART_PRINT("%\n");
	return RetVal;//0 means O.K
}

void PrintFileListProperty(_u16 prop)
{
	printf("Flags : ");
	if (prop & SL_FS_INFO_MUST_COMMIT) {
		printf("Open file commit,");
	}
	if (prop & SL_FS_INFO_BUNDLE_FILE) {
		printf("Open bundle commit,");
	}
	if (prop & SL_FS_INFO_PENDING_COMMIT) {
		printf("Pending file commit,");
	}
	if (prop & SL_FS_INFO_PENDING_BUNDLE_COMMIT) {
		printf("Pending bundle commit,");
	}
	if (prop & SL_FS_INFO_SECURE) {
		printf("Secure,");
	}
	if (prop & SL_FS_INFO_NOT_FAILSAFE) {
		printf("File safe,");
	}
	if (prop & SL_FS_INFO_SYS_FILE) {
		printf("System,");
	}
	if (prop & SL_FS_INFO_NOT_VALID) {
		printf("No valid copy,");
	}
	if (prop & SL_FS_INFO_PUBLIC_WRITE) {
		printf("Public write,");
	}
	if (prop & SL_FS_INFO_PUBLIC_READ) {
		printf("Public read,");
	}
}

int Network(void *arg)//wifi with device
{
	unsigned char serviceType = 0xff, messageType = 0xff;
	_i16 Ret_UDP_Send = 0, Sd_UDP_TMP, recv_Length = 0;
	unsigned char *temp;
	int length = 0, i = 0;
	unsigned int FileLength = 0, crcValue, count = 0, while_count = 0;
	unsigned int second_from_1970_1_1, year = 0, month = 0, day = 0, hour, minute,
	                                   second;
	unsigned long ulToken;
	SlSockAddrIn_t Addr_UDP_TMP;

	RestartSimplink();

	//WriteFileToDevice(&ulToken);
	//ReadFileFromDevice(ulToken);
	//Send_suggested_response();
	//读取配置文件
	//取得当天文件名称
	//打开当天统计文件，获得handle
	//UART1 串口接收，中断配置
	readConfigFile(TokenConfig);  //读取配置文件
	Get_Statistics_FileList();
	sl_FsDel("781118.sat", 0);

	MAP_UARTConfigSetExpClk(UARTA1_BASE, MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
	                        921600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                                 UART_CONFIG_PAR_NONE));
	MAP_UARTIntRegister(UARTA1_BASE, UARTIntHandler);
	UARTFIFOLevelSet(UARTA1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
	MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX);

	setSSID();

	Addr_UDP_TMP.sin_family = SL_AF_INET;
	Addr_UDP_TMP.sin_port = sl_Htons(6666);
	Addr_UDP_TMP.sin_addr.s_addr = SL_INADDR_ANY;

	//_i16 AddrSize = sizeof(SlSockAddrIn_t);

	//while(1)
	{
		//创建UDP socket
		Sd_UDP = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
		if (Sd_UDP < 0) {
			Message("\r\n error , can not create UDP socket! ");
			//continue;
		} else {
			Message("\r\n create UDP socket success!");
		}
		Status_UDP = sl_Bind(Sd_UDP, (SlSockAddr_t *)&Addr_UDP_TMP,
		                     sizeof(SlSockAddrIn_t));
		if (Status_UDP) { //
			// error
			Message("\r\nsl_Bind , UDP error!");
			//continue;
		} else {
			UDP_RUN_Flag = 1;
			Message("\r\n sl_Bind , UDP socket success!");
		}

		TCP_RUN_Flag = 1;
		//char *paraes_RAW_TASK=NULL;
		//xTaskCreate((TaskFunction_t)RAW_TASK,"RAW_TASK",1024,(void *)&paraes_RAW_TASK,1,&xHandle_RAW_TASK);

		//消息处理
		while (1) {
			while_count++;
			if (readIndex < writeIndex) {
				if ((recvBuffFromMCU[readIndex % UART_BUF_SIZE] == 0xcc)
				    && (recvBuffFromMCU[(readIndex + 1) % UART_BUF_SIZE] == 0xcc)
				    && (recvBuffFromMCU[(readIndex + 2) % UART_BUF_SIZE] == 0xcc)
				    && (recvBuffFromMCU[(readIndex + 3) % UART_BUF_SIZE] == 0xcc)) {
					length = recvBuffFromMCU[(readIndex + 8) % UART_BUF_SIZE] +
					         recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE] * 256;
					/*if((writeIndex - readIndex - length) < 0)
					{
					  for(i=0;i<120;i++)
					    UART_PRINT("%02x ",recvBuffFromMCU[readIndex%UART_BUF_SIZE + i]);
					  UART_PRINT("\r\n over");
					}
					*/
					if (((writeIndex - readIndex) > 12)) { //可以判断
						if (checkUartFrame(&recvBuffFromMCU[readIndex % UART_BUF_SIZE])) {
							if ((writeIndex - readIndex - length) >= 0) { //  有个正确完整的数据帧
								{
									//Message("\r\n received one frame data");
									serviceType = recvBuffFromMCU[(readIndex + 10) % UART_BUF_SIZE];
									messageType = recvBuffFromMCU[(readIndex + 11) % UART_BUF_SIZE];
									//if(serviceType == SERVICE_WARNING)
									//{
									//UART_PRINT("\n\rserviceType = %d,",serviceType);
									//Message("\n\r 串口报警报文:");
									//for(int i = readIndex ; i < readIndex + length ; i++)
									//UART_PRINT("%02x ",recvBuffFromMCU[i%UART_BUF_SIZE]);
									//}

									if (serviceType == NO_SERVICE) {                  //无服务
										Message("\r\n FROM MCU NO_SERVICE!");
									} else if (serviceType == SERVICE_FILE) {         //文件服务回复
										//Message("\r\n MCU文件服务!");
										switch (messageType) {
										case MSG_FILE_READ_RESP :                     //读取文件回复 , 第一包数据
											while (File_Upload_Ctrl.FileDataUpLoad_Flag == 1);            //TCP未上传完毕
											FileLength = recvBuffFromMCU[(readIndex + 8) % UART_BUF_SIZE] +
											             recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE] * 256 - 16 - 76 -
											             10; //从串口接收到的数据长度 - 16头 - 76 TP_FILE_PARA - 10 TP_FILE_PACKET

											File_Upload_Ctrl.FILE_READ_RESP[0] = File_Upload_Ctrl.FILE_READ_RESP[1] =
											        File_Upload_Ctrl.FILE_READ_RESP[2] = File_Upload_Ctrl.FILE_READ_RESP[3] = 0xaa;
											//File_Upload_Ctrl.FILE_READ_RESP[4]=(FileLength + 16 + 72 + ) % 256;File_Upload_Ctrl.FILE_READ_RESP[5]=(FileLength + 16 + 72) / 256;File_Upload_Ctrl.FILE_READ_RESP[6]=0;//长度
											File_Upload_Ctrl.FILE_READ_RESP[7] = 0x02;
											File_Upload_Ctrl.FILE_READ_RESP[8] = recvBuffFromMCU[(readIndex + 16 + 76 + 6) %
											                                     UART_BUF_SIZE];
											File_Upload_Ctrl.FILE_READ_RESP[9] = recvBuffFromMCU[(readIndex + 16 + 76 + 6 +
											                                     1) % UART_BUF_SIZE];
											File_Upload_Ctrl.FILE_READ_RESP[10] = SERVICE_FILE;
											File_Upload_Ctrl.FILE_READ_RESP[11] = MSG_FILE_READ_RESP;


											for (i = 0; i < 72 ; i++) { //File name , File Length ,File CRC
												File_Upload_Ctrl.FILE_READ_RESP[16 + i] = recvBuffFromMCU[(readIndex + 16 + 4 +
												        i) % UART_BUF_SIZE];
											}

											for (i = 0;
											     i < (recvBuffFromMCU[(readIndex + 16 + 76 + 8) % UART_BUF_SIZE] +
											          recvBuffFromMCU[(readIndex + 16 + 76 + 9) % UART_BUF_SIZE] * 256) ;
											     i++) { //Filedata
												File_Upload_Ctrl.FILE_READ_RESP[16 + 72 + i] = recvBuffFromMCU[(readIndex + 16 +
												        76 + 10 + i) % UART_BUF_SIZE];
											}

											File_Upload_Ctrl.FileUartReceive_Length = recvBuffFromMCU[(readIndex + 8) %
											        UART_BUF_SIZE] + recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE] * 256;
											File_Upload_Ctrl.FileDataUpLoad_Length = 16 + 72 + recvBuffFromMCU[(readIndex +
											        16 + 76 + 8) % UART_BUF_SIZE] + recvBuffFromMCU[(readIndex + 16 + 76 + 9) %
											                UART_BUF_SIZE] * 256;

											File_Upload_Ctrl.FILE_READ_RESP[4] = File_Upload_Ctrl.FileDataUpLoad_Length %
											                                     256; File_Upload_Ctrl.FILE_READ_RESP[5] =
											                                             File_Upload_Ctrl.FileDataUpLoad_Length / 256;
											File_Upload_Ctrl.FILE_READ_RESP[6] = 0;

											//消息体 CRC校验
											crcValue = GetCrc32(&File_Upload_Ctrl.FILE_READ_RESP[16],
											                    72 + recvBuffFromMCU[(readIndex + 16 + 76 + 8) % UART_BUF_SIZE] +
											                    recvBuffFromMCU[(readIndex + 16 + 76 + 9) % UART_BUF_SIZE] * 256);
											File_Upload_Ctrl.FILE_READ_RESP[12] = crcValue & 0xff;
											File_Upload_Ctrl.FILE_READ_RESP[13] = (crcValue >> 8) & 0xff;
											File_Upload_Ctrl.FILE_READ_RESP[14] = (crcValue >> 16) & 0xff;
											File_Upload_Ctrl.FILE_READ_RESP[15] = (crcValue >> 24) & 0xff;
											//UART_PRINT("\n\r");
											//for(int i =0; i< File_Upload_Ctrl.FileDataUpLoad_Length ;i++)        //头 16 + TLV 86 + 文件长度 Status - 72 -16
											//{
											//UART_PRINT("%2x",File_Upload_Ctrl.FILE_READ_RESP[i]);
											//}

											if ((recvBuffFromMCU[(readIndex + 16 + 80) % UART_BUF_SIZE] ==
											     recvBuffFromMCU[(readIndex + 16 + 82) % UART_BUF_SIZE])
											    && (recvBuffFromMCU[(readIndex + 16 + 81) % UART_BUF_SIZE] ==
											        recvBuffFromMCU[(readIndex + 16 + 83) % UART_BUF_SIZE])) {
												File_Upload_Ctrl.UpLoad_Over = 1;
											}
											File_Upload_Ctrl.FileDataUpLoad_Flag = 1; //收到数据
											break;

										case MSG_FILE_READ_MORE_RESP :                //读取文件服务
											while (File_Upload_Ctrl.FileDataUpLoad_Flag == 1);            //TCP未上传完毕
											FileLength = recvBuffFromMCU[(readIndex + 8) % UART_BUF_SIZE] +
											             recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE] * 256 - 16 - 10;
											File_Upload_Ctrl.FILE_READ_RESP[0] = File_Upload_Ctrl.FILE_READ_RESP[1] =
											        File_Upload_Ctrl.FILE_READ_RESP[2] = File_Upload_Ctrl.FILE_READ_RESP[3] = 0xaa;
											File_Upload_Ctrl.FILE_READ_RESP[4] = (FileLength + 16 + 72) % 256;
											File_Upload_Ctrl.FILE_READ_RESP[5] = (FileLength + 16 + 72) / 256;
											File_Upload_Ctrl.FILE_READ_RESP[6] = 0; //长度
											File_Upload_Ctrl.FILE_READ_RESP[7] = 0x02;
											File_Upload_Ctrl.FILE_READ_RESP[8] = recvBuffFromMCU[(readIndex + 16 + 76 + 6) %
											                                     UART_BUF_SIZE];
											File_Upload_Ctrl.FILE_READ_RESP[9] = recvBuffFromMCU[(readIndex + 16 + 76 + 6 +
											                                     1) % UART_BUF_SIZE]; //序列号
											File_Upload_Ctrl.FILE_READ_RESP[10] = SERVICE_FILE;
											File_Upload_Ctrl.FILE_READ_RESP[11] =
											    MSG_FILE_READ_MORE_RESP;       //服务类型  消息类型

											for (int i = 0;
											     i < (recvBuffFromMCU[(readIndex + 16 + 8) % UART_BUF_SIZE] +
											          recvBuffFromMCU[(readIndex + 16 + 9) % UART_BUF_SIZE] * 256) ; i++) { //Filedata
												File_Upload_Ctrl.FILE_READ_RESP[16 + 72 + i] = recvBuffFromMCU[(readIndex + 16 +
												        10 + i) % UART_BUF_SIZE];
											}
											File_Upload_Ctrl.FileUartReceive_Length = recvBuffFromMCU[(readIndex + 8) %
											        UART_BUF_SIZE] + recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE] * 256;
											File_Upload_Ctrl.FileDataUpLoad_Length = 16 + 72 + recvBuffFromMCU[(readIndex +
											        16 + 8) % UART_BUF_SIZE] + recvBuffFromMCU[(readIndex + 16 + 9) % UART_BUF_SIZE]
											        * 256;

											//消息体CRC校验
											crcValue = GetCrc32(&File_Upload_Ctrl.FILE_READ_RESP[16],
											                    72 + recvBuffFromMCU[(readIndex + 16 + 8) % UART_BUF_SIZE] +
											                    recvBuffFromMCU[(readIndex + 16 + 9) % UART_BUF_SIZE] * 256);
											File_Upload_Ctrl.FILE_READ_RESP[12] = crcValue & 0xff;
											File_Upload_Ctrl.FILE_READ_RESP[13] = (crcValue >> 8) & 0xff;
											File_Upload_Ctrl.FILE_READ_RESP[14] = (crcValue >> 16) & 0xff;
											File_Upload_Ctrl.FILE_READ_RESP[15] = (crcValue >> 24) & 0xff;

											//for(int i =0; i< File_Upload_Ctrl.FileDataUpLoad_Length ;i++)        //头 16 + TLV 86 + 文件长度 Status - 72 -16
											//{
											//UART_PRINT("%2x",File_Upload_Ctrl.FILE_READ_RESP[i]);
											//}

											if ((recvBuffFromMCU[(readIndex + 16 + 4) % UART_BUF_SIZE] ==
											     recvBuffFromMCU[(readIndex + 16 + 6) % UART_BUF_SIZE])
											    && (recvBuffFromMCU[(readIndex + 16 + 5) % UART_BUF_SIZE] ==
											        recvBuffFromMCU[(readIndex + 16 + 7) % UART_BUF_SIZE])) {
												File_Upload_Ctrl.UpLoad_Over = 1;
											}
											File_Upload_Ctrl.FileDataUpLoad_Flag = 1; //收到数据
											break;

										case MSG_FILE_WRITE_RESP :                    //写文件回应
											Write_File_Resp = 1;
											Message("\r\n MSG_FILE_WRITE_RESP");
											break;
										case MSG_FILE_WRITE_MORE_RESP :              //写文件回应
											Write_File_Resp_More = 1;
											Message("\r\n MSG_FILE_WRITE_MORE_RESP");
											break;
										}
									} else if (serviceType == SERVICE_SETTING) {      //参数配置服务
										SendBuff2APP[7] = 2;
										unsigned int crcValue;
										//Message("\r\n MCU参数配置服务!");
										switch (messageType) {
										case MSG_PARA_READ_RESP:      //读取参数 回复
											APP_PARA_READ_resp.head[0] = APP_PARA_READ_resp.head[1] =
											                                 APP_PARA_READ_resp.head[2] = APP_PARA_READ_resp.head[3] = 0xaa;
											APP_PARA_READ_resp.head[4] = 120; APP_PARA_READ_resp.head[5] = 0;
											APP_PARA_READ_resp.head[6] = 0; //length
											APP_PARA_READ_resp.head[7] = 2;
											APP_PARA_READ_resp.head[8] = APP_PARA_READ_resp.head[9] = 0; //序列号
											APP_PARA_READ_resp.head[10] = 3; APP_PARA_READ_resp.head[11] = 1;
											for (i = 0; i < 64 ; i++) {                             //Copy TP_MH_VER
												APP_PARA_READ_resp.TP_MH_VER[i] = recvBuffFromMCU[(readIndex + 16 + 16 + i) %
												                                  UART_BUF_SIZE];
											}
											for (i = 0; i < 16; i++) {                              //Copy TP_DEV_VER
												APP_PARA_READ_resp.TP_DEV_VER[i] = recvBuffFromMCU[(readIndex + 16 + i) %
												                                   UART_BUF_SIZE];
											}
											for (i = 0; i < 8; i++) {                              //Copy P_FPGA_VER
												APP_PARA_READ_resp.TP_FPGA_VER[i] = recvBuffFromMCU[(readIndex + 16 + 16 + 64 +
												                                    i) % UART_BUF_SIZE];
											}

											APP_PARA_READ_resp.TP_GATE_VER[0] = TP_GATE_VER_ID;
											APP_PARA_READ_resp.TP_GATE_VER[1] = 0;
											APP_PARA_READ_resp.TP_GATE_VER[2] = 8; APP_PARA_READ_resp.TP_GATE_VER[3] = 0;
											APP_PARA_READ_resp.TP_GATE_VER[4] = MAIN_WIFI_VER % 256;
											APP_PARA_READ_resp.TP_GATE_VER[5] = MAIN_WIFI_VER / 256;
											APP_PARA_READ_resp.TP_GATE_VER[6] = SUB_WIFI_VER % 256;
											APP_PARA_READ_resp.TP_GATE_VER[7] = SUB_WIFI_VER / 256;

											for (i = 0; i < 8; i++) {                              //Copy P_FPGA_VER
												APP_PARA_READ_resp.TP_DVR_VER[i] = recvBuffFromMCU[(readIndex + 16 + 16 + 64 + 8
												                                   + i) % UART_BUF_SIZE];
											}

											crcValue = GetCrc32(APP_PARA_READ_resp.TP_DEV_VER, 96 + 8);
											APP_PARA_READ_resp.head[12] = crcValue & 0xff;
											APP_PARA_READ_resp.head[13] = (crcValue >> 8) & 0xff;
											APP_PARA_READ_resp.head[14] = (crcValue >> 16) & 0xff;
											APP_PARA_READ_resp.head[15] = (crcValue >> 24) & 0xff;

											temp = APP_PARA_READ_resp.head;
											//UART_PRINT("\n\r回复APP一下报文\n\r");
											//for(int i =0; i< 16+16+8+64+8 ;i++)        //头 16 + TLV 86 + 文件长度 Status - 72 -16
											//{
											//UART_PRINT("%2x",temp[i]);
											//}
											unsigned char *temp = APP_PARA_READ_resp.head;
											Message("\n\r APP读取参数报文1:");
											for (i = 0 ;
											     i < APP_PARA_READ_resp.head[4] + APP_PARA_READ_resp.head[5] * 256 + 8; i++) {
												UART_PRINT("%02x ", temp[i]);
											}

											APP_PARA_READ_resp.Realdy = 1;
											break;

										case MSG_PARA_SET_RESP:
											Message("\r\n MCU写参数回复!");
											break;
										case MSG_PARA_GET_TIME_RESP:                  //获取时间回复
											//Message("\r\n 获取时间回复!");
											if (Config_timeUpdateFlag == 1) {
												if (CheckCRC(&recvBuffFromMCU[readIndex % UART_BUF_SIZE])) {
													second_from_1970_1_1 = recvBuffFromMCU[(readIndex + 16 + 4) % UART_BUF_SIZE] +
													                       recvBuffFromMCU[(readIndex + 16 + 5) % UART_BUF_SIZE] * 256 +
													                       recvBuffFromMCU[(readIndex + 16 + 6) % UART_BUF_SIZE] * 256 * 256 +
													                       recvBuffFromMCU[(readIndex + 16 + 7) % UART_BUF_SIZE] * 256 * 256 * 256;
													//UART_PRINT("\n\r total second = %d",second_from_1970_1_1);
													gettime(second_from_1970_1_1);
													//移动文件列表
													Date_temp_str[0] = (tm.year % 1000) / 10 + '0';
													Date_temp_str[1] = (tm.year % 1000) % 10 + '0';
													Date_temp_str[2] = tm.month / 10 + '0'; Date_temp_str[3] = tm.month % 10 + '0';
													Date_temp_str[4] = tm.day / 10 + '0'; Date_temp_str[5] = tm.day % 10 + '0';
													Date_temp_str[6] = '\0';
													if (compareFileName(&Date_temp_str[0], Statistics_FileList[0]) != 0) {
														Statistics_FileList_Num++;
														for (int m = 0; m < Statistics_FileList_Num; m++) {
															for (int n = 0; n < 6; n++) {
																Statistics_FileList[m + 1][n] = Statistics_FileList[m][n];
															}
														}
														for (int n = 0; n < 6; n++) {
															Statistics_FileList[0][n] = Date_temp_str[n];
														}
													}

													//比较获取的日期，与配置文件中的日期，若日期更改则关闭文件并重新打开新文件，文件名为当天日期
													unsigned char FileName[6];
													FileName[0] = ((tm.year % 100) / 10) + '0';
													FileName[1] = (tm.year % 10) + '0'; //年
													FileName[2] = ((tm.month % 100) / 10) + '0';
													FileName[3] = (tm.month % 10) + '0';//月
													FileName[4] = ((tm.day % 100) / 10) + '0';
													FileName[5] = (tm.day % 10) + '0'; //日

													if ((ConfigBuf[0] == FileName[0]) && (ConfigBuf[1] == FileName[1])
													    && (ConfigBuf[2] == FileName[2]) && (ConfigBuf[3] == FileName[3])
													    && (ConfigBuf[4] == FileName[4]) && (ConfigBuf[5] == FileName[5])) {
													} else {
														ConfigBuf[0] = FileName[0];
														ConfigBuf[1] = FileName[1];
														ConfigBuf[2] = FileName[2];
														ConfigBuf[3] = FileName[3];
														ConfigBuf[4] = FileName[4];
														ConfigBuf[5] = FileName[5];
														//保存文件
														unsigned long ulToke_tmp;
														writeConfigFile(ConfigBuf, ulToke_tmp);
													}
												}
											}
											break;
										}
									} else if (serviceType == SERVICE_WARNING) {      //报警服务
										switch (messageType) {
										case MSG_WARNING_REQ:
											if (CheckCRC(&recvBuffFromMCU[readIndex % UART_BUF_SIZE])) {
												//统计
												dealWarning(&recvBuffFromMCU[readIndex % UART_BUF_SIZE],
												            recvBuffFromMCU[(readIndex + 8) % UART_BUF_SIZE] + recvBuffFromMCU[(readIndex +
												                    9) % UART_BUF_SIZE] * 256);
												//上报APP 报警数据
												SendBuff2APP[7] = 0;
												//Message("\n\r APP报警报文1:");
												//for(i = 0 ; i < SendBuff2APP[4] + SendBuff2APP[5]*256 ; i++)
												//UART_PRINT("%02x ",SendBuff2APP[i]);
												ChangeUartData2WifiData();
												if (UDP_Recv_Flag == 1) {
													for (i = 0; i < device_Connect_num ; i++) {
														if (heartbeat_Time_sencond_from_1970[i] != 0) {
															Ret_UDP_Send = sl_SendTo(Sd_UDP, SendBuff2APP,
															                         SendBuff2APP[4] + SendBuff2APP[5] * 256, 0, (SlSockAddr_t *)&Addr_UDP[i],
															                         sizeof(SlSockAddr_t));
															if (Ret_UDP_Send < 0) {
																Message("\r\n 报警上传失败!");
															} else {
																//Message("\r\n 上传报警信息成功!");
																//Message("\n\r 上发APP报警报文2:");
																//for(i = 0 ; i < SendBuff2APP[4] + SendBuff2APP[5]*256 ; i++)
																//UART_PRINT("%02x ",SendBuff2APP[i]);
															}
														}
													}
												}
											}
											break;
										case MSG_WARNING_DAY_STAT_RESP:
											break;
										}
									} else if (serviceType == SERVICE_HEARTBEAT) {      //心跳服务
										Message("\r\n MCU心跳服务!");
									} else if (serviceType == SERVICE_CMD) {          //命令服务
										Message("\r\n MCU命令服务!");
										SendBuff2APP[7] = 2;
										switch (messageType) {
										case MSG_CMD_RESET_RESP:
											ChangeUartData2WifiData();
											if (UDP_Recv_Flag == 1) {
												for (i = 0; i < device_Connect_num; i++) {
													if (heartbeat_Time_sencond_from_1970[i] != 0) {
														Ret_UDP_Send = sl_SendTo(Sd_UDP, SendBuff2APP,
														                         SendBuff2APP[4] + SendBuff2APP[5] * 256, 0, (SlSockAddr_t *)&Addr_UDP[i],
														                         sizeof(SlSockAddr_t));
														if (Ret_UDP_Send < 0) {
															Message("\r\n 回复APP 复位系统命令回复  失败!");
														} else {
															Message("\r\n 回复APP 复位系统命令回复  成功!");
														}
													}
												}
											}
											break;
										case MSG_CMD_RESET_ME_RESP:
											ChangeUartData2WifiData();
											if (UDP_Recv_Flag == 1) {
												for (i = 0; i < device_Connect_num; i++) {
													if (heartbeat_Time_sencond_from_1970[i] != 0) {
														Ret_UDP_Send = sl_SendTo(Sd_UDP, SendBuff2APP,
														                         SendBuff2APP[4] + SendBuff2APP[5] * 256, 0, (SlSockAddr_t *)&Addr_UDP[i],
														                         sizeof(SlSockAddr_t));
														if (Ret_UDP_Send < 0) {
															Message("\r\n 回复APP 复位ME 失败!");
														} else {
															Message("\r\n 回复APP 复位ME 成功!");
														}
													}
												}
											}

											Message("\r\n MSG_CMD_RESET_ME_REQ");
											RESET_ME_resp.Head[0] = RESET_ME_resp.Head[1] = RESET_ME_resp.Head[2] =
											                            RESET_ME_resp.Head[3] = 0xaa; //头
											RESET_ME_resp.Head[4] = recvBuffFromMCU[(readIndex + 8) % UART_BUF_SIZE];
											RESET_ME_resp.Head[5] = recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE];
											RESET_ME_resp.Head[6] = 0; RESET_ME_resp.Head[7] = 2;
											RESET_ME_resp.Head[8] = 0; RESET_ME_resp.Head[9] = 0;
											RESET_ME_resp.Head[10] = SERVICE_CMD; RESET_ME_resp.Head[11] = MSG_CMD_TEST_REQ;
											temp = &RESET_ME_resp.Head[16];
											for (i = 16; i < RESET_ME_resp.Head[4] + RESET_ME_resp.Head[5] * 256; i++) {
												temp[i] = recvBuffFromMCU[(readIndex + i) % UART_BUF_SIZE];
											}

											crcValue = GetCrc32(RESET_ME_resp.TP_DELAY, 8);
											RESET_ME_resp.Head[12] = crcValue & 0xff;
											RESET_ME_resp.Head[13] = (crcValue >> 8) & 0xff;
											RESET_ME_resp.Head[14] = (crcValue >> 16) & 0xff;
											RESET_ME_resp.Head[15] = (crcValue >> 24) & 0xff;
											RESET_ME_resp.RESET_ME_RESP_rdy = 1;
											break;
										case MSG_CMD_RESET_DVR_RESP:
											ChangeUartData2WifiData();
											if (UDP_Recv_Flag == 1) {
												for (i = 0; i < device_Connect_num; i++) {
													if (heartbeat_Time_sencond_from_1970[i] != 0) {
														Ret_UDP_Send = sl_SendTo(Sd_UDP, SendBuff2APP,
														                         SendBuff2APP[4] + SendBuff2APP[5] * 256, 0, (SlSockAddr_t *)&Addr_UDP[i],
														                         sizeof(SlSockAddr_t));
														if (Ret_UDP_Send < 0) {
															Message("\r\n 回复APP 复位DVR 失败!");
														} else {
															Message("\r\n 回复APP 复位DVR 成功!");
														}
													}
												}
											}
											break;
										case MSG_CMD_POWER_OFF_RESP:
											ChangeUartData2WifiData();
											if (UDP_Recv_Flag == 1) {
												for (i = 0; i < device_Connect_num; i++) {
													if (heartbeat_Time_sencond_from_1970[i] != 0) {
														Ret_UDP_Send = sl_SendTo(Sd_UDP, SendBuff2APP,
														                         SendBuff2APP[4] + SendBuff2APP[5] * 256, 0, (SlSockAddr_t *)&Addr_UDP[i],
														                         sizeof(SlSockAddr_t));
														if (Ret_UDP_Send < 0) {
															Message("\r\n 回复APP 关闭系统 失败!");
														} else {
															Message("\r\n 回复APP 关闭系统 成功!");
														}
													}
												}
											}
											break;
										case MSG_CMD_CALI_RESP:
											ChangeUartData2WifiData();
											if (UDP_Recv_Flag == 1) {
												for (i = 0; i < device_Connect_num; i++) {
													if (heartbeat_Time_sencond_from_1970[i] != 0) {
														Ret_UDP_Send = sl_SendTo(Sd_UDP, SendBuff2APP,
														                         SendBuff2APP[4] + SendBuff2APP[5] * 256, 0, (SlSockAddr_t *)&Addr_UDP[i],
														                         sizeof(SlSockAddr_t));
														if (Ret_UDP_Send < 0) {
															Message("\r\n 回复APP 校准指令 失败!");
														} else {
															Message("\r\n 回复APP 校准之类 成功!");
														}
													}
												}
											}
											break;
										case MSG_CMD_TEST_REQ :
											Message("\r\n MSG_CMD_TEST_REQ");
											TEST_CMD_resp.Head[0] = TEST_CMD_resp.Head[1] = TEST_CMD_resp.Head[2] =
											                            TEST_CMD_resp.Head[3] = 0xaa; //头
											TEST_CMD_resp.Head[4] = recvBuffFromMCU[(readIndex + 8) % UART_BUF_SIZE];
											TEST_CMD_resp.Head[5] = recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE];
											TEST_CMD_resp.Head[6] = 0; TEST_CMD_resp.Head[7] = 2;
											TEST_CMD_resp.Head[8] = 0; TEST_CMD_resp.Head[9] = 0;
											TEST_CMD_resp.Head[10] = SERVICE_CMD; TEST_CMD_resp.Head[11] = MSG_CMD_TEST_REQ;

											unsigned char *temp = &TEST_CMD_resp.Head[16];
											for (i = 16; i < TEST_CMD_resp.Head[4] + TEST_CMD_resp.Head[5] * 256; i++) {
												temp[i] = recvBuffFromMCU[(readIndex + i) % UART_BUF_SIZE];
											}
											//CRC
											crcValue = GetCrc32(TEST_CMD_resp.TP_WORK_TIME, 8);
											TEST_CMD_resp.Head[12] = crcValue & 0xff;
											TEST_CMD_resp.Head[13] = (crcValue >> 8) & 0xff;
											TEST_CMD_resp.Head[14] = (crcValue >> 16) & 0xff;
											TEST_CMD_resp.Head[15] = (crcValue >> 24) & 0xff;
											TEST_CMD_resp.Ready = 1;
											break;
										}
									} else if (serviceType == SERVICE_DVR) {    //行车记录仪服务
										switch (messageType) {
										case MSG_DVR_FILE_LIST_REQ :            //获取文件列表，如视频文件列表，照片列表
											Message("\r\n 获取到文件列表");
											DVR_FILE_LIST_resp.head[0] = DVR_FILE_LIST_resp.head[1] =
											                                 DVR_FILE_LIST_resp.head[2] = DVR_FILE_LIST_resp.head[3] = 0xaa;
											DVR_FILE_LIST_resp.head[4] = recvBuffFromMCU[(readIndex + 8) % UART_BUF_SIZE];
											DVR_FILE_LIST_resp.head[5] = recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE];
											DVR_FILE_LIST_resp.head[6] = 0;
											DVR_FILE_LIST_resp.head[7] = 2;
											DVR_FILE_LIST_resp.head[10] = 0x07; DVR_FILE_LIST_resp.head[11] = 0x02;
											//DVR_FILE_LIST_resp.head[8]=DVR_FILE_LIST_resp.head[9]=0;
											temp = DVR_FILE_LIST_resp.TP_DVR_FILE_TYPE;
											//打包体
											for (i = 0;
											     i < DVR_FILE_LIST_resp.head[4] + DVR_FILE_LIST_resp.head[5] * 256 - 16; i++) {
												temp[i] = recvBuffFromMCU[(readIndex + i + 16) % UART_BUF_SIZE];
											}
											DVR_FILE_LIST_resp.DVR_FILE_LIST_RES_RDY = 1;
											crcValue = GetCrc32(DVR_FILE_LIST_resp.TP_DVR_FILE_TYPE,
											                    DVR_FILE_LIST_resp.head[4] + DVR_FILE_LIST_resp.head[5] * 256 - 16);
											DVR_FILE_LIST_resp.head[12] = crcValue & 0xff;
											DVR_FILE_LIST_resp.head[13] = (crcValue >> 8) & 0xff;
											DVR_FILE_LIST_resp.head[14] = (crcValue >> 16) & 0xff;
											DVR_FILE_LIST_resp.head[15] = (crcValue >> 24) & 0xff;
											//temp = DVR_FILE_LIST_resp.head;
											break;
										}
									} else if (serviceType == SERVICE_DEBUG) {        //调试服务
										switch (messageType) {
										case DEBUG_MCU_COMMAND:
											Message("\r\n MCU调试服务!");
											DEBUG_CMD_resp.Head[0] = DEBUG_CMD_resp.Head[1] = DEBUG_CMD_resp.Head[2] =
											                             DEBUG_CMD_resp.Head[3] = 0xaa; //头
											DEBUG_CMD_resp.Head[4] = recvBuffFromMCU[(readIndex + 8) % UART_BUF_SIZE];
											DEBUG_CMD_resp.Head[5] = recvBuffFromMCU[(readIndex + 9) % UART_BUF_SIZE];
											DEBUG_CMD_resp.Head[6] = 0; DEBUG_CMD_resp.Head[7] = 2;
											DEBUG_CMD_resp.Head[8] = 0; DEBUG_CMD_resp.Head[9] = 0;
											DEBUG_CMD_resp.Head[10] = SERVICE_DEBUG;
											DEBUG_CMD_resp.Head[11] = DEBUG_MCU_COMMAND;

											unsigned char *temp = DEBUG_CMD_resp.Head;
											for (i = 16; i < DEBUG_CMD_resp.Head[4] + DEBUG_CMD_resp.Head[5] * 256; i++) {
												temp[i] = recvBuffFromMCU[(readIndex + i) % UART_BUF_SIZE];
											}
											crcValue = GetCrc32(DEBUG_CMD_resp.TP_DEBUG_CMD, 24);
											DEBUG_CMD_resp.Head[12] = crcValue & 0xff;
											DEBUG_CMD_resp.Head[13] = (crcValue >> 8) & 0xff;
											DEBUG_CMD_resp.Head[14] = (crcValue >> 16) & 0xff;
											DEBUG_CMD_resp.Head[15] = (crcValue >> 24) & 0xff;
											DEBUG_CMD_resp.ready = 1;
											Message("\r\n DEBUG_CMD_resp.ready = 1!");
										}
									} else {
										//error
									}
									readIndex += length;
									//UART_PRINT("\n\rwriteIndex = %d,readIndex=%d",writeIndex,readIndex);
									vTaskDelay(1);
								}
								if ((writeIndex - readIndex) >= UART_BUF_SIZE) {
									readIndex++;
								}
							} else { //数据帧不完整
								vTaskDelay(1);
							}
						} else { //错误帧
							readIndex++;
						}
					}
					//else if(((writeIndex - readIndex) > 9) && ((writeIndex - readIndex - length) < 0))
					//{
					//readIndex++;
					//}
					else { //头部不完整
						vTaskDelay(1);
					}
					// 这里没有delay   同时 一直死循环导致
				} else {
					if (readIndex < writeIndex - 60) {
						readIndex++;    //移动指针
					}
				}
			} else {
				vTaskDelay(1);
			}

		}
	}
}


/********************************************************************************
*TCP task
*Deal with SERVICE_FILE , SERVICE_SETTING
*
********************************************************************************/
static SlSockAddrIn_t Addr_TCP;
static unsigned char RecvBuff_Tcp[4096];
static unsigned char SendBuff_Mcu[1024 + 4096];

static bool need_send_file = false;
static bool is_fisrt_pack = true;
static bool recive_file_is_finished = false; //
static bool send_file_is_finished = false; //
static bool is_the_last_pack =
    false;// 1 : the last pack , 0 : is not the last pack
static unsigned int received_file_length = 0;
static unsigned int need_receive_file_length = 0;
static unsigned short receive_Buf_index = 0;
static unsigned short send_pack_count = 0;

int totalPackage = 0; unsigned int FileSendLength_Inc = 0;
void TCP_TASK()
{
	static unsigned int year;
	static unsigned int month;
	static unsigned int day;

	_i16  i16Status = 0;
	int receiveState; unsigned char serviceType = 0xff;
	unsigned char messageType = 0xff;
	_i16 recv_Length = 0;
	int receiveNum; int packageLength = 0; int writed_FileLength = 0; _i16 Sd;
	_i16 ClientSd_TCP; _i16 retVal; _i16 AddrSize; unsigned int crc32Value; int i;
	static unsigned char times = 0; unsigned int length; unsigned char *temp;
	unsigned int crcValue = 0; unsigned int pos = 0; int32_t         nonBlocking;
	struct SlTimeval_t TimeVal;
	unsigned int File_Length = 0; SlSockAddrIn_t Addr_UDP_TMP;
	AddrSize = sizeof(SlSockAddrIn_t);
	//Message("\r\n tcp_task begin!");
	Addr_TCP.sin_family = SL_AF_INET;
	Addr_TCP.sin_port = sl_Htons(6667);
	Addr_TCP.sin_addr.s_addr = SL_INADDR_ANY;

	TimeVal.tv_sec = 5; // Seconds
	TimeVal.tv_usec = 0; // Microseconds. 10000 microseconds resolution
	//创建TCP socket
	while (TCP_RUN_Flag == 0) {
		vTaskDelay(20);
	}
	while (1) {
		Sd = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, 0);
		if (Sd < 0) {
			al_printf("\r\n error , can not create TCP socket! ");
			continue;
		} else {
			al_printf("\r\n create TCP socket success!");
		}
		i16Status = sl_Bind(Sd, (SlSockAddr_t *)&Addr_TCP, sizeof(SlSockAddrIn_t));
		if (i16Status) {
			// error
			al_printf("\r\n error ,bind TCP socket failed!");
			sl_Close(Sd);
			continue;
		} else {
			al_printf("\r\n sl_Bind , TCP socket success!");
		}

		i16Status = sl_SetSockOpt(Sd, SL_SOL_SOCKET, SL_SO_RCVTIMEO, (_u8 *)&TimeVal,
		                          sizeof(TimeVal));
		if (i16Status) {
			Message("\r\n error , set rev time out failed");
		} else {
			//Message("\r\n set rev time out success");
		}
		i16Status = sl_Listen(Sd, 1);
		if (i16Status) {
			UART_PRINT("[line:%d, error:%d] %s\n\r", __LINE__, i16Status);
			//Message("\r\n error , listen TCP socket failed!");
		} else {
			Message("\r\n sl_Listen , TCP socket success!");
		}

		while (1) {
			receiveState = 0; receiveNum = 0; totalPackage = 0; writed_FileLength = 0;
			while ((ClientSd_TCP = sl_Accept(Sd, (SlSockAddr_t *)&Addr_TCP,
			                                 &AddrSize)) < 0) {
				vTaskDelay(200);//sleep
				break;
			}
			al_printf("\r\n sl_Accept , TCP socket success!");
			while (receiveState == 0) {
				//处理 TCP消息
				//MAP_UARTIntDisable(UARTA1_BASE, UART_INT_RX | UART_INT_RT |
				//UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
				if (receive_Buf_index == 0) {
					i16Status = sl_Recv(ClientSd_TCP, RecvBuff_Tcp, 4096, SL_MSG_DONTWAIT);
				} else {
					i16Status = sl_Recv(ClientSd_TCP, RecvBuff_Tcp, 4096 - receive_Buf_index,
					                    SL_MSG_DONTWAIT);
				}
				//MAP_UARTIntEnable(UARTA1_BASE,UART_INT_RX);
				if ((SL_ERROR_BSD_EAGAIN == i16Status)) {
					//need debug
					UART_PRINT("L:%d", __LINE__); //need_handle---------------------
					continue;
				} else if (i16Status < 0) {
					UART_PRINT("L:%d", __LINE__); //need_handle---------------------
					receiveState = 1;
					break;
				} else {
					receiveNum++;
				}
				recv_Length = Status;
				if (0 > i16Status) {
					receiveNum = 1;
					Message("\r\n error , Receive socket failed!");
					UART_PRINT("\n\r Status = %d", Status);
					break;
				} else if (i16Status > 0) {
					//处理消息
					//Message("\r\nReceive success , TCP!");
					if (receiveNum == 1) { //从Wifi收到的第一包
						serviceType = RecvBuff_Tcp[0x0a];
						messageType = RecvBuff_Tcp[0x0b];
						//totalPackage = (RecvBuff_Tcp[4] + RecvBuff_Tcp[5]*256 + RecvBuff_Tcp[6]*65536) / 4096 +1;         //包总数
					}
					if (serviceType == NO_SERVICE) {            //无服务
						Message("\r\n\r\n\r\n\r\n\r\n\r\n NO_SERVICE!");
						i16Status = sl_Send(ClientSd_TCP, RecvBuff_Tcp, 16, 0);
						receiveState = 1;
					} else if (serviceType == SERVICE_FILE) {   //文件服务
						//Message("\r\n 文件服务");
						switch (messageType) {
						case MSG_FILE_READ_REQ :
							//Message(",请求读取文件!");
							SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] = 0xcc;
							SendBuff_Mcu[8] = (16 + 76) % 256; SendBuff_Mcu[9] = (16 + 76) / 256; //消息长度
							SendBuff_Mcu[10] = RecvBuff_Tcp[10]; SendBuff_Mcu[11] = RecvBuff_Tcp[11];
							//TLV ， TP_FILE_PARA
							SendBuff_Mcu[16] = TP_FILE_PARA_ID; SendBuff_Mcu[17] = 0;       //Type
							SendBuff_Mcu[18] = 76; SendBuff_Mcu[19] = 0;       //Length
							for (i = 0; i < 72;
							     i++) {                                  //Filename , File length , CRC32
								SendBuff_Mcu[20 + i] = RecvBuff_Tcp[16 + i];
							}
							//数据包CRC校验
							crc32Value = GetCrc32(&SendBuff_Mcu[8], 84); //TLV 72
							SendBuff_Mcu[4] = crc32Value & 0xff;
							SendBuff_Mcu[5] = (crc32Value >> 8) & 0xff;
							SendBuff_Mcu[6] = (crc32Value >> 16) & 0xff;
							SendBuff_Mcu[7] = (crc32Value >> 24) & 0xff;

							//UART_PRINT("\n\r");
							//串口发送 , 读取文件请求指令
							while (UART1_STATE == 1) {
								Message("\r\n 2");
								vTaskDelay(1);
							}
							UART1_STATE = 1;
							Message("读取文件指令报文:");
							for (i = 0; i < 16 + 76 ; i++) {  //头 16 + TLV 86 + 文件长度 Status - 72 -16
								UARTCharPut(UARTA1_BASE, SendBuff_Mcu[i]);
								UART_PRINT("%02x ", SendBuff_Mcu[i]);
							}
							UART1_STATE = 0;
							while (1) {
								//串口回复
								times = 0;
								while (File_Upload_Ctrl.FileDataUpLoad_Flag != 1) { //没有数据要上传，等待
									times ++;
									if (times >= 500) {
										break;
									}
									vTaskDelay(100);
								}
								if (File_Upload_Ctrl.FileDataUpLoad_Flag == 1) {
									File_Upload_Ctrl.FileDataUpLoad_Flag = 0;            //上传

									//Message("文件内容:");
									unsigned char *temp = File_Upload_Ctrl.FILE_READ_RESP;
									for (int i = 0; i < File_Upload_Ctrl.FileDataUpLoad_Length ;
									     i++) {  //头 16 + TLV 86 + 文件长度 Status - 72 -16
										//UARTCharPut(UARTA1_BASE , temp[i]);
										UART_PRINT("%02x ", temp[i]);
									}

									i16Status = sl_Send(ClientSd_TCP, File_Upload_Ctrl.FILE_READ_RESP,
									                    File_Upload_Ctrl.FileDataUpLoad_Length, 0);
									if (File_Upload_Ctrl.UpLoad_Over == 1) {
										File_Upload_Ctrl.UpLoad_Over = 0;
									}
									break;
								}
								if (times >= 180) {
									File_Upload_Ctrl.FileDataUpLoad_Flag = 0;
									File_Upload_Ctrl.UpLoad_Over = 0;
									break;
								}
							}
							receiveState = 1;
							break;
						case MSG_FILE_WRITE_REQ :                         //请求写入文件
							Message("\r\n请求写入文件!");
							need_send_file = true;
							is_fisrt_pack = false;
							send_file_is_finished = false;
							if (receiveNum == 1) {
								is_fisrt_pack = true;
								received_file_length = 0;
								pos = 0;
								Message("\n\r收到第一包数据报文:");

								temp = RecvBuff_Tcp;
								//UART1_STATE = 1;
								//for(int i =0; i< recv_Length ;i++)        //头 16 + TLV 86 + 文件长度 Status - 72 -16
								//{
								//UART_PRINT("%2x,",temp[i]);
								//}
								//UART1_STATE = 0;

								if (((RecvBuff_Tcp[16 + 64] + RecvBuff_Tcp[16 + 65] * 256 + RecvBuff_Tcp[16 +
								        66] * 256 * 256 + RecvBuff_Tcp[16 + 67] * 256 * 256 * 256) % 4096) == 0) {
									totalPackage = (RecvBuff_Tcp[16 + 64] + RecvBuff_Tcp[16 + 65] * 256 +
									                RecvBuff_Tcp[16 + 66] * 256 * 256 + RecvBuff_Tcp[16 + 67] * 256 * 256 * 256) /
									               4096;
								} else {
									totalPackage = (RecvBuff_Tcp[16 + 64] + RecvBuff_Tcp[16 + 65] * 256 +
									                RecvBuff_Tcp[16 + 66] * 256 * 256 + RecvBuff_Tcp[16 + 67] * 256 * 256 * 256) /
									               4096  +  1;
								}
								File_Length = RecvBuff_Tcp[80] + RecvBuff_Tcp[81] * 256 + RecvBuff_Tcp[82] * 256
								              * 256 + RecvBuff_Tcp[83] * 256 * 256 * 256;
								FileSendLength_Inc = recv_Length - 72 - 16;
								UART_PRINT("\r\nFileSendLength_Inc = %d,", FileSendLength_Inc);
								UART_PRINT("\r\nFile_Length = %d,", File_Length);

								tcpFileWrite_Resp.head[4] = (16 + 72) % 256;
								tcpFileWrite_Resp.head[5] = (16 + 72) / 256;
								tcpFileWrite_Resp.head[6] = 0; //写文件回复包中  ,  收到文件长度
								for (i = 0; i < 68; i++) {        //Filename
									tcpFileWrite_Resp.TP_FILE_PARA[i] = RecvBuff_Tcp[16 + i];
								}
								tcpFileWrite_Resp.TP_FILE_PARA[16 + 68] = tcpFileWrite_Resp.TP_FILE_PARA[16 +
								        69] = tcpFileWrite_Resp.TP_FILE_PARA[16 + 70] =
								                  tcpFileWrite_Resp.TP_FILE_PARA[16 + 71] = 0xbb;

								//TLV 初始化
								//TP_FILE_PARA
								FilePackage.TP_FILE_PARA[0] = TP_FILE_PARA_ID % 256;
								FilePackage.TP_FILE_PARA[1] = TP_FILE_PARA_ID / 256; //Type
								FilePackage.TP_FILE_PARA[2] = 76;
								FilePackage.TP_FILE_PARA[3] = 0;                 //Length
								for (i = 4; i < 72 + 4 ;
								     i++) {                                               //FileName  , File Length   ,  CRC
									FilePackage.TP_FILE_PARA[i] = RecvBuff_Tcp[16 + i - 4];
								}
								need_receive_file_length = File_Length;


								//文件内容
								for (i = 0; i < i16Status - 72 - 16 ;
								     i++, receive_Buf_index++, received_file_length++) {
									FilePackage.Data[receive_Buf_index] = RecvBuff_Tcp[88 + i];
									//crcValue = GetCrc32_One(FilePackage.Data[i],crcValue,pos);
									//pos++;
								}
								if ((receive_Buf_index == 4096)
								    || (received_file_length == need_receive_file_length)) {
									recive_file_is_finished = true; send_file_is_finished = false;
									for (i = 0; i < 1000 ; i++) { //等待发送结束
										if (send_file_is_finished == true) {
											break;
										}
										vTaskDelay(10);
									}
									if (i == 1000) {
										receiveState = 1;
										receive_Buf_index = 0;
										send_pack_count = 0;
										UART_PRINT("\r\nwrite file time out");
									}
								}
								recive_file_is_finished = false;
							} else {
								while (recive_file_is_finished) {
									vTaskDelay(100);
								}

								FileSendLength_Inc += recv_Length;

								if (send_pack_count == 0) { //第一包
									for (i = 0; i < i16Status ; i++, receive_Buf_index++, received_file_length++) {
										FilePackage.Data[receive_Buf_index] = RecvBuff_Tcp[i];
									}
									if ((receive_Buf_index == 4096)
									    || (received_file_length == need_receive_file_length)) {
										recive_file_is_finished = true; send_file_is_finished = false;
										for (i = 0; i < 1000 ; i++) { //等待发送结束
											if (send_file_is_finished == true) {
												break;
											}
											vTaskDelay(10);
										}
										if (i == 1000) {
											receiveState = 1;
											receive_Buf_index = 0;
											send_pack_count = 0;
											UART_PRINT("\r\nwrite file time out");
										}
										receive_Buf_index = 0;
									}
									recive_file_is_finished = false;
								} else if (receive_Buf_index == 0) { //后续包，头

									//文件内容
									for (int i = 0; i < i16Status ;
									     i++, receive_Buf_index++, received_file_length++) {
										FileFollowPackage.Data[receive_Buf_index] = RecvBuff_Tcp[i];
									}

									if ((receive_Buf_index == 4096)
									    || (received_file_length == need_receive_file_length)) { //接收结束
										recive_file_is_finished = true; send_file_is_finished = false;

										for (i = 0; i < 1000 ; i++) { //等待发送结束
											if (send_file_is_finished == true) {
												break;
											}
											vTaskDelay(10);
										}
										if (i == 1000) {
											receiveState = 1;
											receive_Buf_index = 0;
											send_pack_count = 0;
											UART_PRINT("\r\nwrite file time out");
										}
									}
									recive_file_is_finished = false;
								} else { //后续包尾
									//文件内容
									for (int i = 0; i < i16Status ;
									     i++, receive_Buf_index++, received_file_length++) {
										FileFollowPackage.Data[receive_Buf_index] = RecvBuff_Tcp[i];
									}

									if ((receive_Buf_index == 4096)
									    || (received_file_length == need_receive_file_length)) { //接收结束
										recive_file_is_finished = true; send_file_is_finished = false;

										for (i = 0; i < 1000 ; i++) { //等待发送结束
											if (send_file_is_finished == true) {
												break;
											}
											vTaskDelay(10);
										}
										if (i == 1000) {
											receiveState = 1;
											receive_Buf_index = 0;
											send_pack_count = 0;
											UART_PRINT("\r\nwrite file time out");
										}
									}
									recive_file_is_finished = false;
								}
							}

							if (receiveNum == 1) {
								tcpFileWrite_Resp.head[0] = tcpFileWrite_Resp.head[1] =
								                                tcpFileWrite_Resp.head[2] = tcpFileWrite_Resp.head[3] = 0xaa;
								tcpFileWrite_Resp.head[7] = 0x02;
								tcpFileWrite_Resp.head[10] = 1;
								tcpFileWrite_Resp.head[11] = 0x20; //服务类型，消息类型
								crc32Value = GetCrc32(tcpFileWrite_Resp.TP_FILE_PARA, 72);
								tcpFileWrite_Resp.head[12] = crc32Value & 0xff;
								tcpFileWrite_Resp.head[13] = (crc32Value >> 8) & 0xff;
								tcpFileWrite_Resp.head[14] = (crc32Value >> 16) & 0xff;
								tcpFileWrite_Resp.head[15] = (crc32Value >> 24) & 0xff; //CRC校验
							}
							//UART_PRINT(",%d",receiveNum);
							if ((File_Length == FileSendLength_Inc)) { //接收完毕
								receiveState = 1;//readIndex = writeIndex;
								Message("\r\n Write File success!");
								UART_PRINT("\r\n   CRC 值 = %x", crcValue);
								//回复
								i16Status = sl_Send(ClientSd_TCP, tcpFileWrite_Resp.head, 16 + 72, 0);
								if (i16Status > 0) {
									Message("\r\n ACK 2 APP success");
								} else {
									Message("\r\n ACK 2 APP failed!");
								}
							}
							break;

						default :
							receiveState = 1;
							break;

						}
					}

					else if (serviceType == SERVICE_SETTING) {  //参数配置服务
						unsigned int crc32Value;
						Message("\r\n 参数配置服务");
						switch (messageType) {
						case MSG_PARA_READ_REQ :                //参数读取请求
							Message(",请求读取参数!");
							MCU_PARA_read.head[0] = MCU_PARA_read.head[1] = MCU_PARA_read.head[2] =
							                            MCU_PARA_read.head[3] = 0xcc;
							MCU_PARA_read.head[8] = (16 + 16 + 64 + 8) % 256;
							MCU_PARA_read.head[9] = (16 + 16 + 64 + 8) / 256; //length
							MCU_PARA_read.head[0x0a] = 3;
							MCU_PARA_read.head[0x0b] = 1;         //服务类型，消息类型
							MCU_PARA_read.head[12] = MCU_PARA_read.head[13] = MCU_PARA_read.head[14] =
							                             MCU_PARA_read.head[15] = 0;

							//TLV TP_DEV_VER
							MCU_PARA_read.TP_DEV_VER[0] = TP_DEV_VER_ID % 256;
							MCU_PARA_read.TP_DEV_VER[1] = TP_DEV_VER_ID / 256;
							MCU_PARA_read.TP_DEV_VER[2] = 16; MCU_PARA_read.TP_DEV_VER[3] = 0;

							//TLV TP_MH_VER
							MCU_PARA_read.TP_MH_VER[0] = TP_MH_VER_ID % 256;
							MCU_PARA_read.TP_MH_VER[1] = TP_MH_VER_ID / 256;
							MCU_PARA_read.TP_MH_VER[2] = 64; MCU_PARA_read.TP_MH_VER[3] = 0;

							//TLV TP_FPGA_VER
							MCU_PARA_read.TP_FPGA_VER[0] = TP_FPGA_VER_ID % 256;
							MCU_PARA_read.TP_FPGA_VER[1] = TP_FPGA_VER_ID / 256;
							MCU_PARA_read.TP_FPGA_VER[2] = 8; MCU_PARA_read.TP_FPGA_VER[3] = 0;

							crc32Value = GetCrc32(&MCU_PARA_read.head[8],
							                      8 + 16 + 64 + 8); //TLV 86 + 文件长度 Status - 72 -16
							MCU_PARA_read.head[4] = crc32Value & 0xff;
							MCU_PARA_read.head[5] = (crc32Value >> 8) & 0xff;
							MCU_PARA_read.head[6] = (crc32Value >> 16) & 0xff;
							MCU_PARA_read.head[7] = (crc32Value >> 24) & 0xff;

							temp = MCU_PARA_read.head;
							Message("\n\r读取参数指令报文:");
							//串口发送
							while (UART1_STATE == 1) {
								Message("\r\n 5");
								vTaskDelay(1);
							}
							UART1_STATE = 1;
							for (int i = 0; i < 16 + 16 + 64 + 8 ;
							     i++) { //头 16 + TLV 86 + 文件长度 Status - 72 -16
								UARTCharPut(UARTA1_BASE, temp[i]);
								//UART_PRINT("%2x,",temp[i]);
							}
							UART1_STATE = 0;

							//等待回复指令
							times = 0;
							while (APP_PARA_READ_resp.Realdy == 0) {
								if (times >= 30) {
									break;
								}
								times ++;
								vTaskDelay(30);
							}
							//发送回复指令给APP

							if (APP_PARA_READ_resp.Realdy == 1) {
								unsigned char *temp = APP_PARA_READ_resp.head;
								Message("\n\r 上发APP读取参数报文:");
								for (i = 0 ; i < 120 ; i++) {
									UART_PRINT("%02x ", temp[i]);
								}

								i16Status = sl_Send(ClientSd_TCP, APP_PARA_READ_resp.head, 120, 0);
								if (i16Status > 0) {
									Message("\r\n success , Read Para Resp!");
								} else {
									Message("\r\n failed , Read Para Resp!");
								}
								APP_PARA_READ_resp.Realdy = 0;
							}
							break;

						case MSG_PARA_SET_REQ :                 //参数写入请求
							Message(",请求写入参数!");
							SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] = 0xcc;
							SendBuff_Mcu[8] = RecvBuff_Tcp[4]; SendBuff_Mcu[9] = RecvBuff_Tcp[5];
							SendBuff_Mcu[10] = 3; SendBuff_Mcu[11] = 3;
							SendBuff_Mcu[12] = SendBuff_Mcu[13] = SendBuff_Mcu[14] = SendBuff_Mcu[15] = 0;
							for (i = 16 ; i < 16 + RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 ; i++) {
								SendBuff_Mcu[i] = RecvBuff_Tcp[i];
							}
							crc32Value = GetCrc32(&SendBuff_Mcu[8],
							                      RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 - 8);           //Crc
							SendBuff_Mcu[4] = crc32Value & 0xff;
							SendBuff_Mcu[5] = (crc32Value >> 8) & 0xff;
							SendBuff_Mcu[6] = (crc32Value >> 16) & 0xff;
							SendBuff_Mcu[7] = (crc32Value >> 24) & 0xff;

							//串口发送
							while (UART1_STATE == 1) {
								Message("\r\n 6");
								vTaskDelay(1);
							}
							UART1_STATE = 1;
							temp = SendBuff_Mcu;
							Message("\n\r写入参数指令报文:");
							for (int i = 0; i < SendBuff_Mcu[8] + SendBuff_Mcu[9] * 256 ;
							     i++) { //头 16 + TLV 86 + 文件长度 Status - 72 -16
								UARTCharPut(UARTA1_BASE, temp[i]);
								UART_PRINT("%2x,", temp[i]);
							}
							UART1_STATE = 0;

							Message("\n\r配置时间:");
							gettime(SendBuff_Mcu[20] + SendBuff_Mcu[21] * 256 + SendBuff_Mcu[22] * 256 * 256
							        + SendBuff_Mcu[23] * 256 * 256 * 256);
							dateTime.tm_year = tm.year;
							dateTime.tm_mon = tm.month;
							dateTime.tm_day = tm.day;
							dateTime.tm_hour = tm.hour;
							dateTime.tm_min = tm.minute;
							dateTime.tm_sec = tm.second;

							Get_Set_Time(set_TIME);
							//等待串口回复
							//回复APP
							SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] = 0xaa;
							SendBuff_Mcu[4] = 16; SendBuff_Mcu[5] = 0; SendBuff_Mcu[6] = 0;
							SendBuff_Mcu[7] = 2;
							SendBuff_Mcu[8] = RecvBuff_Tcp[8]; SendBuff_Mcu[9] = RecvBuff_Tcp[9];
							SendBuff_Mcu[10] = 3; SendBuff_Mcu[11] = 2;
							SendBuff_Mcu[12] = SendBuff_Mcu[13] = SendBuff_Mcu[14] = SendBuff_Mcu[15] =
							        0xbb;
							i16Status = sl_Send(ClientSd_TCP, SendBuff_Mcu, 16, 0);
							if (i16Status > 0) {
								Message("\r\n success , Write Para Resp!");
							} else {
								Message("\r\n failed , Write Para Resp!");
							}

							Config_timeUpdateFlag = 1;
							break;
						default :
							break;
						}
						receiveState = 1;
					}

					else if (serviceType == SERVICE_WARNING) {
						switch (messageType) {
						case MSG_WARNING_DAY_STAT_REQ:
							/*
							Message(",按天报警请求!");
							SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] = 0xcc;
							SendBuff_Mcu[8] = RecvBuff_Tcp[4];SendBuff_Mcu[9]=RecvBuff_Tcp[5];
							SendBuff_Mcu[10] = 2;SendBuff_Mcu[11] = 2;
							SendBuff_Mcu[12] = SendBuff_Mcu[13] = SendBuff_Mcu[14] = SendBuff_Mcu[15] = 0;
							for(i = 16 ; i< 16 + RecvBuff_Tcp[4] + RecvBuff_Tcp[5]*256 ; i++)
							 SendBuff_Mcu[i] = RecvBuff_Tcp[i];
							crc32Value = GetCrc32(&SendBuff_Mcu[8] , RecvBuff_Tcp[4]+RecvBuff_Tcp[5]*256 - 8);           //Crc
							SendBuff_Mcu[4] = crc32Value&0xff;SendBuff_Mcu[5] = (crc32Value>>8)&0xff;
							SendBuff_Mcu[6] = (crc32Value>>16)&0xff;SendBuff_Mcu[7] = (crc32Value>>24)&0xff;

							temp = NULL;
							//串口发送
							while(UART1_STATE == 1)
							{
							  Message("\r\n 7");
							  vTaskDelay(1);
							}
							UART1_STATE = 1;
							Message("\n\r写入参数指令报文:");
							for(i =0; i< SendBuff_Mcu[8] + SendBuff_Mcu[9]*256 ;i++)        //头 16 + TLV 86 + 文件长度 Status - 72 -16
							{
							  UARTCharPut(UARTA1_BASE , temp[i]);
							}
							UART1_STATE = 0;

							times = 0;
							while(WARNING_DAY_STAT_resp.WARNING_DAY_STAT_RESP_RDY == 0) //没有数据要上传，等待
							{
							  times ++;
							  if(times == 30)
							    break;
							  vTaskDelay(20);
							}
							if(WARNING_DAY_STAT_resp.WARNING_DAY_STAT_RESP_RDY == 1)
							{
							  Status = sl_Send(ClientSd_TCP , WARNING_DAY_STAT_resp.head, 16+784, 0);
							  WARNING_DAY_STAT_resp.WARNING_DAY_STAT_RESP_RDY = 0;
							}
							*/


							unsigned char *temp = RecvBuff_Tcp;
							Message("\n\r请求报警统计报文:");
							for (int i = 0; i < RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 ;
							     i++) { //头 16 + TLV 86 + 文件长度 Status - 72 -16
								UART_PRINT("%02x ", temp[i]);
							}


							static unsigned int Date_Index;
							//unsigned char *P_Date_Index = &RecvBuff_Tcp[20];
							//Date_Index = *P_Date_Index;


							//将BCD码转成十进制数据
							Date_Index = (RecvBuff_Tcp[20] & 0x0f) + ((RecvBuff_Tcp[20] & 0xf0) >> 4) * 10
							             + ((RecvBuff_Tcp[21] & 0x0f)) * 100 + ((RecvBuff_Tcp[21] & 0xf0) >> 4) * 1000
							             + ((RecvBuff_Tcp[22] & 0x0f)) * 10000 + ((RecvBuff_Tcp[22] & 0xf0) >> 4) *
							             100000 + ((RecvBuff_Tcp[23] & 0x0f)) * 1000000 + ((RecvBuff_Tcp[23] & 0xf0) >>
							                     4) * 10000000;

							if (Date_Index <= 1000) {
								if (Date_Index == 0) {
									//打包
									WARNING_DAY_STAT_resp.head[0] = WARNING_DAY_STAT_resp.head[1] =
									                                    WARNING_DAY_STAT_resp.head[2] = WARNING_DAY_STAT_resp.head[3] = 0xaa;
									WARNING_DAY_STAT_resp.head[4] = (16 + 784) % 256;
									WARNING_DAY_STAT_resp.head[5] = (16 + 784) / 256;
									WARNING_DAY_STAT_resp.head[6] = 0;
									WARNING_DAY_STAT_resp.head[7] = 2;
									WARNING_DAY_STAT_resp.head[8] = 0; WARNING_DAY_STAT_resp.head[9] = 0;
									WARNING_DAY_STAT_resp.head[10] = 2; WARNING_DAY_STAT_resp.head[11] = 2;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[0] = TP_WARN_DAY_STAT_ID;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[1] = 0;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[2] = 784 % 256;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[3] = 784 / 256;

									month = tm.month;
									year = tm.year;
									day = tm.day;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[4] = ((((day - Date_Index) / 10) << 4) &
									        0xf0) + (((day - Date_Index) % 10) & 0x0f);
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[5] = (((month / 10) << 4) & 0xf0) + ((
									            month % 10) & 0x0f);
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[6] = ((((year % 1000) / 10) << 4) & 0xf0)
									        + ((year % 10) & 0x0f);
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[7] = 0;

									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[8] = 10;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[9] = 10;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[10] = 2;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[11] = 0;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[12] = Date_Index % 256;
									WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[13] = Date_Index / 256;

									for (i = 16; i < 784 - 16 ; i++) {
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[i] = pStat24Hour[i - 16];
									}

									crcValue = GetCrc32(WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT, 784);
									WARNING_DAY_STAT_resp.head[12] = crcValue & 0xff;
									WARNING_DAY_STAT_resp.head[13] = (crcValue >> 8) & 0xff;
									WARNING_DAY_STAT_resp.head[14] = (crcValue >> 16) & 0xff;
									WARNING_DAY_STAT_resp.head[15] = (crcValue >> 24) & 0xff;

									//UART1_STATE = 1;
									temp = WARNING_DAY_STAT_resp.head;
									Message("\n\r报警统计报文:");
									for (int i = 0;
									     i < WARNING_DAY_STAT_resp.head[4] + WARNING_DAY_STAT_resp.head[5] * 256 ;
									     i++) {      //头 16 + TLV 86 + 文件长度 Status - 72 -16
										UART_PRINT("%02x ", temp[i]);
									}
									//UART1_STATE = 0;
								} else {
									if (Date_Index < Statistics_FileList_Num) {
										//打包
										WARNING_DAY_STAT_resp.head[0] = WARNING_DAY_STAT_resp.head[1] =
										                                    WARNING_DAY_STAT_resp.head[2] = WARNING_DAY_STAT_resp.head[3] = 0xaa;
										WARNING_DAY_STAT_resp.head[4] = (16 + 784) % 256;
										WARNING_DAY_STAT_resp.head[5] = (16 + 784) / 256;
										WARNING_DAY_STAT_resp.head[6] = 0;
										WARNING_DAY_STAT_resp.head[7] = 2;
										WARNING_DAY_STAT_resp.head[8] = 0; WARNING_DAY_STAT_resp.head[9] = 0;
										WARNING_DAY_STAT_resp.head[10] = 2; WARNING_DAY_STAT_resp.head[11] = 2;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[0] = TP_WARN_DAY_STAT_ID;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[1] = 0;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[2] = 784 % 256;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[3] = 784 / 256;

										unsigned int DateTime_temp1 = 0; unsigned int DateTime_temp2 = 0;
										DateTime_temp1 = Statistics_FileList[Date_Index][4];
										DateTime_temp2 = Statistics_FileList[Date_Index][5];
										DateTime_temp1 = ((DateTime_temp1 - '0') << 4) & 0xf0;
										DateTime_temp2 = (DateTime_temp2 - '0') & 0x0f;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[4] = (unsigned char)DateTime_temp1 +
										        (unsigned char)DateTime_temp2;//日

										DateTime_temp1 = Statistics_FileList[Date_Index][2];
										DateTime_temp2 = Statistics_FileList[Date_Index][3];
										DateTime_temp1 = ((DateTime_temp1 - '0') << 4) & 0xf0;
										DateTime_temp2 = (DateTime_temp2 - '0') & 0x0f;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[5] = (unsigned char)DateTime_temp1 +
										        (unsigned char)DateTime_temp2;;//月

										DateTime_temp1 = Statistics_FileList[Date_Index][0];
										DateTime_temp2 = Statistics_FileList[Date_Index][1];
										DateTime_temp1 = ((DateTime_temp1 - '0') << 4) & 0xf0;
										DateTime_temp2 = (DateTime_temp2 - '0') & 0x0f;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[6] = (unsigned char)DateTime_temp1 +
										        (unsigned char)DateTime_temp2;;//年

										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[7] = 0;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[8] = 10;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[9] = 10;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[10] = 2;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[11] = 0;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[12] = Date_Index % 256;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[13] = Date_Index / 256;


										//打开文件
										unsigned char Sat_buf[24 * 32]; unsigned long Token; unsigned char FileName[11];
										long lRetVal = -1;
										for (int j = 0; j < 10; j++) {
											FileName[j] = Statistics_FileList[Date_Index][j];
										}
										FileName[10] = '\0';

										long FileHandle = sl_FsOpen(FileName, SL_FS_READ, &Token);
										if (FileHandle >= 0) {
											lRetVal = sl_FsRead(FileHandle, 0, Sat_buf, sizeof(Sat_buf));
											if ((lRetVal >= 0) && (lRetVal == 24 * 32)) {
												for (int j = 16; j < 784 - 16 ; j++) {
													WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[j] = Sat_buf[j - 16];    //打包体
												}
											}
											sl_FsClose(FileHandle, 0, 0, 0);
										}

										crcValue = GetCrc32(WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT, 784);
										WARNING_DAY_STAT_resp.head[12] = crcValue & 0xff;
										WARNING_DAY_STAT_resp.head[13] = (crcValue >> 8) & 0xff;
										WARNING_DAY_STAT_resp.head[14] = (crcValue >> 16) & 0xff;
										WARNING_DAY_STAT_resp.head[15] = (crcValue >> 24) & 0xff;

										temp = WARNING_DAY_STAT_resp.head;
										Message("\n\r报警统计报文:");
										for (int i = 0;
										     i < WARNING_DAY_STAT_resp.head[4] + WARNING_DAY_STAT_resp.head[5] * 256 ;
										     i++) {      //头 16 + TLV 86 + 文件长度 Status - 72 -16
											UART_PRINT("%02x ", temp[i]);
										}
									} else {
										//打包
										WARNING_DAY_STAT_resp.head[0] = WARNING_DAY_STAT_resp.head[1] =
										                                    WARNING_DAY_STAT_resp.head[2] = WARNING_DAY_STAT_resp.head[3] = 0xaa;
										WARNING_DAY_STAT_resp.head[4] = (16 + 784) % 256;
										WARNING_DAY_STAT_resp.head[5] = (16 + 784) / 256;
										WARNING_DAY_STAT_resp.head[6] = 0;
										WARNING_DAY_STAT_resp.head[7] = 2;
										WARNING_DAY_STAT_resp.head[8] = 0; WARNING_DAY_STAT_resp.head[9] = 0;
										WARNING_DAY_STAT_resp.head[10] = 2; WARNING_DAY_STAT_resp.head[11] = 2;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[0] = TP_WARN_DAY_STAT_ID;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[1] = 0;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[2] = 784 % 256;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[3] = 784 / 256;

										month = tm.month;
										year = tm.year;
										day = tm.day;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[4] = ((((day - Date_Index) / 10) << 4) &
										        0xf0) + (((day - Date_Index) % 10) & 0x0f);
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[5] = (((month / 10) << 4) & 0xf0) + ((
										            month % 10) & 0x0f);
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[6] = ((((year % 1000) / 10) << 4) & 0xf0)
										        + ((year % 10) & 0x0f);
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[7] = 0;

										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[8] = 10;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[9] = 10;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[10] = 0;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[11] = 0;                   //runtime  0
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[12] = Date_Index % 256;
										WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[13] = Date_Index / 256;

										for (i = 16; i < 784 - 16 ; i++) {
											WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[i] = pStat24Hour[i - 16];
										}

										crcValue = GetCrc32(WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT, 784);
										WARNING_DAY_STAT_resp.head[12] = crcValue & 0xff;
										WARNING_DAY_STAT_resp.head[13] = (crcValue >> 8) & 0xff;
										WARNING_DAY_STAT_resp.head[14] = (crcValue >> 16) & 0xff;
										WARNING_DAY_STAT_resp.head[15] = (crcValue >> 24) & 0xff;

										//UART1_STATE = 1;
										temp = WARNING_DAY_STAT_resp.head;
										Message("\n\r报警统计报文:");
										for (int i = 0;
										     i < WARNING_DAY_STAT_resp.head[4] + WARNING_DAY_STAT_resp.head[5] * 256 ;
										     i++) {      //头 16 + TLV 86 + 文件长度 Status - 72 -16
											UART_PRINT("%02x ", temp[i]);
										}
									}
								}

								i16Status = sl_Send(ClientSd_TCP, WARNING_DAY_STAT_resp.head, 16 + 784, 0);
							} else if (Date_Index >= 160101) {
								/*
								  //打包
								    WARNING_DAY_STAT_resp.head[0]=WARNING_DAY_STAT_resp.head[1]=WARNING_DAY_STAT_resp.head[2]=WARNING_DAY_STAT_resp.head[3]=0xaa;
								    WARNING_DAY_STAT_resp.head[4] = (16+784)%256;WARNING_DAY_STAT_resp.head[5] = (16+784)/256;WARNING_DAY_STAT_resp.head[6] = 0;
								    WARNING_DAY_STAT_resp.head[7] = 2;
								    WARNING_DAY_STAT_resp.head[8]=0;WARNING_DAY_STAT_resp.head[9]=0;
								    WARNING_DAY_STAT_resp.head[10]=2;WARNING_DAY_STAT_resp.head[11]=2;
								    WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[0]=TP_WARN_DAY_STAT_ID;WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[1]=0;
								    WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[2]=784%256;WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[3]=784/256;


								    WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[4] = ((((Statistics_FileList[Date_Index][4] - '0')/10)<<4) & 0xf0) + (((Statistics_FileList[Date_Index][5] - '0')%10) & 0x0f);//日
								    WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[5] = ((((Statistics_FileList[Date_Index][2] - '0')/10)<<4) & 0xf0) + (((Statistics_FileList[Date_Index][3] - '0')%10) & 0x0f);//月
								    WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[6] = ((((Statistics_FileList[Date_Index][0] - '0')/10)<<4) & 0xf0) + (((Statistics_FileList[Date_Index][1] - '0')%10) & 0x0f);//年
								    WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[7] = 0;

								    WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[8] = 10;WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[9] = 10;
								    WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[10]=2;WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[11]=0;
								    WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[12]=Date_Index%256;WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[13]=Date_Index/256;


								    //打开文件
								    unsigned char Sat_buf[24*32];unsigned long Token;unsigned char FileName[11];long lRetVal = -1;
								    for(int j = 0; j< 6; j++)
								      FileName[j] = ;
								    FileName[10] = '\0';

								    long FileHandle = sl_FsOpen(FileName,SL_FS_READ,&Token);
								    if(FileHandle >= 0)
								    {
								      lRetVal = sl_FsRead(FileHandle,0,Sat_buf, sizeof(Sat_buf));
								      if((lRetVal >= 0)&&(lRetVal == 24*32))
								      {
								        for(int j = 16; j < 784-16 ; j++)
								          WARNING_DAY_STAT_resp.TP_WARN_DAY_STAT[j] = Sat_buf[j - 16];    //打包体
								      }
								      sl_FsClose(FileHandle, 0, 0, 0);
								    }

								    Message("\n\r报警统计报文:");
								    for(int i =0; i< WARNING_DAY_STAT_resp.head[4] + WARNING_DAY_STAT_resp.head[5]*256 ;i++)        //头 16 + TLV 86 + 文件长度 Status - 72 -16
								    {
								      UART_PRINT("%02x ",temp[i]);
								    }
								    */
							}
							receiveState = 1;
							break;
						}
						receiveState = 1;
					} else if (serviceType == SERVICE_DVR) {    //行车记录仪服务
						switch (messageType) {
						case MSG_DVR_FILE_LIST_REQ :            //获取文件列表，如视频文件列表，照片列表

							//for(i=0;i<RecvBuff_Tcp[4]+RecvBuff_Tcp[5]*256;i++)
							//UART_PRINT("%02x,",RecvBuff_Tcp[i]);

							//Message(",获取文件列表!");
							//打包头部
							SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] =
							                                        0xcc; //起始标志
							SendBuff_Mcu[8] = RecvBuff_Tcp[4]; SendBuff_Mcu[9] = RecvBuff_Tcp[5]; //消息长度
							SendBuff_Mcu[10] = RecvBuff_Tcp[10];
							SendBuff_Mcu[11] = RecvBuff_Tcp[11]; //服务类型消息类型
							SendBuff_Mcu[12] = SendBuff_Mcu[13] = SendBuff_Mcu[14] = SendBuff_Mcu[15] =
							        0x00;//保留

							//打包体
							length = SendBuff_Mcu[8] + SendBuff_Mcu[9] * 256;
							for (i = 16 ; i < length ; i++) {
								SendBuff_Mcu[i] =  RecvBuff_Tcp[i];
							}
							if (length >= 16) {
								crcValue = GetCrc32(&SendBuff_Mcu[8],
								                    8 + RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 - 16);
								SendBuff_Mcu[4] = crcValue & 0xff;
								SendBuff_Mcu[5] = (crcValue >> 8) & 0xff;
								SendBuff_Mcu[6] = (crcValue >> 16) & 0xff;
								SendBuff_Mcu[7] = (crcValue >> 24) & 0xff;
							} else {

							}


							//vTaskDelay(1500);
							//发送数据给MCU
							while (UART1_STATE == 1) {
								Message("\r\n 8");
								vTaskDelay(100);
							}

							//char *paraes_UART_TASK=NULL;
							//xTaskCreate(UART_TASK,"UART_TASK",500,(void *)&paraes_UART_TASK,4,&xHandle_UART_TASK);

							UART1_STATE = 1;
							//UART1_STATE = 0;
							for (int i = 0; i < SendBuff_Mcu[8] + SendBuff_Mcu[9] * 256 ; i++) {
								UARTCharPut(UARTA1_BASE, SendBuff_Mcu[i]);
								UART_PRINT("%02x ", SendBuff_Mcu[i]);
							}
							UART1_STATE = 0;
							//vTaskDelay(500);
							//vTaskDelay(10);

							//等待MCU回应
							times = 0;
							while (DVR_FILE_LIST_resp.DVR_FILE_LIST_RES_RDY != 1) {
								Message("\r\n TCP 01!");
								times++;
								if (times >= 480) {
									break;
								}
								vTaskDelay(50);
							}
							Message("\r\n TCP 02!");
							DVR_FILE_LIST_resp.head[8] = RecvBuff_Tcp[8];
							DVR_FILE_LIST_resp.head[9] = RecvBuff_Tcp[9];

							length = DVR_FILE_LIST_resp.head[4] + DVR_FILE_LIST_resp.head[5] * 256;
							UART_PRINT("\n\rlength = %d,", length);
							if (DVR_FILE_LIST_resp.DVR_FILE_LIST_RES_RDY == 1) {
								i16Status = sl_Send(ClientSd_TCP, DVR_FILE_LIST_resp.head,
								                    DVR_FILE_LIST_resp.head[4] + DVR_FILE_LIST_resp.head[5] * 256, 0);
								if (i16Status > 0) {
									Message("\r\n success , return file list!");
								} else {
									Message("\r\n failed , return file list!");
								}
								DVR_FILE_LIST_resp.DVR_FILE_LIST_RES_RDY = 0;

								Message("\r\n DVR文件列表 转发给APP 成功!");
								temp = DVR_FILE_LIST_resp.head;
								for (i = 0; i < DVR_FILE_LIST_resp.head[4] + DVR_FILE_LIST_resp.head[5] * 256;
								     i++) {
									UART_PRINT("%02x ", temp[i]);
								}
							}
							DVR_FILE_LIST_resp.DVR_FILE_LIST_RES_RDY = 0;

							receiveState = 1;
							break;

						case MSG_DVR_PLAY_FILE_REQ:
							Message(",播放文件!");
							//打包头部
							SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] =
							                                        0xcc; //起始标志
							SendBuff_Mcu[8] = RecvBuff_Tcp[4]; SendBuff_Mcu[9] = RecvBuff_Tcp[5]; //消息长度
							SendBuff_Mcu[10] = RecvBuff_Tcp[10];
							SendBuff_Mcu[11] = RecvBuff_Tcp[11]; //服务类型消息类型
							SendBuff_Mcu[12] = SendBuff_Mcu[13] = SendBuff_Mcu[14] = SendBuff_Mcu[15] =
							        0x00;//保留

							//打包体
							length = SendBuff_Mcu[8] + SendBuff_Mcu[9] * 256;
							for (i = 16 ; i < length ; i++) {
								SendBuff_Mcu[i] =  RecvBuff_Tcp[i];
							}
							if (length > 16) {
								crcValue = GetCrc32(&SendBuff_Mcu[8],
								                    8 + RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 - 16);
								SendBuff_Mcu[4] = crcValue & 0xff;
								SendBuff_Mcu[5] = (crcValue >> 8) & 0xff;
								SendBuff_Mcu[6] = (crcValue >> 16) & 0xff;
								SendBuff_Mcu[7] = (crcValue >> 24) & 0xff;
							} else if (length == 16) {
								SendBuff_Mcu[4] = 0xbb;
								SendBuff_Mcu[5] = 0xbb;
								SendBuff_Mcu[6] = 0xbb;
								SendBuff_Mcu[7] = 0xbb;
							} else {

							}

							//发送数据给MCU
							while (UART1_STATE == 1) {
								Message("\r\n 9");
								vTaskDelay(1);
							}
							UART1_STATE = 1;
							for (i = 0; i < SendBuff_Mcu[8] + SendBuff_Mcu[9] * 256 ; i++) {
								UARTCharPut(UARTA1_BASE, SendBuff_Mcu[i]);
							}
							UART1_STATE = 0;
							receiveState = 1;
							break;
						}
					} else if (serviceType == SERVICE_CMD) {    //命令服务
						switch (messageType) {
						case MSG_CMD_TEST_REQ :
							SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] = 0xcc;
							SendBuff_Mcu[8] = RecvBuff_Tcp[4]; SendBuff_Mcu[9] = RecvBuff_Tcp[5];
							SendBuff_Mcu[10] = SERVICE_CMD; SendBuff_Mcu[11] = MSG_CMD_TEST_REQ;
							SendBuff_Mcu[12] = SendBuff_Mcu[13] = SendBuff_Mcu[14] = SendBuff_Mcu[15] = 0;
							for (i = 16 ; i < 16 + RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 ; i++) {
								SendBuff_Mcu[i] = RecvBuff_Tcp[i];
							}
							crc32Value = GetCrc32(&SendBuff_Mcu[8],
							                      RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 - 8);           //Crc
							SendBuff_Mcu[4] = crc32Value & 0xff; SendBuff_Mcu[5] = (crc32Value >> 8) & 0xff;
							SendBuff_Mcu[6] = (crc32Value >> 16) & 0xff;
							SendBuff_Mcu[7] = (crc32Value >> 24) & 0xff;

							//串口发送
							//vTaskDelay(3000);
							Message("\n\r 命令，请求模拟数据:");
							UART1_STATE = 1;
							//UART1_STATE = 0;
							for (int i = 0; i < SendBuff_Mcu[8] + SendBuff_Mcu[9] * 256 ; i++) {
								UARTCharPut(UARTA1_BASE, SendBuff_Mcu[i]);
								UART_PRINT("%02x ", SendBuff_Mcu[i]);
							}
							UART1_STATE = 0;

							//串口接收
							times = 0;
							while (TEST_CMD_resp.Ready != 1) {
								times++;
								if (times >= 180) {
									break;
								}
								vTaskDelay(100);
							}
							//回复wifi
							if (TEST_CMD_resp.Ready == 1) {
								i16Status = sl_Send(ClientSd_TCP, TEST_CMD_resp.Head,
								                    TEST_CMD_resp.Head[4] + TEST_CMD_resp.Head[5] * 256, 0);
								if (i16Status > 0) {
									Message("\r\n success , resp test cmd!");
								} else {
									Message("\r\n failed , resp test cmd!");
								}
								TEST_CMD_resp.Ready = 0;

								Message("\r\n 请求测试指令回复,请求模拟,转发给APP成功!");
								//temp = DVR_FILE_LIST_resp.head;
								//for(i=0;i<DVR_FILE_LIST_resp.head[4] +DVR_FILE_LIST_resp.head[5]*256;i++)
								//UART_PRINT("%02x ",temp[i]);
							}
							TEST_CMD_resp.Ready = 0;
							receiveState = 1;
							break;

						case MSG_CMD_RESET_ME_REQ:
							SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] = 0xcc;
							SendBuff_Mcu[8] = RecvBuff_Tcp[4]; SendBuff_Mcu[9] = RecvBuff_Tcp[5];
							SendBuff_Mcu[10] = SERVICE_CMD; SendBuff_Mcu[11] = MSG_CMD_RESET_ME_REQ;
							SendBuff_Mcu[12] = SendBuff_Mcu[13] = SendBuff_Mcu[14] = SendBuff_Mcu[15] = 0;
							for (i = 16 ; i < 16 + RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 ; i++) {
								SendBuff_Mcu[i] = RecvBuff_Tcp[i];
							}
							crc32Value = GetCrc32(&SendBuff_Mcu[8],
							                      RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 - 8);           //Crc
							SendBuff_Mcu[4] = crc32Value & 0xff; SendBuff_Mcu[5] = (crc32Value >> 8) & 0xff;
							SendBuff_Mcu[6] = (crc32Value >> 16) & 0xff;
							SendBuff_Mcu[7] = (crc32Value >> 24) & 0xff;

							Message("\n\r 命令，请求重启mobile eye:");

							UART1_STATE = 1;
							//UART1_STATE = 0;
							for (int i = 0; i < SendBuff_Mcu[8] + SendBuff_Mcu[9] * 256 ; i++) {
								UARTCharPut(UARTA1_BASE, SendBuff_Mcu[i]);
								UART_PRINT("%02x ", SendBuff_Mcu[i]);
							}
							UART1_STATE = 0;

							//串口接收
							times = 0;
							while (RESET_ME_resp.RESET_ME_RESP_rdy != 1) {
								times++;
								if (times >= 180) {
									break;
								}
								vTaskDelay(100);
							}

							if (RESET_ME_resp.RESET_ME_RESP_rdy == 1) {
								i16Status = sl_Send(ClientSd_TCP, RESET_ME_resp.Head,
								                    RESET_ME_resp.Head[4] + RESET_ME_resp.Head[5] * 256, 0);
								if (i16Status > 0) {
									Message("\r\n success , resp reset me cmd!");
								} else {
									Message("\r\n failed , resp reset me cmd!");
								}
								RESET_ME_resp.RESET_ME_RESP_rdy = 0;

								Message("\r\n 请求测试指令回复,请求模拟,转发给APP成功!");
							}
							RESET_ME_resp.RESET_ME_RESP_rdy = 0;
							receiveState = 1;
							break;
							/*
							case MSG_CMD_CAN_DATA_REQ:
							  SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] = 0xcc;
							  SendBuff_Mcu[8] = RecvBuff_Tcp[4];SendBuff_Mcu[9]=RecvBuff_Tcp[5];
							  SendBuff_Mcu[10] = SERVICE_CMD;SendBuff_Mcu[11] = MSG_CMD_CAN_DATA_REQ;
							  SendBuff_Mcu[12] = SendBuff_Mcu[13] = SendBuff_Mcu[14] = SendBuff_Mcu[15] = 0;

							  UART1_STATE = 1;
							  //UART1_STATE = 0;
							  for(int i =0; i< SendBuff_Mcu[8] + SendBuff_Mcu[9]*256 ;i++)
							  {
							    UARTCharPut(UARTA1_BASE , SendBuff_Mcu[i]);
							    UART_PRINT("%02x ",SendBuff_Mcu[i]);
							  }
							  UART1_STATE = 0;

							  //串口接收
							  times = 0;
							  while(can_data.Rdy  != 1)
							  {
							    times++;
							    if(times >= 180)
							      break;
							    vTaskDelay(100);
							  }

							  if(can_data.Rdy == 1)
							  {
							    Status = sl_Send(ClientSd_TCP , can_data.head, can_data.head[4] + can_data.head[5]*256, 0);
							    if(Status > 0)
							      Message("\r\n success , CAN DATA RESP!");
							    else
							      Message("\r\n failed , CAN DATA RESP!");
							    can_data.Rdy = 0;

							    Message("\r\n 请求测试指令回复,请求模拟,转发给APP成功!");
							  }
							  can_data.Rdy = 0;
							  receiveState = 1;

							 break;
							 */
						}
					} else if (serviceType == SERVICE_DEBUG) {
						switch (messageType) {
						case DEBUG_MCU_COMMAND :
							Message("\n\r DEBUG_MCU_COMMAND:");

							SendBuff_Mcu[0] = SendBuff_Mcu[1] = SendBuff_Mcu[2] = SendBuff_Mcu[3] = 0xcc;
							SendBuff_Mcu[8] = RecvBuff_Tcp[4]; SendBuff_Mcu[9] = RecvBuff_Tcp[5];
							SendBuff_Mcu[10] = SERVICE_DEBUG; SendBuff_Mcu[11] = DEBUG_MCU_COMMAND;
							SendBuff_Mcu[12] = SendBuff_Mcu[13] = SendBuff_Mcu[14] = SendBuff_Mcu[15] = 0;
							for (i = 16 ; i < 16 + RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 ; i++) {
								SendBuff_Mcu[i] = RecvBuff_Tcp[i];
							}
							crc32Value = GetCrc32(&SendBuff_Mcu[8],
							                      RecvBuff_Tcp[4] + RecvBuff_Tcp[5] * 256 - 8);           //Crc
							SendBuff_Mcu[4] = crc32Value & 0xff; SendBuff_Mcu[5] = (crc32Value >> 8) & 0xff;
							SendBuff_Mcu[6] = (crc32Value >> 16) & 0xff;
							SendBuff_Mcu[7] = (crc32Value >> 24) & 0xff;

							Message("\n\r 调试服务，MCU调试服务:");

							UART1_STATE = 1;
							//UART1_STATE = 0;
							for (int i = 0; i < SendBuff_Mcu[8] + SendBuff_Mcu[9] * 256 ; i++) {
								UART_PRINT("%02x ", SendBuff_Mcu[i]);
								UARTCharPut(UARTA1_BASE, SendBuff_Mcu[i]);
							}
							UART1_STATE = 0;


							//串口接收
							if ((SendBuff_Mcu[20] == 0x0e) && (SendBuff_Mcu[21] == 0xe0)) {
								Message("\r\n 结束校准！");
							} else {
								times = 0;
								while (DEBUG_CMD_resp.ready != 1) {
									times++;
									if (times >= 200) {
										break;
									}
									vTaskDelay(100);
								}

								if (DEBUG_CMD_resp.ready == 1) {

									Message("\r\n 校准数据，回复报文！");
									unsigned char *temp = DEBUG_CMD_resp.Head;
									UART1_STATE = 1;
									//UART1_STATE = 0;
									for (int i = 0; i < temp[4] + temp[5] * 256 ; i++) {
										UART_PRINT("%02x ", temp[i]);
									}
									UART1_STATE = 0;

									i16Status = sl_Send(ClientSd_TCP, DEBUG_CMD_resp.Head,
									                    DEBUG_CMD_resp.Head[4] + DEBUG_CMD_resp.Head[5] * 256, 0);
									if (i16Status > 0) {
										Message("\r\n success , DEBUG_CMD!");
									} else {
										Message("\r\n failed , DEBUG_CMD!");
									}
									DEBUG_CMD_resp.ready = 0;

									Message("\r\n DEBUG_CMD成功!");
									DEBUG_CMD_resp.ready = 0;
								} else {
									i16Status = sl_Send(ClientSd_TCP, SendBuff_Mcu,
									                    SendBuff_Mcu[8] + SendBuff_Mcu[9] * 256, 0);
									Message("\r\n 等待校准数据超时!");
								}
							}
							receiveState = 1;
							break;
						}
					} else if (serviceType == SERVICE_WARNING) {
						switch (messageType) {
						case MSG_WARNING_DAY_STAT_REQ:
							//
							//返回统计数据
							break;
						}
					} else {
						receiveState = 1;
						//error
					}
				}
				vTaskDelay(10);
			}
			sl_Close(ClientSd_TCP);
			Message("\r\n tcp_task over!");
			vTaskDelay(20);
		}
		sl_Close(Sd);
	}
}

#define onlie_time_out    20   //20s  
_i16 AddrSize = 0;
void UDP_TASK()
{
	unsigned char serviceType = 0xff; unsigned char messageType = 0xff;
	unsigned int crcValue; int length; int i; int j;
	_i16 recv_Length;
	SlSockAddrIn_t Addr_UDP_TMP; unsigned char *tmpP1, *tmpP2;
	int time_now_second_from_1970;
	/*
	  while(1)
	  {
	    Message("\r\n udp_task begin!");
	    Status_UDP = sl_SendTo(Sd_UDP, recvBuffFromAPP, 16, 0, (SlSockAddr_t*)&Addr_UDP,sizeof(SlSockAddr_t));
	    if(Status_UDP < 0)
	      Message("\r\n sl_SendTo failed!");
	    vTaskDelay(2);
	  }*/
	while (1) {
		while (UDP_RUN_Flag == 0) {
			vTaskDelay(20);
		}

		Message("\r\n udp_task begin!");
		AddrSize = sizeof(SlSockAddrIn_t);
		//处理UDP 消息  ,  收集在线设备
		Status_UDP = sl_RecvFrom(Sd_UDP, recvBuffFromAPP, 4096, 0,
		                         (SlSockAddr_t *)&Addr_UDP_TMP, &AddrSize);

		Get_Set_Time(get_TIME);
		time_now_second_from_1970 = calc_sec1970(dateTime.tm_year, dateTime.tm_mon,
		                            dateTime.tm_day, dateTime.tm_hour, dateTime.tm_min, dateTime.tm_sec);

		//判断设备是否在线超时
		for (i = 0; i < device_Connect_num; i++) {
			if (heartbeat_Time_sencond_from_1970[i] == 0) {
				continue;
			}
			if (time_now_second_from_1970 - heartbeat_Time_sencond_from_1970[i] >
			    onlie_time_out) { //长时间没收到此设备的heat beat信号
				tmpP2 = (unsigned char *)&Addr_UDP[i];
				for (j = 0; j < sizeof(SlSockAddr_t); j++) {
					tmpP2[j] = 0;
				}
				heartbeat_Time_sencond_from_1970[i] = 0;
			}
		}
		tmpP1 = (unsigned char *)&Addr_UDP_TMP;
		for (i = 0; i < device_Connect_num; i++) {
			if (heartbeat_Time_sencond_from_1970[i] == 0) {
				continue;
			}
			tmpP2 = (unsigned char *)&Addr_UDP[i];
			for (j = 2; j < sizeof(SlSockAddr_t) - 2 - 8; j++) {
				if (tmpP1[j] != tmpP2[j]) {             //列表中的 这个地址 不是连接的地址
					break;
				}
			}
			if (j == sizeof(SlSockAddr_t) - 2 - 8) { //在地址列表中，找到了连接设备的地址
				break;
			}
		}
		if (i == device_Connect_num) { //在地址列表中未找到 连接设备的地址
			for (i = 0; i < device_Connect_num; i++)
				if (heartbeat_Time_sencond_from_1970[i] == 0) {
					heartbeat_Time_sencond_from_1970[i] = time_now_second_from_1970;
					break;
				}
			if (i == device_Connect_num) {
				Message("\r\nToo many devices");
			} else {
				tmpP1 = (unsigned char *)&Addr_UDP_TMP;  tmpP2 = (unsigned char *)&Addr_UDP[i];
				for (i = 0; i < sizeof(SlSockAddr_t); i++) {
					tmpP2[i] = tmpP1[i];
				}
			}
		}

		recv_Length = Status_UDP;
		if (0 > Status_UDP) {
			// error
			Message("\r\nsl_RecvFrom , UDP error!");
		} else if (0 < Status_UDP) {
			UDP_Recv_Flag = 1;

			serviceType = recvBuffFromAPP[0x0a];
			messageType = recvBuffFromAPP[0x0b];

			if (serviceType == NO_SERVICE) {            //无服务
				//for(i = 0;i < device_Connect_num; i++)
				sl_SendTo(Sd_UDP, recvBuffFromAPP, 16, 0, (SlSockAddr_t *)&Addr_UDP_TMP,
				          sizeof(SlSockAddr_t));
				//sl_SendTo(Sd_UDP, SendBuff2APP, SendBuff2APP[4] + SendBuff2APP[5]*256, 0, (SlSockAddr_t*)&Addr_UDP,sizeof(SlSockAddr_t));
				//Message("\r\n\r\n\r\n\r\n\r\nUDP NO_SERVICE!");

			} else if (serviceType == SERVICE_HEARTBEAT) { //心跳服务
				//Message("\r\n 心跳服务!");
				recvBuffFromAPP[7] = 2;                 //回应
				//Message("\r\n 222222222222222222222222222222!");
				Status_UDP = sl_SendTo(Sd_UDP, recvBuffFromAPP, 16, 0,
				                       (SlSockAddr_t *)&Addr_UDP_TMP, sizeof(SlSockAddr_t)); //回复心跳包
				//Message("\r\n 333333333333333333333333333333!");
			} else if (serviceType == SERVICE_CMD) {    //命令服务
				Message("\r\n 命令服务");
				switch (messageType) {
				case MSG_CMD_RESET_REQ :                //复位系统命令。设备在收到此命令后，回复一个回应消息，整个开始重新启动。
					Message(",复位系统!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_CMD_RESET_ME_REQ :             //复位Mobileye命令。
					Message(",复位mobile!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_CMD_RESET_DVR_REQ :            //复位DVR命令
					Message(",复位DVR");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_CMD_UPDATE_GATE_REQ :          //更新ADASGate命令。
					Message(",更新ADASGate!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				//case MSG_CMD_TEST_REQ :                 //测试命令。设备在收到此命令后，发送测试数据给显示屏和手机。
				//Message(",测试命令!");
				//ChangeWifiData2UartData();            //更换消息头
				//SendData2MCU();                       //发送数据给MCU
				//break;
				case MSG_CMD_SWITCH_SCREEN :            //切换视频源  0:查询当前视频源号 1:原车屏幕  2:Mobileye  3:DVR
					Message(",切换屏幕!");
					//打包头部
					SendBuff2MCU[0] = SendBuff2MCU[1] = SendBuff2MCU[2] = SendBuff2MCU[3] =
					                                        0xcc; //起始标志
					SendBuff2MCU[8] = 16 + 5; SendBuff2MCU[9] = 0; //消息长度
					SendBuff2MCU[10] = recvBuffFromAPP[10];
					SendBuff2MCU[11] = recvBuffFromAPP[11]; //服务类型消息类型
					SendBuff2MCU[12] = SendBuff2MCU[13] = SendBuff2MCU[14] = SendBuff2MCU[15] =
					        0x00;//保留
					//打包体
					length = SendBuff2MCU[8] + SendBuff2MCU[9] * 256;
					for (i = 16 ; i < length ; i++) {
						SendBuff2MCU[i] =  recvBuffFromAPP[i];
					}
					crcValue = GetCrc32(&SendBuff2MCU[8],
					                    8 + SendBuff2MCU[8] + SendBuff2MCU[9] * 256 - 16);
					SendBuff2MCU[4] = crcValue & 0xff;
					SendBuff2MCU[5] = (crcValue >> 8) & 0xff;
					SendBuff2MCU[6] = (crcValue >> 16) & 0xff;
					SendBuff2MCU[7] = (crcValue >> 24) & 0xff;

					SendData2MCU();

					break;
				case MSG_CMD_POWER_OFF_REQ :            //关闭系统，HI3518同时需要发送关机消息给DVR。关机时间为1分钟。用于HI3与MCU或DVR间通信。
					Message(",关闭系统!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_CMD_SAVE_FRAMES_REQ :          //截取3518的VI口输入的视频数据，保存为文件。参数为需要截取的帧数
					Message(",截取3518VI口输入的视频数据!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_CMD_STA_MODE_REQ :             //要求将3518的wifi切换成STA模式
					Message(",请求将3518的wifi切换成STA模式!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_CMD_CALI_REQ :                 //用于校准  0:查询校准状态   1:开始校准/正在校准   2:停止校准
					Message(",校准!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_CMD_CALI_RESULT_INFO :         //校准结果，由3518主动发送
					//Message(",校准结束!");
					//ChangeWifiData2UartData();            //更换消息头
					//SendData2MCU();                       //发送数据给MCU
					break;
				default :
					break;
				}
			} else if (serviceType == SERVICE_DVR) {    //行车记录仪服务
				Message("\r\n 行车记录仪服务");
				switch (messageType) {
				case MSG_DVR_KEY :                      //按键指令。由App发送，用于控制行车记录的按键操作。
					Message(",按键指令!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_DVR_PLAY_FILE_REQ :            //播放DVR文件
					Message(",播放DVR!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_DVR_RECORD :                   //进入录像界面
					Message(",进入录像界面!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_DVR_FORMAT :                   //格式化TF卡
					Message("格式化TF卡!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_DVR_STATUS :                   //行车记录仪当前状态
					Message(",行车记录仪当前状态!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				case MSG_DVR_DEL_FILE :                 //删除文件
					Message(",删除文件!");
					ChangeWifiData2UartData();            //更换消息头
					SendData2MCU();                       //发送数据给MCU
					break;
				default :
					break;
				}
			}
			/*
			else if(serviceType == SERVICE_DEBUG)       //调试服务
			{
			  switch (messageType)
			  {
			    Message("\r\n 调试服务");
			    case LOG_ADASGATE_SWITCH :              //ADASGATE日志开关
			      Message(",ADASGATE日志开关!");
			      if((recvBuffFromAPP[16 + 4] & 0x01) != 0)
			        switch_flag |= 0x02;
			      else
			        switch_flag &= ~0x02;
			      break;
			    case LOG_MCU_SWITCH :                   //MCU日志开关
			      Message(",MCU日志开关!");
			      if((recvBuffFromAPP[16 + 4] & 0x01) != 0)
			        switch_flag |= 0x01;
			      else
			        switch_flag &= ~0x01;
			      SendData2MCU();                       //发送数据给MCU
			      break;
			    case DEBUG_MCU_COMMAND :                //MCU调试用指令
			      Message(",MCU调试用指令!");
			      ChangeWifiData2UartData();            //更换消息头
			      SendData2MCU();                       //发送数据给MCU
			      break;
			    case DEBUG_FPGA_COMMAND :               //FPGA调试用指令
			      Message(",FPGA调试用指令!");
			      ChangeWifiData2UartData();            //更换消息头
			      SendData2MCU();                       //发送数据给MCU
			      break;
			    case DEBUG_DVR_COMMAND :                //DVR调试用指令
			      Message(",DVR调试用指令!");
			      ChangeWifiData2UartData();            //更换消息头
			      SendData2MCU();                       //发送数据给MCU
			      break;
			    default :
			      break;
			  }
			}
			*/
			else {
				//error
			}
		}
		//Message("\r\n udp_task over!");
		vTaskDelay(5);
	}
}


void WRITE_FILE_TASK()
{
	unsigned int crc32Value = 0;
	unsigned char *temp;
	int times;
	while (true) {
		//getDeviceTime();
		vTaskDelay(100);
		if (recive_file_is_finished) { // receive ok
			send_pack_count++;
			if (send_pack_count == 1) {
				//打包
				//头
				FilePackage.head[0] = FilePackage.head[1] = FilePackage.head[2] =
				                          FilePackage.head[3] = 0xcc;
				FilePackage.head[8] = (16 + 86 + receive_Buf_index) % 256;
				FilePackage.head[9] = (16 + 86 + receive_Buf_index) /
				                      256; //长度 = 头 16 + TLV 86 + 内容 4096 -16 -72
				FilePackage.head[10] = 1; //务类型 - 文件
				FilePackage.head[11] = 0x20;

				//TP_FILE_PACKET
				FilePackage.TP_FILE_PACKET[0] = TP_FILE_PACKET_ID % 256;
				FilePackage.TP_FILE_PACKET[1] = TP_FILE_PACKET_ID /
				                                256;                          //Type

				FilePackage.TP_FILE_PACKET[2] = 10 % 256;
				FilePackage.TP_FILE_PACKET[3] = 10 / 256;   //Tlv length

				FilePackage.TP_FILE_PACKET[4] = totalPackage % 256;
				FilePackage.TP_FILE_PACKET[5] = totalPackage / 256;        //Total Packets
				FileFollowPackage.TP_FILE_PACKET[6] = (send_pack_count) % 256;
				FileFollowPackage.TP_FILE_PACKET[7] = (send_pack_count) /
				                                      256;      //Cur Packet
				FilePackage.TP_FILE_PACKET[8] = (receive_Buf_index) % 256;
				FilePackage.TP_FILE_PACKET[9] = (receive_Buf_index) / 256;


				//数据包CRC校验
				crc32Value = GetCrc32(&FilePackage.head[8],
				                      8 + 86 + receive_Buf_index); //TLV 86 + 文件长度 Status - 72 -16
				FilePackage.head[4] = crc32Value & 0xff;
				FilePackage.head[5] = (crc32Value >> 8) & 0xff;
				FilePackage.head[6] = (crc32Value >> 16) & 0xff;
				FilePackage.head[7] = (crc32Value >> 24) & 0xff;


				temp = FilePackage.head;
				//串口发送
				//UART_PRINT("\n\r");
				while (UART1_STATE == 1) {
					Message("\r\n 3");
					vTaskDelay(1);
				}
				//Message("\n\r下发给MCU数据包:");
				UART1_STATE = 1;
				for (int i = 0; i < 16 + 86 + receive_Buf_index;
				     i++) {   //头 16 + TLV 86 + 文件长度 Status - 72 -16
					UARTCharPut(UARTA1_BASE, temp[i]);
					//UART_PRINT("%02x,",temp[i]);
				}
				UART1_STATE = 0;


				UART1_STATE = 1;
				UART_PRINT("\r\n文件长度:%d ,已写文件长度:%d , 总包数:%d 第%d包 , packet length = %d , CRC 值 = %x",
				           need_receive_file_length, FileSendLength_Inc, totalPackage, send_pack_count,
				           receive_Buf_index, crc32Value);
				Write_File_Resp = 0; Write_File_Resp_More = 0;
				UART1_STATE = 0;

				//等待串口回复
				times = 0;
				while ((Write_File_Resp == 0) & (Write_File_Resp_More == 0)) {
					if (times >= 500) {
						break;
					}
					times ++;
					vTaskDelay(10);
				}
				if (times == 500) {
					Message("\r\nTIME OUT");
				} else {
					Write_File_Resp = 0; Write_File_Resp_More = 0; send_file_is_finished = true;
				}
			} else {
				//头
				FileFollowPackage.head[0] = FileFollowPackage.head[1] =
				                                FileFollowPackage.head[2] = FileFollowPackage.head[3] = 0xcc;
				FileFollowPackage.head[8] = (16 + 10 + receive_Buf_index) % 256;
				FileFollowPackage.head[9] = (16 + 10 + receive_Buf_index) / 256;
				FileFollowPackage.head[10] = 1; //务类型 - 文件
				FileFollowPackage.head[11] = 0x21;
				//TLV 初始化
				//TP_FILE_PACKET
				FileFollowPackage.TP_FILE_PACKET[0] = TP_FILE_PACKET_ID;
				FileFollowPackage.TP_FILE_PACKET[1] = 0;
				FileFollowPackage.TP_FILE_PACKET[2] = 10 % 256;
				FileFollowPackage.TP_FILE_PACKET[3] = 10 / 256; //Tlv length
				FileFollowPackage.TP_FILE_PACKET[4] = totalPackage % 256;
				FileFollowPackage.TP_FILE_PACKET[5] = totalPackage /
				                                      256;            //Total Packets
				FileFollowPackage.TP_FILE_PACKET[6] = (send_pack_count) % 256;
				FileFollowPackage.TP_FILE_PACKET[7] = (send_pack_count) /
				                                      256;      //Cur Packet
				FileFollowPackage.TP_FILE_PACKET[8] = (receive_Buf_index) % 256;
				FileFollowPackage.TP_FILE_PACKET[9] = (receive_Buf_index) /
				                                      256;                        //Cur data length

				//数据包CRC校验
				crc32Value = GetCrc32(&FileFollowPackage.head[8],
				                      8 + 10 + receive_Buf_index); //TLV 10 + 文件长度 Status
				FileFollowPackage.head[4] = crc32Value & 0xff;
				FileFollowPackage.head[5] = (crc32Value >> 8) & 0xff;
				FileFollowPackage.head[6] = (crc32Value >> 16) & 0xff;
				FileFollowPackage.head[7] = (crc32Value >> 24) & 0xff;

				temp = FileFollowPackage.head;
				//串口发送
				while (UART1_STATE == 1) {
					Message("\r\n 4");
					vTaskDelay(1);
				}
				UART1_STATE = 1;
				for (int i = 0; i < 16 + 10 + receive_Buf_index ;
				     i++) {  //头 16 + TLV 86 + 文件长度 Status - 72 -16
					UARTCharPut(UARTA1_BASE, temp[i]);
				}
				UART1_STATE = 0;
				UART1_STATE = 1;
				UART_PRINT("\r\n文件长度:%d ,已写文件长度:%d , 总包数:%d 第%d包 , packet length = %d , CRC 值 = %x",
				           need_receive_file_length, FileSendLength_Inc, totalPackage, send_pack_count,
				           receive_Buf_index, crc32Value);
				Write_File_Resp = 0; Write_File_Resp_More = 0;
				UART1_STATE = 0;

				//等待串口回复
				times = 0;
				while ((Write_File_Resp == 0) & (Write_File_Resp_More == 0)) {
					if (times >= 500) {
						break;
					}
					times ++;
					vTaskDelay(10);
				}
				if (times == 500) {
					Message("\r\nTIME OUT");
				} else {
					Write_File_Resp = 0; Write_File_Resp_More = 0; send_file_is_finished = true;
				}
			}
			recive_file_is_finished = false;
			receive_Buf_index = 0;

			if ((need_receive_file_length != 0) && (received_file_length != 0)
			    && (received_file_length == need_receive_file_length)) {
				send_pack_count = 0;
				need_receive_file_length = 0;
				received_file_length = 0;
				send_file_is_finished = true;
			}
		}
	}

}

