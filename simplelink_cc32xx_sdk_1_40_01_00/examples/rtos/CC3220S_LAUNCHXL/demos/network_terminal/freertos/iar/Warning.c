#include <ti/drivers/net/wifi/netapp.h>
//Service type
#define NO_SERVICE 			0x00
#define SERVICE_FILE		        0x01
#define SERVICE_WARNING		        0x02
#define SERVICE_SETTING		        0x03
#define SERVICE_HEARTBEAT	        0x05
#define SERVICE_CMD			0x06
#define SERVICE_DVR			0x07
#define SERVICE_DEBUG		        0x08

#define MSG_DVR_KEY			0x01
#define MSG_DVR_FILE_LIST   		0x02
#define MSG_DVR_PLAY_FILE   	        0x03
#define MSG_DVR_FORMAT		        0x05
#define MSG_DVR_STATUS		        0x06
#define MSG_DVR_DEL_FILE		0x07

//Tlv type
#define TP_FILE_PARA		0x02  //文件相关参数
#define TP_WARNING			0x03  //报警信息
#define TP_FILE_PACKET		0x04  //文件分包情况
#define TP_WIFI_PASSWORD	0x05
#define TP_TIME				0x06
#define TP_MH_VER			0x07  //MH 设备序列号
#define TP_DEV_VER			0x08  //MCU设备号序列号
#define TP_DELAY			0x09  //延时(单位ms)
#define TP_WORK_TIME		0x0A  //测试持续时间
#define TP_DVR_KEY			0x0B
#define TP_SWITCH_VIDEO		0x0C  //切换图像
#define TP_FPGA_VER			0x0D  //FPGA设备序列号
#define TP_RESOLUTION		0x0E  //图像分辨率
#define TP_WIFI_NAME		0x0F  //WIFI  名称
#define TP_GATE_VER			0x10  //ADASGATE version
#define TP_WARN_DAY_STAT	0x11
#define TP_WARN_MONTH_STAT	0x12
#define TP_FRAME_COUNT	 	0x13
#define TP_DVR_FILE_TYPE	0x14
#define TP_DVR_FILE_LIST	0x15
#define TP_DVR_PLAY_FILE	0x16
#define TP_LOG_SWITCH		0x17
#define TP_LOG_CONTENT		0x18
#define TP_DEBUG_CMD		0x19
#define TP_DVR_RESOLUTION	0x1A
#define TP_CALIBRATE_CMD	0x1B
#define TP_CALIBRATE_RESULT     0x1C
#define TP_DVR_STATUS		0x1D

#define FILE_NAME_SIZE          64

unsigned char FileName_Warn_Stat_Today[6];//当天报警文件名称
unsigned char FileName_Warn_Stat_Index[6];//按index 文件名称
long FileHandle_Warn_Stat_Today;
long FileHandle_Warn_Stat_Index;
extern unsigned int Config_timeUpdateFlag;

typedef struct {
	unsigned char head[16];
	unsigned char TLV_DAY_STAT[784];
} WARN_DAY_STAT;

static WARN_DAY_STAT WARN_DAY_STAT_TODAY;
static WARN_DAY_STAT WARN_DAY_STAT_RESP;
unsigned char *p_WARN_DAY_STAT_TODAY = WARN_DAY_STAT_TODAY.head;
unsigned char *p_WARN_DAY_STAT_RESP = WARN_DAY_STAT_RESP.head;

struct eventRecord {
	double longitude;
	double latitude;
	int type;
	int desc;
};

struct hourRecord {
	unsigned int count[7];
	unsigned int reserved;
};

struct dayStatHead {
	unsigned int date;
	char ver;
	char reserver;
	unsigned short mileage;
	unsigned short runtime;
	char reservered[6];
};

struct dayRecord {
	unsigned int day;
	unsigned int count[7];
};
typedef struct {
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
	int weekdays;
} my_tm;
extern my_tm tm;
struct mcuHeader {
	unsigned int startflag;
	unsigned int crc32;
	unsigned short len;
	unsigned char     serviceType;
	unsigned char     messageType;
	unsigned int reserve;
};
struct typeLen {
	unsigned short type;
	unsigned short length;
	//	char *valuep;
};
#define MAXDAY 365
//Warning type
#define WT_FCW			0
#define WT_UFCW			1
#define WT_LDW_LEFT		2
#define WT_LDW_RIGHT	        3
#define WT_PCW			4
#define WT_OVER_SPEED	        5
#define WT_HMW			6

struct hourRecord dayStat[24];
unsigned char *pStat24Hour = (unsigned char *)dayStat;
struct dayRecord monthStat[MAXDAY];
int lastDate = -1;
int dayfd = -1;
int monthfd = -1;

static int fcw = 0;
static int ufcw = 0;
static int ldwl = 0;
static int ldwr = 0;
static int pcw = 0;
static int speeding = 0;
static int emergency_brake = 0;
static int hmw = 0;
int checkSystemTime()
{

}

int checkFCW(char *warndata, struct eventRecord *recordp)
{
	if ((warndata[0] & 0xC0) == 0xC0) {
		if (fcw == 0) {
			recordp->type = WT_FCW;
			fcw = 1;
			return 1;
		}
	} else {
		fcw = 0;
	}
	return 0;
}

int checkUFCW(char *warndata, struct eventRecord *recordp)
{
	if ((warndata[0] & 0xC0) == 0x80) {
		if (ufcw == 0) {
			recordp->type = WT_UFCW;
			ufcw = 1;
			return 1;
		}
	} else {
		ufcw = 0;
	}
	return 0;
}

int checkPCW(char *warndata, struct eventRecord *recordp)
{
	if ((warndata[0] & 0x8) > 0) {
		if (pcw == 0) {
			recordp->type = WT_PCW;
			pcw = 1;
			return 1;
		}
	} else {
		pcw = 0;
	}
	return 0;
}

int checkLDW(char *warndata, struct eventRecord *recordp)
{
	if ((warndata[0] & 0x20) > 0) {
		if (ldwl == 0) {
			recordp->type = WT_LDW_LEFT;
			ldwl = 1;
			return 1;
		}
	} else {
		ldwl = 0;
	}

	if ((warndata[0] & 0x10) > 0) {
		if (ldwr == 0) {
			recordp->type = WT_LDW_RIGHT;
			ldwr = 1;
			return 1;
		}
	} else {
		ldwr = 0;
	}
	return 0;
}

int checkSpeeding(char *warndata, struct eventRecord *recordp)
{
	if ((warndata[0] & 0x2) > 0) {
		if (speeding == 0) {
			recordp->type = WT_OVER_SPEED;
			recordp->desc = warndata[2];
			speeding = 1;
			return 1;
		}
	} else {
		speeding = 0;
	}
	return 0;
}

int checkHMW(char *warndata, struct eventRecord *recordp)
{
	return 0;
}

int checkEmergencyBrake(char *warndata, struct eventRecord *recordp)
{
	if ((warndata[0] & 0x1) > 0) {
		if (emergency_brake == 0) {
			emergency_brake = 1;
			return 1;
		}
	} else {
		emergency_brake = 0;
	}
	return 0;
}


int checkTakePhoto(char *warndata)
{
	//if ((warndata[0] & 0x89) > 0)
	if ((warndata[0] & 0x01) > 0) {
		return 1;
	} else {
		return 0;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////
void updateDayStat(struct eventRecord record)
{
	dayStat[tm.hour].count[record.type] += 1;
}

void takePhoto()
{
	unsigned char key;
	unsigned char buf[0x16];
	int len = 0x16;
	char *hexStr;
	struct mcuHeader *headp;
	struct typeLen *tlp;

	key = 0x06;
	//fprintf(stdout, "DVR key=%d\n", key);

	memset(buf, 0, len);
	headp = (struct mcuHeader *)buf;
	headp->startflag = 0xcccccccc;
	headp->len = len;
	headp->serviceType = SERVICE_DVR;
	headp->messageType = MSG_DVR_KEY;

	tlp = (struct typeLen *)(buf + 0x10);
	tlp->type = TP_DVR_KEY;
	tlp->length = 6;
	buf[0x14] = key;
	buf[0x15] = 1;

	headp->crc32 = GetCrc32(buf + 8, len - 8);

	//sendCmd2DVR(buf, len);  .............................
}



void saveDayStat()
{
	unsigned char FileName[10]; unsigned long ulToken; unsigned long lRetVal;
	FileName[0] = ((tm.year % 100) / 10) + '0';
	FileName[1] = (tm.year % 10) + '0'; //年
	FileName[2] = ((tm.month % 100) / 10) + '0';
	FileName[3] = (tm.month % 10) + '0';//月
	FileName[4] = ((tm.day % 100) / 10) + '0';
	FileName[5] = (tm.day % 10) + '0'; //日

	FileName[6] = '.'; FileName[7] = 's';
	FileName[8] = 'a'; FileName[9] = 't';
	//打开文件
	long lFileHandle = sl_FsOpen(FileName,
	                             SL_FS_CREATE | SL_FS_OVERWRITE | SL_FS_CREATE_MAX_SIZE(1024), &ulToken);
	//write day stat
	unsigned int len = sizeof(struct hourRecord);
	for (int i = 0; i < 24; i++) {
		lRetVal = sl_FsWrite(lFileHandle, i * len, (unsigned char *)&dayStat[i], len);
		if (lRetVal < 0) {
			Message("\r\n 统计文件写入失败!");
		}
	}
	lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
	if (lRetVal < 0) {
		Message("\r\n 统计文件关闭失败!");
	}
}

void saveWarnStat()
{
	if (Config_timeUpdateFlag == 1) {
		saveDayStat();
	}
	//saveMonthStat();
}
static int count = 1000;
void dealWarning(char *buf, int len)
{
	char *warnData;
	struct eventRecord record;
	record.longitude = 0;
	record.latitude = 0;
	//record.time = ts;

	warnData = buf + 0x14;

	//check warning and record to file
	//update dayStat
	if (checkFCW(warnData, &record)) {
		updateDayStat(record);
	}

	if (checkUFCW(warnData, &record)) {
		updateDayStat(record);
	}

	if (checkLDW(warnData, &record)) {
		updateDayStat(record);
	}

	if (checkPCW(warnData, &record)) {
		updateDayStat(record);
	}

	if (checkSpeeding(warnData, &record)) {
		updateDayStat(record);
	}

	if (checkHMW(warnData, &record)) {
		updateDayStat(record);
	}

	//send to dvr every 4 messages
	if (checkTakePhoto(warnData) && ((count % 4) == 0)) {
		takePhoto();
	}


	if (count > 200) {
		saveWarnStat();
		count = 0;
	}
	count++;
}



/*

long openWarningStatFile(unsigned long *ulToken)
{
    long lRetVal = -1;
    int iLoopCnt = 0;
    long lFileHandle;
    //
    //  create a user file
    //
    lFileHandle = sl_FsOpen((unsigned char *)USER_FILE_NAME,
                        SL_FS_CREATE|SL_FS_OVERWRITE|SL_FS_CREATE_MAX_SIZE( MaxSize ),
                        ulToken);
    if(lFileHandle < 0)
    {
        //创建失败
		return -10;
    }

    //
    // write "Old MacDonalds" child song as many times to get just below a 64KB file
    //
    for (iLoopCnt = 0;
            iLoopCnt < (SL_MAX_FILE_SIZE / sizeof(gaucOldMacDonald));
            iLoopCnt++)
    {
        lRetVal = sl_FsWrite(lFileHandle,
                    (unsigned int)(iLoopCnt * sizeof(gaucOldMacDonald)),
                    (unsigned char *)gaucOldMacDonald, sizeof(gaucOldMacDonald));
        if (lRetVal < 0)
        {
            lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
            //ASSERT_ON_ERROR(FILE_WRITE_FAILED);
			//写入失败
			return -11;
        }
    }

    //
    // close the user file
    //
    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != lRetVal)
    {
        //ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
		//关闭失败
		return -12;
    }

    return 0;//success
}
*/

