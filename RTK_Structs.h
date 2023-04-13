/****************************************************************************
目的：    定义GPS+BDS RTK软件需要的常量和结构体
编写时间：2022.1.10
作者：    王甫红
版本:     V1.0
版权：    武汉大学测绘学院
****************************************************************************/
#include <iostream>
#include<stdio.h>
#include<windows.h>
#include<winsock.h>

#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)

#ifndef _GNSS_RTK_H_
#define _GNSS_RTK_H_

#define PAI 3.1415926535898
#define PAI2 (2.0*PAI)                    /* 2pi */
#define Rad (PAI/180.0)                  /* Radians per degree */
#define Deg (180.0/PAI)                  /* Degrees per radian */
#define C_Light 299792458.0         /* Speed of light  [m/s]; IAU 1976  */

#define R_WGS84  6378137.0          /* Radius Earth [m]; WGS-84  */
#define F_WGS84  1.0/298.257223563  /* Flattening; WGS-84   */
#define Omega_WGS 7.2921151467e-5   /*[rad/s], the earth rotation rate */
#define GM_Earth   398600.5e+9      /* [m^3/s^2]; WGS-84 */
#define R_CGS2K  6378137.0          /* Radius Earth [m]; CGCS2000  */
#define F_CGS2K  1.0/298.257222101  /* Flattening; CGCS2000   */
#define Omega_BDS 7.2921150e-5      /*[rad/s], the earth rotation rate */
#define GM_BDS   398600.4418e+9     /* [m^3/s^2]; CGCS2000  */


/* some constants about GPS satellite signal */
#define  FG1_GPS  1575.42E6             /* L1信号频率 */
#define  FG2_GPS  1227.60E6             /* L2信号频率 */
#define  FG12R    (77/60.0)             /* FG1_Freq/FG2_Freq */
#define  FG12R2   (5929/3600.0)
#define  WL1_GPS  (C_Light/FG1_GPS)
#define  WL2_GPS  (C_Light/FG2_GPS)

/* some constants about Compass satellite signal */
#define  FG1_BDS  1561.098E6              /* B1信号的基准频率 */
#define  FG2_BDS  1207.140E6              /* B2信号的基准频率 */
#define  FG3_BDS  1268.520E6              /* B3信号的基准频率 */
#define  FC12R    (FG1_BDS/FG2_BDS)       /* FG1_BDS/FG2_BDS */
#define  FC12R2   (FC12R*FC12R)           /* FG1_BDS^2/FG2_BDS^2 */
#define  FC13R    (FG1_BDS/FG3_BDS)       /* FG1_BDS^2/FG3_BDS^2 */
#define  FC13R2   (FC13R*FC13R)
#define  WL1_BDS  (C_Light/FG1_BDS)
#define  WL2_BDS  (C_Light/FG2_BDS)
#define  WL3_BDS  (C_Light/FG3_BDS)

#define GPST_BDT   14                      /* GPS时与北斗时的差值[s] */
#define MAXCHANNUM 36
#define MAXGPSNUM  32
#define MAXBDSNUM  63
#define MAXRAWLEN  40960
#define MAXNOVDLEN 20*1024
#define DELTATIME  2                      /*基站流动站最大相差时间*/

#define PsrDelta   1
#define AdrDelta   1e-3 
#define PsrW       2*PsrDelta*PsrDelta
#define AdrW       2*AdrDelta*AdrDelta
constexpr auto POLYCRC32 =  0xEDB88320u /* CRC32 polynomial */;;

/* 导航卫星系统定义 */
enum GNSSSys { UNKS=0, GPS, BDS, GLONASS, GALILEO, QZSS};

struct COMMONTIME   /* 通用时间定义 */
{
    short Year;
    unsigned short Month;
    unsigned short Day;
    unsigned short Hour;
    unsigned short Minute;
    double         Second;
    
	COMMONTIME()
	{
		Year	= 0;
		Month	= 0;
		Day		= 0;
		Hour	= 0;
		Minute	= 0;
		Second	= 0.0;
	}
};

struct GPSTIME              /* GPS时间定义 */
{
    unsigned short Week;          
    double         SecOfWeek;
    
    GPSTIME()
    {
        Week = 0;
        SecOfWeek = 0.0;
    }
};

struct MJDTIME             /* 简化儒略日 */
{
    int Days;             
    double FracDay;
    
    MJDTIME()
    {
        Days = 0;
        FracDay = 0.0;
    }
};

// GPS+BDS广播星历
struct GPSEPHREC
{
	unsigned short PRN;
	GNSSSys     Sys;
	GPSTIME  	TOC, TOE;
	short		SVHealth;
	double		ClkBias, ClkDrift, ClkDriftRate;
	double		IODE, IODC;
	double      TGD1, TGD2;
	double		SqrtA, e, M0, OMEGA, i0, omega, OMEGADot, iDot, DeltaN;
	double		Crs, Cuc, Cus, Cic, Cis, Crc;
    double		SVAccuracy;

	GPSEPHREC() {
		PRN = SVHealth = 0;
		Sys = UNKS;
		ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
		SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
		Crs = Cuc = Cus = Cic = Cis = Crc = SVAccuracy = 0.0;
	}
};

/*  每颗卫星的观测数据定义  */
struct SATOBS
{
    short    Prn;
  	GNSSSys  System;
	//double   c1, p2, l1, l2, d1, d2;   // m
	double   Psr[2], Adr[2];
	float    Dopper[2];
	double   cn0[2], LockTime[2];
	unsigned char half[2];
	bool     Valid;
   
    SATOBS()
    {
        Prn = 0;
        System = UNKS;
        //c1=p2=l1=l2=d1=d2=0.0;
		Psr[0] = Psr[1] = Adr[0] = Adr[1]=0;
		Dopper[0] = Dopper[1]=0;
		Valid = false;
    }
};

struct MWGF
{
	short Prn;//卫星号
	GNSSSys Sys;
	double MW, GF, PIF;

	int n;

	MWGF()
	{
		Prn = n = 0;
		Sys = UNKS;
		MW = GF = PIF = 0.0;
	}
};

/* 每颗卫星位置、速度和钟差等的中间计算结果 */
struct SATMIDRES
{
    double SatPos[3],SatVel[3];
    double SatClkOft,SatClkSft;
    double Elevation, Azimuth;
    double TropCorr;
    double Tgd1, Tgd2;
    bool Valid;  //false=没有星历或星历过期,true-计算成功
 
    SATMIDRES()
    {
        SatPos[0]=SatPos[1]=SatPos[2]=0.0;
        SatVel[0]=SatVel[1]=SatVel[2]=0.0;
        Elevation=PAI/2.0;
        SatClkOft=SatClkSft=0.0;
        Tgd1=Tgd2=TropCorr = 0.0;
        Valid=false;
    }
};

/*  每个历元的观测数据定义  */
struct EPOCHOBS
{
    GPSTIME    Time;
    short      SatNum;
    SATOBS     SatObs[MAXCHANNUM];
	SATMIDRES  SatPVT[MAXCHANNUM]; // 卫星位置等计算结果，数组索引与SatObs相同
	MWGF       ComObs[MAXCHANNUM];  // 当前历元的组合观测值，数组索引与SatObs相同
	double     Pos[3];      // 保存基站或NovAtel接收机定位结果
    
    EPOCHOBS()
    {
        SatNum    = 0;
		Pos[0] = Pos[1] = Pos[2] = 0.0;
    }
};

/*  每颗卫星的单差观测数据定义  */
struct SDSATOBS
{
	short    Prn;
	GNSSSys  System;
	bool     Valid;
	double   dP[2], dL[2];  // P，L双频单差,伪距，相位，伪距单位为米，相位单位为周
	short    nBas, nRov;    // 存储单差观测值对应的基准和流动站的数值索引号

	SDSATOBS()
	{
		Prn = nBas = nRov = 0;
		System = UNKS;
		dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
		Valid = false;
	}
};

/*  每个历元的单差观测数据定义  */
struct SDEPOCHOBS
{
	GPSTIME    Time;
	short      SatNum;
	SDSATOBS   SdSatObs[MAXCHANNUM];
	MWGF       SdCObs[MAXCHANNUM];

	SDEPOCHOBS()
	{
		SatNum = 0;
	}
};

/*  双差相关的数据定义  */
struct DDCOBS
{
	int RefPrn[2], RefPos[2];         // 参考星卫星号与存储位置，0=GPS; 1=BDS
	int Sats, DDSatNum[2];            // 待估的双差模糊度数量，0=GPS; 1=BDS
	double FixedAmb[MAXCHANNUM * 4];  // 包括双频最优解[0,AmbNum]和次优解[AmbNum,2*AmbNum]
	double ResAmb[2], Ratio;          // LAMBDA浮点解中的模糊度残差
	float  FixRMS[2];                 // 固定解定位中rms误差
	double dPos[3];                   // 基线向量
	bool bFixed;                      // true为固定，false为未固定

	DDCOBS()
	{
		int i;
		for (i = 0; i<2; i++) 
		{
			DDSatNum[i] = 0;    // 各卫星系统的双差数量
			RefPos[i] = RefPrn[i] = -1;
		}
		Sats = 0;              // 双差卫星总数
		dPos[0] = dPos[1] = dPos[2] = 0.0;
		ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = Ratio = 0.0;
		bFixed = false;
		for (i = 0; i<MAXCHANNUM * 2; i++)
		{
			FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
		}
	}
};

/* 每个历元单点定位和测速的结果及其精度指标 */
struct PPRESULT
{
    GPSTIME Time;
    double Position[3];
    double Velocity[3];
    double RcvClkOft[2];               /* 0 为GPS钟差; 1=BDS钟差 */
    double RcvClkSft;
    double PDOP, SigmaPos, SigmaVel;  // 精度指标
	short  GPSSatNum, BDSSatNum;      /* 单点定位使用的GPS卫星数 */
	short  AllSatNum;                /* 观测历元的所有卫星数   */
	bool   IsSuccess;                /* 单点定位是否成功, 1为成功, 0为失败 */

	PPRESULT()
	{
		for (int i=0; i<3; i++)		Position[i] = Velocity[i] = 0.0;
		RcvClkOft[0] = RcvClkOft[1] = RcvClkSft = 0.0;
		PDOP = SigmaPos = SigmaVel = 999.9;
		GPSSatNum = BDSSatNum = AllSatNum = 0;
		IsSuccess = false;
	}
};

/*  RTK定位的数据定义  */
struct RAWDAT {
	EPOCHOBS BasEpk;
	EPOCHOBS RovEpk;
	SDEPOCHOBS SdObs;
	DDCOBS DDObs;
	GPSEPHREC GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM];
};

struct RTKEKF
{
	GPSTIME Time;
	double X[3 + MAXCHANNUM * 2], P[(3 + MAXCHANNUM * 2)*(3 + MAXCHANNUM * 2)];
	int Index[MAXCHANNUM], nSats, nPos[MAXCHANNUM];
	int FixAmb[MAXCHANNUM];          // 时间更新后上个历元已经固定并传递的模糊度， 1=已固定，-1=未固定或有周跳
	DDCOBS DDObs, CurDDObs;           // 上一个历元和当前历元的双差观测值信息
	SDEPOCHOBS SDObs;                 // 上一个历元的单差观测值
	double X0[3 + MAXCHANNUM * 2], P0[(3 + MAXCHANNUM * 2)*(3 + MAXCHANNUM * 2)];  // 状态备份
	bool IsInit;                      // 滤波是否初始化

	RTKEKF() {
		IsInit = false;
		nSats = 0;
		for (int i = 0; i < MAXCHANNUM; i++) nPos[i] = Index[i] = FixAmb[i] = -1;
		for (int i = 0; i < 3 + MAXCHANNUM * 2; i++) {
			X[i] = X0[i] = 0.0;
			for (int j = 0; j < 3 + MAXCHANNUM * 2; j++) P[i*(3 + MAXCHANNUM * 2) + j] = P0[i*(3 + MAXCHANNUM * 2) + j] = 0.0;
		}
	}
};

struct ROVERCFGINFO   // 配置信息
{
	short  IsFileData, RTKProcMode;      // 1=FILE, 0=COM, 1=EKF, 2=LSQ
	int    RovPort, RovBaud;             // COM端口设置
	char   BasNetIP[20], RovNetIP[20];   // ip address
	short  BasNetPort, RovNetPort;       // port
	double CodeNoise, CPNoise;           // 伪距噪声
    double ElevThreshold;                // 高度角阈值
	double RatioThres;                   // Ratio检验阈值
    
	char  BasObsDatFile[256], RovObsDatFile[256];    //  观测数据的文件名
	char  ResFile[256];            //  结果数据文件名

    
    ROVERCFGINFO()
    {
		IsFileData = RTKProcMode = 1;
		RovPort = RovBaud = BasNetPort = RovNetPort = 0;
		CodeNoise = CPNoise = ElevThreshold = 0.0;
		RatioThres = 3.0;
	}
};

bool ReadSATODSConfigInfo( const char FName[], ROVERCFGINFO& cfg );

/* 通用时,GPS时和简化儒略日之间的相互转换函数*/
void CommonTimeToMJDTime( const COMMONTIME* CT, MJDTIME* MJDT);
void MJDTimeToCommonTime( const MJDTIME* MJDT, COMMONTIME* CT );
void GPSTimeToMJDTime( const GPSTIME* GT, MJDTIME* MJDT );
void MJDTimeToGPSTime ( const MJDTIME* MJDT, GPSTIME* GT );
void CommonTimeToGPSTime ( const COMMONTIME* CT, GPSTIME* GT );
void GPSTimeToCommonTime ( const GPSTIME* GT, COMMONTIME* CT );
double GetDiffTime( const GPSTIME* GT2, const GPSTIME* GT1 );

/* 空间直角坐标,大地坐标的相互转换函数 */
void XYZToBLH( const double xyz[3], double blh[3], const double R, const double F );
void BLHToXYZ( const double BLH[3], double XYZ[3], const double R, const double F );
void BLHToNEUMat(const double Blh[], double Mat[]);
void CompSatElAz(const double Xr[], const double Xs[], double *Elev, double *Azim); //卫星高度角方位角计算函数
void Comp_dEnu(const double X0[], const double Xr[], double dNeu[]);  //定位误差计算函数

/*矩阵运算*/
bool matrix_add(const int row1, const int col1, const int row2, const int col2, const double a[], const double b[], double c[]);
bool matrix_minus(const int row1, const int col1, const int row2, const int col2, const double a[], const double b[], double c[]);
bool matrix_multiply(const int row1, const int col1, const int row2, const int col2, const double a[], const double b[], double c[]);
bool matrix_transfer(const int row1, const int col1, const double a[], double b[]);
bool matrix_Inv(int n, const double a[], double b[]);
double vector_dot(const int row1, const int col1, const int row2, const int col2, const double a[], const double b[]);
bool vector_cross(const int row1, const int col1, const int row2, const int col2, const double a[], const double b[], double c[]);

// NovAtel OEM7数据解码函数
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[]);
int DecodeNovOem7Soc(unsigned char Buff[], int& Len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[]);
int decode_rangeb_oem7(unsigned char *buff, EPOCHOBS* obs);
int decode_gpsephem(unsigned char* buff, GPSEPHREC* eph);
int decode_bdsephem(unsigned char* buff, GPSEPHREC* eph);
//int decode_psrpos(unsigned char* buff, POSRES* pos);
unsigned int crc32(const unsigned char *buff, int len);
int decode(EPOCHOBS* Epoch, FILE* stream, RAWDAT* Raw, unsigned char Buff[], int& LenRead, int& LenRem);
int decode(EPOCHOBS* Epoch, SOCKET& Sock, RAWDAT* Raw, unsigned char Buff[], int& LenRead, int& LenRem);
// 广播星历
bool CompSatClkOff( const int Prn, const GNSSSys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATMIDRES* Mid);
bool CompGPSSatPVT( const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);
bool CompBDSSatPVT( const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);
void ComputeGPSSatOrbitAtSignalTrans(const EPOCHOBS* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double RcvPos[], SATMIDRES* MidRes);

double hopfield(double hgt,double elev);

// SPP & SPV
void DetectOutlier(EPOCHOBS* Obs);  // 线性组合探测粗差
bool SPP(EPOCHOBS* Epoch, RAWDAT* Raw, PPRESULT* Result);
void SPV(EPOCHOBS* Epoch, PPRESULT* Result );

// RTK
//int GetSynObs(FILE* FBas, FILE* FRov, SOCKET& BasSock, CSerial& RovCom, SOCKET& RovSock, RAWDAT* Raw);
int GetSynObs(FILE* FBas, FILE* FRov, RAWDAT* Raw);
int GetSynObs(SOCKET& BasSock, SOCKET& RovSock, RAWDAT* Raw);

void FormSDEpochObs(const EPOCHOBS *EpkA, const EPOCHOBS *EpkB, SDEPOCHOBS *SDObs);
void DetectCycleSlip(SDEPOCHOBS* Obs);
void DetRefSat(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs);

//0=ok,1=false
bool RTKFloat(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov);
void RTKFixed(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov);


// lambda
int LD(int n, const double *Q, double *L, double *D);
void gauss(int n, double *L, double *Z, int i, int j);
void perm(int n, double *L, double *D, int j, double del, double *Z);
void reduction(int n, double *L, double *D, double *Z);
int search(int n, int m, const double *L, const double *D, const double *zs, double *zn, double *s);
int lambda(int n, int m, const double *a, const double *Q, double *F, double *s);

//// Socket网络
bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);
void CloseSocket(SOCKET& sock);

/*
Prn检索，根据输入卫星Prn，在观测值中检索对应卫星，取得对应卫星观测值中的索引
参数：
      Prn：                  需检索的Prn
      EPOCHOBS/SDEPOCHOBS：  在对应观测值/单差观测值内搜索对应卫星
	  Sys：                  该卫星所属系统(GPS=1,BDS=2)
	  EpochIndex/SDObsIndex：检索得到的对应卫星在对应观测值数组内索引
返回值：
      true  = 卫星检索成功
	  false = 观测值列表中不含该卫星，检索失败
*/
bool PrnSearching(const short Prn, const EPOCHOBS* Epoch, const int Sys, int* EpochIndex);
bool PrnSearching(const short Prn, const SDEPOCHOBS* SDObs, const int Sys, int* SDObsIndex);

void MatrixDisplay(int m, int n, double Mat[]);

void SaveSocketData(SOCKET Soc,FILE* Dat);
#endif
