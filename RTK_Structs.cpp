#include "RTK_Structs.h"
#include <iostream>
////配置文件
//bool WriteSATODSConfigInfo(const char FName[])
//{
//    FILE* cfg;
//    if (fopen_s(&cfg, FName, "w+") != 0)
//    {
//        printf("Cannot open this cfg file!");
//        return false;
//    }
//
//    fprintf(cfg, "[GNSS RTK Configuration file]\n");
//
//
//
//    
//}
//读取配置文件
bool ReadSATODSConfigInfo(const char FName[], ROVERCFGINFO& cfg)
{

    return false;
}
//通用时, GPS时和简化儒略日之间的相互转换函数
void CommonTimeToMJDTime(const COMMONTIME* CT, MJDTIME* MJDT)
{
    //年，月，日，时，分，秒  
    int Y, M, D, H, Min, y, m;
    double S, UT, MJD;
    Y = CT->Year;
    M = CT->Month;
    D = CT->Day;
    H = CT->Hour;
    Min = CT->Minute;
    S = CT->Second;
    UT = H + double(Min / 60) + S / 3600;
    if (M <= 2)
    {
        y = Y - 1;
        m = M + 12;
    }
    else
    {
        y = Y;
        m = M;
    }
    MJD = int(365.25 * y) + int(30.6001 * (m + 1)) + D + UT / 24 - 679019;
    MJDT->Days = int(MJD);
    MJDT->FracDay = MJD - MJDT->Days;
}
void MJDTimeToCommonTime(const MJDTIME* MJDT, COMMONTIME* CT)
{
    int a, b, c, d, e, Y, M, D, H, Min;
    double JD, S;
    JD = MJDT->Days + MJDT->FracDay + 2400000.5;
    a = int(JD + 0.5);
    b = a + 1537;
    c = int((b - 122.1) / 365.25);
    d = int(365.25 * c);
    e = int((b - d) / 30.6001);
    M = e - 1 - 12 * int(e / 14);//月
    Y = c - 4715 - int((7 + M) / 10);//年
    D = b - d - int(30.6001 * e);//日
    H = int(MJDT->FracDay * 24);//时
    Min = int((MJDT->FracDay * 24 - H) * 60);//分
    S = double((((MJDT->FracDay * 24 - H) * 60) - Min) * 60);//秒
    CT->Year = Y;
    CT->Month = M;
    CT->Day = D;
    CT->Hour = H;
    CT->Minute = Min;
    CT->Second = S;
}
void GPSTimeToMJDTime(const GPSTIME* GT, MJDTIME* MJDT)
{
    double MJD;
    MJD = 44244 + GT->Week * 7 + GT->SecOfWeek / 86400;
    MJDT->Days = int(MJD);
    MJDT->FracDay = MJD - MJDT->Days;
}
void MJDTimeToGPSTime(const MJDTIME* MJDT, GPSTIME* GT)
{
    //GPS时的起点为1980年1月6日0点，其MJD时间表示为44244
    double MJD = MJDT->Days + MJDT->FracDay;
    GT->Week = int((MJD - 44244) / 7);
    GT->SecOfWeek = (MJDT->Days - 44244 - GT->Week * 7 + MJDT->FracDay) * 86400.0;
}
void CommonTimeToGPSTime(const COMMONTIME* CT, GPSTIME* GT)
{
    MJDTIME MJDT;
    CommonTimeToMJDTime(CT, &MJDT);
    MJDTimeToGPSTime(&MJDT, GT);
}
void GPSTimeToCommonTime(const GPSTIME* GT, COMMONTIME* CT)
{
    MJDTIME MJDT;
    GPSTimeToMJDTime(GT, &MJDT);
    MJDTimeToCommonTime(&MJDT, CT);
}
double GetDiffTime(const GPSTIME* GT2, const GPSTIME* GT1)
{
    return 0.0;
}

//空间直角坐标,大地坐标的相互转换函数
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F)
{
    double N;//卯酉圈半径
    double e = sqrt(2 * F - F * F);
    N = R / sqrt(1 - (e * sin(BLH[0])) * (e * sin(BLH[0])));
    XYZ[0] = (N + BLH[2]) * cos(BLH[0]) * cos(BLH[1]);
    XYZ[1] = (N + BLH[2]) * cos(BLH[0]) * sin(BLH[1]);
    XYZ[2] = (sin(BLH[0]) * (N * (1 - e * e) + BLH[2]));
}
void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F)
{
    double e = sqrt(2 * F - F * F);
    int iterator;
    double X, Y, Z, B;
    double dZ,dZ0; //Z的变化量，需要迭代计算，初值设为dZ=e^2*Z;
    double N;//卯酉圈半径
    X = xyz[0]; Y = xyz[1]; Z = xyz[2];
    dZ0 = e * e * Z;
    iterator = 0;
    
    while (true)
    {
        B = (Z + dZ0) / sqrt(X * X + Y * Y + (Z + dZ0) * (Z + dZ0));//Sin(B)
        N = R / sqrt(1 - (e * B) * (e * B));
        dZ = N * e * e * B;
        if (fabs(dZ-dZ0) < 1e-4 || iterator >= 10)
        {
            break;
        }
        dZ0 = dZ;
        iterator++;
    }
    blh[1] = atan2(Y, X);
    blh[0] = atan2((Z + dZ), (sqrt(X * X + Y * Y)));
    blh[2] = sqrt(X * X + Y * Y + (Z + dZ) * (Z + dZ)) - N;
}
void BLHToNEUMat(const double Blh[], double Mat[])//将Blh转换为以Mat为站型心的NEU坐标系，结果给到Mat中。
{
    double xyz[3]{ 0 };
    double PosBlh[3]{ 0 };
    BLHToXYZ(Blh, xyz, R_WGS84, F_WGS84);
    XYZToBLH(Mat, PosBlh, R_WGS84, F_WGS84);
    double Dx[3]{ 0 };
    Dx[0] = xyz[0] - Mat[0];
    Dx[1] = xyz[1] - Mat[1];
    Dx[2] = xyz[2] - Mat[2];
    double B = PosBlh[0];
    double L = PosBlh[1];
    double H[9] = { -sin(B) * cos(L),-sin(B) * sin(L),cos(B),
                    -sin(L)         , cos(L)         ,     0,
                     cos(B) * cos(L), cos(B) * sin(L),sin(B) };
    matrix_multiply(3, 3, 3, 1, H, Dx, Mat);//Mat:NEU
}
void CompSatElAz(const double Xr[], const double Xs[], double* Elev, double* Azim)
{
    double r;
    double BLH[3] = { 0,0,0 };
    double NEUMat[3] = {Xr[0],Xr[1],Xr[2]};
    XYZToBLH(Xs, BLH, R_WGS84, F_WGS84);
    BLHToNEUMat(BLH,NEUMat);
    r = sqrt(NEUMat[0] * NEUMat[0] + NEUMat[1] * NEUMat[1] + NEUMat[2] * NEUMat[2]);
    *Elev = asin(NEUMat[2] / r);
    *Azim = atan2(*Elev, NEUMat[0]);
    double P=*Elev* Deg;//test
}//卫星高度角方位角计算
void Comp_dEnu(const double X0[], const double Xr[], double dNeu[])
{
    
}

//矩阵运算
bool matrix_add(const int row1, const int col1, const int row2, const int col2, const double a[], const double b[], double c[])
{
    int i, j;

    if (row1 != row2 || col1 != col2)
    {
        printf("出错，相加的两矩阵行列不等");
        return false;
    }

    for (i = 0; i < row1; i++)
    {
        for (j = 0; j < col1; j++)
        {
            c[i * col1 + j] = a[i * col1 + j] + b[i * col1 + j];
        }
    }
    return true;

}
bool matrix_minus(const int row1, const int col1, const int row2, const int col2, const double a[],const double b[], double c[])
{
    int i, j;

    if (row1 != row2 || col1 != col2)
    {
        printf("出错，相减的两矩阵行列不等");
        return false;
    }

    for (i = 0; i < row1; i++)
    {
        for (j = 0; j < col1; j++)
        {
            c[i * col1 + j] = a[i * col1 + j] - b[i * col1 + j];
        }
    }
    return true;
}
bool matrix_multiply(const int row1, const int col1, const int row2, const int col2, const double a[],const double b[], double c[])
{
    int i, j, k;
    double val;

    if (col1 != row2)
    {
        printf("不能进行矩阵相乘，因为前矩阵的列数不等于后矩阵的行数");
        return false;
    }

    for (i = 0; i < row1; i++)
    {
        for (j = 0; j < col2; j++)
        {
            val = 0.0;
            for (k = 0; k < col1; k++)
            {
                val += a[i * col1 + k] * b[k * col2 + j];
            }
            c[i * col2 + j] = val;
        }
    }
    return true;
}
bool matrix_transfer(const int row1, const int col1, const double a[], double b[])
{
    int row2, col2;
    row2 = col1;//b矩阵的列数是原a矩阵的行数
    col2 = row1;//b矩阵的行数是原a矩阵的列数
    int i, j;

    for (i = 0; i < row1; i++)
    {
        for (j = 0; j < col1; j++)
        {
            b[j * col2 + i] = a[i * col1 + j];
        }
    }
    return true;
}
bool matrix_Inv(int n, const double a[], double b[])
{
    int i, j, k, l, u, v, is[250] = { 0 }, js[250] = { 0 };   /* matrix dimension <= 250 */
    double d, p;

    if (n <= 0)
    {
        printf("Error dimension in MatrixInv!\n");
        return false;
    }

    /* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            b[i * n + j] = a[i * n + j];
        }
    }

    for (k = 0; k < n; k++)
    {
        d = 0.0;
        for (i = k; i < n; i++)   /* 查找右下角方阵中主元素的位置 */
        {
            for (j = k; j < n; j++)
            {
                l = n * i + j;
                p = fabs(b[l]);
                if (p > d)
                {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }

        if (d < 1E-12)   /* 主元素接近于0，矩阵不可逆 */
        {
            //   printf("Divided by 0 in MatrixInv!\n");
            return false;
        }

        if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = is[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = i * n + js[k];
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        l = k * n + k;
        b[l] = 1.0 / b[l];  /* 初等行变换 */
        for (j = 0; j < n; j++)
        {
            if (j != k)
            {
                u = k * n + j;
                b[u] = b[u] * b[l];
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                for (j = 0; j < n; j++)
                {
                    if (j != k)
                    {
                        u = i * n + j;
                        b[u] = b[u] - b[i * n + k] * b[k * n + j];
                    }
                }
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                u = i * n + k;
                b[u] = -b[u] * b[l];
            }
        }
    }

    for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
    {
        if (js[k] != k)
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = js[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if (is[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = is[k] + i * n;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
    }

    return true;
}
bool vector_cross(const int row1, const int col1, const int row2, const int col2, const double a[],const double b[], double c[])
{
    if (row1 != 1 || row2 != 1 || col1 != 3 || col2 != 3)
    {
        printf("输入了非法矢量");
        return false;
    }

    c[0] = a[1] * b[2] - b[1] * a[2];
    c[1] = a[0] * b[2] - b[0] * a[2];
    c[2] = a[0] * b[1] - b[0] * a[1];

    return true;
}
double vector_dot(const int row1, const int col1, const int row2, const int col2, const double a[],const double b[])
{
    if (row1 != 1 || row2 != 1 || col1 != 3 || col2 != 3)
    {
        printf("输入了非法矢量");
        return 0;
    }

    double c = 0.0;
    int i;
    for (i = 0; i < 3; i++)
    {
        c += a[i] * b[i];
    }

    return c;
}

// NovAtel OEM7数据解码函数
double R8(unsigned char* p)
{
    double r;
    memcpy(&r, p, 8);
    return r;
}
unsigned short US2(unsigned char* p)
{
    unsigned short r;
    memcpy(&r, p, 2);
    return r;
}
unsigned long UL4(unsigned char* p)
{
    unsigned long r;
    memcpy(&r, p, 4);
    return r;
}
float F4(unsigned char* p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}
unsigned int crc32(const unsigned char* buff, int len)
{
    int i, j;
    unsigned int crc = 0;
    for (i = 0; i < len; i++)
    {
        crc ^= buff[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}
int decode(EPOCHOBS* Epoch, FILE* stream, RAWDAT* Raw, unsigned char Buff[], int& LenRead, int& LenRem)//解码小函数 0=无观测值；1=有观测值；-1=读到文件尾
{
    int Status = 0;
    LenRead = fread_s(Buff + LenRem, MAXNOVDLEN, sizeof(unsigned char), MAXNOVDLEN - (LenRem), stream);
    if (LenRead < (MAXNOVDLEN - LenRem))
    {
        printf("finished!\n");
        fclose(stream);
        return -1;//读到文件尾
    }
    // 调用解码模块，得到观测值和星历
    Status = DecodeNovOem7Dat(Buff, LenRem, Epoch, Raw->GpsEph, Raw->BdsEph);
    return Status;
}
int decode(EPOCHOBS* Epoch, SOCKET& Sock, RAWDAT* Raw, unsigned char Buff[], int& LenRead, int& LenRem)
{
    int Status = 0;
    LenRead = recv(Sock, (char*)Buff + LenRem, MAXNOVDLEN - LenRem, 0);
    if (LenRead < 0)
    {
        printf("Socket no data!\n");
        CloseSocket(Sock);
        return -1;
    }
    //解码
    LenRem += LenRead;
    Status = DecodeNovOem7Soc(Buff, LenRem, Epoch, Raw->GpsEph, Raw->BdsEph);
    return Status;
}
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBS* obs)
{
    int n, i, j, f;
    unsigned short prn;
    unsigned char* p = buff + 28;
    unsigned int track;
    GNSSSys sys;
    obs->Time.Week = US2(buff + 14);
    obs->Time.SecOfWeek = UL4(buff + 16) * 1e-3;
    obs->SatNum = UL4(p);
    memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBS));
    n = 0;
    for (p += 4, j = 0; j < obs->SatNum; j++, p += 44)
    {
        //卫星系统
        track = UL4(p + 40);
        switch ((track >> 16) & 7)
        {
        case 0:sys = GPS;
            break;
            //	case 3:obs->satobs[j].sys = GALILEO;
            //		break;
        case 4:sys = BDS;
            break;
            //	case 5:obs->satobs[j].sys = QZSS;
            //	break;
        default:sys = UNKS;
            break;
        }

        if (sys == UNKS) continue;

        //信号类型
        if (sys == GPS)
        {
            switch ((track >> 21) & 0x1F)
            {
            case 0:f = 0; break;
            case 9:f = 1; break;
            default:f = 2; break;
            }
        }
        if (sys == BDS)
        {
            switch ((track >> 21) & 0x1F)
            {
            case 0:f = 0; break;
            case 2:f = 1; break;
            case 4:f = 0; break;
            case 6:f = 1; break;
            default:f = 2; break;
            }
        }
        if (f == 2) continue;

        prn = US2(p);
        for (i = 0; i < n; i++)
        {
            if (obs->SatObs[i].System == sys && obs->SatObs[i].Prn == prn)
            {
                n = i;
                break;
            }
        }
        obs->SatObs[n].Prn = prn;
        obs->SatObs[n].System = sys;
        obs->SatObs[n].Psr[f] = R8(p + 4);
        obs->SatObs[n].Adr[f] = -R8(p + 16);
        obs->SatObs[n].Dopper[f] = -F4(p + 28);
        obs->SatObs[n].cn0[f] = F4(p + 32);
        n++;
    }
    obs->SatNum = n;
    return 1;
}
int decode_gpsephem(unsigned char* buff, GPSEPHREC* eph)
{
    int prn, num;
    prn = UL4(buff + 28);
    num = prn - 1;
    eph[num].PRN = prn;
    eph[num].Sys = GPS;
    eph[num].TOE.Week = UL4(buff + 28 + 24);
    eph[num].TOC.Week = UL4(buff + 28 + 24);
    eph[num].TOE.SecOfWeek = R8(buff + 28 + 32);
    eph[num].TOC.SecOfWeek = R8(buff + 28 + 164);
    eph[num].TGD1 = R8(buff + 28 + 172);
    eph[num].ClkBias = R8(buff + 28 + 180);
    eph[num].ClkDrift = R8(buff + 28 + 188);
    eph[num].ClkDriftRate = R8(buff + 28 + 196);
    eph[num].Cic = R8(buff + 28 + 112);
    eph[num].Cis = R8(buff + 28 + 120);
    eph[num].Crc = R8(buff + 28 + 96);
    eph[num].Crs = R8(buff + 28 + 104);
    eph[num].Cuc = R8(buff + 28 + 80);
    eph[num].Cus = R8(buff + 28 + 88);
    eph[num].iDot = R8(buff + 28 + 136);
    eph[num].DeltaN = R8(buff + 28 + 48);
    eph[num].OMEGADot = R8(buff + 28 + 152);
    eph[num].e= R8(buff + 28 + 64);
    eph[num].i0 = R8(buff + 28 + 128);
    eph[num].M0 = R8(buff + 28 + 56);
    eph[num].OMEGA = R8(buff + 28 + 72);
    eph[num].omega = R8(buff + 28 + 144);
    eph[num].SqrtA = sqrt(R8(buff + 28 + 40));

    return 0;
}
int decode_bdsephem(unsigned char* buff, GPSEPHREC* eph)
{
    int prn, num;
    prn = UL4(buff + 28);
    num = prn - 1;
    eph[num].PRN = prn;
    eph[num].Sys = BDS;
    eph[num].TOE.Week = UL4(buff + 28 + 4) + 1356;
    eph[num].TOC.Week = UL4(buff + 28 + 4) + 1356;//与GPS开始时间对齐
    eph[num].TOE.SecOfWeek = UL4(buff + 28 + 72);
    eph[num].TOC.SecOfWeek = UL4(buff + 28 + 40);
    eph[num].TGD1 = R8(buff + 28 + 20);
    eph[num].TGD2 = R8(buff + 28 + 28);
    eph[num].ClkBias = R8(buff + 28 + 44);
    eph[num].ClkDrift = R8(buff + 28 + 52);
    eph[num].ClkDriftRate = R8(buff + 28 + 60);
    eph[num].Cic = R8(buff + 28 + 180);
    eph[num].Cis = R8(buff + 28 + 188);
    eph[num].Crc = R8(buff + 28 + 164);
    eph[num].Crs = R8(buff + 28 + 172);
    eph[num].Cuc = R8(buff + 28 + 148);
    eph[num].Cus = R8(buff + 28 + 156);
    eph[num].iDot = R8(buff + 28 + 140);
    eph[num].DeltaN = R8(buff + 28 + 100);
    eph[num].OMEGADot = R8(buff + 28 + 124);
    eph[num].e = R8(buff + 28 + 84);
    eph[num].i0 = R8(buff + 28 + 132);
    eph[num].M0 = R8(buff + 28 + 108);
    eph[num].OMEGA = R8(buff + 28 + 92);
    eph[num].omega = R8(buff + 28 + 116);
    eph[num].SqrtA = R8(buff + 28 + 76);
    return 0;
}
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[])//有观测值返回1，无观测值返回0
{
    int i, MsgLen{}, MsgID;
    int Status = 0;
    // 找AA4412并解码
    for (i = 0; i < MAXNOVDLEN - 2; i = i + MsgLen)
    {
        if ((Buff[i] == 0xAA && Buff[i + 1] == 0x44 && Buff[i + 2] == 0x12) == false)
        {
            MsgLen = 3;
            continue;
        }
        if ((i + 28) >= MAXNOVDLEN) break;
        MsgID = US2(Buff + i + 4);
        MsgLen = US2(Buff + i + 8) + 32;

        if ((i + MsgLen) >= MAXNOVDLEN) break;
        unsigned int crc = crc32(Buff + i, MsgLen - 4);
        switch (MsgID)
        {
        case 43:
            Status = decode_rangeb_oem7(Buff + i, obs);
            Len = 0;
            for (i = i + MsgLen; i < MAXNOVDLEN; i++)
            {
                Buff[Len] = Buff[i];
                Len++;
            }
            return Status;
        case 7:
            decode_gpsephem(Buff + i, geph);

            break;
        case 1696:
            decode_bdsephem(Buff + i, beph);

            break;
            //case 1336:
               //	DecodeQZSSEph()
        case 42:
            //DecodePSRPOS(buff + i, Pos);
            break;
        default:
            break;
        }
    }

    //缓冲区内容读取完毕后将剩余未读取内容移至缓冲区头部
    Len = 0;
    for (; i < MAXNOVDLEN; i++)
    {
        Buff[Len] = Buff[i];
        Len++;
    }
    return Status;
}
int DecodeNovOem7Soc(unsigned char Buff[], int& Len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[])
{
    int i, MsgLen{}, MsgID;
    int Status = 0;
    int DecodeLen = Len;
    // 找AA4412并解码
    for (i = 0; i < DecodeLen - 2; i = i + MsgLen)
    {
        if ((Buff[i] == 0xAA && Buff[i + 1] == 0x44 && Buff[i + 2] == 0x12) == false)
        {
            MsgLen = 3;
            continue;
        }
        if ((i + 28) >= DecodeLen) break;
        MsgID = US2(Buff + i + 4);
        MsgLen = US2(Buff + i + 8) + 32;

        if ((i + MsgLen) >= DecodeLen) break;
        unsigned int crc = crc32(Buff + i, MsgLen - 4);
        switch (MsgID)
        {
        case 43:
            Status = decode_rangeb_oem7(Buff + i, obs);
            break;
        case 7:
            decode_gpsephem(Buff + i, geph);
            break;
        case 1696:
            decode_bdsephem(Buff + i, beph);
            break;
            //case 1336:
               //	DecodeQZSSEph()
        case 42:
            //DecodePSRPOS(buff + i, Pos);
            break;
        default:
            break;
        }
    }

    //缓冲区内容读取完毕后将剩余未读取内容移至缓冲区头部
    Len = 0;
    for (; i < DecodeLen; i++)
    {
        Buff[Len] = Buff[i];
        Len++;
    }
    return Status;
}
// 广播星历
bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATMIDRES* Mid)
{
    if (fabs(t->SecOfWeek) > 7200)
    {
        return false;
    }
    GPSEPHREC EPH;
    if (Sys == GPS)
    {
        EPH = GPSEph[Prn - 1];
    }
    else if (Sys == BDS)
    {
        EPH = BDSEph[Prn - 1];
    }
    else
    {
        return false;
    }
    if (EPH.PRN == 0)
    {
        return false;
    }
    Mid->SatClkOft = EPH.ClkBias + EPH.ClkDrift*(t->SecOfWeek)+EPH.ClkDriftRate*pow((t->SecOfWeek),2)-EPH.TGD1;
    Mid->SatClkSft = EPH.ClkDrift + 2 * EPH.ClkDriftRate * (t->SecOfWeek);
    return true;
}
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
    double tk, A;
    //tk = t_gps.SecOfWeek - Geph.toe.SecOfWeek + 604800 * (t_gps.Week - Geph.toe.Week);
    tk = t->SecOfWeek - Eph[Prn - 1].TOE.SecOfWeek + 604800 * (t->Week - Eph[Prn - 1].TOE.Week);
    double n,n0,Mk, Ek0, Ek, vk, e, Phik, Omek, duk, drk, dik;
    double uk, rk, ik, xk1, yk1, xk, yk, zk;
    int iteration;
    //卫星位置
    A = pow(Eph[Prn - 1].SqrtA, 2);
    //平均运动角速度n0
    n0 = sqrt(GM_Earth / pow(A, 3));
    //平均运动角速度n改正
    n = n0 + Eph[Prn - 1].DeltaN;
    //平近点角Mk
    Mk = Eph[Prn - 1].M0 + n * tk;
    //偏近点角Ek迭代计算
    Ek0 = Mk;
    iteration = 0;
    while (true)
    {
        Ek = Mk + Eph[Prn - 1].e * sin(Ek0);
        if (abs(Ek - Ek0) < 1 / R_WGS84)
        {
            break;
        }
        Ek0 = Ek;
        iteration++;
        if (iteration > 100)
        {
            iteration = 0;
            break;
        }
    }
    //真近点角
    e = Eph[Prn - 1].e;
    vk = atan2((sqrt(1 - e * e) * sin(Ek)) / (1 - e * cos(Ek)), (cos(Ek) - e) / (1 - e * cos(Ek)));
    //升交角距phik
    Phik = vk + Eph[Prn - 1].OMEGA;
    //升交角距改正数
    duk = Eph[Prn - 1].Cus * sin(2 * Phik) + Eph[Prn - 1].Cuc * cos(2 * Phik);
    //向径改正数
    drk = Eph[Prn - 1].Crs * sin(2 * Phik) + Eph[Prn - 1].Crc * cos(2 * Phik);
    //轨道倾角改正数
    dik = Eph[Prn - 1].Cis * sin(2 * Phik) + Eph[Prn - 1].Cic * cos(2 * Phik);
    //升交角距改正
    uk = Phik + duk;
    //向径改正
    rk = A * (1 - e * cos(Ek)) + drk;
    //轨道倾角改正
    ik = Eph[Prn - 1].i0 + dik + (Eph[Prn - 1].iDot) * tk;
    //计算卫星在轨道面上的位置
    xk1 = rk * cos(uk);
    yk1 = rk * sin(uk);
    //升交点经度改正
    Omek = Eph[Prn - 1].omega + (Eph[Prn - 1].OMEGADot - Omega_WGS) * tk - Omega_WGS * Eph[Prn - 1].TOE.SecOfWeek;
    //计算地固坐标系下位置
    xk = xk1 * cos(Omek) - yk1 * cos(ik) * sin(Omek);
    yk = xk1 * sin(Omek) + yk1 * cos(ik) * cos(Omek);
    zk = yk1 * sin(ik);

    Mid->SatPos[0] = xk;
    Mid->SatPos[1] = yk;
    Mid->SatPos[2] = zk;



    //卫星速度计算
    double  v_Ek, v_Phik, v_Omek;
    double v_uk, v_rk, v_ik, v_xk1, v_yk1;
    v_Ek = n / (1 - e * cos(Ek));
    v_Phik = sqrt((1 + e) / (1 - e)) * pow(cos(vk / 2) / cos(Ek / 2), 2) * v_Ek;
    v_uk = 2 * (Eph[Prn-1].Cus * cos(2 * Phik) - Eph[Prn - 1].Cuc * sin(2 * Phik)) * v_Phik + v_Phik;
    v_rk = A * e * sin(Ek) * v_Ek + 2 * (Eph[Prn - 1].Crs * cos(2 * Phik) - Eph[Prn - 1].Crc * sin(2 * Phik)) * v_Phik;
    v_ik = Eph[Prn - 1].iDot + 2 * (Eph[Prn - 1].Cis * cos(2 * Phik) - Eph[Prn - 1].Cic * sin(2 * Phik)) * v_Phik;
    v_Omek = Eph[Prn - 1].OMEGADot - Omega_WGS;
    v_xk1 = v_rk * cos(uk) - rk * v_uk * sin(uk);
    v_yk1 = v_rk * sin(uk) + rk * v_uk * cos(uk);
    double v_R[12] = { cos(Omek),-sin(Omek) * cos(ik),-(xk1 * sin(Omek) + yk1 * cos(Omek) * cos(ik)),  yk1 * sin(Omek) * sin(ik),
                       sin(Omek), cos(Omek) * cos(ik),  xk1 * cos(Omek) - yk1 * sin(Omek) * cos(ik) , -yk1 * cos(Omek) * sin(ik),
                       0        ,           sin(ik)  ,                                             0,              yk1 * cos(ik) };
    double mid_res[4] = { v_xk1,v_yk1,v_Omek,v_ik };
    matrix_multiply(3, 4, 4, 1, v_R, mid_res, Mid->SatVel);

    Mid->SatClkOft = Mid->SatClkOft + (-2 * sqrt(GM_Earth) / pow(C_Light, 2)) * e * sqrt(A) * sin(Ek);
    Mid->SatClkSft = Mid->SatClkSft + (-2 * sqrt(GM_Earth) / pow(C_Light, 2)) * e * sqrt(A) * cos(Ek) * v_Ek;


    return true;
}
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
    double tk, A;

    //tk = t_gps.SecOfWeek - Geph.toe.SecOfWeek + 604800 * (t_gps.Week - Geph.toe.Week);
    tk = t->SecOfWeek - Eph[Prn - 1].TOE.SecOfWeek + 604800 * (t->Week - Eph[Prn - 1].TOE.Week);
    double n, n0, Mk, Ek0, Ek, vk, e, Phik, Omek, duk, drk, dik;
    double uk, rk, ik, xk1, yk1, xk, yk, zk;
    int iteration;
    //卫星位置
    A = pow(Eph[Prn - 1].SqrtA, 2);
    //平均运动角速度n0
    n0 = sqrt(GM_BDS / pow(A, 3));
    //平均运动角速度n改正
    n = n0 + Eph[Prn - 1].DeltaN;
    //平近点角Mk
    Mk = Eph[Prn - 1].M0 + n * tk;
    //偏近点角Ek迭代计算
    Ek0 = Mk;
    iteration = 0;
    while (true)
    {
        Ek = Mk + Eph[Prn - 1].e * sin(Ek0);
        if (abs(Ek - Ek0) < 1 / R_CGS2K)
        {
            break;
        }
        Ek0 = Ek;
        iteration++;
        if (iteration > 100)
        {
            iteration = 0;
            break;
        }
    }
    //真近点角
    e = Eph[Prn - 1].e;
    vk = atan2((sqrt(1 - e * e) * sin(Ek)) / (1 - e * cos(Ek)), (cos(Ek) - e) / (1 - e * cos(Ek)));
    //升交角距phik
    Phik = vk + Eph[Prn - 1].OMEGA;
    //升交角距改正数
    duk = Eph[Prn - 1].Cus * sin(2 * Phik) + Eph[Prn - 1].Cuc * cos(2 * Phik);
    //向径改正数
    drk = Eph[Prn - 1].Crs * sin(2 * Phik) + Eph[Prn - 1].Crc * cos(2 * Phik);
    //轨道倾角改正数
    dik = Eph[Prn - 1].Cis * sin(2 * Phik) + Eph[Prn - 1].Cic * cos(2 * Phik);
    //升交角距改正
    uk = Phik + duk;
    //向径改正
    rk = A * (1 - e * cos(Ek)) + drk;
    //轨道倾角改正
    ik = Eph[Prn - 1].i0 + dik + (Eph[Prn - 1].iDot) * tk;
    //计算卫星在轨道面上的位置
    xk1 = rk * cos(uk);
    yk1 = rk * sin(uk);
    if (Eph[Prn-1].i0 < (30 * Rad))
    {
        //GEO卫星
        //升交点经度改正
        Omek = Eph[Prn-1].omega + Eph[Prn - 1].OMEGADot * tk - Omega_BDS * Eph[Prn - 1].TOE.SecOfWeek;
        //计算自定义坐标系下位置
        xk = xk1 * cos(Omek) - yk1 * cos(ik) * sin(Omek);
        yk = xk1 * sin(Omek) + yk1 * cos(ik) * cos(Omek);
        zk = yk1 * sin(ik);
        //坐标转换至BDCS坐标系
        double Rx_matrix[9] = { 1,             0,            0,
                                0, cos(-5 * Rad),sin(-5 * Rad),
                                0,-sin(-5 * Rad),cos(-5 * Rad) };
        double Rz_matrix[9] = { cos(Omega_BDS * tk),sin(Omega_BDS * tk),0,
                               -sin(Omega_BDS * tk),cos(Omega_BDS * tk),0,
                                  0                ,0                  ,1 };
        double Result1[9];
        double res[3] = { xk,yk,zk };
        matrix_multiply(3, 3, 3, 3, Rz_matrix, Rx_matrix, Result1);
        matrix_multiply(3, 3, 3, 1, Result1, res, Mid->SatPos);
    }
    else
    {
        //MEO/IGSO卫星
        //升交点经度改正 (gps:Omek = Geph.Ω0 + (Geph.dΩ - GPS_ω) * tk-GPS_ω * Geph.toe.SecOfWeek);
        Omek = Eph[Prn - 1].omega + (Eph[Prn - 1].OMEGADot - Omega_BDS) * tk - Omega_BDS * Eph[Prn - 1].TOE.SecOfWeek;
        //卫星位置
        xk = xk1 * cos(Omek) - yk1 * cos(ik) * sin(Omek);
        yk = xk1 * sin(Omek) + yk1 * cos(ik) * cos(Omek);
        zk = yk1 * sin(ik);
        Mid->SatPos[0] = xk;
        Mid->SatPos[1] = yk;
        Mid->SatPos[2] = zk;
    }



    //卫星速度计算
    double  v_Ek, v_Phik, v_Omek;
    double v_uk, v_rk, v_ik, v_xk1, v_yk1;
    v_Ek = n / (1 - e * cos(Ek));
    v_Phik = sqrt((1 + e) / (1 - e)) * pow(cos(vk / 2) / cos(Ek / 2), 2) * v_Ek;
    v_uk = 2 * (Eph[Prn - 1].Cus * cos(2 * Phik) - Eph[Prn - 1].Cuc * sin(2 * Phik)) * v_Phik + v_Phik;
    v_rk = A * e * sin(Ek) * v_Ek + 2 * (Eph[Prn - 1].Crs * cos(2 * Phik) - Eph[Prn - 1].Crc * sin(2 * Phik)) * v_Phik;
    v_ik = Eph[Prn - 1].iDot + 2 * (Eph[Prn - 1].Cis * cos(2 * Phik) - Eph[Prn - 1].Cic * sin(2 * Phik)) * v_Phik;
    v_Omek = Eph[Prn - 1].OMEGADot - Omega_BDS;
    v_xk1 = v_rk * cos(uk) - rk * v_uk * sin(uk);
    v_yk1 = v_rk * sin(uk) + rk * v_uk * cos(uk);
    double v_R[12] = { cos(Omek),-sin(Omek) * cos(ik),-(xk1 * sin(Omek) + yk1 * cos(Omek) * cos(ik)),  yk1 * sin(Omek) * sin(ik),
                       sin(Omek), cos(Omek) * cos(ik),  xk1 * cos(Omek) - yk1 * sin(Omek) * cos(ik) , -yk1 * cos(Omek) * sin(ik),
                       0        ,           sin(ik)  ,                                             0,              yk1 * cos(ik) };
    double mid_res[4] = { v_xk1,v_yk1,v_Omek,v_ik };
    matrix_multiply(3, 4, 4, 1, v_R, mid_res, Mid->SatVel);

    Mid->SatClkOft = Mid->SatClkOft + (-2 * sqrt(GM_BDS) / pow(C_Light, 2)) * e * sqrt(A) * sin(Ek);
    Mid->SatClkSft = Mid->SatClkSft + (-2 * sqrt(GM_BDS) / pow(C_Light, 2)) * e * sqrt(A) * cos(Ek) * v_Ek;


    return true;
}
void ComputeGPSSatOrbitAtSignalTrans(const EPOCHOBS* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double RcvPos[], SATMIDRES* MidRes)
{
    double RcvBLH[3]{ 0 };
    for (int i = 0; i < Epk->SatNum; i++)
    {
        double PIF = Epk->ComObs[i].PIF;
        int Num = Epk->SatObs[i].Prn-1;
        GPSTIME GT = Epk->Time;
        GPSTIME DTC,TK;
        if (Epk->SatObs[i].System == GPS)
        {
            GT.SecOfWeek = GT.SecOfWeek - PIF / C_Light;

            DTC.SecOfWeek = (GT.Week - GPSEph[Num].TOC.Week) * 604800 + GT.SecOfWeek - GPSEph[Num].TOC.SecOfWeek;
        }
        else if (Epk->SatObs[i].System == BDS)
        {
            GT.SecOfWeek = GT.SecOfWeek-GPST_BDT- PIF / C_Light;

            DTC.SecOfWeek = (GT.Week - BDSEph[Num].TOC.Week) * 604800 + GT.SecOfWeek - BDSEph[Num].TOC.SecOfWeek;
        }
        else
        {
            continue;
        }
        MidRes[i].Valid = CompSatClkOff(Epk->SatObs[i].Prn, Epk->SatObs[i].System, &DTC, GPSEph, BDSEph, &MidRes[+i]);
        GT.SecOfWeek = GT.SecOfWeek - MidRes[i].SatClkOft;
        if (MidRes[i].Valid == true && Epk->SatObs[i].Valid==true && Epk->SatObs[i].System == GPS)
        {
            CompGPSSatPVT(Epk->SatObs[i].Prn, &GT, GPSEph, MidRes+i);
            double alpha, dt;
            double R = sqrt(pow(RcvPos[0] - MidRes[i].SatPos[0], 2) + pow(RcvPos[1] - MidRes[i].SatPos[1], 2) + pow(RcvPos[2] - MidRes[i].SatPos[2], 2));
            dt = R / C_Light;
            alpha = Omega_WGS * dt;
            double R_z[9] = { cos(alpha),sin(alpha),0,
                             -sin(alpha),cos(alpha),0,
                                       0,         0,1 };
            matrix_multiply(3, 3, 3, 1, R_z, MidRes[i].SatPos, MidRes[i].SatPos);
            matrix_multiply(3, 3, 3, 1, R_z, MidRes[i].SatVel, MidRes[i].SatVel);
            CompSatElAz(RcvPos, MidRes[i].SatPos, &MidRes[i].Elevation, &MidRes[i].Azimuth);
            XYZToBLH(RcvPos, RcvBLH, R_WGS84, F_WGS84);
            MidRes[i].TropCorr = hopfield(RcvBLH[2], MidRes[i].Elevation);
            //printf("G%d\tX:%f\tY:%f\tZ:%f\tClk:%fE-5\tVX:%f\tVY:%f\tVZ:%f\tClkd:%fE-12\tPIF:%f\tTrop:%f\tElev:%f\n",
            //	Num + 1,
            //	MidRes[i].SatPos[0], MidRes[i].SatPos[1], MidRes[i].SatPos[2], MidRes[i].SatClkOft * 1e5,
            //	MidRes[i].SatVel[0], MidRes[i].SatVel[1], MidRes[i].SatVel[2], MidRes[i].SatClkSft * 1e12,
            //	PIF, MidRes[i].TropCorr,MidRes[i].Elevation*Deg);
        }
        else if (MidRes[i].Valid == true && Epk->SatObs[i].Valid == true && Epk->SatObs[i].System == BDS)
        {
            CompBDSSatPVT(Epk->SatObs[i].Prn, &GT, BDSEph, MidRes+i);
            double alpha, dt;
            double R = sqrt(pow(RcvPos[0] - MidRes[i].SatPos[0], 2) + pow(RcvPos[1] - MidRes[i].SatPos[1], 2) + pow(RcvPos[2] - MidRes[i].SatPos[2], 2));
            dt = R / C_Light;
            alpha = Omega_BDS * dt;
            double R_z[9] = { cos(alpha),sin(alpha),0,
                             -sin(alpha),cos(alpha),0,
                                       0,         0,1 };
            matrix_multiply(3, 3, 3, 1, R_z, MidRes[i].SatPos, MidRes[i].SatPos);
            matrix_multiply(3, 3, 3, 1, R_z, MidRes[i].SatVel, MidRes[i].SatVel);
            CompSatElAz(RcvPos, MidRes[i].SatPos, &MidRes[i].Elevation, &MidRes[i].Azimuth);
            XYZToBLH(RcvPos, RcvBLH, R_CGS2K, F_CGS2K);
            MidRes[i].TropCorr = hopfield(RcvPos[2], MidRes[i].Elevation);
            //printf("C%d\tX:%15.5f\tY:%15.5f\tZ:%15.5f\tClk:13.5%E\tVX:%15.5f\tVY:%15.5f\tVZ:%15.5f\tClkd:%15.5E\tPIF:%15.5f\tTrop:%15.5f\tElev:%15.5f\n",
            //    Num + 1,
            //    MidRes[i].SatPos[0], MidRes[i].SatPos[1], MidRes[i].SatPos[2], MidRes[i].SatClkOft,
            //    MidRes[i].SatVel[0], MidRes[i].SatVel[1], MidRes[i].SatVel[2], MidRes[i].SatClkSft,
            //    PIF, MidRes[i].TropCorr, MidRes[i].Elevation * Deg);
        }
    }

}
double hopfield(double hgt, double elev)
{
    //elev = 58.763 * Rad;
    //标准气象元素
    const double H0 = 0.0;//海平面
    const double T0 = 15 + 273.16;//温度
    const double p0 = 1013.25;//气压 mbar
    const double RH0 = 0.5;//相对湿度
    if (hgt < -35000 || hgt>18000)
    {
        return 0;//高度异常
    }
    //Hopefield模型
    double RH, p, T, Hf_e, h_w, h_d, K_w, K_d, d_Trop;
    RH = RH0 * exp(-0.0006396 * (hgt - H0));
    p = p0 * pow((1 - 0.0000226 * (hgt - H0)), 5.225);
    T = T0 - 0.0065 * (hgt - H0);
    Hf_e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
    h_w = 11000;
    h_d = 40136 + 148.72 * (T0 - 273.16);
    K_w = 155.2e-7 * 4810 / pow(T, 2) * Hf_e * (h_w - hgt);
    K_d = 155.2e-7 * p / T * (h_d - hgt);
    d_Trop = K_d / sin((sqrt(pow(elev * Deg, 2) + 6.25)) * Rad) + K_w / sin((sqrt(pow(elev * Deg, 2) + 2.25)* Rad));//高度角单位为角度，计算三角函数时转换为弧度
    return d_Trop;
}

//SPP & SPV
void DetectOutlier(EPOCHOBS* Obs)
{
    MWGF ComObs[MAXCHANNUM];
    memset(ComObs, 0, MAXCHANNUM * sizeof(MWGF));
    for (int i = 0; i < Obs->SatNum; i++)
    {
        double FG1, FG2;
        FG1 = FG2 = 0.0;
        int SatIndex = -1;
        double MW_ave = 0;
        //数据完整性
        if (abs(Obs->SatObs[i].Psr[0]) < 1e-5 || abs(Obs->SatObs[i].Psr[1]) < 1e-5 || abs(Obs->SatObs[i].Adr[0]) < 1e-5 || abs(Obs->SatObs[i].Adr[1]) < 1e-5)
        {
            Obs->SatObs[i].Valid = false;
            continue;
        }
        ComObs[i].Prn = Obs->SatObs[i].Prn;
        ComObs[i].Sys = Obs->SatObs[i].System;
        if (ComObs[i].Sys == GPS)
        {
            FG1 = FG1_GPS;
            FG2 = FG2_GPS;
        }
        else if (ComObs[i].Sys == BDS)
        {
            FG1 = FG1_BDS;
            FG2 = FG3_BDS;
        }
        else
        {
            Obs->SatObs[i].Valid = false;
            continue;
        }
        ComObs[i].n = 1;
        ComObs[i].GF = C_Light * (Obs->SatObs[i].Adr[0] / FG1) - C_Light * (Obs->SatObs[i].Adr[1] / FG2);

        ComObs[i].MW = (C_Light * (Obs->SatObs[i].Adr[0] - Obs->SatObs[i].Adr[1])) / (FG1 - FG2) -
                       (Obs->SatObs[i].Psr[0] * FG1 + Obs->SatObs[i].Psr[1] * FG2) / (FG1 + FG2);

        if (PrnSearching(ComObs[i].Prn, Obs, ComObs[i].Sys, &SatIndex))
        {
            if (Obs->ComObs[SatIndex].n == 0)
            {
                Obs->SatObs[SatIndex].Valid = true;
                memcpy(&Obs->ComObs[SatIndex], &ComObs[i], sizeof(MWGF));              
            }
            else 
            {
                double D_MW = Obs->ComObs[SatIndex].MW - ComObs[i].MW;
                double D_GF = Obs->ComObs[SatIndex].GF - ComObs[i].GF;

                if (abs(D_GF) >= 0.05 || abs(D_MW) >= 5)
                {
                    Obs->SatObs[SatIndex].Valid = false;
                    Obs->ComObs[SatIndex].n = 0;
                }
                else
                {
                    Obs->ComObs[SatIndex].n += 1;
                    MW_ave = ((Obs->ComObs[SatIndex].n - 1) * Obs->ComObs[SatIndex].MW + ComObs[i].MW) / Obs->ComObs[SatIndex].n;//计算平滑值
                    Obs->ComObs[SatIndex].GF = ComObs[i].GF;
                    Obs->ComObs[SatIndex].MW = MW_ave;
                    Obs->SatObs[SatIndex].Valid = true;
                }
            }
        }

        if (Obs->SatObs[SatIndex].Valid == true)
        {
            Obs->ComObs[SatIndex].PIF = (pow(FG1, 2) * Obs->SatObs[i].Psr[0] - pow(FG2, 2) * Obs->SatObs[i].Psr[1]) / (pow(FG1, 2) - pow(FG2, 2));
        }
    }
}
bool SPP(EPOCHOBS* Epoch, RAWDAT* Raw, PPRESULT* Result)
{
    double delta[5]={0,0,0,0,0};		   //最小二乘的待估参数
    double B[5 * MAXCHANNUM]{0};           //系数矩阵
    double L[MAXCHANNUM]{ 0 }; 
    double M[MAXCHANNUM]{ 0 };
    double N[MAXCHANNUM]{ 0 };
    double W[MAXCHANNUM]{ 0 };
    double R;								//卫星到接收机的距离
    double BT[5 * MAXCHANNUM];				//B矩阵的转置
    double BTB[25];							//BT*B
    double BTBinv[25];						//BT*B的转置矩阵
    double BTL[5];							//BT*L
    double V[MAXCHANNUM];					//残差矩阵
    double VT[MAXCHANNUM];				    //残差矩阵的转置
    double BX[MAXCHANNUM];			    	//B*delta
    double VTV[1];							//VT*V
    int GPSSATNUM ;                         //可用GPS卫星数
    int BDSSATNUM ;                         //可用BDS卫星数
    int iteration=0;
    int j;
    int row;
    //设置初始位置
    //static double RcvPos[3] = { -2267802.8844 ,5009341.3028 , 3220991.8127 };
    static double RcvPos[3] = { -2267809,5009323,3221015 };
    //double RcvPos[3] = { -2267802.8844 ,5009341.3028 , 3220991.8127 };
    do
    {
        GPSSATNUM = 0;
        BDSSATNUM = 0;


        Result->Position[0] = RcvPos[0] + delta[0];
        Result->Position[1] = RcvPos[1] + delta[1];
        Result->Position[2] = RcvPos[2] + delta[2];
        Result->RcvClkOft[0] = Result->RcvClkOft[0] + delta[3];
        Result->RcvClkOft[1] = Result->RcvClkOft[1] + delta[4];
        
        //卫星位置计算
        ComputeGPSSatOrbitAtSignalTrans(Epoch, Raw->GpsEph, Raw->BdsEph, Result->Position, Epoch->SatPVT);

        //观测方程线性化
        j = 0;
        for (int i = 0; i < Epoch->SatNum; i++)
        {
            if (Epoch->SatPVT[i].Valid == false || Epoch->SatObs[i].Valid == false) continue;
            if (Epoch->SatPVT[i].Elevation < 10 * Rad) continue;   //舍弃高度角过低的卫星
            //if (((Epoch->SatObs[i].Prn >= 1 && Epoch->SatObs[i].Prn <= 5) || (Epoch->SatObs[i].Prn >= 59 && Epoch->SatObs[i].Prn <= 63))&&
            //    Epoch->SatObs[i].System==BDS) continue;//暂时跳过BDS GEO卫星,BUG探测
            if (Epoch->SatObs[i].System == GPS)
            {
                R = sqrt(pow(Result->Position[0] - Epoch->SatPVT[i].SatPos[0], 2) + pow(Result->Position[1] - Epoch->SatPVT[i].SatPos[1], 2) + pow(Result->Position[2] - Epoch->SatPVT[i].SatPos[2], 2));
                L[j] = (Result->Position[0] - Epoch->SatPVT[i].SatPos[0]) / R;
                M[j] = (Result->Position[1] - Epoch->SatPVT[i].SatPos[1]) / R;
                N[j] = (Result->Position[2] - Epoch->SatPVT[i].SatPos[2]) / R;             
                W[j] = Epoch->ComObs[i].PIF - (R + Result->RcvClkOft[0] - Epoch->SatPVT[i].SatClkOft * C_Light + Epoch->SatPVT[i].TropCorr);
                GPSSATNUM += 1;
                j++;
            }
            else if (Epoch->SatObs[i].System == BDS)
            {
                R = sqrt(pow(Result->Position[0] - Epoch->SatPVT[i].SatPos[0], 2) + pow(Result->Position[1] - Epoch->SatPVT[i].SatPos[1], 2) + pow(Result->Position[2] - Epoch->SatPVT[i].SatPos[2], 2));
                L[j] = (Result->Position[0] - Epoch->SatPVT[i].SatPos[0]) / R;
                M[j] = (Result->Position[1] - Epoch->SatPVT[i].SatPos[1]) / R;
                N[j] = (Result->Position[2] - Epoch->SatPVT[i].SatPos[2]) / R;
                W[j] = Epoch->ComObs[i].PIF - (R + Result->RcvClkOft[1] - Epoch->SatPVT[i].SatClkOft * C_Light + Epoch->SatPVT[i].TropCorr);
                BDSSATNUM += 1;
                j++;
            }
            else continue;
        }

        if (j < 5) return false;//可用卫星数量少于5，不解算
        
        if (BDSSATNUM == 0&&GPSSATNUM!=0)
        {
            row = 4;
            for (int i = 0; i < GPSSATNUM; i++)
            {
                B[i * row + 0] = L[i];
                B[i * row + 1] = M[i];
                B[i * row + 2] = N[i];
                B[i * row + 3] = 1;
            }
        }
        else if (GPSSATNUM == 0 && BDSSATNUM != 0)
        {
            row = 4;
            for (int i = 0; i < GPSSATNUM; i++)
            {
                B[i * row + 0] = L[i];
                B[i * row + 1] = M[i];
                B[i * row + 2] = N[i];
                B[i * row + 3] = 1;
            }
        }
        else if (GPSSATNUM != 0 && BDSSATNUM != 0)
        {
            int i;
            row = 5;
            for (i = 0; i < GPSSATNUM; i++)
            {
                B[i * row + 0] = L[i];
                B[i * row + 1] = M[i];
                B[i * row + 2] = N[i];
                B[i * row + 3] = 1;
                B[i * row + 4] = 0;
            }
            for (; i < GPSSATNUM + BDSSATNUM; i++)
            {
                B[i * row + 0] = L[i];
                B[i * row + 1] = M[i];
                B[i * row + 2] = N[i];
                B[i * row + 3] = 0;
                B[i * row + 4] = 1;
            }
        }//系数矩阵

        matrix_transfer(j, row, B, BT);
        matrix_multiply(row, j, j, row, BT, B, BTB);
        matrix_Inv(row, BTB, BTBinv);
        matrix_multiply(row, j, j, 1, BT, W, BTL);
        matrix_multiply(row, row, row, 1, BTBinv, BTL, delta);
        iteration++;
    
    } while(sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]) > 1e-2 && iteration<10);
    
    Result->Position[0] = Result->Position[0] + delta[0];
    Result->Position[1] = Result->Position[1] + delta[1];
    Result->Position[2] = Result->Position[2] + delta[2];
    Result->RcvClkOft[0] = Result->RcvClkOft[0] + delta[3];
    Result->RcvClkOft[1] = Result->RcvClkOft[1] + delta[4];
    Result->BDSSatNum = BDSSATNUM;
    Result->GPSSatNum = GPSSATNUM;
    Result->AllSatNum = j;
    Result->Time = Epoch->Time;
    


    //精度评定
    matrix_multiply(j, row, row, 1, B, delta, BX);
    matrix_minus(j, 1, j, 1, BX, W, V);
    matrix_transfer(j, 1, V, VT);
    matrix_multiply(1, j, j, 1, VT, V, VTV);
    Result->SigmaPos = sqrt(VTV[0] / (j - 4));
    Result->PDOP = sqrt(BTBinv[0] + BTBinv[row + 1] + BTBinv[2 * (row + 1)]);

    if (Result->SigmaPos < 2.5)
    {
        RcvPos[0] = Result->Position[0];
        RcvPos[1] = Result->Position[1];
        RcvPos[2] = Result->Position[2];
    }
    return true;
}
void SPV(EPOCHOBS* Epoch, PPRESULT* Result)
{
    double delta[4] = { 0,0,0,0};		   //最小二乘的待估参数
    double B[4 * MAXCHANNUM]{ 0 };           //系数矩阵
    double L[MAXCHANNUM]{ 0 };
    double M[MAXCHANNUM]{ 0 };
    double N[MAXCHANNUM]{ 0 };
    double W[MAXCHANNUM]{ 0 };
    double tropDelay = 0;					//对流层延迟（米）
    double R;								//卫星到接收机的距离
    double BT[4 * MAXCHANNUM];				//B矩阵的转置
    double BTB[16];							//BT*B
    double BTBinv[16];						//BT*B的转置矩阵
    double BTL[4];							//BT*L
    double V[MAXCHANNUM];					//残差矩阵
    double VT[MAXCHANNUM];				    //残差矩阵的转置
    double BX[MAXCHANNUM];			    	//B*delta
    double VTV[1];							//VT*V
    double Dopper;
    double dR;
    int GPSSATNUM;                      //可用GPS卫星数
    int j;
    int row;
    GPSSATNUM = Result->GPSSatNum;

    //观测方程线性化
    j = 0; row = 0;
    for (int i = 0; i < Epoch->SatNum; i++)
    {
        if (Epoch->SatPVT[i].Valid == false || Epoch->SatObs[i].Valid == false) continue;
        if (Epoch->SatPVT[i].Elevation < 10 * Rad) continue;   //舍弃高度角过低的卫星
        if (Epoch->SatObs[i].System == GPS)
        {
            R = sqrt(pow(Result->Position[0] - Epoch->SatPVT[i].SatPos[0], 2) + pow(Result->Position[1] - Epoch->SatPVT[i].SatPos[1], 2) + pow(Result->Position[2] - Epoch->SatPVT[i].SatPos[2], 2));
            L[j] = (Result->Position[0] - Epoch->SatPVT[i].SatPos[0]) / R;
            M[j] = (Result->Position[1] - Epoch->SatPVT[i].SatPos[1]) / R;
            N[j] = (Result->Position[2] - Epoch->SatPVT[i].SatPos[2]) / R;
            Dopper = Epoch->SatObs[i].Dopper[0] * C_Light / FG1_GPS;
            dR = -L[j] * Epoch->SatPVT[i].SatVel[0] - M[j] * Epoch->SatPVT[i].SatVel[1] - N[j] * Epoch->SatPVT[i].SatVel[2];
            W[j] = Dopper - (dR - C_Light * Epoch->SatPVT[i].SatClkSft);
            j++;
        }
        else continue;
    }

    row = 4;
    for (int i = 0; i < GPSSATNUM; i++)
    {
        B[i * row + 0] = L[i];
        B[i * row + 1] = M[i];
        B[i * row + 2] = N[i];
        B[i * row + 3] = 1;
    }//系数矩阵

    matrix_transfer(j, row, B, BT);
    matrix_multiply(row, j, j, row, BT, B, BTB);
    matrix_Inv(row, BTB, BTBinv);
    matrix_multiply(row, j, j, 1, BT, W, BTL);
    matrix_multiply(row, row, row, 1, BTBinv, BTL, delta);

    Result->Velocity[0] = delta[0];
    Result->Velocity[1] = delta[1];
    Result->Velocity[2] = delta[2];
    Result->RcvClkSft=delta[3];

    //精度评定
    matrix_multiply(j, row, row, 1, B, delta, BX);
    matrix_minus(j, 1, j, 1, BX, W, V);
    matrix_transfer(j, 1, V, VT);
    matrix_multiply(1, j, j, 1, VT, V, VTV);
    Result->SigmaVel = sqrt(VTV[0] / (j - 4));
}

//RTK
//时间同步 -1=文件结束,1=同步成功,0=解码未解到观测值
int GetSynObs(FILE* FBas, FILE* FRov, RAWDAT* Raw)
{
    double deltaT=0;
    int Status1 = 0;
    int Status2 = 0;
    static int LenRemBas=0;//剩余的字节
    static int LenRemRov=0;
    int LenReadRov;//已读取字节
    int LenReadBas;
    static unsigned char BuffBas[MAXNOVDLEN] = { 0 };
    static unsigned char BuffRov[MAXNOVDLEN] = { 0 };
    //读取一个历元流动站观测值
    do
    {
        Status1 = decode(&Raw->RovEpk, FRov, Raw, BuffRov, LenReadRov, LenRemRov);
        if (Status1 == -1) return -1;
    } while (Status1 != 1);
    //直接和基站求时间差
    deltaT = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek;
    
    //满足要求
    if (fabs(deltaT) <= DELTATIME) return 1;

    while (fabs(deltaT) > DELTATIME)
    {
        while (deltaT>0)//Rov时间大于Bas
        {
            do
            {
                Status2 = decode(&Raw->BasEpk, FBas, Raw, BuffBas, LenReadBas, LenRemBas);
                if (Status2 == -1) return -1;
            } while (Status2 != 1);

            deltaT = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek;
            if (fabs(deltaT) < DELTATIME) return 1;
        } 

        while (deltaT<0)//Rov时间小于Bas时间
        {
            do
            {
                Status1 = decode(&Raw->RovEpk, FRov, Raw, BuffRov, LenReadRov, LenRemRov);
                if (Status1 == -1) return -1;
            } while (Status1 != 1);

            deltaT = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek;
            if (fabs(deltaT) < DELTATIME) return 1;
        } 
    }
}

int GetSynObs(SOCKET& BasSock, SOCKET& RovSock, RAWDAT* Raw)
{
    double deltaT = 0;
    int Status1 = 0;
    int Status2 = 0;
    static int LenRemBas = 0;//剩余的字节
    static int LenRemRov = 0;
    int LenReadRov;//已读取字节
    int LenReadBas;
    static unsigned char BuffBas[MAXNOVDLEN] = { 0 };
    static unsigned char BuffRov[MAXNOVDLEN] = { 0 };
    //读取一个历元流动站观测值
    do
    {
        Status1 = decode(&Raw->RovEpk, RovSock, Raw, BuffRov, LenReadRov, LenRemRov);
        if (Status1 == -1) return -1;
    } while (Status1 != 1);
    //直接和基站求时间差
    deltaT = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek;

    //满足要求
    if (fabs(deltaT) <= DELTATIME) return 1;

    while (fabs(deltaT) > DELTATIME)
    {
        while (deltaT > 0)//Rov时间大于Bas
        {
            do
            {
                Status2 = decode(&Raw->BasEpk, BasSock, Raw, BuffBas, LenReadBas, LenRemBas);
                if (Status2 == -1) return -1;
            } while (Status2 != 1);

            deltaT = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek;
            if (fabs(deltaT) < DELTATIME) return 1;
        }

        while (deltaT < 0)//Rov时间小于Bas时间
        {
            do
            {
                Status1 = decode(&Raw->RovEpk, RovSock, Raw, BuffRov, LenReadRov, LenRemRov);
                if (Status1 == -1) return -1;
            } while (Status1 != 1);

            deltaT = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek;
            if (fabs(deltaT) < DELTATIME) return 1;
        }
    }
}
void FormSDEpochObs(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs)//EpkA=Rov,EpkB=Bas
{
    int  j=-1;
    int  SatNum = 0;
    bool Status = false;                       //Prn是否对齐，观测值完整，对齐成功且完整为true，否则为false
    int  SdSatObsIndex = 0;               
    SDObs->Time = EpkA->Time;
    for (int i = 0; i < EpkA->SatNum; i++)
    {
        if (EpkA->SatObs[i].Valid == false)continue;
        Status = PrnSearching(EpkA->SatObs[i].Prn, EpkB, EpkA->SatObs[i].System, & j);
        if (Status == true)
        {
            if      (EpkA->SatObs[i].System == GPS) { SDObs->SdSatObs[SdSatObsIndex].System = GPS; }
            else if (EpkA->SatObs[i].System == BDS) { SDObs->SdSatObs[SdSatObsIndex].System = BDS; }
            else continue;

            SDObs->SdSatObs[SdSatObsIndex].Prn = EpkA->SatObs[i].Prn;

            SDObs->SdSatObs[SdSatObsIndex].dP[0] = EpkA->SatObs[i].Psr[0] - EpkB->SatObs[j].Psr[0];
            SDObs->SdSatObs[SdSatObsIndex].dP[1] = EpkA->SatObs[i].Psr[1] - EpkB->SatObs[j].Psr[1];

            SDObs->SdSatObs[SdSatObsIndex].dL[0] = (EpkA->SatObs[i].Adr[0] - EpkB->SatObs[j].Adr[0]) ;
            SDObs->SdSatObs[SdSatObsIndex].dL[1] = (EpkA->SatObs[i].Adr[1] - EpkB->SatObs[j].Adr[1]) ;

            SatNum++;
            SdSatObsIndex++;
        }
    }
    SDObs->SatNum = SatNum;
}
void DetectCycleSlip(SDEPOCHOBS* Obs)
{
    for (int i = 0; i < Obs->SatNum; i++)
    {
        double MW, GF;
        double FG1, FG2;
        MW = GF = FG1 = FG2 = 0.0;
        //数据完整性
        if (abs(Obs->SdSatObs[i].dP[0]) < 1e-5 || abs(Obs->SdSatObs[i].dP[1]) < 1e-5 || abs(Obs->SdSatObs[i].dL[0]) < 1e-5 || abs(Obs->SdSatObs[i].dL[1]) < 1e-5)
        {
            Obs->SdSatObs[i].Valid = false;
            continue;
        }
        Obs->SdCObs[i].Prn = Obs->SdSatObs[i].Prn;
        Obs->SdCObs[i].Sys = Obs->SdSatObs[i].System;
        if (Obs->SdCObs[i].Sys == GPS)
        {
            FG1 = FG1_GPS;
            FG2 = FG2_GPS;
        }
        else if (Obs->SdCObs[i].Sys == BDS)
        {
            FG1 = FG1_BDS;
            FG2 = FG3_BDS;
        }
        else
        {
            Obs->SdSatObs[i].Valid = false;
            continue;
        }
        GF = C_Light * (Obs->SdSatObs[i].dL[0] / FG1) - C_Light * (Obs->SdSatObs[i].dL[1] / FG2);
        MW = (C_Light * (Obs->SdSatObs[i].dL[0] - Obs->SdSatObs[i].dL[1])) / (FG1 - FG2) - (Obs->SdSatObs[i].dP[0] * FG1 + Obs->SdSatObs[i].dP[1] * FG2) / (FG1 + FG2);
        double MW_ave;
        if (Obs->SdCObs[i].n == 0)//第一个历元不认为有粗差
        {
            Obs->SdCObs[i].GF = GF;
            Obs->SdCObs[i].MW = MW;
            Obs->SdCObs[i].n += 1;
            Obs->SdSatObs[i].Valid = true;
        }
        else if (Obs->SdCObs[i].n > 0)
        {
            //做差
            double dMW = Obs->SdCObs[i].MW - MW;
            double dGF = Obs->SdCObs[i].GF - GF;
            if (abs(dGF) >= 0.05 || abs(dMW) >= 5)
            {
                Obs->SdSatObs[i].Valid = false;
                Obs->SdCObs[i].n = 0;
            }
            else
            {
                Obs->SdCObs[i].n += 1;
                MW_ave = ((Obs->SdCObs[i].n - 1) * Obs->SdCObs[i].MW + MW) / Obs->SdCObs[i].n;//计算平滑值
                Obs->SdCObs[i].GF = GF;
                Obs->SdCObs[i].MW = MW_ave;
                Obs->SdSatObs[i].Valid = true;
            }
        }
        if (Obs->SdSatObs[i].Valid == true)
        {
            Obs->SdCObs[i].PIF = (pow(FG1, 2) * Obs->SdSatObs[i].dP[0] - pow(FG2, 2) * Obs->SdSatObs[i].dP[1]) / (pow(FG1, 2) - pow(FG2, 2));
        }

    }
}
void DetRefSat(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs)//EpkA=Rov,EpkB=Bas
{
    int    SatIndex[MAXCHANNUM]{ 0 };                         //符合条件的卫星Index
    bool   Status[2]{ false };                                //false=未选取参考星，true=已选取参考星;Status[0]=GPS,Status[1]=BDS
    double StandardAngle[2]{ 0 };                             //选取高度角大的卫星，StandardAngle[0]=GPS,StandardAngle[1]=BDS
    int    StandardIndex[2]{ 0 };                             //对应高度角卫星Index，StandardIndex[0]=GPS,StandardIndex[1]=BDS
    int    j;                                                 //循环变量，循环完成后，j=符合条件卫星数
    int    DDGpsSatNum, DDBdsSatNum;
    bool   StatusEpkB  = false;                               //流动站，基准站卫星Prn是否配对成功，true=成功，false=失败
    bool   StatusSDObs = false;                               //流动站,单差观测值卫星Prn是否匹配成功，true=成功，false=失败
    int    EpkBIndex = -1;                                    //基准站索引，记录与流动站对应卫星在基准站观测值中的位置
    int    SDObsIndex = -1;                                   //单差观测值索引，记录与流动站对应卫星在单差观测值中的位置
    j = 0;
    DDGpsSatNum = DDBdsSatNum = 0;

    for (int i = 0; i < EpkA->SatNum; i++)
    {
        StatusEpkB = false;
        StatusSDObs = false;
        if (EpkA->SatObs[i].Valid == true && EpkA->SatPVT[i].Valid == true)
        {
            StatusEpkB = PrnSearching(EpkA->SatObs[i].Prn, EpkB, EpkA->SatObs[i].System, &EpkBIndex);
            StatusSDObs = PrnSearching(EpkA->SatObs[i].Prn, SDObs, EpkA->SatObs[i].System, &SDObsIndex);
        }
        else continue;
        if (StatusEpkB == false || StatusSDObs == false) continue;
        if ( EpkB->SatObs[EpkBIndex].Valid == true && EpkB->SatPVT[EpkBIndex].Valid == true && SDObs->SdSatObs[SDObsIndex].Valid == true)
        {
            SatIndex[j] = i;
            j++;
        }
        else{}

        
        
    }
    for (int i = 0; i < j; i++)
    {
        int Index = SatIndex[i];
        int RefIndex;

        if (EpkA->SatObs[Index].System == GPS)      { RefIndex = 0; DDGpsSatNum++; }
        else if (EpkA->SatObs[Index].System == BDS) { RefIndex = 1; DDBdsSatNum++; }
        else continue;

        if (Status[RefIndex] == true) continue;
        if (EpkA->SatObs[Index].Prn == DDObs->RefPrn[RefIndex])//上一历元为参考星
        {
            DDObs->RefPos[RefIndex] = Index;
            DDObs->RefPrn[RefIndex] = EpkA->SatObs[Index].Prn;
            Status[RefIndex] = true;
            continue;
        }
        if (EpkA->SatPVT[Index].Elevation > StandardAngle[RefIndex])
        {
            StandardAngle[RefIndex] = EpkA->SatPVT[Index].Elevation;
            StandardIndex[RefIndex] = Index;
        }	
    }
    for (int i = 0; i < 2; i++)
    {
        if (Status[i] == false && StandardAngle[i]!=0 && StandardIndex[i]!=0)
        {
            DDObs->RefPos[i] = StandardIndex[i];
            DDObs->RefPrn[i] = EpkA->SatObs[StandardIndex[i]].Prn;
            Status[i] = true;
        }
    }
    if (DDGpsSatNum != 0)DDObs->DDSatNum[0] = DDGpsSatNum - 1;
    else DDObs->DDSatNum[0] = 0;
    if (DDBdsSatNum != 0)DDObs->DDSatNum[1] = DDBdsSatNum - 1;
    else DDObs->DDSatNum[1] = 0;
    DDObs->Sats = DDObs->DDSatNum[0] + DDObs->DDSatNum[1];
}

//0=RTKFixed,-1=RTKFloat,1=false
bool RTKFloat(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov)
{
    bool        StatusSDObs = false;                                      //流动站卫星是否与单差观测值卫星匹配成功
    int         SDObsIndex = -1;                                          //与流动站对应卫星在单差观测值中索引,0=Gps,1=Bds

    bool        StatusEpkB = false;                                       //流动站卫星是否与基准站观测值卫星匹配成功
    int         EpkBIndex = -1;                                           //与流动站对应卫星在基站观测值中索引,0=Gps,1=Bds

    bool        StatusRefSDObs = false;                                   //基准星是否在单差观测值卫星中匹配成功
    int         SDObsRefPos = -1;                                         //基准星在单差观测值中的索引,0=Gps,1=Bds

    bool        StatusRefEpk = false;                                     //基准星是否在基站观测值卫星中匹配成功
    int         EpkBRefPos = -1;                                          //基准星在基站观测卫星中的索引,0=Gps,1=Bds

    double      L;
    double      M;
    double      N;                                                        //系数矩阵内系数l,m,n 

    double      W[4*MAXCHANNUM]{ 0 };    
    double      B[4 * MAXCHANNUM * (3 + MAXCHANNUM * 2)]{ 0 };            //系数矩阵 4*DDSatNum × 3 + DDSatNum * 2
    double      P[4 * MAXCHANNUM * 4 * MAXCHANNUM]{ 0 };                  //权矩阵
    double      Q[4 * (MAXCHANNUM - 3) * 4 * (MAXCHANNUM - 3)]{ 0 };      //模糊度Q矩阵
    double      BT[(3 + MAXCHANNUM * 2) * 4 * MAXCHANNUM]{ 0 };
    double      BTP[(3 + MAXCHANNUM * 2) * 4 * MAXCHANNUM]{ 0 };
    double      BTPB[(3 + MAXCHANNUM * 2)* (3 + MAXCHANNUM * 2)]{ 0 };
    double      BTPW[3 + MAXCHANNUM * 2]{ 0 };
    double      invBTPB[4 * MAXCHANNUM * 4 * MAXCHANNUM]{ 0 };

    int         j;
    double      F[2]{ 0 };                                                //GPS(L1/L2)或BDS(B1/B3)信号频率  F
    double      R[2]{ 0 };
    double      Lan[2]{ 0 };                                              //GPS(L1/L2)或BDS(B1/B3)信号波长  Lan=C_Light/F
    double      iteration = 0;                                            //循环次数
    double      deltaPos[3]{ 0 };                                         //待估流动站位置改正数
    double      deltaNbr[MAXCHANNUM * 2]{ 0 };                            //模糊度参数
    double      Nbr[MAXCHANNUM * 2]{ 0 };                                 //模糊度
    bool        NbrValid = false;                                         //false=模糊度未初始化，true=模糊度已经初始化

    int         SysIndex = -1;                                            //0=Gps,1=BDS,-1=Unknow
    double      RBasIg[2]{ 0 }, RBasJg[2]{ 0 };
    double      RRovIg[2]{ 0 }, RRovJg[2]{ 0 };

                                                                          

    do
    {
        
        Rov->Position[0] = Rov->Position[0] + deltaPos[0];
        Rov->Position[1] = Rov->Position[1] + deltaPos[1];
        Rov->Position[2] = Rov->Position[2] + deltaPos[2];

        for (int i = 0; i < 2 * MAXCHANNUM;i++ )
        {
            Nbr[i] = Nbr[i] + deltaNbr[i];
        }


        if (Raw->DDObs.Sats < 5) return false;
        j = 0;
        for (int i = 0; i < Raw->RovEpk.SatNum; i++)
        {
            StatusEpkB     = false;
            StatusSDObs    = false;
            StatusRefSDObs = false;
            StatusRefEpk   = false;

            //跳过基准星
            if ((Raw->RovEpk.SatObs[i].Prn == Raw->DDObs.RefPrn[0] && Raw->RovEpk.SatObs[i].System == GPS) ||
                (Raw->RovEpk.SatObs[i].Prn == Raw->DDObs.RefPrn[1] && Raw->RovEpk.SatObs[i].System == BDS))
                continue;                                                                       

            if      (Raw->RovEpk.SatObs[i].System == GPS) SysIndex = 0;
            else if (Raw->RovEpk.SatObs[i].System == BDS) SysIndex = 1;
            else continue;

            switch (SysIndex)
            {

            case 0:
                F[0] = FG1_GPS;
                Lan[0] = C_Light / F[0];

                F[1] = FG2_GPS;
                Lan[1] = C_Light / F[1];

                break;
            case 1:
                F[0] = FG1_BDS;
                Lan[0] = C_Light / F[0];

                F[1] = FG3_BDS;
                Lan[1] = C_Light / F[1];

                break;

            default:
                break;
            }
            
            if (Raw->DDObs.RefPrn[SysIndex] != -1 && Raw->DDObs.DDSatNum[SysIndex] > 0)
            {
                //获取SDObs中基准星索引
                StatusRefSDObs = PrnSearching(Raw->DDObs.RefPrn[SysIndex], &Raw->SdObs, Raw->RovEpk.SatObs[Raw->DDObs.RefPos[SysIndex]].System, &SDObsRefPos);
                if (StatusRefSDObs != true) return false;

                //获取BasObs中基准星索引
                StatusRefEpk = PrnSearching(Raw->DDObs.RefPrn[SysIndex], &Raw->BasEpk, Raw->RovEpk.SatObs[Raw->DDObs.RefPos[SysIndex]].System, &EpkBRefPos);
                if (StatusRefEpk != true) return false;

                //基站到参考星距离
                RBasIg[SysIndex] = sqrt(pow(Base->Position[0] - Raw->BasEpk.SatPVT[EpkBRefPos].SatPos[0], 2) +
                                        pow(Base->Position[1] - Raw->BasEpk.SatPVT[EpkBRefPos].SatPos[1], 2) +
                                        pow(Base->Position[2] - Raw->BasEpk.SatPVT[EpkBRefPos].SatPos[2], 2));

                //流动站到参考星距离
                RRovIg[SysIndex] = sqrt(pow(Rov->Position[0] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[0], 2) +
                                        pow(Rov->Position[1] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[1], 2) +
                                        pow(Rov->Position[2] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[2], 2));
            }


            if (Raw->RovEpk.SatObs[i].Valid == true && Raw->RovEpk.SatPVT[i].Valid == true)
            {
                StatusSDObs = PrnSearching(Raw->RovEpk.SatObs[i].Prn, &Raw->SdObs,  Raw->RovEpk.SatObs[i].System, &SDObsIndex);  //获取SDObs中卫星索引
                StatusEpkB  = PrnSearching(Raw->RovEpk.SatObs[i].Prn, &Raw->BasEpk, Raw->RovEpk.SatObs[i].System, &EpkBIndex );  //获取BasObs中卫星索引
            }

            if(StatusEpkB && StatusSDObs && Raw->BasEpk.SatObs[EpkBIndex].Valid && Raw->BasEpk.SatPVT[EpkBIndex].Valid && Raw->SdObs.SdSatObs[SDObsIndex].Valid)
            {
                //模糊度初始化
                if (NbrValid == false)
                {
                    Nbr[2 * j + 0] = (Raw->SdObs.SdSatObs[SDObsIndex].dL[0] * Lan[0] - Raw->SdObs.SdSatObs[SDObsIndex].dP[0]) / Lan[0];
                    Nbr[2 * j + 1] = (Raw->SdObs.SdSatObs[SDObsIndex].dL[1] * Lan[1] - Raw->SdObs.SdSatObs[SDObsIndex].dP[1]) / Lan[1];
                }

                RBasJg[SysIndex] = sqrt(pow(Base->Position[0] - Raw->BasEpk.SatPVT[EpkBIndex].SatPos[0], 2) +
                                        pow(Base->Position[1] - Raw->BasEpk.SatPVT[EpkBIndex].SatPos[1], 2) +
                                        pow(Base->Position[2] - Raw->BasEpk.SatPVT[EpkBIndex].SatPos[2], 2));

                RRovJg[SysIndex] = sqrt(pow(Rov->Position[0] - Raw->RovEpk.SatPVT[i].SatPos[0], 2) +
                                        pow(Rov->Position[1] - Raw->RovEpk.SatPVT[i].SatPos[1], 2) +
                                        pow(Rov->Position[2] - Raw->RovEpk.SatPVT[i].SatPos[2], 2));

                R[SysIndex] = RRovJg[SysIndex] - RRovIg[SysIndex] - RBasJg[SysIndex] + RBasIg[SysIndex];

                L = (Rov->Position[0] - Raw->RovEpk.SatPVT[i].SatPos[0]) / RRovJg[SysIndex] - (Rov->Position[0] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[0]) / RRovIg[SysIndex];
                M = (Rov->Position[1] - Raw->RovEpk.SatPVT[i].SatPos[1]) / RRovJg[SysIndex] - (Rov->Position[1] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[1]) / RRovIg[SysIndex];
                N = (Rov->Position[2] - Raw->RovEpk.SatPVT[i].SatPos[2]) / RRovJg[SysIndex] - (Rov->Position[2] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[2]) / RRovIg[SysIndex];

                //系数矩阵B
                for (int k = 0; k < 4; k++)
                {
                    B[(k + 4 * j) * (3 + (Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]) * 2) + 0] = L;
                    B[(k + 4 * j) * (3 + (Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]) * 2) + 1] = M;
                    B[(k + 4 * j) * (3 + (Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]) * 2) + 2] = N;
                }

                B[(2 + 4 * j) * (3 + (Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]) * 2) + (2 * j + 3)] = Lan[0];
                B[(3 + 4 * j) * (3 + (Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]) * 2) + (2 * j + 4)] = Lan[1];

                //W矩阵
                W[4 * j + 0] = Raw->SdObs.SdSatObs[SDObsIndex].dP[0] - Raw->SdObs.SdSatObs[SDObsRefPos].dP[0] - R[SysIndex];
                W[4 * j + 1] = Raw->SdObs.SdSatObs[SDObsIndex].dP[1] - Raw->SdObs.SdSatObs[SDObsRefPos].dP[1] - R[SysIndex];
                W[4 * j + 2] = Raw->SdObs.SdSatObs[SDObsIndex].dL[0] * Lan[0] - Raw->SdObs.SdSatObs[SDObsRefPos].dL[0] * Lan[0] - R[SysIndex] - Lan[0] * Nbr[2 * j + 0];
                W[4 * j + 3] = Raw->SdObs.SdSatObs[SDObsIndex].dL[1] * Lan[1] - Raw->SdObs.SdSatObs[SDObsRefPos].dL[1] * Lan[1] - R[SysIndex] - Lan[1] * Nbr[2 * j + 1];
                
                //P矩阵
                double PsrFactor =  ((Raw->DDObs.DDSatNum[SysIndex] + 1) * PsrW);     //伪距双差观测值权重分母
                double AdrFactor =  ((Raw->DDObs.DDSatNum[SysIndex] + 1) * AdrW);     //相位双差观测值权重分母
                double rat;                                                           //权重分子，对角线上分子为双差卫星数，非对角线为-1
                int    Ndd;                                                           //双差卫星数

                int col = 4 * (Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]);      //P矩阵列数
                if (SysIndex == 0)
                {
                    for (Ndd = 0; Ndd < Raw->DDObs.DDSatNum[0]; Ndd++)
                    {
                        if (j == Ndd) rat = Raw->DDObs.DDSatNum[0];
                        else          rat = -1;

                        P[(4 * j + 0) * col + 4 * Ndd + 0] = rat / PsrFactor;
                        P[(4 * j + 1) * col + 4 * Ndd + 1] = rat / PsrFactor;
                        P[(4 * j + 2) * col + 4 * Ndd + 2] = rat / AdrFactor;
                        P[(4 * j + 3) * col + 4 * Ndd + 3] = rat / AdrFactor;
                    }
                }
                else if (SysIndex == 1)
                {
                    for (Ndd = Raw->DDObs.DDSatNum[0]; Ndd < Raw->DDObs.DDSatNum[1] + Raw->DDObs.DDSatNum[0]; Ndd++)
                    {
                        if (j == Ndd) rat = Raw->DDObs.DDSatNum[1];
                        else          rat = -1;

                        P[(4 * j + 0) * col + 4 * Ndd + 0] = rat / PsrFactor;
                        P[(4 * j + 1) * col + 4 * Ndd + 1] = rat / PsrFactor;
                        P[(4 * j + 2) * col + 4 * Ndd + 2] = rat / AdrFactor;
                        P[(4 * j + 3) * col + 4 * Ndd + 3] = rat / AdrFactor;
                    }
                }
                else continue;
                j++;
            }
            else continue;
        }

        if (j != Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]) return false;//BUGTEST
        if (j < 5) return false;
        NbrValid = true;


        //MatrixTest
        //MatrixDisplay(4 * (Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]), 3 + (Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]) * 2, B);
        //MatrixDisplay(4 * (Raw->DDObs.DDSatNum[0] + Raw->DDObs.DDSatNum[1]), 1, W);
        //MatrixDisplay(4 * j, 4 * j , P);

        double MidDelta[3 + 2 * MAXCHANNUM]{ 0 };
        matrix_transfer(4 * j, (3 + j * 2), B, BT);
        matrix_multiply((3 + j * 2), 4 * j, 4 * j, 4 * j, BT, P, BTP);
        matrix_multiply((3 + j * 2), 4 * j, 4 * j, (3 + j * 2), BTP, B, BTPB);
        matrix_Inv((3 + j * 2), BTPB, invBTPB);
        matrix_multiply((3 + j * 2), 4 * j, 4 * j, 1, BTP, W, BTPW);
        matrix_multiply((3 + j * 2), (3 + j * 2), (3 + j * 2), 1, invBTPB, BTPW, MidDelta);
        for (int i = 0; i < 3; i++)
        {
            deltaPos[i] = MidDelta[i];
        }
        for (int i = 3; i < 3 + 2 * j; i++)
        {
            deltaNbr[i - 3] = MidDelta[i];
        }
        iteration++;
    } while (sqrt(deltaPos[0] * deltaPos[0] + deltaPos[1] * deltaPos[1] + deltaPos[2] * deltaPos[2]) > 1e-2 && iteration < 10);

    Rov->Position[0] = Rov->Position[0] + deltaPos[0];
    Rov->Position[1] = Rov->Position[1] + deltaPos[1];
    Rov->Position[2] = Rov->Position[2] + deltaPos[2];

    for (int i = 0; i < 2 * MAXCHANNUM; i++)
    {
        Nbr[i] = Nbr[i] + deltaNbr[i];
    }
    for (int i = 0; i < 3; i++)
    {
        Raw->DDObs.dPos[i] = Rov->Position[i] - Base->Position[i];   
    }
    //printf("ObsTime:%13.5F DX:%13.5F DY:%13.5F DZ:%13.5F\n",Rov->Time.SecOfWeek, Raw->DDObs.dPos[0], Raw->DDObs.dPos[1], Raw->DDObs.dPos[2]);
    // 
    //提取模糊度Q矩阵
    for (int i = 3; i < 2 * Raw->DDObs.Sats + 3; i++)
    {
        for (int j = 3; j < 2 * Raw->DDObs.Sats + 3; j++)
        {
            int p = i - 3; int q = j - 3;
            Q[p * (2 * Raw->DDObs.Sats) + q] = invBTPB[i * (2 * Raw->DDObs.Sats + 3) + j];
        }
    }

    //printf("InvBTPB:\n");
    //MatrixDisplay(2 * Raw->DDObs.Sats + 3, 2 * Raw->DDObs.Sats + 3, invBTPB);
    //printf("Nbr Qxx:\n");
    //MatrixDisplay(2 * Raw->DDObs.Sats, 2 * Raw->DDObs.Sats, Q);
    

    
    //模糊度固定 0:ok,other:error
    if (lambda(2 * Raw->DDObs.Sats, 2, Nbr, Q, Raw->DDObs.FixedAmb, Raw->DDObs.ResAmb) == 0)
    {
        //固定完成,模糊度检验
        Raw->DDObs.Ratio = Raw->DDObs.ResAmb[1] / Raw->DDObs.ResAmb[0];
        //模糊度检验通过
        if (Raw->DDObs.Ratio > 3)
        {
            RTKFixed(Raw, Base, Rov);
            Raw->DDObs.bFixed = true;
        }
        //不通过则认为固定失败
        else Raw->DDObs.bFixed = false;      
    }
    return true;
    
}
void RTKFixed(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov)
{
    bool        StatusSDObs = false;                                      //流动站卫星是否与单差观测值卫星匹配成功
    int         SDObsIndex = -1;                                          //与流动站对应卫星在单差观测值中索引,0=Gps,1=Bds

    bool        StatusEpkB = false;                                       //流动站卫星是否与基准站观测值卫星匹配成功
    int         EpkBIndex = -1;                                           //与流动站对应卫星在基站观测值中索引,0=Gps,1=Bds

    bool        StatusRefSDObs = false;                                   //基准星是否在单差观测值卫星中匹配成功
    int         SDObsRefPos = -1;                                         //基准星在单差观测值中的索引,0=Gps,1=Bds

    bool        StatusRefEpk = false;                                     //基准星是否在基站观测值卫星中匹配成功
    int         EpkBRefPos = -1;                                          //基准星在基站观测卫星中的索引,0=Gps,1=Bds

    double      L;
    double      M;
    double      N;                                                        //系数矩阵内系数l,m,n 

    double      W[2 * MAXCHANNUM]{ 0 };
    double      B[2 * 3 * MAXCHANNUM]{ 0 };                               //系数矩阵 2*DDSATNUM × 3
    double      BT[2 * 3 * MAXCHANNUM]{ 0 };
    double      P[2 * MAXCHANNUM * 2 * MAXCHANNUM]{ 0 };                  //权阵 2*DDSATNUM×2*DDSATNUM
    double      BTP[2 * 3 * MAXCHANNUM]{ 0 };                             //3×2*DDSATNUM
    double      BTPB[9]{ 0 };                                             //3×3
    double      BTPW[3 + MAXCHANNUM * 2]{ 0 };                            //3×1
    double      invBTPB[9]{ 0 };                                          //3×3

    int         j;
    double      F[2]{ 0 };                                                //GPS(L1/L2)或BDS(B1/B3)信号频率  F
    double      R[2]{ 0 };
    double      Lan[2]{ 0 };                                              //GPS(L1/L2)或BDS(B1/B3)信号波长  Lan=C_Light/F
    double      iteration = 0;                                            //循环次数
    double      deltaPos[3]{ 0 };                                         //待估流动站位置改正数
    double      Nbr[MAXCHANNUM * 2]{ 0 };                                 //已固定模糊度
    bool        NbrValid = false;                                         //false=模糊度未初始化，true=模糊度已经初始化


    int         SysIndex = -1;                                            //0=Gps,1=BDS,-1=Unknow
    double      RBasIg[2]{ 0 }, RBasJg[2]{ 0 };
    double      RRovIg[2]{ 0 }, RRovJg[2]{ 0 };

    //精度评定相关定义
    double delta2 = 0;                                                    //单位权中误差
    double WT[4 * MAXCHANNUM]{ 0 };
    double WTP[4 * MAXCHANNUM]{ 0 };
    double WTPW[1]{ 0 };

    double BasPos[3] = { -2267804.5263, 5009342.3723, 3220991.8632 };

    //RTKFixed
    do
    {
        for (int i = 0; i < 3; i++)
        {
            Rov->Position[i] = Rov->Position[i] + deltaPos[i];
        }

        j = 0;
        for (int i = 0; i < Raw->RovEpk.SatNum; i++)
        {
            StatusEpkB = false;
            StatusSDObs = false;
            StatusRefSDObs = false;
            StatusRefEpk = false;

            //跳过基准星
            if ((Raw->RovEpk.SatObs[i].Prn == Raw->DDObs.RefPrn[0] && Raw->RovEpk.SatObs[i].System == GPS) ||
                (Raw->RovEpk.SatObs[i].Prn == Raw->DDObs.RefPrn[1] && Raw->RovEpk.SatObs[i].System == BDS))
                continue;

            if (Raw->RovEpk.SatObs[i].System == GPS) SysIndex = 0;
            else if (Raw->RovEpk.SatObs[i].System == BDS) SysIndex = 1;
            else continue;

            switch (SysIndex)
            {

            case 0:
                F[0] = FG1_GPS;
                Lan[0] = C_Light / F[0];

                F[1] = FG2_GPS;
                Lan[1] = C_Light / F[1];

                break;
            case 1:
                F[0] = FG1_BDS;
                Lan[0] = C_Light / F[0];

                F[1] = FG3_BDS;
                Lan[1] = C_Light / F[1];

                break;

            default:
                break;
            }
         
            if (Raw->DDObs.RefPrn[SysIndex] != -1 && Raw->DDObs.DDSatNum[SysIndex] > 0)
            {
                if (!PrnSearching(Raw->DDObs.RefPrn[SysIndex], &Raw->SdObs, Raw->RovEpk.SatObs[Raw->DDObs.RefPos[SysIndex]].System, &SDObsRefPos)) continue;
                if (!PrnSearching(Raw->DDObs.RefPrn[SysIndex], &Raw->BasEpk, Raw->RovEpk.SatObs[Raw->DDObs.RefPos[SysIndex]].System, &EpkBRefPos)) continue;

                //基站到参考星距离
                RBasIg[SysIndex] = sqrt(pow(Base->Position[0] - Raw->BasEpk.SatPVT[EpkBRefPos].SatPos[0], 2) +
                    pow(Base->Position[1] - Raw->BasEpk.SatPVT[EpkBRefPos].SatPos[1], 2) +
                    pow(Base->Position[2] - Raw->BasEpk.SatPVT[EpkBRefPos].SatPos[2], 2));

                //流动站到参考星距离
                RRovIg[SysIndex] = sqrt(pow(Rov->Position[0] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[0], 2) +
                    pow(Rov->Position[1] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[1], 2) +
                    pow(Rov->Position[2] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[2], 2));
            }

            if (Raw->RovEpk.SatObs[i].Valid == true && Raw->RovEpk.SatPVT[i].Valid == true)
            {
                StatusSDObs = PrnSearching(Raw->RovEpk.SatObs[i].Prn, &Raw->SdObs, Raw->RovEpk.SatObs[i].System, &SDObsIndex);  //获取SDObs中卫星索引
                StatusEpkB = PrnSearching(Raw->RovEpk.SatObs[i].Prn, &Raw->BasEpk, Raw->RovEpk.SatObs[i].System, &EpkBIndex);  //获取BasObs中卫星索引
            }

            if (StatusEpkB && StatusSDObs && Raw->BasEpk.SatObs[EpkBIndex].Valid && Raw->BasEpk.SatPVT[EpkBIndex].Valid && Raw->SdObs.SdSatObs[SDObsIndex].Valid)
            {
                //模糊度初始化
                if (NbrValid == false)
                {
                    Nbr[2 * j + 0] = Raw->DDObs.FixedAmb[2 * j + 0];
                    Nbr[2 * j + 1] = Raw->DDObs.FixedAmb[2 * j + 1];
                }

                RBasJg[SysIndex] = sqrt(pow(Base->Position[0] - Raw->BasEpk.SatPVT[EpkBIndex].SatPos[0], 2) +
                    pow(Base->Position[1] - Raw->BasEpk.SatPVT[EpkBIndex].SatPos[1], 2) +
                    pow(Base->Position[2] - Raw->BasEpk.SatPVT[EpkBIndex].SatPos[2], 2));

                RRovJg[SysIndex] = sqrt(pow(Rov->Position[0] - Raw->RovEpk.SatPVT[i].SatPos[0], 2) +
                    pow(Rov->Position[1] - Raw->RovEpk.SatPVT[i].SatPos[1], 2) +
                    pow(Rov->Position[2] - Raw->RovEpk.SatPVT[i].SatPos[2], 2));

                R[SysIndex] = RRovJg[SysIndex] - RRovIg[SysIndex] - RBasJg[SysIndex] + RBasIg[SysIndex];

                L = (Rov->Position[0] - Raw->RovEpk.SatPVT[i].SatPos[0]) / RRovJg[SysIndex] - (Rov->Position[0] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[0]) / RRovIg[SysIndex];
                M = (Rov->Position[1] - Raw->RovEpk.SatPVT[i].SatPos[1]) / RRovJg[SysIndex] - (Rov->Position[1] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[1]) / RRovIg[SysIndex];
                N = (Rov->Position[2] - Raw->RovEpk.SatPVT[i].SatPos[2]) / RRovJg[SysIndex] - (Rov->Position[2] - Raw->RovEpk.SatPVT[Raw->DDObs.RefPos[SysIndex]].SatPos[2]) / RRovIg[SysIndex];

                //系数矩阵B
                for (int k = 0; k < 2; k++)
                {
                    B[(k + 2 * j) * 3 + 0] = L;
                    B[(k + 2 * j) * 3 + 1] = M;
                    B[(k + 2 * j) * 3 + 2] = N;
                }

                //W矩阵
                W[2 * j + 0] = Raw->SdObs.SdSatObs[SDObsIndex].dL[0] * Lan[0] - Raw->SdObs.SdSatObs[SDObsRefPos].dL[0] * Lan[0] - R[SysIndex] - Lan[0] * Nbr[2 * j + 0];
                W[2 * j + 1] = Raw->SdObs.SdSatObs[SDObsIndex].dL[1] * Lan[1] - Raw->SdObs.SdSatObs[SDObsRefPos].dL[1] * Lan[1] - R[SysIndex] - Lan[1] * Nbr[2 * j + 1];

                //P矩阵(暂时设为单位阵)
                //double PsrFactor = ((Raw->DDObs.DDSatNum[SysIndex] + 1) * PsrW);     //伪距双差观测值权重分母
                double AdrFactor = ((Raw->DDObs.DDSatNum[SysIndex] + 1) * AdrW);      //相位双差观测值权重分母
                double rat;                                                           //权重分子，对角线上分子为1，非对角线为0
                int    Ndd;                                                           //双差卫星数

                int col = 2 * (Raw->DDObs.Sats);                                      //P矩阵列数
                if (SysIndex == 0)
                {
                    for (Ndd = 0; Ndd < Raw->DDObs.DDSatNum[0]; Ndd++)
                    {
                        if (j == Ndd) rat = Raw->DDObs.DDSatNum[0];
                        else          rat = -1;

                        P[(2 * j + 0) * col + 2 * Ndd + 0] = rat/AdrFactor;
                        P[(2 * j + 1) * col + 2 * Ndd + 1] = rat/AdrFactor;
                    }
                }
                else if (SysIndex == 1)
                {
                    for (Ndd = Raw->DDObs.DDSatNum[0]; Ndd < Raw->DDObs.Sats; Ndd++)
                    {
                        if (j == Ndd) rat = Raw->DDObs.DDSatNum[1];
                        else          rat = -1;

                        P[(2 * j + 0) * col + 2 * Ndd + 0] = rat/AdrFactor;
                        P[(2 * j + 1) * col + 2 * Ndd + 1] = rat/AdrFactor;
                    }
                }
                else continue;
                j++;
            }
            else continue;


        }
        NbrValid = true;

        //MatrixTest
        //MatrixDisplay(2*Raw->DDObs.Sats, 3, B);
        //MatrixDisplay(2*Raw->DDObs.Sats, 1, W);
        //MatrixDisplay(2 * j, 2 * j , P);

        //LS
        matrix_transfer(2 * Raw->DDObs.Sats, 3, B, BT);
        matrix_multiply(3, 2 * Raw->DDObs.Sats, 2 * Raw->DDObs.Sats, 2 * Raw->DDObs.Sats, BT, P, BTP);
        matrix_multiply(3, 2 * Raw->DDObs.Sats, 2 * Raw->DDObs.Sats, 3, BTP, B, BTPB);
        matrix_Inv(3, BTPB, invBTPB);
        matrix_multiply(3, 2 * Raw->DDObs.Sats, 2 * Raw->DDObs.Sats, 1, BTP, W, BTPW);
        matrix_multiply(3, 3, 3, 1, invBTPB, BTPW, deltaPos);
        iteration++;
    } while (sqrt(vector_dot(1,3,1,3,deltaPos,deltaPos))>1e-2||iteration<10);

    for (int i = 0; i < 3; i++)
    {
        Rov->Position[i] = Rov->Position[i] + deltaPos[i];
    }
    for (int i = 0; i < 3; i++)
    {
        Raw->DDObs.dPos[i] = Rov->Position[i] - Base->Position[i];
    }

    //精度评定
    //double      W[2 * MAXCHANNUM]{ 0 };
    //double      B[2 * 3 * MAXCHANNUM]{ 0 };                               //系数矩阵 2*DDSATNUM × 3
    //double      BT[2 * 3 * MAXCHANNUM]{ 0 };
    //double      P[2 * MAXCHANNUM * 2 * MAXCHANNUM]{ 0 };                  //权阵 2*DDSATNUM×2*DDSATNUM
    //double      BTP[2 * 3 * MAXCHANNUM]{ 0 };                             //3×2*DDSATNUM
    //double      BTPB[9]{ 0 };                                             //3×3
    //double      BTPW[3 + MAXCHANNUM * 2]{ 0 };                            //3×1
    //double      invBTPB[9]{ 0 };                                          //3×3
    //matrix_transfer(2 * Raw->DDObs.Sats, 1, W, WT);
    //matrix_multiply(1, 2 * Raw->DDObs.Sats, 2 * Raw->DDObs.Sats, 2 * Raw->DDObs.Sats, WT, P, WTP);
    //matrix  
}
//Test
bool PrnSearching(const short Prn, const EPOCHOBS* Epoch ,const int Sys,int* EpochIndex)
{
    bool StatusEpkB = false;
    int Index = -1;
    int j = 0;
    for (int i=0; i < Epoch->SatNum; i++)
    {
        if (Epoch->SatObs[i].Prn == Prn && Epoch->SatObs[i].System==Sys)
        {
            StatusEpkB = true;
            Index = i;
            break;
        }
    }
    if (Index > MAXCHANNUM)
    {
        StatusEpkB = false;
    }
    *EpochIndex = Index;
    return StatusEpkB;
}
bool PrnSearching(const short Prn, const SDEPOCHOBS* SDObs, const int Sys, int* SDObsIndex)
{
    bool StatusSDObs = false;
    int Index = -1;
    int j = 0;
    for (int i=0; i < SDObs->SatNum; i++)
    {
        if (SDObs->SdSatObs[i].Prn == Prn && SDObs->SdSatObs[i].System==Sys)
        {
            StatusSDObs = true; 
            Index = i;
            break;
        }
    }
    if (Index > MAXCHANNUM)
    {
        StatusSDObs = false;
    }
    *SDObsIndex = Index;
    return StatusSDObs;
}
void MatrixDisplay(int m, int n, double Mat[])
{
    int i, j;
    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)  printf("%13.5E\t", Mat[i * n + j]);
        printf("\n");
    }
    printf("\n");
}
void SaveSocketData(SOCKET Soc, FILE* Dat)
{
    void *buff = malloc(sizeof(unsigned char) * MAXNOVDLEN);
    unsigned char *Buff;
    Buff = (unsigned char*)buff;
    recv(Soc, (char*)Buff, MAXNOVDLEN, 0);
    fwrite((char*)Buff, sizeof(unsigned char),MAXNOVDLEN,Dat);
}
//Socket
bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port)
{
    WSADATA wsaData;
    SOCKADDR_IN addrSrv;

    if (!WSAStartup(MAKEWORD(1, 1), &wsaData))
    {
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) != INVALID_SOCKET)
        {
            addrSrv.sin_addr.S_un.S_addr = inet_addr(IP);
            addrSrv.sin_family = AF_INET;
            addrSrv.sin_port = htons(Port);
            connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
            return true;
        }
    }
    return false;
}
void CloseSocket(SOCKET& sock)
{
    closesocket(sock);
    WSACleanup();
}



