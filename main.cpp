// Spp.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include"RTK_Structs.h"

int main()
{
	FILE*         FBas;
	FILE*         FRov;

	FILE*         res;
	FILE*         LineLog;

	SOCKET        BasSoc ;
	SOCKET        RovSoc ;

	PPRESULT      Res[2];                       //0为流动站，1为基站

	RAWDAT*       RAW=new RAWDAT;
	int           Status = 0;          

	const char RovIP[] = "8.140.46.126";
	const char BasIP[] = "47.114.134.129";

	const unsigned short RovPort = 5002;
	const unsigned short BasPort = 7190;

	int RTKFixedNum = 0;
	int RTKFloatNum = 0;
	int RTKFailure = 0;
	int SPPFailure = 0;

	double blh[3]{ 0 };
	const double BasPos[3] = { -2267804.5263, 5009342.3723,3220991.8632 };
    //OpenFile
	if (fopen_s(&FRov, "oem719-202203031500-2.bin", "r+b") != 0)
	{
		printf("Problem opening the file\n");
		return -1;
	}
	if (fopen_s(&FBas, "oem719-202203031500-1.bin", "r+b") != 0)
	{
		printf("Problem opening the file\n");
		return -1;
	}

	//OpenSocket
	if (!OpenSocket(BasSoc, BasIP, BasPort))
	{
		printf("Problem opening the socket!\n");
	}
	if (!OpenSocket(RovSoc, RovIP, RovPort))
	{
		printf("Problem opening the socket!\n");
	}

	//SPP Log
	if (fopen_s(&res, "result.oem719.log", "w+") != 0)
	{
		printf("Problem opening the file\n");
		return -1;
	}

	//RTK Log
	if (fopen_s(&LineLog, "RTK.oem719.log", "w+") != 0)
	{
		printf("Problem opening the file\n");
		return -1;
	}

	//RTK
	while (true)
	{
		//时间同步:
		
		//FILE
		//Status = GetSynObs(FBas, FRov, RAW);

		//Socket
		Sleep(999);
		Status = GetSynObs(BasSoc, RovSoc, RAW);

		if (Status == -1) break;

		//粗差探测
		DetectOutlier(&RAW->RovEpk);
		DetectOutlier(&RAW->BasEpk);

		//SPP & SPV  
		memset(blh, 0, sizeof(double) * 3);
		if (Status == 1)//Res[0]为流动站结果，Res[1]为基站结果
		{
			Res[0].IsSuccess = SPP(&RAW->RovEpk, RAW, &Res[0]);
			if (Res[0].IsSuccess == true)
			{
				SPV(&RAW->RovEpk, &Res[0]);
				XYZToBLH(Res[0].Position, blh, R_WGS84, F_WGS84);
				//printf("OBSTIME:%d\t%f\t", Res[0].Time.Week, Res[0].Time.SecOfWeek);
			    //printf("Rov:\tX:%f\tY:%f\tZ:%f\tB:%f\tL:%f\tH:%f\t GPSClk:%13.5f\tBDSClk:%13.5f\tPDOP:%f\tSigma:%f\t", Res[0].Position[0], Res[0].Position[1], Res[0].Position[2], blh[0] * Deg, blh[1] * Deg, blh[2], Res[0].RcvClkOft[0], Res[0].RcvClkOft[1], Res[0].PDOP, Res[0].SigmaPos);
			    //printf("VX:%13.5f\tVY:%f\tVZ:%f\tClkd:%f\tVelSigma:%f\n", Res[0].Velocity[0], Res[0].Velocity[1], Res[0].Velocity[2], Res[0].RcvClkSft, Res[0].SigmaVel);

				fprintf(res,"OBSTIME:%d\t%f\t", Res[0].Time.Week,Res[0].Time.SecOfWeek);
				fprintf(res,"Rov:\tX:%f\tY:%f\tZ:%f\tB:%f\tL:%f\tH:%f\t GPSClk:%13.5f\tPDOP:%f\tSigma:%f\t", Res[0].Position[0], Res[0].Position[1], Res[0].Position[2],blh[0]*Deg,blh[1]*Deg,blh[2],Res[0].RcvClkOft[0],Res[0].PDOP,Res[0].SigmaPos);
				fprintf(res,"VX:%13.5f\tVY:%f\tVZ:%f\tClkd:%f\tVelSigma:%f\n", Res[0].Velocity[0], Res[0].Velocity[1], Res[0].Velocity[2],Res[0].RcvClkSft,Res[0].SigmaVel);
			}
			Res[1].IsSuccess = SPP(&RAW->BasEpk, RAW, &Res[1]);
			if (Res[1].IsSuccess == true)
			{
				SPV(&RAW->BasEpk, &Res[1]);
				XYZToBLH(Res[1].Position, blh, R_WGS84, F_WGS84);
				//printf("OBSTIME:%d\t%f\t", Res[1].Time.Week, Res[1].Time.SecOfWeek);
				//printf("Bas:\tX:%f\tY:%f\tZ:%f\tB:%f\tL:%f\tH:%f\t GPSClk:%13.5f\tPDOP:%f\tSigma:%f\t\n", Res[1].Position[0], Res[1].Position[1], Res[1].Position[2], blh[0] * Deg, blh[1] * Deg, blh[2], Res[1].RcvClkOft[0], Res[1].PDOP, Res[1].SigmaPos);
				//printf("VX:%13.5f\tVY:%f\tVZ:%f\tClkd:%f\tVelSigma:%f\n", Res[1].Velocity[0], Res[1].Velocity[1], Res[1].Velocity[2], Res[1].RcvClkSft, Res[1].SigmaVel);

				fprintf(res,"OBSTIME:%d\t%f\t", Res[1].Time.Week, Res[1].Time.SecOfWeek);
				fprintf(res,"Bas:\tX:%f\tY:%f\tZ:%f\tB:%f\tL:%f\tH:%f\t GPSClk:%13.5f\tPDOP:%f\tSigma:%f\t", Res[1].Position[0], Res[1].Position[1], Res[1].Position[2], blh[0] * Deg, blh[1] * Deg, blh[2], Res[1].RcvClkOft[0], Res[1].PDOP, Res[1].SigmaPos);
				fprintf(res,"VX:%13.5f\tVY:%f\tVZ:%f\tClkd:%f\tVelSigma:%f\n", Res[1].Velocity[0], Res[1].Velocity[1], Res[1].Velocity[2], Res[1].RcvClkSft, Res[1].SigmaVel);
			}
		}
		//RTKfloat
		if (Res[0].IsSuccess && Res[1].IsSuccess)
		{
			//站间单差
			FormSDEpochObs(&RAW->RovEpk, &RAW->BasEpk, &RAW->SdObs);
			//粗差探测
			DetectCycleSlip(&RAW->SdObs);
			//基准星选取
			DetRefSat(&RAW->RovEpk, &RAW->BasEpk, &RAW->SdObs, &RAW->DDObs);
			//RTK定位数学模型建立			
			if (RTKFloat(RAW, &Res[1], &Res[0]))
			{
				for (int i = 0; i < 3; i++)
				{
					RAW->DDObs.dPos[i] = RAW->DDObs.dPos[i] + BasPos[i];
				}
				XYZToBLH(RAW->DDObs.dPos, blh, R_WGS84, F_WGS84);

				if (RAW->DDObs.bFixed)
				{
					RTKFixedNum++;
					printf("Obstime Week:%4hd\tSec:%13.5f\tRTKFixed\tX:%15.5f\tY:%15.5f\tZ:%15.5f\tB:%15.5f\tL:%15.5f\tH:%15.5f\tRatio:%13.5f\tRTKFixedNum:%3d\n",
						RAW->RovEpk.Time.Week, RAW->RovEpk.Time.SecOfWeek, RAW->DDObs.dPos[0], RAW->DDObs.dPos[1], RAW->DDObs.dPos[2], blh[0] * Deg, blh[1] * Deg, blh[2], RAW->DDObs.Ratio, RTKFixedNum);
					fprintf(LineLog, "Obstime Week:%4hd\tSec:%13.5f\tRTKFixed\tX:%15.5f\tY:%15.5f\tZ:%15.5f\tB:%15.5f\tL:%15.5f\tH:%15.5f\tRatio:%13.5f\tRTKFixedNum:%3d\n",
						RAW->RovEpk.Time.Week, RAW->RovEpk.Time.SecOfWeek, RAW->DDObs.dPos[0], RAW->DDObs.dPos[1], RAW->DDObs.dPos[2], blh[0] * Deg, blh[1] * Deg, blh[2], RAW->DDObs.Ratio, RTKFixedNum);
				}
				else 
				{
					RTKFloatNum++;
					printf("Obstime Week:%4hd\tSec:%13.5f\tRTKFloat\tX:%15.5f\tY:%15.5f\tZ:%15.5f\tB:%15.5f\tL:%15.5f\tH:%15.5f\tRatio:%13.5f\tRTKFloatNum:%3d\n",
						RAW->RovEpk.Time.Week, RAW->RovEpk.Time.SecOfWeek, RAW->DDObs.dPos[0], RAW->DDObs.dPos[1], RAW->DDObs.dPos[2], blh[0] * Deg, blh[1] * Deg, blh[2], RAW->DDObs.Ratio, RTKFloatNum);
					fprintf(LineLog, "Obstime Week:%4hd\tSec:%13.5f\tRTKFloat\tX:%15.5f\tY:%15.5f\tZ:%15.5f\tB:%15.5f\tL:%15.5f\tH:%15.5f\tRatio:%13.5f\tRTKFloatNum:%3d\n",
						RAW->RovEpk.Time.Week, RAW->RovEpk.Time.SecOfWeek, RAW->DDObs.dPos[0], RAW->DDObs.dPos[1], RAW->DDObs.dPos[2], blh[0] * Deg, blh[1] * Deg, blh[2], RAW->DDObs.Ratio, RTKFloatNum);
				}
			}
			else
			{
				RTKFailure++;
				printf("Obstime Week:%4hd\tSec:%13.5f RTK解算失败!\tX:%15.5f\tY:%15.5f\tZ:%15.5f\tB:%15.5f\tL:%15.5f\tH:%15.5f\tRTKFailure:%3d\n", RAW->RovEpk.Time.Week, RAW->RovEpk.Time.SecOfWeek, Res[0].Position[0], Res[0].Position[1], Res[0].Position[2], blh[0] * Deg, blh[1] * Deg, blh[2], RTKFailure);
				fprintf(LineLog, "Obstime Week:%4hd\tSec:%13.5f RTK解算失败！\tX:%15.5f\tY:%15.5f\tZ:%15.5f\tB:%15.5f\tL:%15.5f\tH:%15.5f\n", RAW->RovEpk.Time.Week, RAW->RovEpk.Time.SecOfWeek, Res[0].Position[0], Res[0].Position[1], Res[0].Position[2], blh[0] * Deg, blh[1] * Deg, blh[2]);
			}
			
		}
		else
		{
			SPPFailure++;
			printf("Obstime Week:%4hd\tSec:%13.5f SPP解算失败！SPPFailure;%3d\n", RAW->RovEpk.Time.Week, RAW->RovEpk.Time.SecOfWeek,SPPFailure);
			fprintf(LineLog, "Obstime Week:%4hd\tSec:%13.5f SPP解算失败！\n", RAW->RovEpk.Time.Week, RAW->RovEpk.Time.SecOfWeek);
		}
	} 


	delete RAW;

	fclose(res);
	fclose(LineLog);
	return Status;
}

