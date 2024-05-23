#pragma once

#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS.h>
#include "KFApp.h"

#define STATE_INIT (100)
#define STATE_ALNT (200)
#define STATE_CLBT (201)
#define STATE_NAVI (300)

#define INITIM		(5)			// 对准时间
#define ALNTIM		(5)			// 对准时间
#define IKFTIM		(25)		// 对准时间
#define LOGONOFF	(TRUE)
#define THRESHOLD_K 	(114591.55)


class AP_PhiKF
{
public:
	AP_PhiKF(double ts);
	~AP_PhiKF();

	bool init();
	void setIMU(double wx, double wy,double wz, double fx, double fy,double fz);
	void setMag(double mx, double my,double mz);
	void setGNSS(double vE, double vN, double vU, double Lat, double Lng, double Alt);
	void TimeUpdate();
	void MeasureUpdate(CVect3 &fsf, CVect3 &mag, CVect3 &magnk);
	void MeasureUpdate(CVect3 &vn);
	void MeasureUpdate(CVect3 &vn, CVect3 &pos);
	void get_location(Location &loc);

	double ts;
	int k_nav = 0, k_ini = 0, k_aln = 0;    		// Sample of NavApp
	double t_nav = 0;    	// Time of NavApp
	int state = STATE_INIT;

	DataIMU imub;		// 接收imu数据
	DataMag magb;		// 接收磁场数据
	DataAVP ins_avp;
	DataGPS gps;
	CVect3 magn;		// 统计导航系数据
	CVect3 fsfb;		// 统计导航系数据
	CVect3 wibb;		// 统计导航系数据
	CVect3 vngps;		// 统计导航系数据
	CVect3 posgps;		// 统计导航系数据
	CMat3 CRe;			// 速度转移矩阵
	CVect3 localpos, localpos_gps;    // 相对位置
	float kvp = 1;		// 速度位置增益
	double threshold_wm = 0.05*glv.dps*TS;
	double threshold_vm = 0.02*glv.g0*TS;

	double pitch, roll, yaw;
	CVect3 dvn, dpos, vel, gnts;
	CKFApp kf;
	CEarth earth;
	bool gnssUdt = false;
	
	CVect3 vel_nhc_proc(CVect3 &vn, CMat3 &Cnb);
};
