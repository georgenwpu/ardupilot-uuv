#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/psins_output.h>

#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/sensor_gps.h>

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

using namespace time_literals;

class px4NavApp : public ModuleBase<px4NavApp>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	px4NavApp();
	~px4NavApp() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	static constexpr hrt_abstime INTERVAL_US = 10000_us;
	static constexpr int MAX_SENSORS = 4;

	int k_nav = 0, k_ini = 0, k_aln = 0;    		// Sample of NavApp
	double t_nav = 0;    	// Time of NavApp
	int state = STATE_INIT;

	vehicle_acceleration_s acce;
	vehicle_angular_velocity_s gyro;
	vehicle_magnetometer_s mage;
	sensor_gps_s gps_s;

	psins_output_s data{};

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
	CMahony mahony;
	CKFApp kf;
	CSINS sins;
	CEarth earth;
	bool gnssUdt = false;
	
	CVect3 vel_nhc_proc(CVect3 &vn, CMat3 &Cnb);

	void Run() override;

	// Publications
	uORB::Publication<psins_output_s> _orb_psins_pub{ORB_ID(orb_psins_output)};

	// Subscriptions
	uORB::Subscription  _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};   // regular subscription for parameter updates
	uORB::Subscription  _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)}; // regular subscription for additional data
	uORB::Subscription  _vehicle_magnetometer_sub{ORB_ID(vehicle_magnetometer)};   // regular subscription for parameter updates
	uORB::Subscription  _sensor_gps_sub{ORB_ID(sensor_gps)};   					   // regular subscription for parameter updates

	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
