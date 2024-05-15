#include "px4NavApp.h"
#include "KFApp.h"

px4NavApp::px4NavApp() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

px4NavApp::~px4NavApp()
{
	perf_free(_loop_interval_perf);
}

bool px4NavApp::init()
{
	// alternatively, Run on fixed interval
	ScheduleOnInterval(INTERVAL_US);
	magn = CVect3(0.0f, 0.0f, 0.0f);
	wibb = CVect3(0.0f, 0.0f, 0.0f);
	fsfb = CVect3(0.0f, 0.0f, 0.0f);
	vngps = CVect3(0.0f, 0.0f, 0.0f);
	posgps = CVect3(0.0f, 0.0f, 0.0f);
	localpos = CVect3(0.0f, 0.0f, 0.0f);
	earth = CEarth();
	kf = CKFApp();
	PX4_INFO("PSINS Start!!");
	state = STATE_INIT;			// 进入初始对准状态
	return true;
}

void px4NavApp::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_count(_loop_interval_perf);
	k_nav++;
	t_nav = k_nav * INTERVAL_US * 1e-6;

	//  数据惯性读取
	if (_vehicle_acceleration_sub.updated()) {
		if (_vehicle_acceleration_sub.copy(&acce)) {
			imub.vm = CVect3((double)acce.xyz[1], (double)acce.xyz[0], -(double)acce.xyz[2])*TS;
		}
	}
	if (_vehicle_angular_velocity_sub.updated()) {
		if (_vehicle_angular_velocity_sub.copy(&gyro)) {
			imub.wm = CVect3((double)gyro.xyz[1], (double)gyro.xyz[0], -(double)gyro.xyz[2])*TS;
			if (state==STATE_NAVI)		// 导航状态补偿零偏
				imub.wm = imub.wm - wibb;
			if (norm(imub.wm) <= threshold_wm) {
				kvp = norm(imub.wm) * THRESHOLD_K;
			} else {
				kvp = 1;
			}
		}
	}
	if (_vehicle_magnetometer_sub.updated()) {
		if (_vehicle_magnetometer_sub.copy(&mage)) {
			magb.mag = CVect3((double)mage.magnetometer_ga[1], (double)mage.magnetometer_ga[0], -(double)mage.magnetometer_ga[2]);
			kf.SetMeas(imub.vm, magb.mag, magn);
		}
	}

	// 读取卫导数据
	if (_sensor_gps_sub.updated()) {
		if (_sensor_gps_sub.copy(&gps_s)) {
			gps.vn = CVect3(gps_s.vel_e_m_s, gps_s.vel_n_m_s, -gps_s.vel_d_m_s);
			gps.pos = CVect3(gps_s.latitude_deg*glv.deg, gps_s.longitude_deg*glv.deg, gps_s.altitude_msl_m);		// 纬度 精度 高度
			gps.t = gps_s.timestamp;		// GNSS时间戳
			gnssUdt |= true;
			kf.SetMeas(gps.pos, gps.vn);
			if (state == STATE_INIT)
			{
				PX4_INFO("GNSS: %.4f  %.4f  %.4f %.4f  %.4f  %.4f %.4f", gps.vn.i, gps.vn.j, gps.vn.k,
					 gps.pos.i/glv.deg, gps.pos.j/glv.deg, gps.pos.k, t_nav);
			}
		}
	}

	if (state == STATE_INIT)
	{
		if (t_nav < INITIM)
		{
			k_ini++;
			vngps = vngps * (k_ini-1) / (k_ini) + gps.vn / (k_ini);
			posgps = posgps * (k_ini-1) / (k_ini) + gps.pos / (k_ini);
		}
		else
		{
			ins_avp.vn = vngps;
			ins_avp.pos = posgps;
			earth.Update(posgps, vngps, 1);
			gnts= earth.gn*TS;
			CRe = CMat3(0, 1.0/earth.RMh, 0, 1.0/earth.clRNh, 0, 0, 0, 0, 1);
			PX4_INFO("Finish Init: Pos0 = %.4f  %.4f  %.4f", posgps.i, posgps.j, posgps.k);
			state = STATE_ALNT;
		}
	}
	else if (state == STATE_ALNT)		//初始对准过程
	{
		k_aln++;
		fsfb = fsfb * (k_aln-1) / (k_aln) + imub.vm / (k_aln);
		wibb = wibb * (k_aln-1) / (k_aln) + imub.wm / (k_aln);
		magn = magn * (k_aln-1) / (k_aln) + magb.mag / (k_aln);

		pitch = asin(fsfb.j / norm(fsfb));
		roll = atan2(-fsfb.i, fsfb.k);
		yaw = atan2(magn.i * cos(roll) + magn.j * sin(roll),
			magn.i * sin(roll) * sin(pitch) + magn.j * cos(pitch) - magn.k * cos(roll) * sin(pitch));
		ins_avp.att = CVect3(pitch, roll, yaw);

		if (t_nav > ALNTIM){
			state = STATE_CLBT;		// 切换到标定补偿状态
		}
	}
	else if (state == STATE_CLBT)	// 导航定位过程
	{
		if (gnssUdt) {
			kf.Init(CSINS(ins_avp.att, CVect3(0.0,0.0,0.0), ins_avp.pos, t_nav));
			kf.SetHk(magn);
			state = STATE_NAVI;		// 切换到标定补偿状态
		}
	}
	else if (state == STATE_NAVI)	// 导航定位过程
	{
		if (gnssUdt && t_nav<=180) {
			kf.SetMeas(gps.pos, gps.vn);
			gnssUdt = false;
		}

		// 组合导航更新
		kf.Update(&imub.wm, &imub.vm, 1, TS);

		ins_avp.att = kf.sins.att;
		ins_avp.vn = kf.sins.vn;
		ins_avp.pos = kf.sins.pos;
		ins_avp.t = t_nav;

		// ins_avp.vn.k = gps.vn.k;
		// ins_avp.pos.k = gps.pos.k;

		localpos.i = (ins_avp.pos.j - posgps.j)*earth.clRNh;
		localpos.j = (ins_avp.pos.i - posgps.i)*earth.RMh;
		localpos.k = ins_avp.pos.k - posgps.k;
		localpos_gps.i = (gps.pos.j - posgps.j)*earth.clRNh;
		localpos_gps.j = (gps.pos.i - posgps.i)*earth.RMh;
		localpos_gps.k = gps.pos.k - posgps.k;
	}

	if (k_nav%100 == 0 && LOGONOFF)
	{
		if (state == STATE_ALNT)
		{
			PX4_INFO("Alinging:Time: %.4f s", t_nav);
			PX4_INFO("Alinging:Att: %.4f  %.4f  %.4f", pitch, roll, yaw);
		}
		else if (state == STATE_NAVI)
		{
			PX4_INFO("Navi:Time: %.4f s", ins_avp.t);
			PX4_INFO("Navi:Att: %.4f  %.4f  %.4f", ins_avp.att.i/glv.deg, ins_avp.att.j/glv.deg, ins_avp.att.k/glv.deg);
			PX4_INFO("Navi:Vel: %.4f  %.4f  %.4f", ins_avp.vn.i, ins_avp.vn.j, ins_avp.vn.k);
			PX4_INFO("Navi:Pos: %.4f  %.4f  %.4f", localpos.i, localpos.j, localpos.k);
			PX4_INFO("Navi:Pos: %.4f  %.4f  %.4f", localpos_gps.i, localpos_gps.j, localpos_gps.k);
		}
	}
	data.pitch = ins_avp.att.i;
	data.roll  = ins_avp.att.j;
	data.yaw   = ins_avp.att.k;
	data.ve    = ins_avp.vn.i;
	data.vn    = ins_avp.vn.j;
	data.vu    = ins_avp.vn.k;
	data.lat   = ins_avp.pos.i;
	data.lng   = ins_avp.pos.j;
	data.alt   = ins_avp.pos.k;
	data.timestamp = hrt_absolute_time();
	//data.timestamp = t_nav;
	_orb_psins_pub.publish(data);
}

int px4NavApp::task_spawn(int argc, char *argv[])
{
	px4NavApp *instance = new px4NavApp();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int px4NavApp::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int px4NavApp::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

CVect3 px4NavApp::vel_nhc_proc(CVect3 &vn, CMat3 &Cnb)
{
	CVect3 vb = (~Cnb)*vn;
    vb = Cnb*CVect3(0,vb.j,vb.k);
	return vb;
}


extern "C" __EXPORT int px4NavApp_main(int argc, char *argv[])
{
	return px4NavApp::main(argc, argv);
}
