#include "AP_PhiKF.h"
#include "KFApp.h"

AP_PhiKF::AP_PhiKF(double ts0):kf(ts0)
{
	this->ts = ts0;
	init();
}

AP_PhiKF::~AP_PhiKF()
{
	
}

bool AP_PhiKF::init()
{
	magn = CVect3(0.0f, 0.0f, 0.0f);
	wibb = CVect3(0.0f, 0.0f, 0.0f);
	fsfb = CVect3(0.0f, 0.0f, 0.0f);
	vngps = CVect3(0.0f, 0.0f, 0.0f);
	posgps = CVect3(0.0f, 0.0f, 0.0f);
	localpos = CVect3(0.0f, 0.0f, 0.0f);
	earth = CEarth();
	kf = CKFApp(ts);
	state = STATE_INIT;			// 进入初始对准状态
	// GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PhiKF Init Finished!!");
	// GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PhiKF Init Finished!!");
	// GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PhiKF Init Finished!!");
	return true;
}

void AP_PhiKF::setIMU(double wx, double wy,double wz, double fx, double fy,double fz)
{
	imub.wm.i = -wx;
	imub.wm.j = -wy;
	imub.wm.k = -wz;
	imub.vm.i = -fx;
	imub.vm.j = -fy;
	imub.vm.k = -fz;
	imub.wm = imub.wm*DEG*ts;
	imub.vm = imub.vm*ts;
}

void AP_PhiKF::setMag(double mx, double my,double mz)
{
	magb.mag.i = -mx;
	magb.mag.j = -my;
	magb.mag.k = -mz;
}

void AP_PhiKF::setGNSS(double vE, double vN, double vU, double Lat, double Lng, double Alt)
{
	gps.vn.i = vE;
	gps.vn.j = vN;
	gps.vn.k = vU;
	gps.pos.i = Lat*1e-7*DEG;
	gps.pos.j = Lng*1e-7*DEG;
	gps.pos.k = Alt*0.01;
	gnssUdt = true;
}

void AP_PhiKF::TimeUpdate()
{
	k_nav++;
	t_nav = k_nav * ts;

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
			kf.Init(CSINS(ins_avp.att, CVect3(0.0,0.0,0.0), gps.pos, t_nav));
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
}

void AP_PhiKF::MeasureUpdate(CVect3 &fsf, CVect3 &mag, CVect3 &magnk)
{
	kf.SetMeas(fsf, mag, magn);
}

void AP_PhiKF::MeasureUpdate(CVect3 &vn, CVect3 &pos)
{
	kf.SetMeas(pos, vn);
}

void AP_PhiKF::get_location(Location &loc)
{
	loc.lat = (int32_t)ins_avp.pos.i/DEG*1e7;
	loc.lng = (int32_t)ins_avp.pos.j/DEG*1e7;
	loc.alt = (int32_t)ins_avp.pos.k*100;
}

CVect3 AP_PhiKF::vel_nhc_proc(CVect3 &vn, CMat3 &Cnb)
{
	CVect3 vb = (~Cnb)*vn;
    vb = Cnb*CVect3(0,vb.j,vb.k);
	return vb;
}

