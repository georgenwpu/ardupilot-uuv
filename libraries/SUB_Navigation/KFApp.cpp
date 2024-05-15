#include "KFApp.h"

/***************************  class CKFApp  *********************************/
CKFApp::CKFApp(double ts):CSINSGNSS(15, 10, ts)
{
//state: 0-2 phi; 3-5 dvn; 6-8 dpos; 9-11 eb; 12-14 db; 15-17 lever; 18 dt
//meas:  0-1 fsf; 2-3 mag;  4-6 dvn;   7-9 dpos;
}

void CKFApp::Init(const CSINS &sins0, int grade)
{
	CSINSGNSS::Init(sins0);
	Pmax.Set2(fPHI(600,600),  fXXX(500),  fdPOS(1e6),  fDPH3(5000),  fMG3(10), fXXX(10),  0.1);
	Pmin.Set2(fPHI(0.1,1.0),  fXXX(0.001),  fdPOS(0.1),  fDPH3(0.1),  fUG3(10), fXXX(0.01),  0.0001);
	Pk.SetDiag2(fPHI(60,600),  fXXX(1.0),  fdPOS(10.0),  fDPH3(100),  fMG3(3.0), fXXX(1.0),  0.01);
	Qt.Set2(fDPSH3(1.0),  fUGPSHZ3(100.0),  fOOO,  fOO6,	fOOO, 0.0);
	Rt.Set2(fXXZ(0.3,3), fdLLH(10.0,30.0), 0.01,0.01,0.01,0.01);
	SetRmmbt(0.1, 10, 0.6);
	//FBTau.Set(fXX9(0.1), fXX6(1.0));
	FBTau.Set(fXX9(0.6), fXX6(0.0));        // 零偏不反馈
	SetMeasMask(01777);
	SetMeasStop(0xffffffff, 1.0);
}

void CKFApp::SetHk(CVect3& magn)
{
	Hk(6, 1) = -glv.g0;
	Hk(7, 0) =  glv.g0;
	Hk(8, 1) = -magn.k;  Hk(8, 2) =  magn.j;
	Hk(9, 0) =  magn.k;  Hk(9, 2) = -magn.i;
}

void CKFApp::SetFt(int nnq)
{
	CSINSGNSS::SetFt(15);
}


void CKFApp::SetMeas(CVect3& fsf, CVect3& mag, CVect3& magn)
{
	CVect3 df = sins.Cnb * (fsf/TS);
	CVect3 dm = sins.Cnb * mag - magn;
	Zk(6) = df.i;
	Zk(7) = df.j;
	Zk(8) = dm.i;
	Zk(9) = dm.j;
	SetMeasFlag(0001700);
}


void CKFApp::SetMeas(CVect3& vn0)
{
	CVect3 vn1, vb = sins.Cbn * vn0;
	vb.i = 0.0;   vb.k = 0.0;
	vn1 = sins.Cnb * vb;
	*(CVect3*)&Zk.dd[0] = sins.vn - vn1;
	SetMeasFlag(000007);
}

void CKFApp::SetMeas(const CVect3& pgps, const CVect3& vgps)
{
	if (!IsZero(pgps))
	{
		*(CVect3*)&Zk.dd[3] = sins.pos - pgps;
		SetMeasFlag(000070);

		*(CVect3*)&Zk.dd[0] = sins.vn - vgps;
		SetMeasFlag(000007);
	}
}

int CKFApp::Update(const CVect3* pwm, const CVect3* pvm, int nSamples, double ts)
{
	int res = TDUpdate(pwm, pvm, nSamples, ts, 30);
	avpi.Push(sins);

	return res;
}

void CKFApp::Feedback(int nnq, double fbts)
{
	CVect fbvct = dotmul(FBTau,Xk);
	qdelphi(sins.qnb, CVect3(&fbvct(0)));
	sins.vn -= CVect3(&fbvct(3));
	sins.pos -= CVect3(&fbvct(6));
	sins.eb += CVect3(&fbvct(9));
	sins.db += CVect3(&fbvct(12));
	Xk -= fbvct;
}



