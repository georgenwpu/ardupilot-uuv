/* KFApp c++ hearder file KFApp.h */
/*
	By     : Yan Gongmin @ NWPU
	Date   : 2020-12-09
	From   : College of Automation, 
	         Northwestern Polytechnical University, 
			 Xi'an 710072, China
*/

#ifndef _KFAPP_H
#define _KFAPP_H

#include "PSINS.h"

#define FRQ	FRQ100
#define TS	(1.0/FRQ)

class CKFApp:public CSINSGNSS
{
public:

	CKFApp(double ts=TS);
	virtual void Init(const CSINS &sins0, int grade=-1);

	virtual void SetFt(int nnq = 15) override;
	virtual void SetHk(CVect3& mag);
	virtual void SetMeas(CVect3& vn0);
	virtual void SetMeas(CVect3& fsf, CVect3& mag, CVect3& magn);
	virtual void SetMeas(const CVect3& pgps, const CVect3& vgps);
	int Update(const CVect3* pwm, const CVect3* pvm, int nSamples, double ts);
	virtual void Feedback(int nnq, double fbts) override;
};

typedef struct {
	CVect3 wm, vm;
	double t;
} DataIMU;

typedef struct {
	CVect3 mag;
	double t;
} DataMag;

typedef struct {
	CVect3 att, vn, pos;
	double t;
} DataAVP;

typedef struct {
	CVect3 vn, pos;
	double t;
} DataGPS;

#endif

