//-*-Mode: C++;-*-
#ifndef _simVehicleParticular_h_
#define _simVehicleParticular_h_

#include "A2BehavioralModelUtil.h"
#include "A2SimVehicle.h"
#include "AKIProxie.h"

class A2BEHAVIORALEXPORT simVehicleParticular: public A2SimVehicle
{
private:
	double mDesiredDistance;
	bool mIsCacc;
	bool mCaccChecked;
	int mIdLaneType;
	double mMaxAcceleration;
	double mMaxDeceleration;

	StaticInfVeh mStaticInf;
	int mCacc2;
	int mCacc5;
	int mVeh2;
	int mVeh5;

	double mOldReactionTimeAtStop;
	double mOldSpeedAcceptance;
public:
	simVehicleParticular ( void *handlerVehicle, unsigned short idhandler,bool isFictitiousVeh );
	~simVehicleParticular ();

	void setCaccReservedLaneTypeId( int idLaneType );
	void setDesiredDistance(double value);
	double desiredDistance() const;
	void setCaccVehicle( bool value );
	bool isCaccVehicle() const;
	//returns if it's a leader of at least 2 CACC vehicles
	bool isPlatoonLeader( double maxHeadway );
	bool caccChecked() const;
	bool isInCaccReservedLane() const;

	void setMaxAcceleration( double value );
	void setMaxDeceleration( double value );
	double getMaxAcceleration() const;
	double getMaxDeceleration() const;

	void setNoCaccReactionTimeAtStop( double value );
	void setNoCaccSpeedAcceptance( double value );

	void setStaticInf( StaticInfVeh staticInf );
	void setCaccVehicles( int cacc2, int cacc5, int veh2, int veh5 );
	bool isInZone2();

	void restoreParameters();
	void modifyParameters();
};

#endif
