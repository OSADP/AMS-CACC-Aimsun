//-*-Mode: C++;-*-
#ifndef _behavioralModelParticular_h_
#define _behavioralModelParticular_h_

#include "A2BehavioralModelUtil.h"
#include "A2BehavioralModel.h"
#include "simVehicleParticular.h"
#include <map>

enum Type {eNone, eReservedLane, eTurning, eNotAllowedInfluence, eNotAllowed, ePTStopInfluence, eOnRamp, eLaneClosureInfluence, eIncidentInfluence, eLaneClosure, eIncident, ePTStop};


class A2BEHAVIORALEXPORT behavioralModelParticular: public A2BehavioralModel
{
private:
	int seed;
	double p_distance;
public:
	behavioralModelParticular();
	~behavioralModelParticular();	

	virtual double computeCarFollowingDecelerationComponentSpeed (A2SimVehicle *vehicle,double Shift,A2SimVehicle *vehicleLeader,double ShiftLeader,bool controlDecelMax=false);
	virtual A2SimVehicle * arrivalNewVehicle( void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh);
	virtual void removedVehicle( void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh );
	virtual bool evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed);
	virtual int evaluateLaneSelectionDiscretionary(A2SimVehicle *vehicle,bool LeftLanePossible,bool RightLanePossible);
	
	
	virtual bool evaluateLaneChanging( A2SimVehicle *vehicle,int threadId ){ return false; }
	virtual bool avoidCollision(A2SimVehicle *vehicle,A2SimVehicle *vehiclePre,double ShiftPre){ return false; }
	virtual double computeCarFollowingAccelerationComponentSpeed (A2SimVehicle *vehicle,double VelActual,double VelDeseada, double RestoCiclo);
   
private:
	int		mPreceedingVehicles; // number of preceeding vehicles to consider
	double	mKa; // acceleration factor
	double	mKv; // speed factor
	double	mKd; // distance factor
	double  mMaxHeadway; // maximum distance in metres that a CACC vehicle will try to follow its CACC leader
	double  mTargetHeadway; //minimum distance between vehicles
	bool	mDeactivateLaneChanging;
	int		mCaccLaneType;
	int		mCaccModel; // CACC model. 1: CACC, 2: CACC1, 3: CACC2
	int		mCacc02;
	int		mCacc05;
	int		mVeh02;
	int		mVeh05;
	void	*mActiveInRoadTypeAtt;
	void	*mCaccVehicleTypeAtt;
	void	*mApplyingCaccAtt;

	void readSettings();
	bool isInHovlane( A2SimVehicle* vehicle ) const;
	bool isInCaccRoadType( A2SimVehicle* vehicle ) const;
	double caccCarFollowing(simVehicleParticular* simVeh, simVehicleParticular* leader, double shift) const;
	double cacc1CarFollowing(simVehicleParticular* simVeh, simVehicleParticular* leader, double shift) const;
	double cacc2CarFollowing(simVehicleParticular* simVeh, simVehicleParticular* leader, double shift) const;	
	double calculateErrorSpeed(simVehicleParticular* previousVeh, simVehicleParticular* leader, int& preceedingVehicles) const;
};

#endif
