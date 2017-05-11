#include "simVehicleParticular.h"
#include <math.h>

simVehicleParticular:: simVehicleParticular ( void *handlerVehicle, unsigned short idhandler,bool isFictitiousVeh ) : A2SimVehicle( handlerVehicle, idhandler,isFictitiousVeh )
{	
	mDesiredDistance = 0;
	mIsCacc = false;
	mCaccChecked = false;
}

simVehicleParticular::~simVehicleParticular ()
{
}

void simVehicleParticular::setDesiredDistance(double value)
{
	mDesiredDistance = value;
}

double simVehicleParticular::desiredDistance() const
{
	return mDesiredDistance;
}

bool simVehicleParticular::isCaccVehicle() const
{
	return mIsCacc;
}

void simVehicleParticular::setStaticInf( StaticInfVeh info )
{
	mStaticInf = info;
}

void simVehicleParticular::setCaccVehicles( int cacc2, int cacc5, int veh2, int veh5 )
{
	mCacc2 = cacc2;
	mCacc5 = cacc5;
	mVeh2 = veh2;
	mVeh5 = veh5;
}

void simVehicleParticular::setCaccVehicle( bool value )
{
	mIsCacc = value;
	mCaccChecked = true;
}

bool simVehicleParticular::caccChecked() const
{
	return mCaccChecked;
}

bool simVehicleParticular::isPlatoonLeader( double maxHeadway )
{
	bool res = false;
	double shift;
	simVehicleParticular * follower = dynamic_cast<simVehicleParticular*>( getRealFollower( shift ) );
	if( follower ){
		double position, speed, posDw, speedDw;
		double gap = follower->getGap( shift, this, 0, position, speed, posDw, speedDw );
		if( follower->isCaccVehicle() == true && fabs(gap) < maxHeadway ){
			//search for another CACC follower inside the maxHeadway range
			simVehicleParticular * follower2 = dynamic_cast<simVehicleParticular*>( follower->getRealFollower( shift ) );
			if( follower2 ){
				gap = follower2->getGap( shift, this, 0, position, speed, posDw, speedDw );
				if( follower2->isCaccVehicle() == true && fabs(gap) < maxHeadway ){
					res = true;
				}
			}
		}
	}
	return res;
}

void simVehicleParticular::setCaccReservedLaneTypeId( int idLaneType )
{
	mIdLaneType = idLaneType;
}

bool simVehicleParticular::isInCaccReservedLane() const
{
	bool res = false;

	res = isInAReservedLane() == true && getReservedLaneTypeId() == mIdLaneType;
	
	return res; 
}

void simVehicleParticular::setMaxAcceleration( double value )
{
	mMaxAcceleration = value;
}

void simVehicleParticular::setMaxDeceleration( double value )
{
	mMaxDeceleration = value;
}

double simVehicleParticular::getMaxAcceleration() const
{
	return mMaxAcceleration;
}

double simVehicleParticular::getMaxDeceleration() const
{
	return mMaxDeceleration;
}

bool simVehicleParticular::isInZone2()
{
	bool res = false;
	int oldType = mStaticInf.type;
	int newVehType = mStaticInf.type;
	if( AKIVehTypeGetIdVehTypeANG(mStaticInf.type) == mCacc2 ){
		newVehType = AKIVehGetVehTypeInternalPosition( mVeh2 );
	}else if( AKIVehTypeGetIdVehTypeANG(mStaticInf.type) == mCacc5 ){
		newVehType = AKIVehGetVehTypeInternalPosition( mVeh5 );
	}

	mStaticInf.type = newVehType;
	AKIVehSetStaticInf( getId(), mStaticInf );		
	updateLookAheadModel();
	res = isInZone2OfATurning();
	if( res == false ){
		mStaticInf.type = oldType;
		AKIVehSetStaticInf( getId(), mStaticInf );		
		updateLookAheadModel();
	}
	return res;
}

void simVehicleParticular::restoreParameters()
{
	setReactionTimeAtStop( mOldReactionTimeAtStop );
	setSpeedAcceptance( mOldSpeedAcceptance );
}

void simVehicleParticular::modifyParameters()
{
	setReactionTimeAtStop( getReactionTime() );
	setSpeedAcceptance( 1 );
}

void simVehicleParticular::setNoCaccReactionTimeAtStop( double value )
{
	mOldReactionTimeAtStop = value;
}

void simVehicleParticular::setNoCaccSpeedAcceptance( double value )
{
	mOldSpeedAcceptance = value;
}