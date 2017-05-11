#include "behavioralModelParticular.h"
#include "simVehicleParticular.h"
#include "AKIProxie.h"
#include "ANGConProxie.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
using namespace std;
#define Tolerancia 0.01

behavioralModelParticular::behavioralModelParticular(): A2BehavioralModel()
{
   const unsigned short *randomSeedString = AKIConvertFromAsciiString( "GKReplication::randomSeedAtt" );
   seed = ANGConnGetAttributeValueInt( ANGConnGetAttribute( randomSeedString ), ANGConnGetReplicationId() );
   const unsigned short *param0= AKIConvertFromAsciiString( "GKExperiment::p_distance" );
   p_distance = ANGConnGetAttributeValueDouble( ANGConnGetAttribute( param0 ),ANGConnGetExperimentId()); 
   mActiveInRoadTypeAtt = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::ActivateCaccAtt") );
   mCaccVehicleTypeAtt = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::IsCaccAtt") );
   mApplyingCaccAtt = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::ApplyingCaccAtt") );      
   readSettings();
}
 
behavioralModelParticular::~behavioralModelParticular(){}

A2SimVehicle * behavioralModelParticular::arrivalNewVehicle( void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh)
{
	simVehicleParticular * res = new simVehicleParticular(handlerVehicle, idHandler, isFictitiousVeh);		
	res->setDesiredDistance( mTargetHeadway );
	res->setCaccReservedLaneTypeId( mCaccLaneType );
	return res;
}
void behavioralModelParticular::removedVehicle( void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh )
{	
}

void behavioralModelParticular::readSettings()
{	
	int experimentId = ANGConnGetExperimentId();
	void * att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::PreceedingVehicles") );
	mPreceedingVehicles = ANGConnGetAttributeValueInt(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::AccelerationFactor"));
	mKa = ANGConnGetAttributeValueDouble(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::SpeedFactor"));
	mKv = ANGConnGetAttributeValueDouble(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::DistanceFactor"));
	mKd = ANGConnGetAttributeValueDouble(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::Model"));
	mCaccModel = ANGConnGetAttributeValueInt(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::MaxHeadway"));
	mMaxHeadway = ANGConnGetAttributeValueDouble(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::TargetHeadway"));
	mTargetHeadway = ANGConnGetAttributeValueDouble(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::CACCVeh2"));
	mCacc02 = ANGConnGetAttributeValueInt(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::CACCVeh5"));
	mCacc05 = ANGConnGetAttributeValueInt(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::Veh2"));
	mVeh02 = ANGConnGetAttributeValueInt(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::Veh5"));
	mVeh05 = ANGConnGetAttributeValueInt(att, experimentId);

	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::LaneTypeId"));
	mCaccLaneType = ANGConnGetAttributeValueInt(att, experimentId);
	
	mApplyingCaccAtt = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::ApplyingCacc"));	
	att = ANGConnGetAttribute(AKIConvertFromAsciiString("CACC::DisableLaneChanging"));
	mDeactivateLaneChanging = ANGConnGetAttributeValueBool(att, experimentId);
}

bool behavioralModelParticular::isInHovlane(A2SimVehicle* vehicle) const
{
	bool res = vehicle->isInAReservedLane();
	return res;
}

bool behavioralModelParticular::isInCaccRoadType( A2SimVehicle* vehicle ) const
{
	bool res = false;	
	int roadTypeId = vehicle->getIdCurrentSectionRoadType();	
	res = ANGConnGetAttributeValueInt( mActiveInRoadTypeAtt, roadTypeId ) > 0;
	return res;
}

int behavioralModelParticular::evaluateLaneSelectionDiscretionary(A2SimVehicle *vehicle,bool LeftLanePossible,bool RightLanePossible)
{
	int res = -10;// do nothing

	simVehicleParticular * simVeh = dynamic_cast<simVehicleParticular*>(vehicle);
	
	int idVeh = vehicle->getId(); 
	if( mDeactivateLaneChanging == false && simVeh && simVeh->isCaccVehicle() && simVeh->isCurrentLaneOnRamp() == false &&
		simVeh->getNbLaneChanges2ReachNextValidLane() == 0 && simVeh->isInCaccReservedLane() == true ){
		double shift;
		const simVehicleParticular * leader = dynamic_cast<const simVehicleParticular*>( vehicle->getRealLeader( shift ) );
		if( leader && leader->isCaccVehicle() == false && simVeh->isPlatoonLeader( mMaxHeadway ) ){
			res = -10;
		}else{
			if( leader && ( leader->isCaccVehicle() == false || (leader->isCaccVehicle() == true && leader->getNbLaneChanges2ReachNextValidLane() != 0) ) ){//look for any CACC vehicle to follow
				bool leaderFound = false;			
				
				if( LeftLanePossible ){
					A2SimVehicle * vehUp, * vehDown;
					double shiftUp, shiftDw;				
					simVeh->getUpDown( -1, simVeh->getPosition(0), vehUp, shiftUp, vehDown, shiftDw );					
					const simVehicleParticular * newLeader = dynamic_cast<const simVehicleParticular*>( vehDown );
					if( newLeader && newLeader->isCaccVehicle() && newLeader->isInCaccReservedLane() == true ){
						double position, speed, posDw, speedDw;
						double gap = simVeh->getGap( 0, vehDown, shiftDw, position, speed, posDw, speedDw );					
						if( gap < mMaxHeadway ){
							res = -1;
						}
					}
				}
				if( res == -10 && RightLanePossible ){
					A2SimVehicle * vehUp, * vehDown;
					double shiftUp, shiftDw;				
					simVeh->getUpDown( 1, simVeh->getPosition(0), vehUp, shiftUp, vehDown, shiftDw );					
					const simVehicleParticular * newLeader = dynamic_cast<const simVehicleParticular*>( vehDown );	
					
					if( newLeader && newLeader->isCaccVehicle() && newLeader->isInCaccReservedLane() == true ){
						double position, speed, posDw, speedDw;
						double gap = simVeh->getGap( 0, vehDown, shiftDw, position, speed, posDw, speedDw );			
						if( gap < mMaxHeadway ){
							int idLeader = newLeader->getId();
							res = 1;
						}

					}
				}
			}else if( leader && leader->isCaccVehicle() == true ){
				res = 0;
			}
		}
	}

	return res;
}

double behavioralModelParticular::computeCarFollowingAccelerationComponentSpeed (A2SimVehicle *vehicle,double VelActual,double VelDeseada, double RestoCiclo)
{
	double res = -1;
	simVehicleParticular * simVeh = dynamic_cast<simVehicleParticular*>( vehicle );
	if( simVeh && simVeh->isCaccVehicle() && simVeh->isCurrentLaneOnRamp() == false && 
		simVeh->getNbLaneChanges2ReachNextValidLane() == 0 && simVeh->isInCaccReservedLane() == true ){
		
		unsigned int sectionId = simVeh->getIdCurrentSection();
		double speed = AKIInfNetGetSectionANGInf( sectionId ).speedLimit;

		//those following a CACC leader
		double shift;
		simVehicleParticular * leader = dynamic_cast<simVehicleParticular*>( vehicle->getRealLeader( shift ) );
		if( leader && leader->isCaccVehicle() ){	
			res = speed;
		}

		//leaders with followers
		if( simVeh->isPlatoonLeader( mMaxHeadway ) ){
			res = speed;
		}
	}

	return res;
}

double behavioralModelParticular::computeCarFollowingDecelerationComponentSpeed (A2SimVehicle *vehicle,double Shift,A2SimVehicle *vehicleLeader,double ShiftLeader,bool controlDecelMax )
{
	double speed = -1;
	return speed;
}

bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed)
{
	bool res = true;
	simVehicleParticular * simVeh = dynamic_cast<simVehicleParticular*>(vehicle);
	if( vehicle && simVeh && simVeh->isCaccVehicle() == false && simVeh->caccChecked() == false ){
		StaticInfVeh staticInf = AKIVehGetStaticInf( vehicle->getId() );
		int idVehicleType = AKIVehTypeGetIdVehTypeANG( staticInf.type );
		simVeh->setCaccVehicle( ANGConnGetAttributeValueInt( mCaccVehicleTypeAtt, idVehicleType ) > 0 );
		simVeh->setMaxAcceleration( staticInf.maxAcceleration );
		simVeh->setMaxDeceleration( staticInf.maxDeceleration );
		simVeh->setStaticInf( staticInf );
		simVeh->setCaccVehicles( mCacc02, mCacc05, mVeh02, mVeh05 );		
		simVeh->setNoCaccReactionTimeAtStop( simVeh->getReactionTimeAtStop() );
		simVeh->setNoCaccSpeedAcceptance( simVeh->getSpeedAcceptance() );
	}
	int idveh = vehicle->getId();		
	if( vehicle && simVeh && simVeh->isCaccVehicle() && simVeh->isCurrentLaneOnRamp() == false && 
		simVeh->getNbLaneChanges2ReachNextValidLane() == 0 && simVeh->isInCaccReservedLane() == true ){
		if( simVeh->isInZone2() == true ){		
			simVeh->restoreParameters();
			simVeh->setCaccVehicle( false );						
			res = false;
		}else{
			double shift;
			simVehicleParticular * leader = dynamic_cast<simVehicleParticular*>( vehicle->getRealLeader( shift ) );
			if( leader && leader->isCaccVehicle() ){	
				simVeh->modifyParameters();
				double newAcc;
				switch (mCaccModel){
				case 2:
					newAcc = cacc1CarFollowing(simVeh, leader, shift);
					break;
				case 3:
					newAcc = cacc2CarFollowing(simVeh, leader, shift);
					break;
				default:
					newAcc = caccCarFollowing(simVeh, leader, shift);
					break;
				}

				if( newAcc < 0 ){
					newAcc = std::max<double>( newAcc, simVeh->getMaxDeceleration() );
				}else{
					newAcc = std::min<double>( newAcc, simVeh->getMaxAcceleration() );
				}
				
				newspeed = std::max<double>( 0, simVeh->getSpeed(0) + newAcc * AKIGetSimulationStepTime() );
				newspeed = std::min<double>( simVeh->getFreeFlowSpeed(), newspeed );
				newpos = simVeh->getPosition(0) + newspeed * AKIGetSimulationStepTime();				
			}else{
				res = false;
				simVeh->restoreParameters();
			}
		}
	}else{//apply the regular car following by returning false
		res = false;					
		simVeh->restoreParameters();
	}

	return res;
}

double behavioralModelParticular::caccCarFollowing(simVehicleParticular* simVeh, simVehicleParticular* leader, double shift) const
{
	double res;
	double currentSpeed = simVeh->getSpeed(0);
	double desiredSpeed = simVeh->getFreeFlowSpeed();
		
	double acc_ref_v = (desiredSpeed - currentSpeed) / AKIGetSimulationStepTime();	
	double leaderAcc = (leader->getSpeed( 0 ) - leader->getSpeed( 1 )) / AKIGetSimulationStepTime();
	double leaderSpd = leader->getSpeed(0);

	if (leaderAcc < Tolerancia){
		leaderAcc = 0;
	}
	if (leaderSpd < Tolerancia){
		leaderSpd = 0;
	}
	double errorSpd = leaderSpd - currentSpeed;
	double position, speed, posDw, speedDw;
	double gap = simVeh->getGap(0, leader, shift, position, speed, posDw, speedDw);
	double errorDst = gap - simVeh->desiredDistance();

	double desiredAcc = (mKa * leaderAcc) + (mKv * errorSpd) + (mKd * errorDst);
	if (desiredAcc < 0){
		desiredAcc = std::max<double>(desiredAcc, simVeh->getDeceleration());
	}else{
		desiredAcc = std::min<double>(desiredAcc, simVeh->getAcceleration());
	}


	res = std::min<double>(acc_ref_v, desiredAcc);

	return res;
}

double behavioralModelParticular::cacc1CarFollowing( simVehicleParticular* simVeh, simVehicleParticular* realLeader , double shift) const 
{
	double tolerance = 0.01;
	double res = 100000;
	double currentSpeed = simVeh->getSpeed(0);
	double desiredSpeed = simVeh->getFreeFlowSpeed();
	
	const simVehicleParticular * previousVeh = simVeh;
	double aggDesiredDistance = simVeh->desiredDistance();
	double aggGap = 0.0;
	for (int i = 0; i < mPreceedingVehicles && realLeader && realLeader->isCaccVehicle() && aggGap < mMaxHeadway; i++){		
		double leaderSpd = realLeader->getSpeed(0);
				
		if (leaderSpd < Tolerancia){
			leaderSpd = 0;
		}
		double errorSpd = leaderSpd - currentSpeed;	
		double position, speed, posDw, speedDw;	
		double gap = previousVeh->getGap( 0, realLeader, shift, position, speed, posDw, speedDw );
		aggGap += gap;

		double errorDst = aggGap - aggDesiredDistance;

		double acc = (mKv * errorSpd) + (mKd * errorDst);
		if (acc < 0){
			acc = std::max<double>(acc, simVeh->getDeceleration());
		}else{
			acc = std::min<double>(acc, simVeh->getAcceleration());
		}			
		if( (errorDst + tolerance) > 0.0 && (errorDst - tolerance) < 0.0 ){
			acc = 0;//forcing 0 acc when the gap is reached
		}
		res = std::min<double>(acc, res);

		//leader can be fictitious and therefore not have any desired distance
		aggDesiredDistance += realLeader->getLength() + std::max<double>(0, realLeader->desiredDistance());
		
		previousVeh = realLeader;
		realLeader = dynamic_cast<simVehicleParticular*>( realLeader->getRealLeader( shift ) );
		
	}
	if( (res + tolerance) > 100000 && (res - tolerance) < 100000 ){
		double desiredAcc = (desiredSpeed - currentSpeed) / AKIGetSimulationStepTime();
		res = desiredAcc;
	}
	return res;
}


double behavioralModelParticular::cacc2CarFollowing(simVehicleParticular* simVeh, simVehicleParticular* realLeader, double shift) const
{
	double res;
	double currentSpeed = simVeh->getSpeed(0);
	double desiredSpeed = simVeh->getFreeFlowSpeed();
	double desiredAcc = (desiredSpeed - currentSpeed) / AKIGetSimulationStepTime();

	res = desiredAcc;

	double leaderSpd = realLeader->getSpeed(0);

	if (leaderSpd < Tolerancia){
		leaderSpd = 0;
	}
	double errorSpd = leaderSpd - currentSpeed;
	double position, speed, posDw, speedDw;	
	double gap = simVeh->getGap( 0, realLeader, shift, position, speed, posDw, speedDw );
	double errorDst = gap - simVeh->desiredDistance();

	double acc = (mKv * errorSpd) + (mKd * errorDst);
	res = acc;

	int nbPreceedingVehicles = mPreceedingVehicles - 1;
	if (nbPreceedingVehicles > 0){
		double errorSpeed = calculateErrorSpeed(simVeh, realLeader, nbPreceedingVehicles);
		if (nbPreceedingVehicles > 0){
			res += (mKv / nbPreceedingVehicles)*errorSpeed;
		}
	}
	if (res < 0){
		res = std::max<double>(simVeh->getDeceleration(), res);
	}else{
		res = std::min<double>(desiredAcc, res);
	}
	
	double tolerance = 0.01;
	if( (errorDst + tolerance) > 0.0 && (errorDst - tolerance) < 0.0 ){
		res = 0;//forcing 0 acc when the gap is reached
	}
	

	return res;
}

double behavioralModelParticular::calculateErrorSpeed(simVehicleParticular* simVeh, simVehicleParticular* leader, int& preceedingVehicles) const
{
	double res = 0;
	int nbPreceedingVehicles = 0;
	simVehicleParticular * vehDw;

	double shift;
	vehDw = dynamic_cast<simVehicleParticular*>( leader->getRealLeader( shift ) );
	
	for (; nbPreceedingVehicles < preceedingVehicles && vehDw; nbPreceedingVehicles++){
		leader = dynamic_cast<simVehicleParticular*>( vehDw->getRealLeader( shift ) );
		double position, speed, posDw, speedDw;	
		double gap = simVeh->getGap( 0, vehDw, shift, position, speed, posDw, speedDw );
		if( leader && leader->isCaccVehicle() && gap < mMaxHeadway ){
			double currentSpeed = vehDw->getSpeed(0);
			double desiredSpeed = vehDw->getFreeFlowSpeed();
			double leaderSpd = leader->getSpeed(0);

			if (leaderSpd < Tolerancia){
				leaderSpd = 0;
			}
			double errorSpd = leaderSpd - currentSpeed;
			res += errorSpd;
		}

		vehDw = leader;
	}

	preceedingVehicles = nbPreceedingVehicles;

	return res;
}
