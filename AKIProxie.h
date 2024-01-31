//DO NOT MODIFY THIS FILE
#ifndef _AKIProxie_h_
#define _AKIProxie_h_
//SDK
#include "AAPI_Util.h"
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>

extern "C" {

	// .......................................
	// 		ACTIONS MANAGEMENT 
	// .......................................
    AAPIEXPORT int AKIGetNbScenarioActions();
    AAPIEXPORT int AKIGetScenarioActions( int * actions );
    AAPIEXPORT bool AKIIsActionActive( int actionId );
    AAPIEXPORT int AKIGetActionType( int actionId );

	AAPIEXPORT void * AKIActionAddSpeedAction(int sectionId, double newSpeed, int vehTypePos, double acomplianceLevel);
	AAPIEXPORT void * AKIActionAddSpeedActionByID(int idAction, int nbSections, int * sectionIDs, double newSpeed, int vehType, double complianceLevel, bool considerSpeedAcceptance);
	AAPIEXPORT void * AKIActionAddTurnSpeedActionByID( int idAction, int nbTurns, int * turnIDs, double speed, int vehicleType, double complianceLevel, bool considerSpeedAcceptance );
	AAPIEXPORT void * AKIActionAddDetailedSpeedAction(int ang_sectionId, int laneId, int ang_segmentId, double newSpeed, int vehTypePos, double acomplianceLevel, bool considerSpeedAcceptance);
	AAPIEXPORT void * AKIActionAddDetailedSpeedActionByID(int idAction, int nbSections, int * sectionIDs, int laneId, int and_fromSegmentId, int and_toSegmentId, double newSpeed, int vehType, double complianceLevel, bool considerSpeedAcceptance);

	AAPIEXPORT void * AKIActionCloseLaneAction(int sectionId, int alane, int vehTypePos);
	AAPIEXPORT void * AKIActionCloseLaneActionByID(int idAction, int sectionId, int alane, int vehTypePos);
	AAPIEXPORT void * AKIActionCloseLaneActionBySegment(int sectionId, int alane, int ang_segmentFromId, int ang_segmentToId, int vehTypePos, bool apply2LanesCF, double visibilityDistance);
	AAPIEXPORT void * AKIActionCloseLaneActionBySegmentByID(int idAction, int sectionId, int alane, int ang_segmentFromId, int ang_segmentToId, int vehTypePos, bool apply2LanesCF, double visibilityDistance);
	AAPIEXPORT void * AKIActionCloseLaneDetailedAction(int sectionId, int alane, int vehTypePos, bool apply2LanesCF, double visibilityDistance );
	AAPIEXPORT void * AKIActionCloseLaneDetailedActionByID(int idAction, int sectionId, int alane, int vehTypePos, bool apply2LanesCF, double visibilityDistance );
	AAPIEXPORT void * AKIActionAddNextTurningODAction(int sectionId, int anextSection, int aOrigin, int aDest, int vehTypePos, int asectionInPath, double acomplianceLevel, double visibilityDistance);
	AAPIEXPORT void * AKIActionAddNextTurningODActionByID(int idAction, int sectionId, int anextSection, int aOrigin, int aDest, int VehType, int asectionInPath, double acomplianceLevel, double visibilityDistance);
	AAPIEXPORT void * AKIActionAddNextTurningResultAction(int sectionId, int aoldNextSection, int anewNextSection, int vehTypePos, double acomplianceLevel);
	AAPIEXPORT void * AKIActionAddNextTurningResultActionByID(int idAction, int sectionId, int aoldNextSection, int anewNextSection, int VehType, double acomplianceLevel);
	AAPIEXPORT void * AKIActionAddChangeDestAction(int sectionId, int anewDest, int aOrigin, int aDest, int vehTypePos, double acomplianceLevel);
	AAPIEXPORT void * AKIActionChangeTurningProbAction(int sectionId, int nbnewProb, int *aNextSection, double *anewProb, int vehTypePos);
	AAPIEXPORT void * AKIActionDisableReservedLaneAction(int sectionId, int alane, int ang_segmentId);
	AAPIEXPORT void * AKIActionDisableReservedLaneActionByID(int actionId, int sectionId, int alane, int ang_segmentId);
	AAPIEXPORT void * AKIActionAddEnRouteAssignmentAction(int actionId, int ang_sectionId, int sectionInPath, int aOrigin, int aDest, int VehType, double acomplianceLevel, bool reevaluateAction, int routeChoiceType, double routeChoiceParam1, double routeChoiceParam2, double routeChoiceParam3);
	AAPIEXPORT void * AKIActionCongestionPricingODAction(int actionId, int ang_sectionId, int ang_HOTSection, int aOrigin, int aDest, int vehTypePos, double acomplianceLevel, double avisibilityDistance, int aType, bool isTolled, int aGPSection, int amethod, double athreshold, double logitScaleFactor, double penalisationGPL, double penalisationHOTL, int costType, double xshift, double yshift, bool reexecute );


	AAPIEXPORT void AKIActionRemoveAction(void *a2kaction);
	AAPIEXPORT void AKIActionRemoveActionByID( int a2kactionId );
	AAPIEXPORT void AKIActionReset();

	AAPIEXPORT void * AKIActionAddNextSubPathODAction(int ang_sectionId, int nbNextSections, int * anextSections, int aOrigin, int aDest, int vehTypePos, int asectionInPath, double acomplianceLevel, double visibilityDistance);
	AAPIEXPORT void * AKIActionAddNextSubPathODActionByID(int actionId, int ang_sectionId, int nbNextSections, int * anextSections, int aOrigin, int aDest, int VehType, int asectionInPath, double acomplianceLevel, double visibilityDistance);
	AAPIEXPORT void * AKIActionAddNextSubPathResultAction(int ang_sectionId, int nbNextSections, int * anextSections, int vehTypePos, double acomplianceLevel);
	AAPIEXPORT void * AKIActionAddNextSubPathResultActionByID(int actionId, int ang_sectionId, int nbNextSections, int * anextSections, int VehType, double acomplianceLevel);
	AAPIEXPORT void * AKIActionAddNextSubPathPTAction(int ang_sectionId, int nbSections, int * aSections, int idline, int VehType, double acomplianceLevel, double visibilityDistance);
	AAPIEXPORT void * AKIActionAddNextSubPathAndStopsPTAction(int ang_sectionId, int nbSections, int * aSections, int *aStops, double *aDwellTime, double *aoffset, int idline );
	AAPIEXPORT void * AKIActionAddCloseTurningODAction(int sectionId, int anewSection2Close, int aOrigin, int aDest, int vehTypePos, double acomplianceLevel, double visibilityDistance = 200.0, bool localEffect = true, int sectionAffectingPathCostId = -1 );
	AAPIEXPORT void * AKIActionAddCloseTurningODActionByID(int idAction, int sectionId, int anewSection2Close, int aOrigin, int aDest, int vehType, double acomplianceLevel, double visibilityDistance = 200.0, bool localEffect = true, int sectionAffectingPathCostId = -1 );

	AAPIEXPORT void AKIActionModifyNextTurningODAction( void * a2kaction, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyNextTurningODActionByID( int a2kactionId, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyNextTurningResultAction( void * a2kaction, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyNextTurningResultActionByID( int a2kactionId, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyChangeDestAction( void * a2kaction, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyChangeDestActionByID( int a2kactionId, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyNextSubPathResultAction( void * a2kaction, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyNextSubPathResultActionByID( int a2kactionId, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyNextSubPathODAction( void * a2kaction, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyNextSubPathODActionByID( int a2kactionId, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyCloseTurningODAction( void * a2kaction, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyCloseTurningODActionByID( int a2kactionId, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyEnRouteAssignmentAction( void * a2kaction, double acomplianceLevel);
	AAPIEXPORT void AKIActionModifyEnRouteAssignmentActionByID( int a2kactionId, double acomplianceLevel);


	// .......................................
	//		INFORMATION OF FORCE TURNINGS
	// .......................................

	/*! \brief Subpath's information

	  */
	struct AAPIEXPORT A2KSubPathProportion {

		A2KSubPathProportion()
		{
		}

		A2KSubPathProportion( const std::vector<int>& aSections, double aPercentage )
		: mSections( aSections )
		, mPercentage( aPercentage )
		{
		}

		std::vector<int> mSections;
		double mPercentage = 1.0;
	};

	/*! \brief Destination's information

	  */
	struct AAPIEXPORT A2KDestinationProportion {

		A2KDestinationProportion()
		{
		}

		A2KDestinationProportion( int aNewDest, double aPercentage, const std::vector<A2KSubPathProportion*>& aSubpaths )
		: mNewDest( aNewDest )
		, mPercentage( aPercentage )
		, mSubpaths( aSubpaths )
		{
		}

		int mNewDest = -1;
		double mPercentage = 1.0;
		std::vector<A2KSubPathProportion*> mSubpaths;
	};

	AAPIEXPORT void * AKIActionAddForceTurningODActionByID( int idAction, int sectionId, const std::vector<A2KDestinationProportion*>& aNextSections, int aOrigin, int aDest, int vehType, int aSectionInPath, double aComplianceLevel, double visibilityDistance );
	AAPIEXPORT void * AKIActionAddForceTurningODSubPathActionByID( int idAction, int sectionId, const std::vector<A2KDestinationProportion*>& aNextSections, int aOrigin, int aDest, int vehType, int aSectionInPath, double aComplianceLevel, double visibilityDistance );
	AAPIEXPORT void * AKIActionAddForceTurningResultActionByID( int idAction, int sectionId, int aOldNextSection, const std::vector<A2KDestinationProportion*>& aNewNextSections, int  vehType, double aComplianceLevel );
	AAPIEXPORT void * AKIActionAddForceTurningResultSubPathActionByID( int idAction, int sectionId, const std::vector<A2KDestinationProportion*>& aNewNextSections, int vehType, double aComplianceLevel );

	// .......................................
	//		REROUTING
	// .......................................
	AAPIEXPORT void * AKIActionAddChangeDestActionByID(int idAction, int sectionId, const std::vector<A2KDestinationProportion*>& aNewDests, int aOrigin, int aDest, int vehType, double aComplianceLevel);
	AAPIEXPORT void * AKIActionAddChangeDestParkRideActionByID(int idAction, int sectionId,  const std::vector<A2KDestinationProportion*>& aNewDests, int aOrigin, int aDest, int vehType, double complianceLevel, double scaleFactor);

	// .......................................
	// 		INFORMATION OF GENERAL VEHICLES
	// .......................................


	#define AKIInfVehGetMem 			-4001
	#define AKIInfVehUnknownSection 	-4002
	#define AKIInfIndexNotValid 		-4003
	#define AKIInfNotReady 				-4004
	#define AKIInfVehUnknownJunction	-4005
	#define AKIInfVehNotFound			-4006
	#define AKIInfVehInvalidParam		-4007
	#define AKIVehInvalidVehicleTypeId  -4008
	#define AKIInfVehNotAvalaible		-1

	/*!  \brief Vehicle's dynamic information.

	  */
	struct AAPIEXPORT InfVeh{
		//! 0, OK, else error code
		int report;
		//! the vehicle identifier
		int idVeh;
		//! the vehicle type (car, bus, truck, etc.) from 1 to AKIVehGetNbVehTypes ()
		int type;
		//! current id section if it's in a section
		int idSection;
		//! current id segment if it's in a section
		int segment;
		//! current id lane if it's in a section
		int numberLane;
		//! current id junction if it's in a junction
		int idJunction;
		//! the origin section identifier
		int idSectionFrom;
		//! number of lanes of the origin section where the vehicle enters the junction
		int idLaneFrom;
		//! destination section identifier
		int idSectionTo;
		//! number of lanes of the destination section where the vehicle exits the junction
		int idLaneTo;
		//! the position inside the junction = distance (m or feet, depending on the units defined in the network) from the entrance to the junction.
		double CurrentPos;
        //! the distance to end of the turn (m or feet, depending on the units defined in the network).
		double distance2End;
		//! x world coordinates of the middle point of the front bumper of the vehicle.
		double xCurrentPos;
		//! y world coordinates of the middle point of the front bumper of the vehicle.
		double yCurrentPos;
		//! z world coordinates of the middle point of the front bumper of the vehicle.
		double zCurrentPos;
		//! x world coordinates of the middle point of the back bumper of the vehicle.
		double xCurrentPosBack;
		//! y world coordinates of the middle point of the back bumper of the vehicle.
		double yCurrentPosBack;
		//! z world coordinates of the middle point of the back bumper of the vehicle.
		double zCurrentPosBack;
		//! current speed (in km/h or mph, depending on the units defined in the network
		double CurrentSpeed;
		//! speed in the previous simulation step (in km/h or mph, depending on the units defined in the network
		double PreviousSpeed;
		//!  total distance traveled (m or feet)
		double TotalDistance;
		//! the absolute generation time of the vehicle into the system. If no virtual queue found in its entrance section it will be the same as the SystemEntranceT.
		double SystemGenerationT;
		//! the absolute entrance time of the vehicle in the system
		double SystemEntranceT;
		//! the absolute entrance time of the vehicle in last section
		double SectionEntranceT;
		//! the current stop time
		double CurrentStopTime;
        //! true when the vehicle is stopped
        bool stopped;
		//! the number of lost turnings Turns
		unsigned int mNbLostTurnings;
		//! energy state(SOC/Fuel tank level)
		double energyState;
	};

	/*!  \brief Vehicle's static information.
	  */
	struct AAPIEXPORT StaticInfVeh{
		//! 0, OK, else error code
		int report;
		//! vehicle identifier
		int idVeh;
		//! vehicle type (car, bus, truck, etc.)
		int type;
		//! vehicle length (m or feet, depending on the units defined in the network).
		double length;
		//! vehicle width (m or feet, depending on the units defined in the network).
		double width;
		//! Maximum desired speed of the vehicle (km/h or mph, depending on the units defined in the network).
		double maxDesiredSpeed;
		//! Maximum acceleration of the vehicle (m/s2 or ft/ s2, depending on the units defined in the network).
		double maxAcceleration;
		//! Maximum deceleration of the vehicle that can apply under normal conditions (m/s2 or ft/ s2, depending the units defined in the network).
		double normalDeceleration;
		//! Maximum deceleration of the vehicle that can apply under special conditions (m/s2 or ft/ s2, depending the units defined in the network).
		double maxDeceleration;
		//! degree of acceptance of the speed limits.
		double speedAcceptance;
		//! distance that the vehicle keeps between itself and the preceding vehicle (m or feet, depending on the units defined in the network).
		double minDistanceVeh;
		//! time after which the vehicle becomes more aggressive in give way situations (seconds).
		double giveWayTime;
		//! level of compliance of the vehicle to guidance indications (between 0 and 1).
		double guidanceAcceptance;
		//!  0 means vehicle will not change path en route, 1 means vehicle will update path en route depending on the defined percentege of vehicles that update path en route.
		int enrouted;
		//! 0 means vehicle not equipped, 1 means vehicle equipped.
		int equipped;
		//! 0 means vehicle not tracked, 1 means vehicle tracked.
		int tracked;
		//! means vehicle keep fast lane during overtaking
		bool  keepfastLane;
		//! safety margin factor
		double safetyMarginFactor;
		//! minimum headway to keep with its leader
		double headwayMin;
		//! estimation of the acceleration of the leader
		double sensitivityFactor;
		//! reaction time of the vehicle
		double reactionTime;
		//! reaction time at stop of the vehicle
		double reactionTimeAtStop;
		//! reaction time of the vehicle when stopped at a red traffic light and it is the first vehicle
		double reactionTimeAtTrafficLight;
        //! lane-changing cooperation
        bool laneChangingCooperation;
        //! lane-changing aggressiveness level
        double laneChangingAggressivenessLevel;
		//! look-ahead distance factor
        double distanceZoneFactor;
        //! Identifier of centroid origin of the vehicle, when the traffic conditions are defined by an OD matrix.
		int centroidOrigin;	
		//! Identifier of centroid destination of the vehicle, when the traffic conditions are defined by an OD matrix.
		int centroidDest;
		//! Identifier of exit section destination of the vehicle, when the destination centroid uses percentages as destination (otherwise is -1) and the traffic conditions are defined by an OD matrix.
		int idsectionExit;
		//! Identifier of Transit Line, when the vehicle has been generated as a transit vehicle.
		int idLine;
		//!	Pointer to internal veh, only for internal use
		void * internalInfo;
		//! Safety margin to consider in TwoWay Model
		double margin2Overtake;
		//! Manouver Time
		double TM;
		//! Abort Time
		double TA;
		//! Colision Time
		double TC;
		//! Tiempo hasta el final de la zona de adelantamiento
		double TS;
		//! Overtaking Desired Speed
		double VelMaxOvertaking;
		//! factors for acceleration and deceleration for modeling capacity drop
		double AStateFactor;
		double BStateFactor;
		
		//! Sitraffic Dectection pattern info
		int tripNumber;
		short timetableDev;
		int trainLength;
		int manualDirection;
		//! Sitraffic OCIT
		int priority;
		int lineNumber;
		int routeNumber;

		//MFC, emission & consumption models
		int engineTypeId;
		//MFC
		int vehicleSegmentId;
		//LEM
		int EUEmissionId;
		//Energy consumption Model
		double energyCapacity;
	};

	/*! \brief Vehicles using ACC/CACC Model
	 *
	 */
	struct AAPIEXPORT StaticInfVehACCParams
	{
		//! Description
		int report; // Code error. 0 if it's ok.
		//! Description
		int idVeh;
		//! Description
		int accType; // 0: None, 1: ACC, 2: CACC

		// ACC Parameters

		//! The lower threshold for the space between the rear bumper of a vehicle and the front bumper of the following (m).
		double minClearanceDistance;
		//! The upper threshold for the space between the rear bumper of a vehicle and the front bumper of the following (m).
		double maxClearanceDistance;
		//! The gain on the speed difference between the free flow speed and the subject vehicle's current speed (s-1).
		double speedGainFreeFlow;
		//! The gain on the position difference between the preceding vehicle and the subject vehicle (s-2).
		double distanceGain;
		//! The gain on the speed difference between the preceding vehicle and the subject vehicle (s-1).
		double speedGainPrec;
		//! The desired time gap of the ACC controller (s).
		double desiredTimeGap;

		// CACC Parameters

		//! The gain on the position difference between the preceding connected vehicle and the subject CACC vehicle (s-1).
		double connectedDistanceGain;
		//! The gain on the speed difference between the preceding connected vehicle and the subject CACC vehicle.
		double connectedSpeedGain;
		//! The lower threshold for the time gap (s).
		double minTimeGapThreshold;
		//! The upper threshold for the time gap (s).
		double maxTimeGapThreshold;
		//! The constant time gap between the last vehicle of the preceding connected string and the subject CACC vehicle (s).
		double followerTimeGap;
		//! The constant time gap between the last vehicle of the preceding connected string and the subject CACC vehicle (s).
		double leaderTimeGap;
	};

	struct AAPIEXPORT LeaderInfVeh {
		//! Description
		int report; // Code error (AKIInfVehNotFound). 0 if it's ok.
		//! Description
		int idVeh;
		//! Description
		int idLeaderVeh; // Zero in case no leader
		//! The time between the front bumper of a vehicle and the front bumper of the following vehicle (s).
		double headway;
		//! The time between the rear bumper of a vehicle and the front bumper of the following vehicle  (s).
		double gap;
		//! The space between the front bumper of a vehicle and the front bumper of the following vehicle (m).
		double spacing;
		//! The space between the rear bumper of a vehicle and the front bumper of the following (m).
		double clearance;
	};

	/*! \brief Vehicle's world coordinates.
	  */
	struct AAPIEXPORT VehPos{
		//! x world coordinates of the middle point of the front bumper of the vehicle.
		double xPos;
		//! y world coordinates of the middle point of the front bumper of the vehicle.
		double yPos;
		//! z world coordinates of the middle point of the front bumper of the vehicle.
		double zPos;
		//!  x world coordinates of the middle point of the back bumper of the vehicle.
		double xPosBack;
		//!  y world coordinates of the middle point of the back bumper of the vehicle.
		double yPosBack;
		//!  z world coordinates of the middle point of the back bumper of the vehicle.
		double zPosBack;
	};

	/*! \brief  Vehicle's world coordinates during last simulation step.
	  */
	struct AAPIEXPORT InfVehPos{
		//! 0, OK, otherwise an error code
		int	report;
		//! the vehicle identifier
		int idVeh;
		//! number of positions during last simulation step (input parameter).
		int Npos;
		//! Array with the positions during last simulation step.  After using this function, this array must be deallocated using "free" function.
		VehPos *vehiclePos;
	};

	/*! \brief Vehicle's dynamic graphic information.
	  */
	struct AAPIEXPORT GraphicInfVeh{
		//! 0, OK, otherwise an error code
		int report;
		//! the vehicle identifier
		int idVeh;
		//! Indicates if left turn signal is activated.
		bool leftTurnSignal;
		//! Indicates if right turn signal is activated.
		bool rightTurnSignal;
		//! Indicates if brake ligth is activated.
		bool brakeLight;
		//! Indicates if it's driving backwards.
		bool drivingBackwards;
	};
	
	AAPIEXPORT int						AKIRemoveVehicle(int aidSec, int indexveh );

	/* activates the graphical info by vehicle (turn lights, brake lights...). When always false the 
		graphical inf will be updated only when simulating interactively so it will not be updated 
		when simulating in batch or in fast forward mode */
	AAPIEXPORT int						AKIVehEnableGraphicalInf( bool always = false );
	AAPIEXPORT int						AKIVehDisableGraphicalInf();

	/// ACC: Default values, get and set (internal)
	AAPIEXPORT void						AKISetDefaultStaticInfVehACCParams( StaticInfVehACCParams & accParams );
	AAPIEXPORT StaticInfVehACCParams	AKIGetACCParamsFromVeh( void * pVeh );
	AAPIEXPORT int						AKISetACCParamsToVeh( void * pVeh, StaticInfVehACCParams staticinfVehACC );

	AAPIEXPORT int						AKIVehStateGetNbVehiclesSection( int aidSec, bool considerAllSegments );
	AAPIEXPORT InfVeh					AKIVehStateGetVehicleInfSection( int aidSec, int indexveh );
	AAPIEXPORT StaticInfVeh				AKIVehGetVehicleStaticInfSection( int aidSec, int indexveh );
	AAPIEXPORT StaticInfVehACCParams	AKIVehGetVehicleStaticInfACCParamsSection( int aidSec, int indexveh );
	AAPIEXPORT int						AKIVehSetVehicleStaticInfSection( int aidSec, int indexveh, StaticInfVeh staticinfVeh );
	AAPIEXPORT int						AKIVehSetVehicleStaticInfACCParamsSection( int aidSec, int indexveh, StaticInfVehACCParams staticinfVehACC );
	AAPIEXPORT LeaderInfVeh				AKIVehGetLeaderVehInfSection( int aidSec, int indexveh );
	AAPIEXPORT InfVehPos				AKIVehGetVehicleGetPosSection( int asect, int indexveh, int nbPos );
	AAPIEXPORT int						AKIVehGetVehicleGetPosSectionWithStruct( int asect, int indexveh, int nbPos, InfVehPos *pinfVehPos );
	AAPIEXPORT GraphicInfVeh				AKIVehGetVehicleGraphicInfSection( int asect, int indexveh );
	AAPIEXPORT int						AKIVehSetDrivingBackwards( int aidSec, int indexveh, bool value );
	
	AAPIEXPORT int						AKIVehStateGetNbVehiclesJunction( int ajunction );
	AAPIEXPORT InfVeh					AKIVehStateGetVehicleInfJunction( int ajunction, int indexveh );
	AAPIEXPORT StaticInfVeh				AKIVehGetVehicleStaticInfJunction( int ajunction, int indexveh );
	AAPIEXPORT StaticInfVehACCParams	AKIVehGetVehicleStaticInfACCParamsJunction( int ajunction, int indexveh );
	AAPIEXPORT int						AKIVehSetVehicleStaticInfJunction( int ajunction, int indexveh, StaticInfVeh staticinfVeh );
	AAPIEXPORT int						AKIVehSetVehicleStaticInfACCParamsJunction( int ajunction, int indexveh, StaticInfVehACCParams staticinfVehACC );
	AAPIEXPORT InfVehPos				AKIVehGetVehicleGetPosJunction( int ajunction, int indexveh, int nbPos );
	AAPIEXPORT int						AKIVehGetVehicleGetPosJunctionWithStruct( int ajunction, int indexveh, int nbPos, InfVehPos *pinfVehPos );
	AAPIEXPORT GraphicInfVeh			AKIVehGetVehicleGraphicInfJunction( int ajunction, int indexveh );
	AAPIEXPORT LeaderInfVeh				AKIVehGetLeaderVehInfJunction(int ajunction, int indexveh);

	AAPIEXPORT InfVeh					AKIVehGetInf( int aidVeh );
	AAPIEXPORT StaticInfVeh				AKIVehGetStaticInf( int aidVeh );
	AAPIEXPORT StaticInfVehACCParams	AKIVehGetStaticInfACCParams( int aidVeh );
	AAPIEXPORT int						AKIVehSetStaticInf( int aidVeh, StaticInfVeh staticinfVeh );
	AAPIEXPORT int						AKIVehSetStaticInfACCParams( int aidVeh, StaticInfVehACCParams staticinfVehACC );
	AAPIEXPORT LeaderInfVeh				AKIVehGetLeaderInfVeh( int aidVeh );

	AAPIEXPORT int						AKIVehGetNbVehTypes();
	AAPIEXPORT int						AKIVehTypeGetIdVehTypeANG( int vehTypePos );
	AAPIEXPORT double					AKIVehGetMinLengthVehType( int vehTypePos );
	AAPIEXPORT double					AKIVehGetMaxLengthVehType( int vehTypePos );

	// Gets the name of the given vehTypePos it is in utf16 and \0 terminated. The result has to be deleted with delete[]
	AAPIEXPORT const unsigned short		* AKIVehGetVehTypeName( int vehTypePos );

	// Gets the vehTypePos of the given name. Has to be in utf16 and \0 terminated
	AAPIEXPORT int						AKIVehGetVehTypeInternalPosition( int aimsunVehicleTypeId );

    AAPIEXPORT int						AKIVehGetLeaderId( int idVeh );
    AAPIEXPORT int						AKIVehGetFollowerId( int idVeh );

	AAPIEXPORT double					AKIVehGetAccumulatedDelay(int idVeh);

	AAPIEXPORT bool						AKIVehTypeGetImprudentLaneChanging( int idVehicleType );
	AAPIEXPORT int						AKIVehTypeSetImprudentLaneChanging( int idVehicleType, bool value );
	AAPIEXPORT double					AKIVehTypeGetPercentageForStayingInFastLane( int idVehicleType );
	AAPIEXPORT int						AKIVehTypeSetPercentageForStayingInFastLane( int idVehicleType, double newPercentage );
    AAPIEXPORT double					AKIVehTypeGetOvertakeSpeedThreshold( int idVehicleType );
    AAPIEXPORT int						AKIVehTypeSetOvertakeSpeedThreshold( int idVehicleType, double newPercentage );
    AAPIEXPORT double					AKIVehTypeGetLaneRecoverySpeedThreshold( int idVehicleType );
    AAPIEXPORT int						AKIVehTypeSetLaneRecoverySpeedThreshold( int idVehicleType, double newPercentage );

	AAPIEXPORT int						AKIVehStateGetNbSectionsVehiclePathJunction( int aidJunction, int indexveh, int idsection );
	AAPIEXPORT int						AKIVehStateGetIdSectionVehiclePathJunction( int aidJunction, int indexveh, int idsection, int indexsection );
	AAPIEXPORT int						AKIVehStateGetNbSectionsVehiclePathSection( int idSect, int indexveh, int idsection );
	AAPIEXPORT int						AKIVehStateGetIdSectionVehiclePathSection( int idSect, int indexveh, int idsection, int indexsection );
	AAPIEXPORT int						AKIVehTrackedGetNbSectionsVehiclePath( int idveh );
	AAPIEXPORT int						AKIVehTrackedGetIdSectionVehiclePath( int idveh, int indexsection );
	AAPIEXPORT int						AKIRemoveVehicleJunction( int idJunction, int indexveh );
	AAPIEXPORT int						AKIVehTrackedRemove( int idveh );
	AAPIEXPORT int						AKIVehTrackedSetLanesTrajectory( int aidVeh, int nbSegmentsInTrajectory, int * sectIds, int * lanes, int idLastSection );

	// .......................................
	// 		INFORMATION OF DETECTORS AND DETECTORS MEASURES 
	// .......................................


	#define AKIDETUnknownDetector  		-3010
	#define AKIDETIncorrectInterval  	-3011
	#define AKIDETMeasureNotGathered  	-3012
	#define AKIDETNoAggregatedDetection -3013
	#define AKIDETNoInstantDetection	-3015

	/*! \brief Internal Detector information.
	  */
	struct AAPIEXPORT structA2KDetector{
		//! 0, OK, else error code
		int report;
		//! detector identifier
		int Id;
		//! section identifier where the detector is located
		int IdSection;
		//! first lane that the detector covers
		int IdFirstLane;
		//! last lane that the detector covers
		int IdLastLane;
		//! set of bits that codifies the detection capabilities.
		int Capabilities;
		//! position of the beginning of the detector, with respect to the beginning of the section.
		double InitialPosition;
		//! position of the end of the detector, with respect to the beginning of the section.
		double FinalPosition;
	};

	/*! \brief Vehicles dynamic eqquiped information.
	  */
	struct AAPIEXPORT EquippedInfVeh{
		//! 0, OK, else error code
		int report;
		//! the instant when the equipped vehicle crossed the detector.
		double timedetected;
		//! the vehicle identifier
		int idVeh;
		//!  vehicle type position in the list of vehicles types being used. 0 must be used for all vehicle types and a value from 1 to AKIVehGetNbVehTypes (), for a specific vehicle type.
		int vehType;
		//! instant vehicle speed (km/h or mph, depending on the units defined in the network)
		double speed;
		//! instant vehicle headway (sec)
		double headway;
		//! Transit line identifier (-1 when the equipped vehicle does not belongs to any Transit Line)
		int idptline;
	};


	AAPIEXPORT int						AKIDetGetNumberDetectors();
	AAPIEXPORT int						AKIDetGetIdDetector( int nbdet );
	AAPIEXPORT structA2KDetector			AKIDetGetPropertiesDetector( int nbdet );
	AAPIEXPORT structA2KDetector			AKIDetGetPropertiesDetectorById( int iddet );

	AAPIEXPORT bool					AKIDetIsCountGather( int Capability );
	AAPIEXPORT bool					AKIDetIsPresenceGather( int Capability );
	AAPIEXPORT bool					AKIDetIsSpeedGather( int Capability );
	AAPIEXPORT bool					AKIDetIsOccupancyGather( int Capability );
	AAPIEXPORT bool					AKIDetIsHeadwayGather( int Capability );
	AAPIEXPORT bool					AKIDetIsDensityGather( int Capability );
	AAPIEXPORT bool					AKIDetIsInfEquippedVehGather( int Capability );

	AAPIEXPORT double					AKIDetGetIntervalDetection();
	AAPIEXPORT double					AKIDetGetCycleInstantDetection();
	AAPIEXPORT int						AKIDetGetNbMeasuresAvailableInstantDetection();
	AAPIEXPORT double					AKIDetGetEndTimeMeasureAvailableInstantDetection( int elem );

	AAPIEXPORT int						AKIDetGetSCOOTOccupancyCyclebyId( int iddet, int vehTypePos );
	AAPIEXPORT double					AKIDetGetFinTimeOccupedCyclebyId( int iddet, int elem, int vehTypePos );
	AAPIEXPORT double					AKIDetGetIniTimeOccupedCyclebyId( int iddet, int elem, int vehTypePos );
	AAPIEXPORT int						AKIDetGetNbintervalsOccupedCyclebyId( int iddet, int vehTypePos );
	AAPIEXPORT int						AKIDetGetCounterCyclebyId( int iddet, int vehTypePos );
	AAPIEXPORT double					AKIDetGetSpeedCyclebyId( int iddet, int vehTypePos );
	AAPIEXPORT double					AKIDetGetTimeOccupedCyclebyId( int iddet, int vehTypePos );
	AAPIEXPORT int						AKIDetGetPresenceCyclebyId( int iddet, int vehTypePos );
	AAPIEXPORT double					AKIDetGetHeadwayCyclebyId( int iddet, int vehTypePos );
	AAPIEXPORT double					AKIDetGetDensityCyclebyId( int iddet, int vehTypePos );

	AAPIEXPORT int						AKIDetGetNbVehsEquippedInDetectionCyclebyId( int iddet, int vehTypePos );
	AAPIEXPORT StaticInfVeh				AKIDetGetInfVehInDetectionStaticInfVehCyclebyId( int iddet, int elem, int vehTypePos );
	AAPIEXPORT StaticInfVehACCParams		AKIDetGetInfVehInDetectionStaticInfVehACCParamsCyclebyId( int iddet, int elem, int vehTypePos );
	AAPIEXPORT InfVeh					AKIDetGetInfVehInDetectionInfVehCyclebyId( int iddet, int elem, int vehTypePos );

	AAPIEXPORT int						AKIDetGetNbVehsEquippedOverCyclebyId( int iddet, int VehType );
	AAPIEXPORT StaticInfVeh				AKIDetGetInfVehOverStaticInfVehCyclebyId( int iddet, int elem, int VehType );
	AAPIEXPORT StaticInfVehACCParams		AKIDetGetInfVehOverStaticInfVehACCParamsCyclebyId( int iddet, int elem, int VehType );
	AAPIEXPORT InfVeh					AKIDetGetInfVehOverInfVehCyclebyId( int iddet, int elem, int VehType );

	AAPIEXPORT int						AKIDetGetSCOOTOccupancyInstantDetectionbyId( int iddet, int vehTypePos, double endtime );
	AAPIEXPORT double					AKIDetGetIniTimeOccupedInstantDetectionbyId( int iddet, int elem, int vehTypePos, double endtime );
	AAPIEXPORT double					AKIDetGetEndTimeOccupedInstantDetectionbyId( int iddet, int elem, int vehTypePos, double endtime );
	AAPIEXPORT int						AKIDetGetNbintervalsOccupedInstantDetectionbyId( int iddet, int vehTypePos, double endtime );
	AAPIEXPORT int						AKIDetGetCounterInstantDetectionbyId( int iddet, int vehTypePos, double endtime );
	AAPIEXPORT double					AKIDetGetSpeedInstantDetectionbyId( int iddet, int vehTypePos, double endtime );
	AAPIEXPORT double					AKIDetGetTimeOccupedInstantDetectionbyId( int iddet, int vehTypePos, double endtime );
	AAPIEXPORT int						AKIDetGetPresenceInstantDetectionbyId( int iddet, int vehTypePos, double endtime );
	AAPIEXPORT double					AKIDetGetHeadwayInstantDetectionbyId( int iddet, int vehTypePos, double endtime );
	AAPIEXPORT double					AKIDetGetDensityInstantDetectionbyId( int iddet, int vehTypePos, double endtime );

	AAPIEXPORT int						AKIDetGetNbVehsEquippedInDetectionInstantDetectionbyId( int iddet, int vehTypePos, double endtime );
	AAPIEXPORT StaticInfVeh				AKIDetGetInfVehInDetectionStaticInfVehInstantDetectionbyId( int iddet, int elem, int vehTypePos, double endtime );
	AAPIEXPORT StaticInfVehACCParams		AKIDetGetInfVehInDetectionStaticInfVehACCParamsInstantDetectionbyId( int iddet, int elem, int vehTypePos, double endtime );
	AAPIEXPORT InfVeh					AKIDetGetInfVehInDetectionInfVehInstantDetectionbyId( int iddet, int elem, int vehTypePos, double endtime );

	AAPIEXPORT int						AKIDetGetNbVehsEquippedOverInstantDetectionbyId( int iddet, int VehType, double endtime );
	AAPIEXPORT StaticInfVeh				AKIDetGetInfVehInOverStaticInfVehInstantDetectionbyId( int iddet, int elem, int VehType, double endtime );
	AAPIEXPORT StaticInfVehACCParams		AKIDetGetInfVehInOverStaticInfVehACCParamsInstantDetectionbyId( int iddet, int elem, int VehType, double endtime );
	AAPIEXPORT InfVeh					AKIDetGetInfVehOverInfVehInstantDetectionbyId( int iddet, int elem, int VehType, double endtime );

	AAPIEXPORT int						AKIDetGetCounterAggregatedbyId( int iddet, int vehTypePos );
	AAPIEXPORT double					AKIDetGetSpeedAggregatedbyId( int iddet, int vehTypePos );
	AAPIEXPORT double					AKIDetGetTimeOccupedAggregatedbyId( int iddet, int vehTypePos );
	AAPIEXPORT int						AKIDetGetPresenceAggregatedbyId( int iddet, int vehTypePos );
	AAPIEXPORT double					AKIDetGetDensityAggregatedbyId( int iddet, int vehTypePos );
	AAPIEXPORT double					AKIDetGetHeadwayAggregatedbyId( int iddet, int vehTypePos );
	AAPIEXPORT int						AKIDetGetNbVehsInDetectionAggregatedbyId( int iddet, int vehTypePos );
	AAPIEXPORT EquippedInfVeh			AKIDetGetInfVehInDetectionAggregatedbyId( int iddet, int vehTypePos, int elem );

	// .......................................
	// 		DETECTOR CLICKABLE EVENTS   
	// .......................................

	#define AKIDetectorEventsNoTraffic -10000

	AAPIEXPORT int AKIDetectorEventsEnable();
	AAPIEXPORT int AKIDetectorEventsDisable();
	AAPIEXPORT void AKIDetectorEventsAddEvent(int iddet, double aIniTime, double aEndTime, int vehTypePos, double speed, double length, int idPTline);
	AAPIEXPORT int AKIDetectorEventsClear();

	// .......................................
	// 		INFORMATION OF IDENTIFIERS OF SECTIONS AND JUNCTIONS 
	// .......................................

	#define AKIInfNetGetMem 			-5001
	#define AKIInfUnknownId		 		-5002
    #define AKIInfUnknownTurning		-5003
    #define AKIInfUnknownFromSection    -5004
    #define AKIInfUnknownToSection      -5005
    #define AKIInfNoPath                -5006
	/*! \brief Aimsun section's information.
	  */
	struct AAPIEXPORT A2KSectionInf{
		//! 0, OK, otherwise is an error code
		int report;
		//! section identifier
		int id;
		//! section identifier in Aimsun kernel
		int angId;
		//! total number of central lanes
		int nbCentralLanes;
		//! total number of side lanes
		int nbSideLanes;
		//! speed limit of the section (Km/h or mph).
		double speedLimit;
		//! visibility distance of the reserved lanes of the section (meters or feet).
		double reservedLanesVisibilityDistance;
		//! capacity of the section (veh/h).
		double capacity;
        //! Side Lane Cooperation Distance: (m o ft).
		double distance_OnRamp;
		//! Side Lane Merging Distance: (m o ft).
		double distance_OnRampMerge;
		//! lane-changing cooperation percentage of the section.
		double cooperation_OnRamp;
		//! length of the section (m or feet).
		double length;
        //! number of section segments
        int nbSegments;
        //! array of nbSegments positions with the segment slope
        double * slopePercentages;
		//! user-defined cost of the section.
		double userDefinedCost;
		//! reaction time variation of the section.
		int reactionTimeVariation;
		//! reaction time variation of the section.
		double reactionTimeAtTrafficLightVariation;
		//! reaction time variation of the section.
		double reactionTimeAtStopVariation;
		//! if the imprudent lane-changing behavior is activated in this section.
		bool imprudentLaneChanging;
		//! total number of turns that have as origin this section.
		int nbTurnings;
		//! if the icapacity drop model  is activated in this section.
		bool capacityDropModel;

	};

	/*! \brief Section's behavior parameters.

		This class is used to change some and more important parameters.
	  */
	struct AAPIEXPORT A2KSectionBehaviourParam{
		//! speed limit of the section (km/h or mph).
		double speedLimit;
		//! visibility distance of the reserved lanes of the section (meters or feet).
		double reservedLanesVisibilityDistance;
		//! capacity of the section (veh/h).
		double capacity;
		//!  Side Lane Cooperation Distance: (m or ft).
		double distance_OnRamp;
		//! Side Lane Merging Distance: (m or ft).
		double distance_OnRampMerge;
		//! lane-changing cooperation percentage of the section.
		double cooperation_OnRamp;
		double laneChangingAggressiveness;
		//! user-defined cost of the section.
		double userDefinedCost;
		//! reaction time variation of the section.
		int reactionTimeVariation;
		double reactionTimeAtTrafficLightVariation;
		double reactionTimeAtStopVariation;
		bool imprudentLaneChanging;
		//! if the icapacity drop model  is activated in this section.
		bool capacityDropModel;
		bool considerTwoLanesCarFollowing;
		double accelerationVariationFactor;
	};

	/*! \brief Aimsun Microscopic internal centroid information.

	  */
	struct AAPIEXPORT A2KCentroidInf{
		//! 0, OK, else error code
		int report;
		//! centroid identifier
		int id;
		//! Centroid considers destination connectors percentages when the centroid acts as destination
		bool AsDestConsider_percentage;
		//! Centroid considers origin connectors percentages when the centroid acts as origin
		bool AsOrigConsider_percentage;
		//! true if the centroid acts as one origin
		bool IsOrigin;
		//! true if the centroid acts as one destination
		bool IsDestination;
		//! Number of connectors To
		int NumConnecTo;
		//! Number of connectors From
		int NumConnecFrom;
	};

    /*! \brief Aimsun Microscopic internal centroid information.

      */
    struct AAPIEXPORT A2KTurnInf{
        //! 0, OK, else error code
        int report;
        //! turn identifier
        int id;
		//! turn length (m or feet)
        double length;
		//! turn speed limit (Km/h or mph).
		double speedLimit;
		//! turn origin section identifier
        int originSectionId;
        //! turn destination section identifier
        int destinationSectionId;
        //! turn origin from lane
        int originFromLane;
        //! turn origin to lane
        int originToLane;
        //! turn destination from lane
        int destinationFromLane;
        //!turn destination to lane
        int destinationToLane;
		//!turn yellow box behavior
        bool yellowBoxBehaviour;
    };

	/*! \brief Turn's behavior parameters.

		This class is used to change some and more important parameters.
	  */
	struct AAPIEXPORT A2KTurnBehaviourParam{
		//! speed limit of the turn (km/h or mph).
		double speedLimit;
		//! yellow box behavior on/off.
		bool yellowBoxBehaviour;
		//! look-ahead distance (Micro) for the lane-changing model
		double distanceZone1;
		//! critical look-ahead distance (Micro) for the lane-changing model
		double distanceZone2;
		//! waiting time before missing turn
		double waitingTimeBeforeLosingTurn;
		//! safety front margin distance
		double safetyMarginFront;
		//! safety back margin distance
		double safetyMarginBack;
		//! minimum gap
		double minimumGap;
		//! maximum gap
		double maximumGap;
		//! minimum yield time factor
		double minGiveWayTimeFactor;
		//! maximum yield time factor
		double maxGiveWayTimeFactor;
		//! visibility distance 
		double visibilityDistance;
		//! visibility distance main stream 
		double visibilityDistanceMainStream;
	};

	AAPIEXPORT int AKIInfNetGetUnits();
	AAPIEXPORT int AKIInfNetGetWorldCoordinates(double *min_x, double *min_y, double *max_x, double *max_y);
	AAPIEXPORT int AKIInfNetNbSectionsANG();

	AAPIEXPORT int AKIInfNetGetSectionANGId(int element);

	AAPIEXPORT A2KSectionInf AKIInfNetGetSectionANGInf(int aid);
	AAPIEXPORT double AKIInfNetGetSectionSlopeBySegment(int aSectionId, int aSegmentPos);

	AAPIEXPORT int AKIInfNetGetIdSectionANGDestinationofTurning(int aid, int elem);
	AAPIEXPORT int AKIInfNetGetDestinationFromLaneofTurning(int aid, int elem);
	AAPIEXPORT int AKIInfNetGetDestinationToLaneofTurning(int aid, int elem);
	AAPIEXPORT int AKIInfNetGetOriginFromLaneofTurning(int aid, int elem);
	AAPIEXPORT int AKIInfNetGetOriginToLaneofTurning(int aid, int elem);

    AAPIEXPORT int AKIInfNetNbTurns();
    AAPIEXPORT int AKIInfNetGetTurnId(int element);
    AAPIEXPORT A2KTurnInf AKIInfNetGetTurnInf(int aid);

    AAPIEXPORT int AKIInfNetGetNbTurnsInNode(int idnode);
    AAPIEXPORT int AKIInfNetGetOriginSectionInTurn(int idnode, int index);
    AAPIEXPORT int AKIInfNetGetDestinationSectionInTurn(int idnode, int index);
    AAPIEXPORT A2KTurnInf AKIInfNetGetTurnInfo(int idnode, int index);

	AAPIEXPORT int AKIInfNetSetTurnBehaviouralParam(int aid, A2KTurnBehaviourParam behaviourParam);
	AAPIEXPORT A2KTurnBehaviourParam AKIInfNetGetTurnBehaviouralParam( int idturn, int* report );

	AAPIEXPORT int AKIInfNetGetTurningId( int originSection, int destinationSection );
	AAPIEXPORT int AKIInfNetGetTurningDestinationFromLane( int originSection, int destinationSection );
	AAPIEXPORT int AKIInfNetGetTurningDestinationToLane( int originSection, int destinationSection );
	AAPIEXPORT int AKIInfNetGetTurningOriginFromLane( int originSection, int destinationSection );
	AAPIEXPORT int AKIInfNetGetTurningOriginToLane( int originSection, int destinationSection );

	AAPIEXPORT int AKIInfNetSetSectionBehaviouralParam(int aid, A2KSectionBehaviourParam behaviourParam, bool allsegments);
	AAPIEXPORT A2KSectionBehaviourParam AKIInfNetGetSectionBehaviouralParam( int idSection, int* report );
	AAPIEXPORT int AKISetSectionCapacity(int aid, double capacity);
	AAPIEXPORT int AKISetSectionUserDefinedCost(int aid, double userDefinedCost);
	AAPIEXPORT int AKISetSectionUserDefinedCost2(int aid, double userDefinedCost);
	AAPIEXPORT int AKISetSectionUserDefinedCost3(int aid, double userDefinedCost);

	AAPIEXPORT double AKIGetSectionCapacity(int aid);
	AAPIEXPORT double AKIGetSectionUserDefinedCost(int aid);
	AAPIEXPORT double AKIGetSectionUserDefinedCost2(int aid);
	AAPIEXPORT double AKIGetSectionUserDefinedCost3(int aid);

	AAPIEXPORT int AKIInfNetNbJunctions();
	AAPIEXPORT int AKIInfNetGetJunctionId(int element);

	AAPIEXPORT int AKIInfNetNbCentroids();
	AAPIEXPORT int AKIInfNetGetCentroidId(int element);
	AAPIEXPORT A2KCentroidInf AKIInfNetGetCentroidInf(int aid);
	AAPIEXPORT int AKIInfNetGetIdObjectofOriginCentroidConnector(int aid, int elem, bool &isSection);
	AAPIEXPORT int AKIInfNetGetIdObjectofDestinationCentroidConnector(int aid, int elem, bool &isSection);
	AAPIEXPORT int AKIInfNetGetIdObjectANGofOriginCentroidConnector(int aid, int elem, bool &isSection);
	AAPIEXPORT int AKIInfNetGetIdObjectANGofDestinationCentroidConnector(int aid, int elem, bool &isSection);

    AAPIEXPORT int AKIInfNetGetShortestPathNbSections(int fromSection, int toSection, void * sectionColumnCost );
    AAPIEXPORT int AKIInfNetGetShortestPath(int fromSection, int toSection, void * sectionColumnCost, int* path );
	// .......................................
	// 		INFORMATION OF PATH & NAMES 
	// .......................................

	#define	ODMatrixDemand				1
	#define	StateDemand					2

	// Returns utf16 encoded path of the network. The result has to be deleted with delete[]
	AAPIEXPORT const unsigned short * AKIInfNetGetNetworkPath();
	// Returns utf16 encoded name of the network. The result has to be deleted with delete[]
	AAPIEXPORT const unsigned short * AKIInfNetGetNetworkName();
	// Returns utf16 encoded name of current demand. The result has to be deleted with delete[]
	AAPIEXPORT const unsigned short * AKIInfNetGetTrafficDemandName();
	AAPIEXPORT int AKIInfNetGetTrafficDemandType(); 


	// .......................................
	// 		STATISTICAL INFORMATION  
	// .......................................


	#define AKIESTUnknownSection 	-6001
	#define AKIESTNotAvalaible 		-6002
	#define AKIESTUnknownCentroid 	-6004
	#define AKIESTUnknownStream					-6005
	#define AKIESTFuelConsumptionNotAvailable	-6006
	#define AKIESTPollutionEmissionNotAvailable	-6007
	#define AKIESTUnknownPollutant				-6008
	#define AKIESTWrongLane						-6009
    #define AKIESTUnknownNode   				-6010

	/*! \brief System/Replication statistics
	*/
	struct AAPIEXPORT StructAkiEstadSystem{
		//! 0, OK, else error code
		int report;
		//!  Flow  (veh/h)
		int Flow;	/* Flow */
		//! Average travel time(sec/km or sec/mile)
		double TTa;
		//! Deviation travel time(sec/km or sec/mile)
		double TTd;
		//! Average delay time (sec/km or sec/mile)
		double DTa;
		//! Deviation delay time (sec/km or sec/mile)
		double DTd;
		//!  Average speed (km/h or mph)
		double Sa;
		//!  Deviation speed (km/h or mph)
		double Sd;
		//! Average harmonic speed (km/h or mph)
		double SHa;
		//! Deviation harmonic speed (km/h or mph)
		double SHd;
		//! Density (Veh/km or Veh/mile per lane)
		double Density;
		//! Average stop time (sec/km or sec/mile)
		double STa;
		//! Deviation stop time (sec/km or sec/mile)
		double STd;
		//! Number of Stops (#/Veh/km or #/Veh/mile)
		double NumStops;
		//!Total number of Stops
		int totalNumStops;
		//! Average Queue Length
		double LongQueueAvg;
		//! Maximum Queue Length
		double LongQueueMax;	/* Maximum Queue Length */
		//! Total Distance traveled
		double TotalTravel;
		//! Total Time traveled
		double TotalTravelTime;
		//! Virtual average queue length
		double virtualQueueAvg;
		//! Virtual max queue length
		int virtualQueueMax;
        //! volume
        int count;
        //! vehicle flow that have entered the section
        int inputFlow;
        //! vehicle count that have entered the section
        int inputCount;
        //! vehicles waiting to enter
        int vehsWaiting;
        //! vehicles in network
        int vehIn;
		/* vehicles which are left from the network */
		int vehOut;
        //! vehicles lost inside the network
        int vehsLostIn;
        //! vehicle lost outside the network
        int vehsLostOut;
        //! number of missed turns
        int missedTurns;
		//! number of lane changes per km
		double laneChanges;
		//! total number of lane changes
		int totalLaneChanges;
		//! Average time in seconds that vehicles remained waiting in a virtual queue. Only in entry sections.
		double waitingTimeVirtualQueue;
		//! Deviation average time in seconds that vehicles remained waiting in a virtual queue. Only in entry sections.
		double waitingTimeVirtualQueueDev;
	};

	/*! \brief Section's statistics
	*/
	struct AAPIEXPORT StructAkiEstadSection{
		//! \see StructAkiEstadSystem
		int report;	
		//! Section indentifier
		int Id;
		//! \see StructAkiEstadSystem
		int Flow;	/* Flow */
		//! \see StructAkiEstadSystem
		double TTa;
		//! \see StructAkiEstadSystem
		double TTd;		/* Travel Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double DTa;
		//! \see StructAkiEstadSystem
		double DTd;		/* Delay Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double Sa;
		//! \see StructAkiEstadSystem
		double Sd;		/* Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double SHa;
		//! \see StructAkiEstadSystem
		double SHd;		/* Harmonic Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double Density;
		//! \see StructAkiEstadSystem
		double STa;
		//! \see StructAkiEstadSystem
		double STd;		/* Stop Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double NumStops;		/* Number of Stops */
		//! Average Queue Length
		double LongQueueAvg;	/* Average Queue Length */
		//! Maximum Queue Length
		double LongQueueMax;	/* Maximum Queue Length */
		//! \see StructAkiEstadSystem
		double TotalTravel;	/* Total Distance  traveled */
		//! \see StructAkiEstadSystem
		double TotalTravelTime;	/* Total Time  traveled */
		//! Virtual average queue length
		double virtualQueueAvg; /*VirtualQueue Avg*/
		//! Virtual max queue length
		int virtualQueueMax; /*VirtualQueue Max*/
        //! volume
        int count;
        //! vehicle flow that have entered the section
        int inputFlow;
        //! vehicle count that have entered the section
        int inputCount;
        //! flow / section capacity
        double flowCapacity;
		//! nb of lane changes in section per vehicle
        double laneChanges;
		//! total number of lane changes
		double totalLaneChanges;
		//! total number of vehicles in virtual queues
		int numVehiclesInVQ;
		//! Average time in seconds that vehicles remained waiting in a virtual queue. Only in entry sections.
		double waitingTimeVirtualQueue;
		//! Deviation of average time in seconds that vehicles remained waiting in a virtual queue. Only in entry sections.
		double waitingTimeVirtualQueueDev;
	};

	/*! \brief Turn's statistics
	*/
	struct AAPIEXPORT StructAkiEstadTurning{
		//! \see StructAkiEstadSystem
		int report;	
		//! Section from identifier
		int IdSectionFrom;
		//! Section to identifier
		int IdSectionTo;
		//! \see StructAkiEstadSystem
		int count; /* Count */
		//! \see StructAkiEstadSystem
		int Flow;	/* Flow */
		//! \see StructAkiEstadSystem
		int inputCount; /* Count */
		//! \see StructAkiEstadSystem
		int inputFlow;	/* Flow */
		//! \see StructAkiEstadSystem
		double TTa;
		//! \see StructAkiEstadSystem
		double TTd;		/* Travel Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double DTa;
		//! \see StructAkiEstadSystem
		double DTd;		/* Delay Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double Sa;
		//! \see StructAkiEstadSystem
		double Sd;		/* Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double SHa;
		//! \see StructAkiEstadSystem
		double SHd;		/* Harmonic Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double STa;
		//! \see StructAkiEstadSystem
		double STd;		/* Stop Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double NumStops;		/* Number of Stops */
		//! \see StructAkiEstadSection
		double LongQueueAvg;	/* Average Queue Length */
		//! \see StructAkiEstadSection
		double LongQueueMax;	/* Maximum Queue Length */
		//! \see StructAkiEstadSection
		double TotalTravel;	/* Total Distance  traveled */
		//! \see StructAkiEstadSection
		double TotalTravelTime;	/* Total Time  traveled */
		//! \see StructAkiEstadSection
		double virtualQueueAvg; /*VirtualQueue Avg*/
		//! \see StructAkiEstadSection
		int virtualQueueMax; /*VirtualQueue Maximum*/
		//! nb of lane changes in turn per vehicle
		double laneChanges;
		//! total number of lane changes
		double totalLaneChanges;
		/* Total number of lost vehicles */
		int vehLost;
		/* Total number of missed vehicles */
		int vehMissed;
};

	/*!
	 * \brief Section Lane's statistics
	 */
	struct AAPIEXPORT StructAkiEstadSectionLane{
		//! \see StructAkiEstadSystem
		int report;
		//! Section Identifier
		int IdSection;
		//! \see StructAkiEstadSystem
		int count; /* count */
		//! \see StructAkiEstadSystem
		int Flow;	/* Flow */
		//! \see StructAkiEstadSystem
		int inputCount; /* Input Count */
		//! \see StructAkiEstadSystem
		int inputFlow; /* Input Flow */
		//! \see StructAkiEstadSystem
		double TTa;
		//! \see StructAkiEstadSystem
		double TTd;		/* Travel Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double DTa;
		//! \see StructAkiEstadSystem
		double DTd;		/* Delay Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double Sa;
		//! \see StructAkiEstadSystem
		double Sd;		/* Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double SHa;
		//! \see StructAkiEstadSystem
		double SHd;		/* Harmonic Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double Density; /* Density */
		//! \see StructAkiEstadSystem
		double STa;
		//! \see StructAkiEstadSystem
		double STd;		/* Stop Time  : Average & Deviation */
		//! Average Queue Length
		double LongQueueAvg;
		//! Maximum Queue Length
		double LongQueueMax;
		//! Average time in seconds that vehicles remained waiting in a virtual queue. Only in entry sections.
		double waitingTimeVirtualQueue;
		//! Deviation of average time in seconds that vehicles remained waiting in a virtual queue. Only in entry sections.
		double waitingTimeVirtualQueueDev;
	};

	/*! \brief OD pair's statistics
	*/
	struct AAPIEXPORT StructAkiEstadODPair{
		//! \see StructAkiEstadSystem
		int report;
		//! Origin centroid identifier
		int IdOrigin;
		//! Destination centroid identifier
		int IdDest;
		//! \see StructAkiEstadSystem
		int count; /* Count */
		//! \see StructAkiEstadSystem
		int Flow;	/* Flow */
		//! \see StructAkiEstadSystem
		int inputCount; /* Count */
		//! \see StructAkiEstadSystem
		int inputFlow;	/* Flow */
		//! \see StructAkiEstadSystem
		double TTa;
		//! \see StructAkiEstadSystem
		double TTd;		/* Travel Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double DTa;
		//! \see StructAkiEstadSystem
		double DTd;		/* Delay Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double Sa;
		//! \see StructAkiEstadSystem
		double Sd;		/* Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double SHa;
		//! \see StructAkiEstadSystem
		double SHd;		/* Harmonic Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double STa;
		//! \see StructAkiEstadSystem
		double STd;		/* Stop Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double NumStops;		/* Number of Stops */
		//! \see StructAkiEstadSection
		double virtualQueueAvg;	/* Average Virtual Queue */
		//! \see StructAkiEstadSection
		double virtualQueueMax;	/* Maximum Virtual Queue */
		//! \see StructAkiEstadSystem
		double TotalTravel;	/* Total Distance  traveled */
		//! \see StructAkiEstadSystem
		double TotalTravelTime;	/* Total Time  traveled */
		//! Total number of vehicles lost that we doing this trip
		int vehLost;		/* Total number of vehicle lost */
		//! \see StructAkiEstadSystem
		double dwellTime;
		//! \see StructAkiEstadSystem
		double dwellTimeDev; /* Time spends by a veh at a scheduled stop without moving. Only in Transit line vehs. */
		//! \see StructAkiEstadSystem
		double waitingTimeVirtualQueue;
		//! \see StructAkiEstadSystem
		double waitingTimeVirtualQueueDev;/* Average time in seconds that vehicles remained waiting in a virtual queue. Only in entry sections.  */
	};

	/*! \brief Stream's statistics
	*/
	struct AAPIEXPORT StructAkiEstadStream{
		//! \see StructAkiEstadSystem
		int report;	
		//! Stream's identifier
		int Id;
		//! \see StructAkiEstadSystem
		int count; /* Count */
		//! \see StructAkiEstadSystem
		int Flow;	/* Flow */
		//! \see StructAkiEstadSystem
		int inputCount; /* Count */
		//! \see StructAkiEstadSystem
		int inputFlow;	/* Flow */
		//! \see StructAkiEstadSystem
		double TTa;
		//! \see StructAkiEstadSystem
		double TTd;		/* Travel Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double DTa;
		//! \see StructAkiEstadSystem
		double DTd;		/* Delay Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double Sa;
		//! \see StructAkiEstadSystem
		double Sd;		/* Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double SHa;
		//! \see StructAkiEstadSystem
		double SHd;		/* Harmonic Speed  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double STa;
		//! \see StructAkiEstadSystem
		double STd;		/* Stop Time  : Average & Deviation */
		//! \see StructAkiEstadSystem
		double NumStops;		/* Number of Stops */
		//! \see StructAkiEstadSection
		double virtualQueueAvg;	/* Average Virtual Queue */
		//! \see StructAkiEstadSection
		double virtualQueueMax;	/* Maximum Virtual Queue */
		//! \see StructAkiEstadSection
		double TotalTravel;	/* Total Distance  traveled */
		//! \see StructAkiEstadSection
		double TotalTravelTime;	/* Total Time  traveled */
		//! \see StructAkiEstadSection
		int vehLost; /* Total number of lost vehicles */
		//! \see StructAkiEstadSystem
		double dwellTime;
		//! \see StructAkiEstadSystem
		double dwellTimeDev; /* Time spends by a veh at a scheduled stop without moving. Only in Transit line vehs. */
		//! \see StructAkiEstadSystem
		double waitingTimeVirtualQueue;
		//! \see StructAkiEstadSystem
		double waitingTimeVirtualQueueDev;/* Average time in seconds that vehicles remained waiting in a virtual queue. Only in entry sections.  */
	};

	/*! \brief Detector statistics
	*/
	struct AAPIEXPORT StructAkiEstadDetector{
		int report;
		int Id;
		int Flow;	/* Flow */
		double Sa;		/* Speed  : Average */
		double Density; /* Density */
		double Occupancy; /* Occupancy */
		double Headway; /* Headway */
	};


	AAPIEXPORT int AKIIsGatheringStatistics();
	AAPIEXPORT double AKIEstGetIntervalStatistics();
	AAPIEXPORT bool 					AKIEstIsNewStatisticsAvailable();
	AAPIEXPORT StructAkiEstadSection 	AKIEstGetGlobalStatisticsSection(int aidarc, int vehTypePos);
	AAPIEXPORT StructAkiEstadSection 	AKIEstGetParcialStatisticsSection(int aidarc, double timeSta, int vehTypePos);
	AAPIEXPORT StructAkiEstadSection	AKIEstGetCurrentStatisticsSection(int aidarc, int VehType);
	AAPIEXPORT StructAkiEstadSectionLane AKIEstGetGlobalStatisticsSectionLane(int aidarc, int indexLane, int vehTypePos);
	AAPIEXPORT StructAkiEstadSectionLane AKIEstGetParcialStatisticsSectionLane(int aidarc, int indexLane, double timeSta, int vehTypePos);
	AAPIEXPORT StructAkiEstadSectionLane AKIEstGetCurrentStatisticsSectionLane(int aidarc, int indexLane, int VehType);
	AAPIEXPORT StructAkiEstadTurning 	AKIEstGetGlobalStatisticsTurning(int aidsectionFrom, int aidsectionTo, int vehTypePos);
	AAPIEXPORT StructAkiEstadTurning 	AKIEstGetParcialStatisticsTurning(int aidsectionFrom, int aidsectionTo, double timeSta, int vehTypePos);
	AAPIEXPORT StructAkiEstadTurning	AKIEstGetCurrentStatisticsTurning(int aidsectionFrom, int aidsectionTo, int VehType);
	AAPIEXPORT StructAkiEstadTurning 	AKIEstGetGlobalStatisticsLink(int aidsectionFrom, int aidsectionTo, int vehTypePos);
	AAPIEXPORT StructAkiEstadTurning 	AKIEstGetParcialStatisticsLink(int aidsectionFrom, int aidsectionTo, double timeSta, int vehTypePos);
	AAPIEXPORT StructAkiEstadTurning		AKIEstGetCurrentStatisticsLink(int aidsectionFrom, int aidsectionTo, int VehType);
	AAPIEXPORT double					AKIGetTotalLengthSystem();
	AAPIEXPORT StructAkiEstadSystem 	AKIEstGetGlobalStatisticsSystem(int vehTypePos);
	AAPIEXPORT StructAkiEstadSystem 	AKIEstGetParcialStatisticsSystem(double timeSta, int vehTypePos);
	AAPIEXPORT StructAkiEstadODPair 	AKIEstGetGlobalStatisticsODPair(int idOrigin, int idDestination, int vehTypePos);
	AAPIEXPORT StructAkiEstadODPair 	AKIEstGetParcialStatisticsODPair(int idOrigin, int idDestination, double timeSta, int vehTypePos);
	AAPIEXPORT StructAkiEstadStream 	AKIEstGetParcialStatisticsStream(int aidstream, double timeSta, int vehTypePos);
	AAPIEXPORT StructAkiEstadStream 	AKIEstGetGlobalStatisticsStream(int aidstream, int vehTypePos);

	AAPIEXPORT double					AKIEstGetInstantVirtualQueueSection(int aidarc,int vehTypePos);

	AAPIEXPORT int 	AKIEstGetGlobalStatisticsNodeLostVehicles( int aidNode, int vehTypePos );
	AAPIEXPORT int 	AKIEstGetPartialStatisticsNodeLostVehicles( int aidNode, int vehTypePos );
    AAPIEXPORT double AKIEstGetGlobalStatisticsNodeApproachDelay( int aidNode );
    AAPIEXPORT double AKIEstGetPartialStatisticsNodeApproachDelay( int aidNode );
	AAPIEXPORT int 	AKIEstGetGlobalStatisticsNodeMissedTurns( int aidNode, int vehTypePos );
	AAPIEXPORT int 	AKIEstGetPartialStatisticsNodeMissedTurns( int aidNode, int vehTypePos );

	//Pollutants and emission statistics
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsSectionFuelCons(int aidarc, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsSectionFuelCons(int aidarc, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetCurrentStatisticsSectionFuelCons(int aidarc, int VehType);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsTurningFuelCons(int aidsectionFrom, int aidsectionTo, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsTurningFuelCons(int aidsectionFrom, int aidsectionTo, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetCurrentStatisticsTurningFuelCons(int aidsectionFrom, int aidsectionTo, int VehType);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsLinkFuelCons(int aidsectionFrom, int aidsectionTo, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsLinkFuelCons(int aidsectionFrom, int aidsectionTo, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetCurrentStatisticsLinkFuelCons(int aidsectionFrom, int aidsectionTo, int VehType);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsSystemFuelCons(int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsSystemFuelCons(double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsODPairFuelCons(int idOrigin, int idDestination, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsODPairFuelCons(int idOrigin, int idDestination, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsStreamFuelCons(int aidstream, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsStreamFuelCons(int aidstream, int vehTypePos);

	AAPIEXPORT int 	AKIEstGetNbPollutants();
	// Gets the name in utf16 and \0 terminated. The result has to be deleted with delete[]
	AAPIEXPORT const unsigned short *AKIEstGetPollutantName(int index);

	AAPIEXPORT double 	AKIEstGetGlobalStatisticsSectionPollution(int indexPol, int aidarc, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsSectionPollution(int indexPol, int aidarc, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetCurrentStatisticsSectionPollution(int indexPol, int aidarc, int VehType);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsTurningPollution(int indexPol, int aidsectionFrom, int aidsectionTo, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsTurningPollution(int indexPol, int aidsectionFrom, int aidsectionTo, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetCurrentStatisticsTurningPollution(int indexPol, int aidsectionFrom, int aidsectionTo, int VehType);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsLinkPollution(int indexPol, int aidsectionFrom, int aidsectionTo, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsLinkPollution(int indexPol, int aidsectionFrom, int aidsectionTo, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetCurrentStatisticsLinkPollution(int indexPol, int aidsectionFrom, int aidsectionTo, int VehType);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsSystemPollution(int indexPol, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsSystemPollution(int indexPol, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsODPairPollution(int indexPol, int idOrigin, int idDestination, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsODPairPollution(int indexPol, int idOrigin, int idDestination, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetParcialStatisticsStreamPollution(int indexPol, int aidstream, double timeSta, int vehTypePos);
	AAPIEXPORT double 	AKIEstGetGlobalStatisticsStreamPollution(int indexPol, int aidstream, int vehTypePos);

	AAPIEXPORT StructAkiEstadDetector AKIEstGetGlobalStatisticsDetector( int detectorId, int vehType );
	AAPIEXPORT StructAkiEstadDetector AKIEstGetParcialStatisticsDetector( int detectorId, double time, int vehType );
	// .......................................
	// 		VEHICLE ENTRANCE  
	// .......................................

	#define AKIEnterVehUnknownSection 		-7001
	#define AKIEnterVehUnFeasibleLane		-7003
	#define AKIEnterVehNotSpace		 		-7004
	#define AKIEnterVehUnknownCentroid		-7005
	#define AKIEnterVehUnFeasiblePath		-7006
	#define AKIEnterVehNoTrafficFlow		-7008
	#define AKIEnterVehNoTrafficOD			-7009
	#define AKIEnterVehUnknownLane			-7010
	#define AKIEnterVehUnknownNextSection	-7011
	#define AKIEnterVehUnknownVehType		-7016
    #define AKIZeroPedestriansToGenerate	-7018
    #define AKIZeroTimeInterval             -7019
    #define AKIEntranceCentroidNotFound     -7020
    #define AKIExitCentroidNotFound         -7021
    #define AKIRouteNotFound                -7022
    #define AKIEntranceCentroidNotInRoute   -7023
    #define AKIExitCentroidNotInRoute       -7024

	#define RIGHT	-1
	#define NOCHANGE 0
	#define LEFT	1

	AAPIEXPORT int		AKIEnterVehTrafficFlow(int asection, int vehTypePos, int tracking = 0);
	AAPIEXPORT int		AKIEnterVehTrafficOD(int asection, int vehTypePos, int idCentroidOrigin, int idCentroidDest, int tracking = 0);
	/* initPosition and initSpeed in network units (metric: m and km/h and english: feet and mph */
	AAPIEXPORT int		AKIPutVehTrafficFlow(int asection, int idLane, int vehTypePos, double initPosition, double initSpeed, int nextSection, int tracking);
	/* initPosition and initSpeed in network units (metric: m and km/h and english: feet and mph */
	AAPIEXPORT int		AKIPutVehTrafficOD(int asection, int idLane, int vehTypePos, int idCentroidOr, int idCentroidDest, double initPosition, double initSpeed, int tracking, bool use2Dim, double initYPosition, double initYSpeed );

	/*! \brief Vehicle's arrival information.
	  */
	struct AAPIEXPORT InfArrival{
		//! 0, OK, else error code
		int report;
		//! >0 vehicle identifier entered in the system
		int idVeh;
		//! true the vehicle has not enter in the system but has been stored in the virtual entrance Queue, false otherwise
		bool inVirtualQueue;
		//! entrance section identifier where the vehicle has been entered or stored in the virtual entrance queue.
		int entranceSection;
	}; 



	AAPIEXPORT InfArrival AKIGenerateArrivalTrafficFlow(int asection, int vehTypePos, int useVirtualQueue);
	AAPIEXPORT InfArrival AKIGenerateArrivalTrafficOD(int vehTypePos, int idCentroidOrigin, int idCentroidDest, int useVirtualQueue);

	// .......................................
	// 		VEHICLE TRACKED MANAGE  
	// .......................................

	#define AKIVehNotTracked				-7012
	#define AKIVehInvalidParameter			-7013
	#define AKIVehNextSectionUnreachable	-7014

	/*! newSpeed expressed in the network units, either Km/h or Mph */
	AAPIEXPORT int		AKIVehTrackedModifySpeed(int aidVeh, double newSpeed);
	AAPIEXPORT int		AKIVehTrackedForceSpeed(int aidVeh, double newSpeed);
	AAPIEXPORT int		AKIVehTrackedModifyLane(int aidVeh, int nextLane);
	AAPIEXPORT int		AKIVehTrackedModifyNextSection(int aidVeh, int nextSection);
	AAPIEXPORT int		AKIVehTrackedModifyNextSections(int aidVeh, int sizeNextSections, const int *nextSections);
	AAPIEXPORT int		AKIVehTrackedModifyNextTargetLaneInNextSection(int aidVeh, int nextSection, int nextlane);
	AAPIEXPORT int		AKIVehTrackedDelete(int aidVeh);
	AAPIEXPORT int		AKIVehSetAsTracked(int aidVeh);
	AAPIEXPORT int		AKIVehSetAsTrackedbyPointer(void *aVeh);
	AAPIEXPORT int		AKIVehSetAsNoTracked(int aidVeh);
	AAPIEXPORT int		AKIVehTrackedSetParkingManeuver(int aidVeh, int idsection, int lane, double position, double length, double durationParking, double visibilityDistance);
	AAPIEXPORT int		AKIVehTrackedCancelParkingManeuver(int aidVeh);






	// .......................................
	// 		INFORMATION OF VEHICLE TRACKED   
	// .......................................

	/*! \brief Dynamic vehicle's position.

	  This struct is used to modify vehicle's position.
  */
	struct AAPIEXPORT DynInfVeh{
		//! x world coordinates of the middle point of the vehicle front bumper for the current position
		double xCurrentPos;
		//! y world coordinates of the middle point of the vehicle front bumper for the current position
		double yCurrentPos;
		//! x world coordinates of the middle point of the vehicle back bumper for the current position
		double xCurrentPosBack;
		//! y world coordinates of the middle point of the vehicle back bumper for the current position
		double yCurrentPosBack;
		//! current speed (km/h or mph, depending the units defined in the network).
		double currentSpeed;
        //! next turn selected by the vehicle (-1 right turn, 1 left turn, 0 other cases)
		int turning;
	};

 
	AAPIEXPORT StaticInfVeh				AKIVehTrackedGetStaticInf( int aidVeh );
	AAPIEXPORT StaticInfVehACCParams 	AKIVehTrackedGetStaticInfACCParams( int aidVeh );
	AAPIEXPORT InfVeh					AKIVehTrackedGetInf( int aidVeh );
	AAPIEXPORT int						AKIVehTrackedSetStaticInf( int aidVeh, StaticInfVeh staticinfVeh );
	AAPIEXPORT int						AKIVehTrackedSetStaticInfACCParams( int aidVeh, StaticInfVehACCParams staticinfVehACC );
	AAPIEXPORT InfVehPos				AKIVehTrackedGetPos( int anIdVeh, int nbPos );
	AAPIEXPORT int						AKIVehTrackedGetPosWithStruct( int anIdVeh, int nbPos , InfVehPos *pinfVehPos );
	AAPIEXPORT int						AKIVehSetVehicleTrackedDynInf( int anIdVeh, DynInfVeh dynInfVeh );
	AAPIEXPORT GraphicInfVeh			AKIVehTrackedGetGraphicInf( int aidVeh );
	AAPIEXPORT LeaderInfVeh				AKIVehTrackedGetLeaderVehInf( int aidVeh );

	AAPIEXPORT int						AKIVehTrackedGetVehicleCategoryId( int aidVeh );
	AAPIEXPORT int						AKIVehTrackedGetLEMEmissionVehicleTypeId( int aidVeh );

	AAPIEXPORT int						AKIVehTrackedGetVehicleCategoryId( int aidVeh );
	AAPIEXPORT int						AKIVehTrackedGetLEMEmissionVehicleTypeId( int aidVeh );

	// .......................................
	// 		INFORMATION OF RUN TIME SIMULATION   
	// .......................................
	AAPIEXPORT double 	AKIGetCurrentSimulationTime();	
	AAPIEXPORT double 	AKIGetTimeSta();	
	AAPIEXPORT double 	AKIGetIniSimTime();
	AAPIEXPORT double 	AKIGetEndSimTime();
	AAPIEXPORT double 	AKIGetDurationTransTime();
	AAPIEXPORT double 	AKIGetSimulationStepTime();
	AAPIEXPORT int		AKISetEndSimTime( double atime );

	// .......................................
	// 		INFORMATION OF REPLICATION   
	// .......................................

	AAPIEXPORT double 	AKIGetRandomNumber();

	// .......................................
	// 		MANAGING OF INCIDENTS   
	// .......................................

	#define AKIIncidentWrongIniTime		-8001
	#define AKIIncidentWrongPosition	-8002
	#define AKIIncidentUnknownLane		-8003
	#define AKIIncidentUnknownSection	-8004
	#define AKIIncidentNotPresent		-8005
	#define AKIIncidentWrongLength		-8006
	#define AKIIncidentWrongDuration	-8007

	AAPIEXPORT int AKIGenerateIncident(int asection, int alane, double position, double length, double initime, double duration, double visibilityDistance, bool updateIdGroup,
											bool applySpeedReduction, double upstreamDistanceSR, double downstreamDistanceSR, double maxSpeedSR);
	AAPIEXPORT int AKIGenerateIncidentDistancePerVehType(int asection, int alane, double position, double length, double initime, double duration, double visibilityDistanceGeneral, int nbVehTypes, int * vehType, double * visibilityDistances, bool updateidGroup,
	bool applySpeedReduction, double upstreamDistanceSR, double downstreamDistanceSR, double maxSpeedSR);
	AAPIEXPORT int AKIRemoveIncident(int asection, int alane, double position);
	AAPIEXPORT int AKIRemoveAllIncidentsInSection(int asection); 
	AAPIEXPORT int AKIResetAllIncidents();

	// .......................................
	// 		MANAGING OF Transit
	// .......................................

	#define AKIPTNotLoaded					-9001
	#define AKIPTVehUnknown					-9002
	#define AKIPTStopUnknown				-9003
	#define AKIPTLineUnknown				-9005
	#define AKIPTVehNotSpace		 		-9006
	#define AKIPTIndexNotValid		 		-9007
	#define AKIPTVehUnFeasibleLane		 	-9008
	#define AKIPTVehCapacityOverflow	 	-9009
	#define AKIPTNotAvalaible		 		-1

	/*! \brief Transit vehicle's dynamic information.
	  */
	struct AAPIEXPORT InfPTVeh{
		//! \see InfVeh
		int report; // Retorna 0, si OK, sino codi error
		//! \see InfVeh
		int idVeh;
		//! \see InfVeh
		int type;
		//! \see InfVeh
		// Information in Vehicle when it is in a section
		//! \see InfVeh
		int idSection;
		//! \see InfVeh
		int segment;
		//! \see InfVeh
		int numberLane;
		//! \see InfVeh
		// Information in Vehicle when it is in a node
		int idJunction;
		//! \see InfVeh
		int idSectionFrom;
		//! \see InfVeh
		int idLaneFrom;
		//! \see InfVeh
		int idSectionTo;
		//! \see InfVeh
		int idLaneTo;
		//! \see InfVeh
		double CurrentPos;
		//! \see InfVeh
		double distance2End;
		//! \see InfVeh
		double xCurrentPos;
		//! \see InfVeh
		double yCurrentPos;
		//! \see InfVeh
		double zCurrentPos;
		//! \see InfVeh
		double xCurrentPosBack;
		//! \see InfVeh
		double yCurrentPosBack;
		//! \see InfVeh
		double zCurrentPosBack;
		//! \see InfVeh
		double CurrentSpeed;
		//! \see InfVeh
		double PreviousSpeed;
		//! \see InfVeh
		double TotalDistance;
		//! \see InfVeh
		double SystemGenerationT;
		//! \see InfVeh
		double SystemEntranceT;
		//! \see InfVeh
		double SectionEntranceT;
		//! \see InfVeh
		double CurrentStopTime;
		//! \see InfVeh
		bool stopped;
		//! \see InfVeh
		unsigned int mNbLostTurnings;
        //! \see InfVeh
        double energyState;

		// Information about PT
		//! the theoretical Generation Time
		double theoricalGenerationTime;
		//! the number of stops done.
		int nbStopsDone;
		//! Observed Stopped Time of last Stop
		double observedLastStopTime;
		//! Observed Initial Stopped Time of last Stop
		double observedLastInitialStopTime;
		//! the identifier of next stop
		int nextStopId;
		//! offset of next stop
		double offsetInNextStop;
		//! the distance to next stop (meters or feet).
		double distanceNextStop;
		//! the next stop time in the next stop(seconds).
		double nextServiceTime;
		//! the stopped time in a bus stop(seconds).
		double currentStoppedTimeInBusStop;
		//! Current number of people inside the ptvehicle
		int currentLoad;
	};

	/*!  \brief Transit vehicle's static information.
	 */
	struct AAPIEXPORT StaticInfPTVeh{
		//! \see StaticInfVeh
		int report; 
		//! \see StaticInfVeh
		int idVeh;
		//! \see StaticInfVeh
		int type;
		//! \see StaticInfVeh
		double length;
		//! \see StaticInfVeh
		double width;
		//! \see StaticInfVeh
		double maxDesiredSpeed;
		//! \see StaticInfVeh
		double maxAcceleration;
		//! \see StaticInfVeh
		double normalDeceleration;
		//! \see StaticInfVeh
		double maxDeceleration;
		//! \see StaticInfVeh
		double speedAcceptance;
		//! \see StaticInfVeh
		double minDistanceVeh;
		//! \see StaticInfVeh
		double giveWayTime;
		//! \see StaticInfVeh
		double guidanceAcceptance;
		//! \see StaticInfVeh
		int enrouted;
		//! \see StaticInfVeh
		int equipped;
		//! \see StaticInfVeh
		int tracked;
		//! \see StaticInfVeh
		bool  keepfastLane;
		//! \see StaticInfVeh
		double safetyMarginFactor;
		//! \see StaticInfVeh
		double headwayMin;
		//! \see StaticInfVeh
		double sensitivityFactor;
		//! \see StaticInfVeh
		double reactionTime;
		//! \see StaticInfVeh
		double reactionTimeAtStop;
		//! \see StaticInfVeh
		double reactionTimeAtTrafficLight;
        //! \see StaticInfVeh
        bool laneChangingCooperation;
        //! \see StaticInfVeh
        double laneChangingAggressivenessLevel;
        //! \see StaticInfVeh
        double distanceZoneFactor;
		//! Maximum number of people inside the ptvehicle
		int	maxCapacity;
		//! Transit line identifier
		int idLine;

		//MFC, emission & consumption models
		int engineTypeId;
		//LEM
		int EUEmissionId;
		//Energy consumption Model
		double energyCapacity;
	};

	struct AAPIEXPORT ModifyStopTimeRequest{
		enum ReqReason {eAlight, eBoarding, eOther};
		int vehId;
		int stopIndex;
		double timeRequested;
		ReqReason reason;
	};

	struct AAPIEXPORT ModifyStopTimeResult{
		int errorCode;
		double time;
	};

	AAPIEXPORT int		AKIPTGetNumberLines();
	AAPIEXPORT int		AKIPTGetIdLine(int elem);

	AAPIEXPORT int		AKIGetNbVehiclesFollowingPTLine(int lineId);
	AAPIEXPORT int		AKIGetVehicleFollowingPTLine(int lineId, int vehPos);

	AAPIEXPORT int		AKIPTGetNumberSectionsInLine(int lineId);
	AAPIEXPORT int		AKIPTGetIdSectionInLine(int lineId, int elem);

	AAPIEXPORT int		AKIPTGetNumberStopsInLine(int lineId);
	AAPIEXPORT int		AKIPTGetIdStopsInLine(int lineId, int elem);

	AAPIEXPORT int		AKIPTEnterVeh( int lineId, int vehTypePos, bool tracked );
	AAPIEXPORT int		AKIPTVehModifyStopTime( int aidVeh, int nstop, double stopTime );
	AAPIEXPORT ModifyStopTimeResult AKIPTVehRequestModifyStopTime( const ModifyStopTimeRequest & request );
	AAPIEXPORT int		AKIPTVehReroute(int aidVeh, int nbsections, int *asections, int *aaNewstops, double *adwellTime, double *aoffset);



	AAPIEXPORT StaticInfPTVeh 	AKIPTVehGetStaticInf(int aidVeh);
	AAPIEXPORT int				AKIPTVehSetStaticInf(int aidVeh, StaticInfPTVeh staticinfVeh);
	AAPIEXPORT InfPTVeh 		AKIPTVehGetInf(int aidVeh);
	AAPIEXPORT double			AKIPTGetServiceTimeStopsInLine(int aidVeh, int nstop);
	AAPIEXPORT int AKIPTVehSetCurrentLoad( int aidVeh, int currentLoad );
	// .......................................
	// 		PRINT INFORMATION IN XCONSOLE / LOG WINDOW
	// .......................................

	AAPIEXPORT int AKIPrintString(const char *string);
	AAPIEXPORT void AKIPrintAsUNICODEString(const unsigned short *string);
	// remember to delete[] the const char *
	AAPIEXPORT const char *AKIConvertToAsciiString(const unsigned short *string, bool deleteUshortString, bool *anyNonAsciiChar);
	// remember to delete[] the const unsigned short *
	AAPIEXPORT const unsigned short *AKIConvertFromAsciiString(const char *ascii);
	AAPIEXPORT void AKIDeleteUNICODEString( const unsigned short *string );
	AAPIEXPORT void AKIDeleteASCIIString( const char *string );
	//.........................................
	//     ODDemand
	//........................................

	#define AKIODDemandNoTrafficOD 			-11000
	#define AKIODDemandIncorrectNumSlice 	-11002
	#define AKIODDemandUnknownCentroid 		-11003
	#define AKIODDemandUnknownODPair 		-11004


	AAPIEXPORT int AKIODDemandGetNumSlicesOD(int vehTypePos);
	AAPIEXPORT double AKIODDemandGetIniTimeSlice(int vehTypePos, int slice);
	AAPIEXPORT double AKIODDemandGetEndTimeSlice(int vehTypePos, int slice);
	AAPIEXPORT int AKIODDemandGetDemandODPair(int origen, int desti, int vehTypePos, int numslice);
	AAPIEXPORT int AKIODDemandSetDemandODPair(int origen, int desti, int vehTypePos, int numslice, int anewdemand);

	//.........................................
	//     TrafficDemand
	//........................................

	#define AKIStateDemandNoTrafficState		-15000
	#define AKIStateDemandIncorrectNumSlice 	-15002
	#define AKIStateDemandUnknownVehType 		-15004

	AAPIEXPORT int AKIStateDemandGetNumSlices(int vehTypePos);
	AAPIEXPORT double AKIStateDemandGetIniTimeSlice(int vehTypePos, int numslice);
	AAPIEXPORT double AKIStateDemandGetEndTimeSlice(int vehTypePos, int numslice);
	AAPIEXPORT double AKIStateDemandGetDemandSection(int idSection, int vehTypePos, int numslice);
	AAPIEXPORT int AKIStateDemandSetDemandSection(int idSection, int vehTypePos, int numslice, double anewflow);
    AAPIEXPORT int AKIStateDemandSetTurningPercentage( int idSectionFrom, int idSectionTo, int vehType, int numSlice, double newPercentage );
	//.........................................
	//     PastCosts
	//........................................
	#define AKIPastCostUnknownLink 			-12001
	#define AKIPastCostNoPerVehType		 	-12002
	#define AKIPastCostPerVehType 			-12003
	#define AKIPastCostNoReaded 			-12004
	#define AKIPastCostIncorrectTypeName	-12005

	AAPIEXPORT int AKIPastCostAreCostsPerVehicleType();
	AAPIEXPORT double AKIPastCostGetIniTimeReaded();
	AAPIEXPORT double AKIPastCostGetIntervalReaded();
	AAPIEXPORT int AKIPastCostGetNbIntervalsReaded();
	AAPIEXPORT int AKIPastCostSetPastCost(int sectorig, int sectdest, double aTime, int idVehType, double acost, double aocost);
	AAPIEXPORT double AKIPastCostGetPastCost(int sectorig, int sectdest, double aTime, int idVehType);
	AAPIEXPORT double AKIPastCostGetPastOutputCost(int sectorig, int sectdest, double aTime, int idVehType);

// Functions relative to path information
	#define AKIInfVehPathNotAvailable	-13001
	#define AKIInfVehDestinationUnreachable	-13002
	#define AKIInfVehInvalidDestinationCentroid	-13003
	
	/*! \brief Path vehicle's information.
	  */
	struct AAPIEXPORT PathInfVeh {
		//! 0 OK, otherwise check the error code.
		int report;
		//! Vehicle identifier.
		int idVeh;
		/*! Path type:
				-# Route Choice Path
				-# User-defined Route
				-# User-defined Shortest Path Tree
		  */
		int type;
		//! Entrance section identifier. This is the section from where the vehicle enters the network.
		int entranceSectionId;
		//! Total number of sections that have	the path.
		int numSectionsInPath;
		//! Total distance of the path in m.
		double totalDistance;
		//! Total free flow of the path in seconds using the vehicle speed acceptance.
		double totalFreeFlowTravelTime;
	};

	AAPIEXPORT PathInfVeh AKIVehInfPath( int idveh );
	AAPIEXPORT PathInfVeh AKIVehInfPathSection(int aidSec, int indexveh);
	AAPIEXPORT int AKIVehInfPathGetNextSection(int idveh, int fromsection);
	AAPIEXPORT int AKIVehInfSectionInPath(int idveh, int idsection);
	AAPIEXPORT int AKIVehInfPathGetNextSectionInSection(int aidSec, int indexveh, int fromsection);

//Function relative to Legion and the Aimsun pedestrian simulator
    AAPIEXPORT int AKIGeneratePedestrians( int fromCentroid, int toCentroid, int idRoute, double nbPedestrians );
    AAPIEXPORT int AKIGeneratePedestriansInTime( int fromCentroid, int toCentroid, int idRoute, double nbPedestrians, double timeInterval );
    AAPIEXPORT int AKIGeneratePedestriansDefaultRoutes( int fromCentroid, int toCentroid, double nbPedestrians );
    AAPIEXPORT int AKIGeneratePedestriansDefaultRoutesInTime( int fromCentroid, int toCentroid, double nbPedestrians, double timeInterval );

	struct AAPIEXPORT StaticInfPed {
		std::int32_t  report;
		std::uint32_t id;
		std::uint32_t originID;
		std::uint32_t destinationID;
		std::uint32_t typeID;
		double        radius;
		double        preferredWalkSpeed;
		double        preferredRunSpeed;
	};

	struct AAPIEXPORT InfPosition
	{
		double x;
		double y;
		double z;
	};

struct AAPIEXPORT InfPed {
		std::int32_t  report;
		std::uint32_t id;
		InfPosition   position;
		double        speed;
	};

	AAPIEXPORT StaticInfPed AKIPedestrianGetStaticInf( std::uint32_t aidPedestrian );
	AAPIEXPORT void         AKIPedestrianSetStaticInf( StaticInfPed newstaticInfPed );
	AAPIEXPORT InfPed       AKIPedestrianGetInf( std::uint32_t aidPedestrian );

	AAPIEXPORT int AKIConvertLatitudeLongitudeToXY( double latitude, double longitude, double *x, double *y );
	AAPIEXPORT int AKIConvertXYToLatitudeLongitude( double x, double y, double *latitude, double *longitude);
}



#endif
