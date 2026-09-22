/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005 Jean-Luc Rocca-Serra, Mark Grant

  ORBITER vessel module: Connector class

  Project Apollo is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Project Apollo is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Project Apollo; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  See http://nassp.sourceforge.net/license/ for more details.

  **************************************************************************/

#if !defined(_PA_CONNECTOR_H)
#define _PA_CONNECTOR_H

#include "nasspmp_api.h"
#include "replication/KinematicPrediction.h"
#include "replication/ReplicationHub.h"

#include <chrono>
#include <stdio.h>

///
/// \ingroup Connectors
/// \brief Connector type.
///
enum ConnectorType
{
	NO_CONNECTION,				///< Dummy.
	CSM_IU_COMMAND,				///< Passes commands and data between CSM and IU.
	LV_IU_COMMAND,				///< Passes commands between launch vehicle and IU.
	CSM_LEM_DOCKING,			///< Docking connector between CSM and LEM.
	LEM_CSM_POWER,				///< Power connection from CSM to LEM.
	MFD_PANEL_INTERFACE,		///< Connector from an MFD to a panel.
	LEM_CSM_ECS,				///< Tunnel connection from CSM to LEM.
	CHECKLIST_DATA_INTERFACE,	///< Data connector from checklist controller to vessel
	PAYLOAD_SLA_CONNECT,		///< Passes commands and data between payload and SIVb.
	CSM_PAYLOAD_COMMAND,		///< Docking connector between CSM and Payload
	SIVB_SI_COMMAND,			///< Docking connector between S-IVB and S-IB/S-II
	SII_SIC_COMMAND,			///< Docking connector between S-II and S-IC
	RADAR_RF_SIGNAL,			///< Radar connector betwen LM rendezvous radar amd CSM rendezvous radar transponder
	VHF_RNG,
};

#define VIRTUAL_CONNECTOR_PORT	(0xffff)		///< Port ID for 'virtual' connectors which don't physically exist.

//
// The message can pass various different parameters in this union. The receiver
// will determine which is correct based on the message type.
//
// \brief Message value for connector messages.
//
union ConnectorMessageValue
{
	int iValue;				///< Integer message value.
	double dValue;			///< Floating point message value.
	bool bValue;			///< Boolean message value.
	void *pValue;			///< Pointer message value.
	OBJHANDLE hValue;		///< Orbiter handle.
	VECTOR3 vValue;			///< Vector message value;
};

///
/// \ingroup Connectors
/// \brief Connector message to be passed through the system.
///
struct ConnectorMessage
{
	///
	/// \brief What kind of connector should this message go to?
	///
	ConnectorType destination;

	///
	/// \brief Connection-specific message type.
	///
	unsigned int messageType;

	///
	/// \brief Message value 1.
	///
	ConnectorMessageValue val1;

	///
	/// \brief Message value 2.
	///
	ConnectorMessageValue val2;

	///
	/// \brief Message value 3.
	///
	ConnectorMessageValue val3;

	///
	/// \brief Message value 4.
	///
	ConnectorMessageValue val4;
};

///
/// \ingroup Connectors
/// \brief Connector class. Specific connectors will be derived from this class.
///
class Connector {
public:
	///
	/// \brief Constructor.
	///
	Connector();

	///
	/// \brief Destructor. Disconnects the connector when called.
	///
	virtual ~Connector();

	///
	/// \brief Get the type of the connector.
	/// \return Connector type.
	///
	virtual ConnectorType GetType();

	///
	/// \brief Set the type of the connector.
	/// \param t Connector type.
	///
	void SetType(ConnectorType t) { type = t; };

	///
	/// \brief Connect to another connector.
	/// \param other Other end of the connection.
	/// \return True if the connector is the correct type and we connected.
	///
	virtual bool ConnectTo(Connector *other);

	///
	/// \brief Disconnect from the far end of the connection.
	///
	virtual void Disconnect();

	///
	/// \brief Has been disconnected from the far end of the connection.
	///
	virtual void Disconnected();

	///
	/// Send a message through the connector. Note that the receiver can update the value in the
	/// connector message, in order to return data to the caller.
	///
	/// \brief Send message.
	/// \param m Message to send.
	/// \return False if the connector isn't connected to anything or the message wasn't handled
	/// at the far end of the connection.
	///
	virtual bool SendMessage(ConnectorMessage &m);

	///
	/// Receieve a message through the connector. Note that the receiver can update the value in the
	/// connector message, in order to return data to the caller.
	///
	/// The default connector is output-only, and will return an error if the other end of the connection
	/// tries to send it data.
	///
	/// \brief Receive message.
	/// \param from The connector that sent the message.
	/// \param m Message received.
	/// \return False if we don't handle the message.
	///
	virtual bool ReceiveMessage(Connector *from, ConnectorMessage &m);

	///
	/// \brief Connector we're connected to, if any.
	///
	Connector *connectedTo;

protected:
	///
	/// \brief Type of connection.
	///
	ConnectorType type;
};

///
/// \ingroup Connectors
/// \brief Connector class for multiple connectors, typically used for docked ships.
///
class MultiConnector : public Connector
{
public:
	MultiConnector();
	~MultiConnector();

	///
	/// \brief Add another connection to this connector.
	/// \param other Connector to add.
	/// \return True if we could add the new connector.
	///
	virtual bool AddTo(Connector *other);

	bool ReceiveMessage(Connector *from, ConnectorMessage &m);
	void Disconnect();

#define N_MULTICONNECT_INPUTS 16

private:
	Connector *Inputs[N_MULTICONNECT_INPUTS];
};

class PanelSwitches;
class ChecklistController;

///
/// \ingroup Connectors
/// \brief Connector class for panel interface.
///
class PanelConnector : public Connector
{
public:

	///
	/// \ingroup Connectors
	/// \brief Message type to send from the CSM to the SIVb.
	///
	enum PanelConnectorMessageType
	{
		MFD_PANEL_FLASH_ITEM,					///< Turn flash on or off.
		MFD_PANEL_GET_ITEM_STATE,				///< Get the item's current state.
		MFD_PANEL_SET_ITEM_STATE,				///< Set the item's current state.
		MFD_PANEL_GET_FAILED_STATE,				///< Get the item's failed state.
		MFD_PANEL_CHECKLIST_AUTOCOMPLETE,		///< Checklist autocomplete.
		MFD_PANEL_GET_CHECKLIST_ITEM,			///< Get the an checklist item.
		MFD_PANEL_GET_CHECKLIST_LIST,			///< Get list of allowed checklists.
		MFD_PANEL_FAIL_ITEM,					///< Fail a checklist step.
		MFD_PANEL_COMPLETE_ITEM,				///< Complete a checklist step.
		MFD_PANEL_CHECKLIST_AUTOCOMPLETE_QUERY,	///< Find out the autocomplete state.
		MFD_PANEL_CHECKLIST_NAME,				///< Find out the name of the current checklist
		MFD_PANEL_RETRIEVE_CHECKLIST,			///< Get an entire, non-controlled, checklist
		MFD_PANEL_CHECKLIST_FLASHING,			///< Checklist item flashing.
		MFD_PANEL_CHECKLIST_FLASHING_QUERY,		///< Checklist item flashing.
		MFD_PANEL_GET_ITEM_FLASHING,			///< Get the item's current flashing.
		MFD_PANEL_GET_CHECKLIST_AUTOEXECUTE,	///< Get automatic checklist execution.
		MFD_PANEL_SET_CHECKLIST_AUTOEXECUTE,    ///< Set automatic checklist execution.
		MFD_PANEL_GOTO_CHECKLIST_ITEM,          ///< Go to checklist item
		MFD_PANEL_UNDO_CHECKLIST_ITEM           ///< Undo last item   
	};

	PanelConnector(PanelSwitches &p, ChecklistController &c);
	~PanelConnector();

	bool ReceiveMessage(Connector *from, ConnectorMessage &m);

private:
	PanelSwitches &panel;
	ChecklistController &checklist;
};

///
/// ProjectApollo-specific vessel which allows us to get connectors to communicate
/// while docked.
/// \ingroup Connectors
///

class ProjectApolloConnectorVessel : public VESSEL4, public IReplicationProvider
{
public:

	struct ConnectorDefinition
	{
		int port;
		Connector *c;

		ConnectorDefinition() { port = 0; c = 0; };
	};

	///
	/// \brief Constructor.
	///
	ProjectApolloConnectorVessel(OBJHANDLE hObj, int fmodel);

	///
	/// \brief Destructor.
	///
	virtual ~ProjectApolloConnectorVessel();

	///
	/// Other vessels can call this function to get a connector to talk to when they
	/// are docked. They need to specify the docking port number so we can have multiple
	/// connectors of the same time on vessels with multiple ports; for example a space
	/// station may supply power to each docking port.
	///
	/// \brief Get a pointer to a connector of specified type.
	/// \param port Docking port number.
	/// \param t Connector type to look for.
	/// \return Pointer to connector if one is registered, NULL if none registered.
	///
	virtual Connector *GetConnector(int port, ConnectorType t);

	///
	/// This is a sanity-check. If the validation fails, then the vessel probably isn't really of this class!
	///
	/// \return True if this is a valid vessel.
	///
	bool ValidateVessel();

	///
	/// \brief Returns whether this vessel's replication hub role is Replica.
	/// \return True if replica
	///
	bool IsMultiplayerReplica() const { return ReplicationHubInstance.GetRole() == ReplicationRole::Replica; }

	///
	/// \brief Retrieve the replication hub for this vessel
	/// \return Reference to replication hub
	///
	ReplicationHub &GetReplicationHub() { return ReplicationHubInstance; }
	const ReplicationHub &GetReplicationHub() const { return ReplicationHubInstance; }

	///
	/// Converts a simulator date into the timeline shared with the authority. Together with
	/// the timestamp an authority sample carries, this gives a prediction horizon that is a
	/// plain difference of two exchanged timestamps, with no estimated quantity in it.
	/// \brief Maps an Orbiter MJD onto the shared session timeline
	/// \param mjd Orbiter date, normally the mjd argument of the current callback
	/// \param sessionTime Receives seconds since the negotiated session epoch
	/// \return False if shared session timeline is not available
	///
	bool TryGetSessionTime(double mjd, double &sessionTime) const;

	///
	/// This function extrapolates position and orientation between host updates
	/// \brief If vessel is a replica, this prepares its predicted state for the end of the Orbiter physics step
	/// \param simdt Orbiter integration interval following this call
	/// \param mjd Orbiter date at the end of that interval, which is the date being rendered
	///
	void UpdateReplicatedKinematics(double simdt, double mjd);

	/// Provider block
	const char *ComponentKey() const override;
	ProviderResult Describe(ReplicationCatalogBuilder &catalog) const override;
	ProviderResult Capture(const char *groupKey, ReplicationWriter &writer, const CaptureContext &context) override;
	ProviderResult Validate(const char *groupKey, const ReplicationReader &reader, const ApplyContext &context) const override;
	void Apply(const char *groupKey, const ReplicationReader &reader, const ApplyContext &context) override;
	void OnRoleChanged(ReplicationRole role) override;

	///
	/// \brief Set up connectors on docking.
	///
	void DockConnectors(int port);

	///
	/// \brief Disconnect connectors on undocking.
	///
	void UndockConnectors(int port);

public:
	enum class StageEventKind : std::uint8_t
	{
		None,
		S1C,
		S1B,
		Interstage,
		S2,
		SIVB,
		ServiceModule,
		CommandModule,
		Abort,
		PayloadSeparation,
		Liftoff
	};

protected:
	struct StageTransition
	{
		StageEventKind kind = StageEventKind::None;
		std::uint8_t targetStage = 0;
		bool spawnsEntity = false;
		VECTOR3 position = {};
		VECTOR3 velocity = {};
		nasspmp_kinematics::Quaternion orientation;
	};
	// Publishes one authority staging result for a change-replicated vessel group.
	void AnnounceStageEvent(StageEventKind kind, int targetStage, bool spawnsEntity, const VESSELSTATUS &spawnState);
	// Makes the received spawn pose available only while the matching staging call executes.
	void BeginReplicatedStageEvent(const StageTransition &event);
	void EndReplicatedStageEvent();
	void ApplyReplicatedStageSpawnState(VESSELSTATUS &spawnState) const;
	bool IsApplyingReplicatedStageEvent() const { return applyingReplicatedStageEvent; }
	const StageTransition &LatestStageEvent() const { return stageEvent; }
	Revision StageEventRevision() const { return stageEventRevision; }

	///
	/// \brief Register a connector for use by other vessels.
	/// \param port Docking port number.
	/// \param c Pointer to a connector.
	/// \return True if registered, false if not (e.g. too many registered already).
	///
	bool RegisterConnector(int port, Connector *c);

#define PACV_N_VALIDATION	0x5a715a75

	///
	/// We store a known value here for validation. If a vessel is not of this class then it's unlikely to
	/// have the same value at this location and will fail to validate; unfortunately it may also crash if
	/// the vessel happens to have been allocated at the end of the heap!
	///
	/// \brief Validation value.
	///
	unsigned int ValidationValue;

#define PACV_N_CONNECTORS 16

	ConnectorDefinition ConnectorList[PACV_N_CONNECTORS];

	struct ReplicatedKinematics
	{
		// Motion is predicted from this immutable authority sample.
		nasspmp_kinematics::State state;

		// Orbiter needs a separate surface representation for landed vessels.
		VECTOR3 landedOrientation = {};
		double surfaceLongitude = 0.0;
		double surfaceLatitude = 0.0;
		double surfaceHeading = 0.0;
		double landedAltitude = 0.0;
		std::uint8_t flightStatus = 0;

		// Whether position and velocity turn with the reference body or are inertial.
		// Currently not really used (= always inertial) since replication is stable without.
		std::uint8_t frame = 0;

		// Body the authority measured position and velocity against
		OBJHANDLE referenceBody = NULL;

		// Instant the authority captured this sample, in seconds since the session epoch.
		double sampleSessionTime = 0.0;
		bool hasSessionTime = false;
	};
    
	//////////////////////////////////////////////////
	// Kinematic block. Maybe move to separate class?

	ProviderResult ReadKinematics(const ReplicationReader &reader, ReplicatedKinematics *target, const ApplyContext &context) const;
	nasspmp_kinematics::State CurrentKinematicState(OBJHANDLE reference = NULL) const;
	nasspmp_kinematics::State CurrentKinematicState(const VESSELSTATUS2 &status) const;
	void ApplyFreeFlightState(const nasspmp_kinematics::State &state, OBJHANDLE reference);
	void ApplyLandedState(const ReplicatedKinematics &state);
	void OpenKinematicsLog(ReplicationRole role);
	void LogKinematics(const char *event, double sampleSessionTime, std::uint8_t flightStatus,
		double renderSessionTime, double stepSeconds,
		double predictionSeconds, double correctionFraction, int smoothCorrection,
		const nasspmp_kinematics::State *sample, const nasspmp_kinematics::State *current,
		const nasspmp_kinematics::State *target, const nasspmp_kinematics::State *applied,
		const VECTOR3 *positionError, const VECTOR3 *velocityError, const VECTOR3 *orbiterAcceleration);

	ReplicatedKinematics replicatedKinematics;

	// If there was done a replication already and we have a valid authoritative state
	bool hasReplicatedKinematics = false;

	// A visual position/orientation offset is active and is decayed smoothly towards zero
	bool correctionActive = false;

	// If the current landed state was already applied. Application of the landing state is a one-shot event
	bool landedStateApplied = false;

	// Difference between the state on display and the authority target. Adding it keeps the
	// vessel continuous when a new sample arrives, a filter decays it back to zero.
	VECTOR3 positionCorrection = {};
	VECTOR3 velocityCorrection = {};
	nasspmp_kinematics::Quaternion orientationCorrection;

	// Samples velocities to derive the total acceleration, which is supplied to the Replica to improve prediction
	// This could maybe be calculated exactly from gravity, thrust and aerodynamic values

	// TODO: Put in struct?
	bool hasAuthorityVelocitySample = false;
	double authorityVelocitySampleTime = 0.0;
	// Differencing across a change of frame or body would be dangerous.
	std::uint8_t authorityVelocitySampleFrame = 0;
	std::uint64_t authorityVelocitySampleBody = 0;
	VECTOR3 authorityVelocitySample = {};
	VECTOR3 authorityAccelerationSample = {};

	// Measures the acceleration applied by Orbiter between consecutive replica PreSteps.
	// TODO: This is a dirty trick. It predicts orbiters time step and compensates it, because for now we want
	// the authority over the prediction to be here. Unfortunately found no way to disable orbiter kinematics from NASSP.
	// Could later be improved by supplying correct thruster accelerations. Gravity and aerodynamics should be correct already.
	// TODO: Put in struct?
	bool hasReplicaIntegrationSample = false;
	VECTOR3 replicaIntegrationStartVelocity = {};
	double replicaIntegrationStepSeconds = 0.0;
	bool hasReplicaOrbiterAcceleration = false;
	VECTOR3 replicaOrbiterAcceleration = {};

	// Kinematic debug
	bool kinematicsUpdateLogPending = false;
	FILE *kinematicsLogFile = NULL;
	unsigned int kinematicsLogSequence = 0;
	std::chrono::steady_clock::time_point kinematicsLogStarted;
	std::chrono::steady_clock::time_point kinematicsLogLastFlush;

	// Staging related replication values
	StageTransition stageEvent;
	Revision stageEventRevision = 0;
	bool applyingReplicatedStageEvent = false;

	// Owns the registration and role boundary for this replicated vessel entity.
	ReplicationHub ReplicationHubInstance;
};

///
/// \ingroup Connectors
///
/// This function tries to get a connector from the specified docking port on the
/// specified vessel. It tries to determine whether the connector is a Project
/// Apollo vessel and then tries to get the connector pointer if it is.
///
/// \brief Get a connector from a vessel.
/// \param v Vessel, which may or may not be one of ours.
/// \param port Docking port number.
/// \param t Connector type to look for.
/// \return Connector if found, or NULL if not.
///
extern Connector *GetVesselConnector(VESSEL *v, int port, ConnectorType t);

///
/// \ingroup Connectors
/// \brief Radar Messages
///
enum RFconnectorMessageType {
	CW_RADAR_SIGNAL, ///< Continuous Wave Radar Signal
	RR_XPDR_SIGNAL, ///< Radar Transponder Signal
	VHF_RNG_SIGNAL_CSM,
	VHF_RNG_SIGNAL_LM,
};

#endif // _PA_CONNECTOR_H
