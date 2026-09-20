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

// To force Orbitersdk.h to use <fstream> in any compiler version
#pragma include_alias( <fstream.h>, <fstream> )
#include "Orbitersdk.h"

#include "nasspdefs.h"

#include "PanelSDK/PanelSDK.h"
#include "PanelSDK/Internals/Esystems.h"

#include "powersource.h"
#include "connector.h"

#include <algorithm>
#include <cmath>
#include <process.h>
#include <stdio.h>
#include <string.h>

namespace
{
const char *KinematicsGroupKey = "state";
const double KinematicCorrectionDurationSeconds = 0.25;
const double KinematicMinimumSmoothPositionMeters = 0.5;
const double KinematicMinimumSmoothAngleRadians = 0.5 * RAD;
const double KinematicTimingErrorSeconds = 0.05;
const double MaximumPredictionRealAgeSeconds = 0.5;
const std::uint8_t FreeFlightStatus = 0;
const std::uint8_t LandedStatus = 1;
const std::uint8_t DockedStatus = 2;
const DWORD LandedFlightStatusFlag = 1;
const DWORD DockedFlightStatusFlag = 2;

bool IsFinite(const VECTOR3 &value)
{
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsFinite(const nasspmp_kinematics::Quaternion &value)
{
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) && std::isfinite(value.w);
}

const char *KinematicsRoleName(ReplicationRole role)
{
	return role == ReplicationRole::Authority ? "authority" : "replica";
}

void WriteKinematicsStateHeader(FILE *file, const char *prefix)
{
	fprintf(file, ",%s_position_x,%s_position_y,%s_position_z", prefix, prefix, prefix);
	fprintf(file, ",%s_velocity_x,%s_velocity_y,%s_velocity_z", prefix, prefix, prefix);
	fprintf(file, ",%s_acceleration_x,%s_acceleration_y,%s_acceleration_z", prefix, prefix, prefix);
	fprintf(file, ",%s_quaternion_x,%s_quaternion_y,%s_quaternion_z,%s_quaternion_w", prefix, prefix, prefix, prefix);
	fprintf(file, ",%s_angular_velocity_x,%s_angular_velocity_y,%s_angular_velocity_z", prefix, prefix, prefix);
}

void WriteKinematicsState(FILE *file, const nasspmp_kinematics::State *state)
{
	if (!state) {
		for (int index = 0; index < 16; index++)
			fprintf(file, ",nan");
		return;
	}

	fprintf(file, ",%.17g,%.17g,%.17g", state->position.x, state->position.y, state->position.z);
	fprintf(file, ",%.17g,%.17g,%.17g", state->velocity.x, state->velocity.y, state->velocity.z);
	fprintf(file, ",%.17g,%.17g,%.17g", state->acceleration.x, state->acceleration.y, state->acceleration.z);
	fprintf(file, ",%.17g,%.17g,%.17g,%.17g", state->orientation.x, state->orientation.y,
		state->orientation.z, state->orientation.w);
	fprintf(file, ",%.17g,%.17g,%.17g", state->angularVelocity.x, state->angularVelocity.y, state->angularVelocity.z);
}

void WriteKinematicsVectorHeader(FILE *file, const char *prefix)
{
	fprintf(file, ",%s_x,%s_y,%s_z", prefix, prefix, prefix);
}

void WriteKinematicsVector(FILE *file, const VECTOR3 *value)
{
	if (value)
		fprintf(file, ",%.17g,%.17g,%.17g", value->x, value->y, value->z);
	else
		fprintf(file, ",nan,nan,nan");
}
}

Connector::Connector()

{
	type = NO_CONNECTION;
	connectedTo = 0;
}

Connector::~Connector()

{
	//
	// We have to be sure to disconnect before deletion so the other end doesn't try to send
	// us any more data.
	//
	Disconnect();
}

void Connector::Disconnect()

{
	if (connectedTo)
	{
		connectedTo->connectedTo = 0;
		connectedTo = 0;
	}
}

void Connector::Disconnected()

{
	//
	// Do nothing by default.
	//
}

bool Connector::ConnectTo(Connector *other)

{
	//
	// Other may be NULL. If so just clear this connection, otherwise
	// check the other end is the correct type and disconnected.
	//
	if (other)
	{
		if (other->GetType() != type || other->connectedTo)
		{
			return false;
		}

		other->connectedTo = this;
	}

	connectedTo = other;

	return true;
}

bool Connector::SendMessage(ConnectorMessage &m)

{
	if (connectedTo)
	{
		return connectedTo->ReceiveMessage(this, m);
	}

	return false;
}

bool Connector::ReceiveMessage(Connector *from, ConnectorMessage &m)

{
	//
	// The default connector is one-way. Return an error if anything tries to send data to us.
	//
	return false;
}

ConnectorType Connector::GetType()

{
	return type;
}

MultiConnector::MultiConnector()

{
	int i;

	for (i = 0; i < N_MULTICONNECT_INPUTS; i++)
	{
		Inputs[i] = 0;
	}
}

MultiConnector::~MultiConnector()

{
	int i;

	for (i = 0; i < N_MULTICONNECT_INPUTS; i++)
	{
		if (Inputs[i])
		{
			Inputs[i]->connectedTo = 0;
			Inputs[i] = 0;
		}
	}
}

bool MultiConnector::AddTo(Connector *other)

{
	int i;

	if (!other)
		return false;

	//
	// First check this isn't already connected.
	//
	for (i = 0; i < N_MULTICONNECT_INPUTS; i++)
	{
		if (Inputs[i] == other)
		{
			other->connectedTo = this;
			return true;
		}
	}

	//
	// Only one connector of each type.
	//
	ConnectorType otherType = other->GetType();

	for (i = 0; i < N_MULTICONNECT_INPUTS; i++)
	{
		if (Inputs[i] && Inputs[i]->GetType() == otherType)
		{
			return false;
		}
	}

	//
	// Then add it if there's a free slot.
	//
	for (i = 0; i < N_MULTICONNECT_INPUTS; i++)
	{
		if (!Inputs[i])
		{
			Inputs[i] = other;
			other->connectedTo = this;
			return true;
		}
	}

	return false;
}

bool MultiConnector::ReceiveMessage(Connector *from, ConnectorMessage &m)

{
	if (from == connectedTo)
	{
		int i;

		//
		// The message came from the other end of the connection, so forward to the
		// appropriate connection on this side.
		//
		for (i = 0; i < N_MULTICONNECT_INPUTS; i++)
		{
			if (Inputs[i] && Inputs[i]->GetType() == m.destination) 
			{
				return Inputs[i]->ReceiveMessage(from, m);
			}
		}
	}
	else if (connectedTo)
	{
		//
		// The message came from an input, so send to the other end.
		//
		return connectedTo->ReceiveMessage(this, m);
	}

	//
	// Should never happen unless we're disconnected from the other end.
	//
	return false;
}

void MultiConnector::Disconnect()

{
	int i;

	//
	// Tell all our connectors that we've disconnected.
	//
	for (i = 0; i < N_MULTICONNECT_INPUTS; i++)
	{
		if (Inputs[i])
		{
			Inputs[i]->Disconnected();
		}
	}

	Connector::Disconnect();
}

ProjectApolloConnectorVessel::ProjectApolloConnectorVessel(OBJHANDLE hObj, int fmodel) : VESSEL4(hObj, fmodel)

{
	int i;
	for (i = 0; i < PACV_N_CONNECTORS; i++)
	{
		ConnectorList[i].port = 0;
		ConnectorList[i].c = NULL;
	}

	ValidationValue = PACV_N_VALIDATION;
}

ProjectApolloConnectorVessel::~ProjectApolloConnectorVessel()
{
	if (kinematicsLogFile) {
		fclose(kinematicsLogFile);
		kinematicsLogFile = NULL;
	}

	// Clearing stale non-owning entries is safe because UnregisterAll() does not dereference providers.
	ReplicationHubInstance.UnregisterAll();

	//Disconnect all connectors
	int i;
	for (i = 0; i < PACV_N_CONNECTORS; i++)
	{
		if (ConnectorList[i].c)
		{
			ConnectorList[i].c->Disconnect();
		}
	}
}

void ProjectApolloConnectorVessel::OpenKinematicsLog(ReplicationRole role)
{
	if (kinematicsLogFile) {
		fclose(kinematicsLogFile);
		kinematicsLogFile = NULL;
	}
	if (role != ReplicationRole::Authority && role != ReplicationRole::Replica)
		return;

	char vesselName[128];
	const char *sourceName = GetName();
	unsigned int nameLength = 0;
	while (sourceName && sourceName[nameLength] && nameLength < sizeof(vesselName) - 1) {
		const char character = sourceName[nameLength];
		const bool validCharacter = (character >= 'a' && character <= 'z') ||
			(character >= 'A' && character <= 'Z') || (character >= '0' && character <= '9') ||
			character == '-' || character == '_';
		vesselName[nameLength] = validCharacter ? character : '_';
		nameLength++;
	}
	vesselName[nameLength] = 0;
	if (nameLength == 0)
		strcpy(vesselName, "vessel");

	char fileName[256];
	sprintf_s(fileName, sizeof(fileName), "NASSP-MP-Kinematics-%s-%s-%d-%u.csv", vesselName,
		KinematicsRoleName(role), _getpid(), ++kinematicsLogSequence);
	kinematicsLogFile = fopen(fileName, "wt");
	if (!kinematicsLogFile)
		return;

	setvbuf(kinematicsLogFile, NULL, _IOFBF, 64 * 1024);
	fprintf(kinematicsLogFile, "event,wall_seconds,server_tick,flight_status,message_age_seconds,elapsed_seconds,authority_time_scale,prediction_seconds,correction_fraction,smooth_correction");
	WriteKinematicsStateHeader(kinematicsLogFile, "sample");
	WriteKinematicsStateHeader(kinematicsLogFile, "current");
	WriteKinematicsStateHeader(kinematicsLogFile, "target");
	WriteKinematicsStateHeader(kinematicsLogFile, "applied");
	WriteKinematicsVectorHeader(kinematicsLogFile, "position_error");
	WriteKinematicsVectorHeader(kinematicsLogFile, "velocity_error");
	WriteKinematicsVectorHeader(kinematicsLogFile, "orbiter_acceleration");
	fprintf(kinematicsLogFile, "\n");
	fflush(kinematicsLogFile);
	kinematicsLogStarted = std::chrono::steady_clock::now();
	kinematicsLogLastFlush = kinematicsLogStarted;
}

void ProjectApolloConnectorVessel::LogKinematics(const char *event, SimulationTick serverTick,
	std::uint8_t flightStatus, double messageAgeSeconds, double elapsedSeconds, double authorityTimeScale,
	double predictionSeconds, double correctionFraction, int smoothCorrection,
	const nasspmp_kinematics::State *sample, const nasspmp_kinematics::State *current,
	const nasspmp_kinematics::State *target, const nasspmp_kinematics::State *applied,
	const VECTOR3 *positionError, const VECTOR3 *velocityError, const VECTOR3 *orbiterAcceleration)
{
	if (!kinematicsLogFile)
		return;

	const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	const double wallSeconds = std::chrono::duration<double>(now - kinematicsLogStarted).count();
	fprintf(kinematicsLogFile, "%s,%.9f,%llu,%u,%.9f,%.9f,%.9f,%.9f,%.9f,%d", event, wallSeconds,
		static_cast<unsigned long long>(serverTick), static_cast<unsigned int>(flightStatus), messageAgeSeconds,
		elapsedSeconds, authorityTimeScale, predictionSeconds, correctionFraction, smoothCorrection);
	WriteKinematicsState(kinematicsLogFile, sample);
	WriteKinematicsState(kinematicsLogFile, current);
	WriteKinematicsState(kinematicsLogFile, target);
	WriteKinematicsState(kinematicsLogFile, applied);
	WriteKinematicsVector(kinematicsLogFile, positionError);
	WriteKinematicsVector(kinematicsLogFile, velocityError);
	WriteKinematicsVector(kinematicsLogFile, orbiterAcceleration);
	fprintf(kinematicsLogFile, "\n");

	if (std::chrono::duration<double>(now - kinematicsLogLastFlush).count() >= 1.0) {
		fflush(kinematicsLogFile);
		kinematicsLogLastFlush = now;
	}
}

const char *ProjectApolloConnectorVessel::ComponentKey() const
{
	return "vessel.kinematics";
}

ProviderResult ProjectApolloConnectorVessel::Describe(ReplicationCatalogBuilder &catalog) const
{
	// Keep the complete motion sample in one unreliable group. A newer sample can
	// replace an older one without mixing position and orientation from different ticks.
	ReplicationSchemaBuilder schema;
	schema.AddString("project-apollo-kinematics-v2");
	schema.AddString("flight_status:uint2");
	schema.AddString("reference_position:float64x3");
	schema.AddString("reference_velocity:float64x3");
	schema.AddString("reference_acceleration:float64x3");
	schema.AddString("global_orientation_quaternion:float64x4");
	schema.AddString("local_angular_velocity:float64x3");
	schema.AddString("landed_orientation:float64x3-if-landed");
	schema.AddString("surface_coordinates:float64x3-if-landed");
	schema.AddString("landed_altitude:float64-if-landed");

	ReplicationGroupDescriptor state;
	state.key = KinematicsGroupKey;
	state.schemaId = schema.SchemaId();
	state.delivery = ReplicationDelivery::Unreliable;
	state.periodicIntervalMs = 50;
	state.replicateChanges = false;
	state.maximumPayloadBytes = static_cast<std::uint32_t>(1 + 23 * sizeof(double));
	catalog.AddGroup(state);
	return ProviderResult::Success;
}

ProviderResult ProjectApolloConnectorVessel::Capture(const char *groupKey, ReplicationWriter &writer, const CaptureContext &context)
{
	if (strcmp(groupKey, KinematicsGroupKey) != 0)
		return ProviderResult::Unsupported;

	// Extract flight mode from orbiter flight status
	const DWORD orbiterFlightStatus = GetFlightStatus();
	const std::uint8_t flightStatus = orbiterFlightStatus & DockedFlightStatusFlag ? DockedStatus :
		(orbiterFlightStatus & LandedFlightStatusFlag ? LandedStatus : FreeFlightStatus);

	// Capture all live motion from one Orbiter status snapshot.
	VESSELSTATUS2 status = {};
	status.version = 2;
	status.flag = 0;
	GetStatusEx(&status);
	nasspmp_kinematics::State state = CurrentKinematicState(status);

	// Calculate acceleration by difference to improve client-side prediction
	// The finite difference follows Orbiter's actually propagated velocity. This
	// includes gravity, thrust and other forces without duplicating its gravity model.
	if (hasAuthorityVelocitySample && context.simulationTick > authorityVelocitySampleTick) {
		const double elapsedSeconds = static_cast<double>(context.simulationTick - authorityVelocitySampleTick) / 1000.0;
		authorityAccelerationSample = (state.velocity - authorityVelocitySample) / elapsedSeconds;
	} else if (!hasAuthorityVelocitySample || context.simulationTick < authorityVelocitySampleTick) {
		authorityAccelerationSample = {};
	}
	if (!hasAuthorityVelocitySample || context.simulationTick != authorityVelocitySampleTick) {
		authorityVelocitySample = state.velocity;
		authorityVelocitySampleTick = context.simulationTick;
		hasAuthorityVelocitySample = true;
	}
	state.acceleration = authorityAccelerationSample;
	const double notAvailable = std::nan("");
	LogKinematics("capture", context.simulationTick, flightStatus, notAvailable, notAvailable, oapiGetTimeAcceleration(),
		notAvailable, notAvailable, -1, &state, NULL, NULL, NULL, NULL, NULL, NULL);

	// The free-flight fields are always present. Landed placement follows only
	// for landed samples, keeping the common high-rate payload compact.
	writer.WriteScalar(flightStatus, 2)
		.WriteScalar(state.position.x).WriteScalar(state.position.y).WriteScalar(state.position.z)
		.WriteScalar(state.velocity.x).WriteScalar(state.velocity.y).WriteScalar(state.velocity.z)
		.WriteScalar(state.acceleration.x).WriteScalar(state.acceleration.y).WriteScalar(state.acceleration.z)
		.WriteScalar(state.orientation.x).WriteScalar(state.orientation.y).WriteScalar(state.orientation.z).WriteScalar(state.orientation.w)
		.WriteScalar(state.angularVelocity.x).WriteScalar(state.angularVelocity.y).WriteScalar(state.angularVelocity.z);

	if (flightStatus == LandedStatus) {
		writer.WriteScalar(status.arot.x).WriteScalar(status.arot.y).WriteScalar(status.arot.z)
			.WriteScalar(status.surf_lng).WriteScalar(status.surf_lat).WriteScalar(status.surf_hdg)
			.WriteScalar(status.vrot.x);
	}
	return writer ? ProviderResult::Success : ProviderResult::BufferTooSmall;
}

ProviderResult ProjectApolloConnectorVessel::ReadKinematics(const ReplicationReader &reader, ReplicatedKinematics *target, const ApplyContext &context) const
{
	// Decode into a temporary object so malformed payloads cannot partially
	// change the last valid presentation state.
	ReplicatedKinematics decoded;
	reader.ReadScalar(decoded.flightStatus, 2)
		.ReadScalar(decoded.state.position.x).ReadScalar(decoded.state.position.y).ReadScalar(decoded.state.position.z)
		.ReadScalar(decoded.state.velocity.x).ReadScalar(decoded.state.velocity.y).ReadScalar(decoded.state.velocity.z)
		.ReadScalar(decoded.state.acceleration.x).ReadScalar(decoded.state.acceleration.y)
		.ReadScalar(decoded.state.acceleration.z)
		.ReadScalar(decoded.state.orientation.x).ReadScalar(decoded.state.orientation.y)
		.ReadScalar(decoded.state.orientation.z).ReadScalar(decoded.state.orientation.w)
		.ReadScalar(decoded.state.angularVelocity.x).ReadScalar(decoded.state.angularVelocity.y)
		.ReadScalar(decoded.state.angularVelocity.z);

	if (decoded.flightStatus == LandedStatus) {
		reader.ReadScalar(decoded.landedOrientation.x).ReadScalar(decoded.landedOrientation.y)
			.ReadScalar(decoded.landedOrientation.z).ReadScalar(decoded.surfaceLongitude)
			.ReadScalar(decoded.surfaceLatitude).ReadScalar(decoded.surfaceHeading)
			.ReadScalar(decoded.landedAltitude);
	}
	const bool payloadComplete = reader.Finish();
	if (!payloadComplete)
		return ProviderResult::Malformed;
	if (decoded.flightStatus > DockedStatus || !IsFinite(decoded.state.position) || !IsFinite(decoded.state.velocity) ||
		!IsFinite(decoded.state.acceleration) || !IsFinite(decoded.state.orientation) ||
		!IsFinite(decoded.state.angularVelocity))
		return ProviderResult::Rejected;
	const double quaternionLength = std::sqrt(decoded.state.orientation.x * decoded.state.orientation.x +
		decoded.state.orientation.y * decoded.state.orientation.y + decoded.state.orientation.z * decoded.state.orientation.z +
		decoded.state.orientation.w * decoded.state.orientation.w);
	if (quaternionLength < 0.5 || quaternionLength > 1.5)
		return ProviderResult::Rejected;
	if (decoded.flightStatus == LandedStatus && (!IsFinite(decoded.landedOrientation) ||
		!std::isfinite(decoded.surfaceLongitude) || !std::isfinite(decoded.surfaceLatitude) ||
		!std::isfinite(decoded.surfaceHeading) || !std::isfinite(decoded.landedAltitude)))
		return ProviderResult::Rejected;

	decoded.state.orientation = nasspmp_kinematics::Normalize(decoded.state.orientation);
	decoded.serverTick = context.serverTick;
	if (target)
		*target = decoded;
	return ProviderResult::Success;
}

ProviderResult ProjectApolloConnectorVessel::Validate(const char *groupKey, const ReplicationReader &reader, const ApplyContext &context) const
{
	if (strcmp(groupKey, KinematicsGroupKey) != 0 || context.purpose == ApplyPurpose::RemoteInput)
		return ProviderResult::Unsupported;
	return ReadKinematics(reader, NULL, context);
}

void ProjectApolloConnectorVessel::Apply(const char *, const ReplicationReader &reader, const ApplyContext &context)
{
	ReplicatedKinematics incoming;
	if (ReadKinematics(reader, &incoming, context) != ProviderResult::Success)
		return;

	const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	incoming.receivedAt = now;
	incoming.messageAgeSeconds = static_cast<double>(context.estimatedMessageAgeMs) / 1000.0;

	// Server ticks measure simulation time, while arrivals use a monotonic real clock.
	// Their ratio estimates the host time acceleration without synchronized clocks.
	if (hasReplicatedKinematics && context.purpose != ApplyPurpose::Baseline && incoming.serverTick > replicatedKinematics.serverTick) {
		incoming.authorityTimeScale = replicatedKinematics.authorityTimeScale;
		const double elapsedSeconds = std::chrono::duration<double>(now - replicatedKinematics.receivedAt).count();
		const double serverSeconds = static_cast<double>(incoming.serverTick - replicatedKinematics.serverTick) / 1000.0;
		if (elapsedSeconds > 0.001) {
			const double measuredTimeScale = serverSeconds / elapsedSeconds;
			if (std::isfinite(measuredTimeScale) && measuredTimeScale >= 0.0 && measuredTimeScale <= 1000.0)
				incoming.authorityTimeScale = replicatedKinematics.authorityTimeScale * 0.75 + measuredTimeScale * 0.25;
		}
	}

	correctionActive = false;
	bool receivedStateCompared = false;
	if (!hasReplicatedKinematics || replicatedKinematics.flightStatus != LandedStatus || context.purpose == ApplyPurpose::Baseline)
		landedStateApplied = false;

	if (hasReplicatedKinematics && context.purpose != ApplyPurpose::Baseline && incoming.flightStatus != LandedStatus) {
		// Execute prediction, since received vessel state is already as old as the round-trip-time of the message
		const double predictionAgeSeconds = (std::min)(MaximumPredictionRealAgeSeconds, incoming.messageAgeSeconds);
		const double predictionSeconds = predictionAgeSeconds * incoming.authorityTimeScale;
		const nasspmp_kinematics::State target = nasspmp_kinematics::Extrapolate(incoming.state, predictionSeconds);
		const nasspmp_kinematics::State current = CurrentKinematicState();

		// Since position, velocity and orientation might still be deviated when receiving an update, 
		// we perform correction smoothly if the difference is small enough.
		// The threshold is scaled with the respective derivative
		positionCorrection = current.position - target.position;
		velocityCorrection = current.velocity - target.velocity;
		const double positionError = length(positionCorrection);
		const double angleError = nasspmp_kinematics::AngularDistance(current.orientation, target.orientation);
		const double timingWindow = KinematicTimingErrorSeconds * incoming.authorityTimeScale;
		const double smoothPositionLimit = (std::max)(KinematicMinimumSmoothPositionMeters,
			length(target.velocity) * timingWindow);
		const double smoothAngleLimit = (std::max)(KinematicMinimumSmoothAngleRadians,
			length(target.angularVelocity) * timingWindow);

		// A fixed distance threshold would mistake millisecond timing errors at orbital
		// velocity for a teleport. Scale the soft-correction window with current motion.
		const bool smoothCorrection = positionError <= smoothPositionLimit && angleError <= smoothAngleLimit;
		if (smoothCorrection) {
			orientationCorrection = nasspmp_kinematics::Multiply(
				nasspmp_kinematics::Inverse(target.orientation), current.orientation);
			correctionStart = now;
			correctionActive = true;
		}
		LogKinematics("receive", incoming.serverTick, incoming.flightStatus, incoming.messageAgeSeconds, 0.0,
			incoming.authorityTimeScale, predictionSeconds, 0.0, smoothCorrection ? 1 : 0,
			&incoming.state, &current, &target, NULL, &positionCorrection, &velocityCorrection, NULL);
		receivedStateCompared = true;
	}
	if (!receivedStateCompared) {
		const double notAvailable = std::nan("");
		LogKinematics(context.purpose == ApplyPurpose::Baseline ? "baseline" : "receive", incoming.serverTick,
			incoming.flightStatus, incoming.messageAgeSeconds, 0.0, incoming.authorityTimeScale,
			notAvailable, notAvailable, -1, &incoming.state, NULL, NULL, NULL, NULL, NULL, NULL);
	}

	replicatedKinematics = incoming;
	hasReplicatedKinematics = true;
	kinematicsUpdateLogPending = incoming.flightStatus != LandedStatus;
}

void ProjectApolloConnectorVessel::OnRoleChanged(ReplicationRole role)
{
	OpenKinematicsLog(role);
	if (role != ReplicationRole::Authority)
		hasAuthorityVelocitySample = false;
	if (role != ReplicationRole::Replica) {
		hasReplicatedKinematics = false;
		correctionActive = false;
		landedStateApplied = false;
		hasReplicaIntegrationSample = false;
		hasReplicaOrbiterAcceleration = false;
		kinematicsUpdateLogPending = false;
	}
}

void ProjectApolloConnectorVessel::AnnounceStageEvent(StageEventKind kind, int targetStage, bool spawnsEntity, const VESSELSTATUS &spawnState)
{
	if (ReplicationHubInstance.GetRole() != ReplicationRole::Authority)
		return;

	MATRIX3 rotation;
	GetRotationMatrix(rotation);
	stageEvent.kind = kind;
	stageEvent.targetStage = static_cast<std::uint8_t>(targetStage);
	stageEvent.spawnsEntity = spawnsEntity;
	stageEvent.position = spawnState.rpos;
	stageEvent.velocity = spawnState.rvel;
	stageEvent.orientation = nasspmp_kinematics::FromRotationMatrix(rotation);
	++stageEventRevision;
}

void ProjectApolloConnectorVessel::BeginReplicatedStageEvent(const StageTransition &event)
{
	stageEvent = event;
	applyingReplicatedStageEvent = true;
}

void ProjectApolloConnectorVessel::EndReplicatedStageEvent()
{
	applyingReplicatedStageEvent = false;
}

void ProjectApolloConnectorVessel::ApplyReplicatedStageSpawnState(VESSELSTATUS &spawnState) const
{
	const MATRIX3 rotation = nasspmp_kinematics::ToRotationMatrix(stageEvent.orientation);
	spawnState.rpos = stageEvent.position;
	spawnState.rvel = stageEvent.velocity;
	spawnState.arot.x = std::atan2(rotation.m23, rotation.m33);
	spawnState.arot.y = -std::asin((std::max)(-1.0, (std::min)(1.0, rotation.m13)));
	spawnState.arot.z = std::atan2(rotation.m12, rotation.m11);
}

nasspmp_kinematics::State ProjectApolloConnectorVessel::CurrentKinematicState() const
{
	VESSELSTATUS2 status = {};
	status.version = 2;
	status.flag = 0;
	GetStatusEx(&status);
	return CurrentKinematicState(status);
}

nasspmp_kinematics::State ProjectApolloConnectorVessel::CurrentKinematicState(const VESSELSTATUS2 &status) const
{
	// Orbiter reports translation relative to the current gravity reference and
	// attitude in its global frame; the wire sample deliberately uses the same split.
	nasspmp_kinematics::State state;
	state.position = status.rpos;
	state.velocity = status.rvel;
	VECTOR3 angularVelocity;
	MATRIX3 rotation;
	GetAngularVel(angularVelocity);
	state.angularVelocity = angularVelocity;
	GetRotationMatrix(rotation);
	state.orientation = nasspmp_kinematics::FromRotationMatrix(rotation);
	return state;
}

void ProjectApolloConnectorVessel::ApplyFreeFlightState(const nasspmp_kinematics::State &state)
{
	const OBJHANDLE reference = GetGravityRef();
	if (!reference)
		return;

	// Preserve all non-kinematic VESSELSTATUS2 fields while replacing the motion
	// relative to the locally resolved gravity reference.
	VESSELSTATUS2 status = {};
	status.version = 2;
	status.flag = 0;
	GetStatusEx(&status);
	status.version = 2;
	status.flag = 0;
	status.rbody = reference;
	status.status = FreeFlightStatus;
	status.rpos = state.position;
	status.rvel = state.velocity;
	status.vrot = state.angularVelocity;
	DefSetStateEx(&status);

	// DefSetStateEx accepts orientation only as Euler angles in status.arot.
	// Apply the predicted quaternion as an exact matrix after resetting translation.
	SetRotationMatrix(nasspmp_kinematics::ToRotationMatrix(state.orientation));
}

void ProjectApolloConnectorVessel::ApplyLandedState(const ReplicatedKinematics &state)
{
	// Landed samples use Orbiter's surface-coordinate representation instead of
	// trying to extrapolate an inertial state constrained to the ground.
	VESSELSTATUS2 status = {};
	status.version = 2;
	status.flag = 0;
	GetStatusEx(&status);
	status.version = 2;
	status.flag = 0;
	status.rbody = GetGravityRef();
	status.status = LandedStatus;
	status.arot = state.landedOrientation;
	status.surf_lng = state.surfaceLongitude;
	status.surf_lat = state.surfaceLatitude;
	status.surf_hdg = state.surfaceHeading;
	status.vrot = _V(state.landedAltitude, 0.0, 0.0);
	DefSetStateEx(&status);
}

void ProjectApolloConnectorVessel::UpdateReplicatedKinematics(double simdt)
{
	if (ReplicationHubInstance.GetRole() != ReplicationRole::Replica || !hasReplicatedKinematics)
		return;
	const DWORD localFlightStatus = GetFlightStatus();
	if (localFlightStatus & DockedFlightStatusFlag) {
		hasReplicaIntegrationSample = false;
		kinematicsUpdateLogPending = false;
		return;
	}

	if (replicatedKinematics.flightStatus == LandedStatus) {
		hasReplicaIntegrationSample = false;
		hasReplicaOrbiterAcceleration = false;
		kinematicsUpdateLogPending = false;
		if (!landedStateApplied) {
			ApplyLandedState(replicatedKinematics);
			landedStateApplied = true;
		}
		return;
	}

	const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

	// Wall-clock time since message was received
	const double elapsedSeconds = (std::max)(0.0, std::chrono::duration<double>(now - replicatedKinematics.receivedAt).count());
	
	// Estimated wall-clock timestamp of the transmitted state
	// Limit real packet age, not simulation time. At 50x time acceleration a
	// 0.5-second real-time window therefore permits up to 25 simulated seconds.
	const double predictionAgeSeconds = (std::min)(MaximumPredictionRealAgeSeconds,
		replicatedKinematics.messageAgeSeconds + elapsedSeconds);

	// With the elapsed wall-clock time since the transmitted kinematic state we estimate the
	// elapsed sim-time by scaling with the measured timeAcceleration
	const double predictionSeconds = predictionAgeSeconds * replicatedKinematics.authorityTimeScale;
	
	// Local delta time of simulation
	const double integrationStepSeconds = (std::max)(0.0, simdt);

	// Rendering follows the upcoming Orbiter integration step. Predict its end
	// before calculating the matching state which Orbiter needs at its beginning.
	const double presentationPredictionSeconds = predictionSeconds + integrationStepSeconds;

	// Extrapolate received state with 2nd order kinematics to compensate for the elapsed time
	// since the host measured his state
	nasspmp_kinematics::State target = nasspmp_kinematics::Extrapolate(
		replicatedKinematics.state, presentationPredictionSeconds);
	
	const nasspmp_kinematics::State current = CurrentKinematicState();
	double correctionFraction = std::nan("");
	const int smoothCorrection = correctionActive ? 1 : 0;

	// measure the acceleration that the orbiter-internal time-step uses for kinematic propagation
	// This is used to compensate for orbiters calculations since they are not deactivatable
	if (hasReplicaIntegrationSample && replicaIntegrationStepSeconds > 0.0) {
		const VECTOR3 acceleration =
			(current.velocity - replicaIntegrationStartVelocity) / replicaIntegrationStepSeconds;
		if (IsFinite(acceleration)) {
			replicaOrbiterAcceleration = acceleration;
			hasReplicaOrbiterAcceleration = true;
		}
	}
	hasReplicaIntegrationSample = false;

	if (correctionActive) {
		// A cubic Hermite correction brings both position and velocity errors to zero.
		// Matching both derivatives prevents a new authority sample from changing the
		// vessel velocity abruptly while the positional correction remains smooth.
		const double correctionSeconds = std::chrono::duration<double>(now - correctionStart).count();
		const double fraction = (std::min)(1.0, (std::max)(0.0, correctionSeconds / KinematicCorrectionDurationSeconds));
		correctionFraction = fraction;
		const double fractionSquared = fraction * fraction;
		const double fractionCubed = fractionSquared * fraction;
		const double positionErrorScale = 2.0 * fractionCubed - 3.0 * fractionSquared + 1.0;
		const double velocityErrorScale = fractionCubed - 2.0 * fractionSquared + fraction;
		const double positionErrorDerivative = (6.0 * fractionSquared - 6.0 * fraction) / KinematicCorrectionDurationSeconds;
		const double velocityErrorDerivative = 3.0 * fractionSquared - 4.0 * fraction + 1.0;
		const nasspmp_kinematics::Quaternion decayingOrientationCorrection =
			nasspmp_kinematics::Slerp(orientationCorrection, nasspmp_kinematics::Quaternion(), fraction);
		target.position = target.position + positionCorrection * positionErrorScale +
			velocityCorrection * (KinematicCorrectionDurationSeconds * velocityErrorScale);
		target.velocity = target.velocity + positionCorrection * positionErrorDerivative +
			velocityCorrection * velocityErrorDerivative;
		target.orientation = nasspmp_kinematics::Normalize(
			nasspmp_kinematics::Multiply(target.orientation, decayingOrientationCorrection));
		if (fraction >= 1.0)
			correctionActive = false;
	}

	// Feed the state into Orbiter before its physics update, while the vessel and
	// its gravity reference still use the same state buffer. Backstep the motion
	// Orbiter will integrate during this frame using its measured acceleration.
	nasspmp_kinematics::State integrationStart = target;
	if (integrationStepSeconds > 0.0) {
		const VECTOR3 localAcceleration =
			hasReplicaOrbiterAcceleration ? replicaOrbiterAcceleration : _V(0.0, 0.0, 0.0);
		integrationStart.position = target.position - target.velocity * integrationStepSeconds +
			localAcceleration * (0.5 * integrationStepSeconds * integrationStepSeconds);
		integrationStart.velocity = target.velocity - localAcceleration * integrationStepSeconds;
		integrationStart.orientation = nasspmp_kinematics::IntegrateOrientation(
			target.orientation, target.angularVelocity, -integrationStepSeconds);
	}
	ApplyFreeFlightState(integrationStart);

	if (kinematicsUpdateLogPending) {
		LogKinematics("update", replicatedKinematics.serverTick, replicatedKinematics.flightStatus,
			replicatedKinematics.messageAgeSeconds, elapsedSeconds, replicatedKinematics.authorityTimeScale,
			presentationPredictionSeconds, correctionFraction, smoothCorrection, &replicatedKinematics.state, &current,
			&target, &integrationStart, smoothCorrection ? &positionCorrection : NULL,
			smoothCorrection ? &velocityCorrection : NULL,
			hasReplicaOrbiterAcceleration ? &replicaOrbiterAcceleration : NULL);
		kinematicsUpdateLogPending = false;
	}
	replicaIntegrationStartVelocity = integrationStart.velocity;
	replicaIntegrationStepSeconds = integrationStepSeconds;
	hasReplicaIntegrationSample = integrationStepSeconds > 0.0;
}

bool ProjectApolloConnectorVessel::ValidateVessel()

{
	return (ValidationValue == PACV_N_VALIDATION);
}

Connector *ProjectApolloConnectorVessel::GetConnector(int port, ConnectorType t)

{
	int i;
	for (i = 0; i < PACV_N_CONNECTORS; i++)
	{
		if (ConnectorList[i].c && (ConnectorList[i].port == port) && (ConnectorList[i].c->GetType() == t)) {
			return ConnectorList[i].c;
		}
			
	}
	return NULL;
}

bool ProjectApolloConnectorVessel::RegisterConnector(int port, Connector *c)

{
	int i;
	for (i = 0; i < PACV_N_CONNECTORS; i++)
	{
		if (!ConnectorList[i].c)
		{
			ConnectorList[i].c = c;
			ConnectorList[i].port = port;
			return true;
		}
	}

	return false;
}

void ProjectApolloConnectorVessel::UndockConnectors(int port)

{
	int i;
	for (i = 0; i < PACV_N_CONNECTORS; i++)
	{
		if (ConnectorList[i].c && (ConnectorList[i].port == port))
		{
			ConnectorList[i].c->Disconnect();
		}
	}
}

void ProjectApolloConnectorVessel::DockConnectors(int port)

{
	DOCKHANDLE d = GetDockHandle(port);

	if (!d)
		return;

	OBJHANDLE connected = GetDockStatus(d);

	if (!connected)
		return;

	VESSEL *dockedWith = oapiGetVesselInterface(connected);

	if (!dockedWith)
		return;

	int i;
	for (i = 0; i < PACV_N_CONNECTORS; i++)
	{
		if (ConnectorList[i].c && (ConnectorList[i].port == port))
		{
			///
			/// \todo Find correct docking port on other vessel, don't assume we're
			/// always docked to port zero.
			///
			Connector *ours = ConnectorList[i].c;

			for (int j = 0;j < 2;j++)
			{
				Connector *theirs = GetVesselConnector(dockedWith, j, ours->GetType());

				if (theirs)
				{
					ours->ConnectTo(theirs);
					break;
				}
			}
		}
	}
}


Connector *GetVesselConnector(VESSEL *v, int port, ConnectorType t)

{
	//
	// Check for null pointer, just in case!
	//
	if (!v)
		return NULL;

	char *classname = v->GetClassName();

	//
	// If this isn't a project Apollo vessel, assume it's not the
	// correct type.
	//
	if (strnicmp(classname, "ProjectApollo", 13))
		return NULL;

	//
	// Cast it to our vessel on the assumption that it is.
	//
	ProjectApolloConnectorVessel *pacv = static_cast<ProjectApolloConnectorVessel *> (v);

	//
	// Validate it to check that this is probably the right kind of vessel.
	//
	if (!pacv->ValidateVessel()) {
		return NULL;
	}

	//
	// Finally, try to get the connector.
	//
	return pacv->GetConnector(port, t);
}
