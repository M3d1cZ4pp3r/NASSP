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

// Offset below which the filter is treated as settled and switched off.
const double KinematicSettledOffsetMeters = 0.001;

const double KinematicMinimumSmoothPositionMeters = 0.5;
const double KinematicMinimumSmoothAngleRadians = 0.5 * RAD;
const double KinematicTimingErrorSeconds = 0.05;

// Longest span a sample may be extrapolated over, in simulation time. Long enough that a clock
// offset of a few seconds still extrapolates smoothly instead of freezing the replica between samples. 
const double MaximumPredictionSeconds = 5.0;

// Shortest interval the authority acceleration may be differenced over, and the largest
// magnitude the result may have. There were rare occasions where this spiked insanely high, so prevent it.
const double MinimumAccelerationIntervalSeconds = 0.005;
const double MaximumAccelerationMetersPerSecondSquared = 200.0;

// Frames a sample can be expressed in. Both are centred on the gravity reference and differ
// only in whether the frame turns with the body.
const std::uint8_t InertialFrame = 0;
const std::uint8_t BodyFixedFrame = 1;

// Altitude below which the body-fixed frame is used. Close to the surface the rotation of the
// body dominates the inertial motion, and the turning frame removes it from the prediction
// entirely.
//
// Held at zero for now, which selects the inertial frame everywhere.
const double BodyFixedFrameAltitudeMeters = 0.0;

const std::uint8_t FreeFlightStatus = 0;
const std::uint8_t LandedStatus = 1;
const std::uint8_t DockedStatus = 2;

const DWORD LandedFlightStatusFlag = 1;
const DWORD DockedFlightStatusFlag = 2;

// TODO: Move a lot of all this to kinematics library?
// It starts polluting this file

bool IsFinite(const VECTOR3 &value)
{
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// Identity of a celestial body that both machines derive alike. The name is hashed rather
// than an index transmitted, because the order in which bodies are loaded might not be guaranteed
// to match between installations.
std::uint64_t BodyIdentity(OBJHANDLE body)
{
	char name[256] = {};
	oapiGetObjectName(body, name, sizeof(name));

	ReplicationSchemaBuilder identity;
	identity.AddString(name);
	return identity.SchemaId();
}

// Finds the local celestial body carrying an identity, or NULL when this installation has no
// such body.
OBJHANDLE ResolveBody(std::uint64_t identity)
{
	const DWORD count = oapiGetGbodyCount();
	for (DWORD index = 0; index < count; ++index) {
		const OBJHANDLE body = oapiGetGbodyByIndex(static_cast<int>(index));
		if (body && BodyIdentity(body) == identity)
			return body;
	}
	return NULL;
}

// Angular velocity of a celestial body in the global frame.
VECTOR3 BodyAngularVelocity(OBJHANDLE body, const MATRIX3 &rotation)
{
	const double period = oapiGetPlanetPeriod(body);
	if (period == 0.0)
		return _V(0.0, 0.0, 0.0);

	// Orbiter turns every celestial body about its own local y axis.
	return mul(rotation, _V(0.0, PI2 / period, 0.0));
}

// Orientation a body will have after the given span. Needed because a sample is expressed at
// the render date, which is one integration step ahead of the state Orbiter currently holds.
MATRIX3 AdvancedRotationMatrix(OBJHANDLE body, double seconds)
{
	MATRIX3 rotation;
	oapiGetRotationMatrix(body, &rotation);
	const double period = oapiGetPlanetPeriod(body);
	if (period == 0.0 || seconds == 0.0)
		return rotation;

	// The body turns about its own y axis, so the advance composes from the right.
	const double angle = PI2 * seconds / period;
	const double cosine = cos(angle);
	const double sine = sin(angle);
	const MATRIX3 advance = _M(cosine, 0.0, sine, 0.0, 1.0, 0.0, -sine, 0.0, cosine);
	return mul(rotation, advance);
}

// Expresses an inertial body-relative state in the frame that turns with the body. A vessel
// standing on the surface has exactly zero velocity and zero acceleration there, which is what
// takes the rotation of the body out of the prediction.
void ToBodyFixed(OBJHANDLE body, VECTOR3 &position, VECTOR3 &velocity)
{
	MATRIX3 rotation;
	oapiGetRotationMatrix(body, &rotation);

	const VECTOR3 turning = velocity - crossp(BodyAngularVelocity(body, rotation), position);
	position = tmul(rotation, position);
	velocity = tmul(rotation, turning);
}

// Reverses ToBodyFixed at the orientation the body has after the given span.
void FromBodyFixed(OBJHANDLE body, double advanceSeconds, VECTOR3 &position, VECTOR3 &velocity)
{
	const MATRIX3 rotation = AdvancedRotationMatrix(body, advanceSeconds);
	const VECTOR3 angular = BodyAngularVelocity(body, rotation);

	position = mul(rotation, position);
	velocity = mul(rotation, velocity) + crossp(angular, position);
}

// Returns a predicted sample to the inertial frame the vessel state uses, at the body
// orientation belonging to the given span ahead of the current date. A sample that was never
// body-fixed passes through untouched.
void ToInertialFrame(OBJHANDLE body, std::uint8_t frame, double advanceSeconds,
	nasspmp_kinematics::State &state)
{
	if (frame != BodyFixedFrame || !body)
		return;
	FromBodyFixed(body, advanceSeconds, state.position, state.velocity);
}

// Natural frequency of the visual offset filter, chosen so a critically damped response has
// decayed to about two percent after one correction duration.
double OffsetFilterFrequency()
{
	return 6.0 / KinematicCorrectionDurationSeconds;
}

// Remaining share of an exponentially decaying quantity after one step.
double OffsetDecayFactor(double stepSeconds)
{
	return exp(-OffsetFilterFrequency() * stepSeconds);
}

// Decays an offset and its rate towards zero, this is done analytically.
// This smoothes the position offset together with the speed offset
void DecayVisualOffset(VECTOR3 &offset, VECTOR3 &offsetRate, double stepSeconds)
{
	if (stepSeconds <= 0.0)
		return;

	const double frequency = OffsetFilterFrequency();
	const double decay = OffsetDecayFactor(stepSeconds);
	const VECTOR3 slope = offsetRate + offset * frequency;
	const VECTOR3 decayed = (offset + slope * stepSeconds) * decay;

	offsetRate = (offsetRate - slope * (frequency * stepSeconds)) * decay;
	offset = decayed;
}

// Bounds the span an authority sample may be extrapolated over, in either direction. A negative
// span means the local clock runs ahead of the authority, so the sample is carried back to the
// local date rather than shown at its own.
double ClampPredictionSeconds(double seconds)
{
	if (!std::isfinite(seconds))
		return 0.0;
	return (std::max)(-MaximumPredictionSeconds, (std::min)(MaximumPredictionSeconds, seconds));
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
#ifdef LOG_KINEMATICS
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
	fprintf(kinematicsLogFile, "event,wall_seconds,sample_session_time,flight_status,render_session_time,step_seconds,prediction_seconds,correction_fraction,smooth_correction");
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
#endif
}

void ProjectApolloConnectorVessel::LogKinematics(const char *event, double sampleSessionTime,
	std::uint8_t flightStatus, double renderSessionTime, double stepSeconds,
	double predictionSeconds, double correctionFraction, int smoothCorrection,
	const nasspmp_kinematics::State *sample, const nasspmp_kinematics::State *current,
	const nasspmp_kinematics::State *target, const nasspmp_kinematics::State *applied,
	const VECTOR3 *positionError, const VECTOR3 *velocityError, const VECTOR3 *orbiterAcceleration)
{
#ifdef LOG_KINEMATICS
	if (!kinematicsLogFile)
		return;

	const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	const double wallSeconds = std::chrono::duration<double>(now - kinematicsLogStarted).count();
	fprintf(kinematicsLogFile, "%s,%.9f,%.9f,%u,%.9f,%.9f,%.9f,%.9f,%d", event, wallSeconds,
		sampleSessionTime, static_cast<unsigned int>(flightStatus), renderSessionTime, stepSeconds,
		predictionSeconds, correctionFraction, smoothCorrection);
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
#endif
}

const char *ProjectApolloConnectorVessel::ComponentKey() const
{
	return "vessel.kinematics";
}

ProviderResult ProjectApolloConnectorVessel::Describe(ReplicationCatalogBuilder &catalog) const
{
	// Keep the complete motion sample in one unreliable group.
	ReplicationSchemaBuilder schema;
	schema.AddString("project-apollo-kinematics-v4");
	schema.AddString("flight_status:uint2");
	schema.AddString("reference_frame:uint1");
	schema.AddString("reference_body:uint64");
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
	state.maximumPayloadBytes = static_cast<std::uint32_t>(1 + 8 + 23 * sizeof(double));
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

	// Near the surface the sample is expressed in the frame that turns with the reference body,
	// which removes the rotation of the body from the prediction. The receiver reconstructs that
	// rotation from its own clock.
	const std::uint8_t frame = status.rbody && GetAltitude() < BodyFixedFrameAltitudeMeters ?
		BodyFixedFrame : InertialFrame;
	if (frame == BodyFixedFrame)
		ToBodyFixed(status.rbody, state.position, state.velocity);

	// Position and velocity only mean something relative to their body, so the receiver is told
	// which one instead of resolving its own and hoping both sides agree.
	const std::uint64_t referenceBody = status.rbody ? BodyIdentity(status.rbody) : 0;

	// A difference taken across a change of frame or body compares two unrelated quantities.
	// Across a change of sphere of influence the velocity jumps by the orbital speed of the body.
	const bool referenceChanged = frame != authorityVelocitySampleFrame ||
		referenceBody != authorityVelocitySampleBody;
	if (hasAuthorityVelocitySample && referenceChanged)
		hasAuthorityVelocitySample = false;

	// Calculate acceleration by difference to improve client-side prediction
	// The finite difference follows Orbiter's actually propagated velocity. This
	// includes gravity, thrust and other forces.
	if (hasAuthorityVelocitySample && context.hasSessionTime &&
		context.captureSessionTime > authorityVelocitySampleTime + MinimumAccelerationIntervalSeconds) {
		const double elapsedSeconds = context.captureSessionTime - authorityVelocitySampleTime;
		const VECTOR3 difference = (state.velocity - authorityVelocitySample) / elapsedSeconds;

		// A capture stamp that does not match its own state turns the difference into a value
		// no vehicle can produce. Keeping the previous sample rides out that single frame.
		if (length(difference) <= MaximumAccelerationMetersPerSecondSquared)
			authorityAccelerationSample = difference;
	} else if (!hasAuthorityVelocitySample || !context.hasSessionTime ||
		context.captureSessionTime < authorityVelocitySampleTime) {
		authorityAccelerationSample = {};
	}
	if (context.hasSessionTime &&
		(!hasAuthorityVelocitySample || context.captureSessionTime != authorityVelocitySampleTime)) {
		authorityVelocitySample = state.velocity;
		authorityVelocitySampleTime = context.captureSessionTime;
		authorityVelocitySampleFrame = frame;
		authorityVelocitySampleBody = referenceBody;
		hasAuthorityVelocitySample = true;
	}

	state.acceleration = authorityAccelerationSample;

	const double notAvailable = std::nan("");
	LogKinematics("capture", context.captureSessionTime, flightStatus, notAvailable, oapiGetSimStep(),
		notAvailable, notAvailable, -1, &state, NULL, NULL, NULL, NULL, NULL, NULL);

	// The free-flight fields are always present. Landed placement follows only
	// for landed samples, keeping the common high-rate payload compact.
	writer.WriteScalar(flightStatus, 2)
		.WriteScalar(frame, 1)
		.WriteScalar(referenceBody)
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
	// Decode into a temporary object
	ReplicatedKinematics decoded;
	std::uint64_t referenceBody = 0;
	reader.ReadScalar(decoded.flightStatus, 2)
		.ReadScalar(decoded.frame, 1)
		.ReadScalar(referenceBody)
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
	
	// Range checks
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

	// A body this installation does not know makes every coordinate in the sample meaningless,
	// which is worse than dropping it.
	decoded.referenceBody = ResolveBody(referenceBody);
	if (!decoded.referenceBody)
		return ProviderResult::Rejected;

	decoded.state.orientation = nasspmp_kinematics::Normalize(decoded.state.orientation);
	decoded.sampleSessionTime = context.sampleSessionTime;
	decoded.hasSessionTime = context.hasSessionTime;

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

	double liveStateSessionTime = 0.0;
	const bool hasLiveStateTime = TryGetSessionTime(oapiGetSimMJD(), liveStateSessionTime);

	correctionActive = false;
	bool receivedStateCompared = false;
	if (!hasReplicatedKinematics || replicatedKinematics.flightStatus != LandedStatus || context.purpose == ApplyPurpose::Baseline)
		landedStateApplied = false;

	if (hasReplicatedKinematics && hasLiveStateTime && incoming.hasSessionTime &&
		context.purpose != ApplyPurpose::Baseline && incoming.flightStatus != LandedStatus) {

		const double predictionSeconds = ClampPredictionSeconds(liveStateSessionTime - incoming.sampleSessionTime);

		// Extrapolate from the sampled MJD on the authority to our MJD now. This preserves correlation between
		// Bodies and vessels, and is generally a useful absolute reference
		nasspmp_kinematics::State target = nasspmp_kinematics::Extrapolate(incoming.state, predictionSeconds);

		// The comparison is against the live state, so the body orientation of this instant applies.
		ToInertialFrame(incoming.referenceBody, incoming.frame, 0.0, target);

		const nasspmp_kinematics::State current = CurrentKinematicState(incoming.referenceBody);

		// Since position, velocity and orientation might still be deviated when receiving an update, 
		// we perform correction smoothly if the difference is small enough.
		// The threshold is scaled with the respective derivative
		positionCorrection = current.position - target.position;
		velocityCorrection = current.velocity - target.velocity;
		const double positionError = length(positionCorrection);
		const double angleError = nasspmp_kinematics::AngularDistance(current.orientation, target.orientation);

		// A fixed distance threshold would mistake millisecond timing errors at orbital
		// velocity for a teleport. Scale the soft-correction window with current motion.
		const double timingWindow = KinematicTimingErrorSeconds * oapiGetTimeAcceleration();
		const double smoothPositionLimit = (std::max)(KinematicMinimumSmoothPositionMeters,
			length(target.velocity) * timingWindow);
		const double smoothAngleLimit = (std::max)(KinematicMinimumSmoothAngleRadians,
			length(target.angularVelocity) * timingWindow);

		const bool smoothCorrection = positionError <= smoothPositionLimit && angleError <= smoothAngleLimit;

		// Inside the window the difference is assumed to be a timing artefact, so it is absorbed into the
		// visual offset and decayed away. 
		if (smoothCorrection) {
			orientationCorrection = nasspmp_kinematics::Multiply(
				nasspmp_kinematics::Inverse(target.orientation), current.orientation);
			correctionActive = true;
		}

		// Logged before the offset is discarded below, because a rejected difference is the
		// one worth inspecting afterwards.
		LogKinematics("receive", incoming.sampleSessionTime, incoming.flightStatus, liveStateSessionTime,
			oapiGetSimStep(), predictionSeconds, 0.0, smoothCorrection ? 1 : 0,
			&incoming.state, &current, &target, NULL, &positionCorrection, &velocityCorrection, NULL);

		// Outside the window we just snap
		if (!smoothCorrection) {
			positionCorrection = _V(0.0, 0.0, 0.0);
			velocityCorrection = _V(0.0, 0.0, 0.0);
			orientationCorrection = nasspmp_kinematics::Quaternion();
		}
		receivedStateCompared = true;
	}
	if (!receivedStateCompared) {
		const double notAvailable = std::nan("");
		LogKinematics(context.purpose == ApplyPurpose::Baseline ? "baseline" : "receive", incoming.sampleSessionTime,
			incoming.flightStatus, hasLiveStateTime ? liveStateSessionTime : notAvailable,
			oapiGetSimStep(), notAvailable, notAvailable, -1,
			&incoming.state, NULL, NULL, NULL, NULL, NULL, NULL);
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

bool ProjectApolloConnectorVessel::TryGetSessionTime(double mjd, double &sessionTime) const
{
	const ReplicationTimeBase &timeBase = ReplicationHubInstance.GetTimeBase();
	if (!timeBase.valid)
		return false;

	sessionTime = timeBase.SessionTime(mjd);
	return true;
}

nasspmp_kinematics::State ProjectApolloConnectorVessel::CurrentKinematicState(OBJHANDLE reference) const
{
	VESSELSTATUS2 status = {};
	status.version = 2;
	status.flag = 0;
	GetStatusEx(&status);
	nasspmp_kinematics::State state = CurrentKinematicState(status);

	// Measured against the body a received sample names, where Orbiter would otherwise report
	// against whichever body it currently considers the gravity reference.
	if (reference && reference != status.rbody) {
		GetRelativePos(reference, state.position);
		GetRelativeVel(reference, state.velocity);
	}
	return state;
}

nasspmp_kinematics::State ProjectApolloConnectorVessel::CurrentKinematicState(const VESSELSTATUS2 &status) const
{
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

void ProjectApolloConnectorVessel::ApplyFreeFlightState(const nasspmp_kinematics::State &state, OBJHANDLE reference)
{
	if (!reference)
		return;

	// Preserve all non-kinematic VESSELSTATUS2 fields while replacing the motion
	// relative to the body the authority measured it against.
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
	// Apply the predicted quaternion as a matrix.
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
	status.rbody = state.referenceBody;
	status.status = LandedStatus;
	status.arot = state.landedOrientation;
	status.surf_lng = state.surfaceLongitude;
	status.surf_lat = state.surfaceLatitude;
	status.surf_hdg = state.surfaceHeading;
	status.vrot = _V(state.landedAltitude, 0.0, 0.0);
	DefSetStateEx(&status);
}

void ProjectApolloConnectorVessel::UpdateReplicatedKinematics(double simdt, double mjd)
{
	if (ReplicationHubInstance.GetRole() != ReplicationRole::Replica || !hasReplicatedKinematics)
		return;

	const DWORD localFlightStatus = GetFlightStatus();
	// Detect replica-side docking for now, later this should also be only authorized by authority
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

	// Local delta time of simulation, can it ever get negative?
	const double integrationStepSeconds = (std::max)(0.0, simdt);

	// mjd names the start of the step Orbiter is about to integrate, so what ends
	// up on screen is one step later. That's why we need to add dt. 
	// If we would not add it, the extrapolated vessel location is dt * velocity behind.
	// That's not cool, because at 460m/s earth ground speed on the pad, that might be 10m.
	double renderSessionTime = 0.0;
	const bool hasRenderTime = TryGetSessionTime(mjd, renderSessionTime);
	if (hasRenderTime)
		renderSessionTime += integrationStepSeconds;

	const double presentationPredictionSeconds = hasRenderTime && replicatedKinematics.hasSessionTime ?
		ClampPredictionSeconds(renderSessionTime - replicatedKinematics.sampleSessionTime) : 0.0;

	// Extrapolate received state with 2nd order kinematics to compensate for the elapsed time
	// since the host measured his state
	nasspmp_kinematics::State target = nasspmp_kinematics::Extrapolate(
		replicatedKinematics.state, presentationPredictionSeconds);

	// A body-fixed sample is predicted in the turning frame and only then expressed inertially,
	// at the orientation the body reaches by the render date.
	ToInertialFrame(replicatedKinematics.referenceBody, replicatedKinematics.frame, integrationStepSeconds, target);

	// Measured against the sample's body
	const nasspmp_kinematics::State current = CurrentKinematicState(replicatedKinematics.referenceBody);
	double correctionFraction = std::nan("");
	const int smoothCorrection = correctionActive ? 1 : 0;

	// measure the acceleration that the orbiter-internal time-step uses for kinematic propagation
	// This is used to compensate for orbiters calculations since they are not deactivatable
	// This is a dirty trick, see also the comment in the header file
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
		// Show the target plus the offset, which is what keeps the vessel continuous across a
		// new sample, then decay the offset over this step.
		target.position = target.position + positionCorrection;
		target.velocity = target.velocity + velocityCorrection;
		target.orientation = nasspmp_kinematics::Normalize(
			nasspmp_kinematics::Multiply(target.orientation, orientationCorrection));

		DecayVisualOffset(positionCorrection, velocityCorrection, integrationStepSeconds);
		const double decay = OffsetDecayFactor(integrationStepSeconds);
		// Try using the same exponential decay instead of linear
		orientationCorrection = nasspmp_kinematics::Slerp(orientationCorrection,
			nasspmp_kinematics::Quaternion(), 1.0 - decay);

		correctionFraction = length(positionCorrection);
		if (correctionFraction < KinematicSettledOffsetMeters &&
			length(velocityCorrection) < KinematicSettledOffsetMeters) {
			positionCorrection = _V(0.0, 0.0, 0.0);
			velocityCorrection = _V(0.0, 0.0, 0.0);
			orientationCorrection = nasspmp_kinematics::Quaternion();
			correctionActive = false;
		}
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
	ApplyFreeFlightState(integrationStart, replicatedKinematics.referenceBody);

	if (kinematicsUpdateLogPending) {
		LogKinematics("update", replicatedKinematics.sampleSessionTime, replicatedKinematics.flightStatus,
			hasRenderTime ? renderSessionTime : std::nan(""), integrationStepSeconds,
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
