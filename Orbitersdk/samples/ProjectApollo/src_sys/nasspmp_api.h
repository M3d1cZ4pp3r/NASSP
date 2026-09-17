#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

// Versioned POD contract used through VESSEL::clbkGeneric. The caller owns
// every buffer, so no NASSP object or allocator crosses the DLL boundary.
namespace nasspmp_api
{
constexpr int MessageId = 0x4E4D5001;
namespace replication
{
constexpr int Version = 9;
constexpr std::size_t ReplicationKeyCapacity = 64;

using ComponentId = std::uint32_t;
using ReplicationGroupId = std::uint32_t;
using Revision = std::uint64_t;
using SimulationTick = std::uint64_t;

enum class Operation : std::uint32_t
{
	// Queries whether the entity sealed a compatible replication catalog.
	GetEntityStatus = 0,
	// Copies every component group descriptor into caller-owned storage.
	ReadCatalog = 1,
	// Changes the common role of every provider in the entity.
	SetRole = 2,
	// Reads an optional dirty revision for one component group.
	GetGroupRevision = 3,
	// Captures one group into a caller-owned payload buffer.
	CaptureGroup = 4,
	// Validates and applies a caller-owned multi-group payload batch.
	ApplyBatch = 5
};

enum class Result : std::uint32_t
{
	// The requested operation completed successfully.
	Ok = 0,
	// The request shape or an enum value is invalid.
	InvalidRequest = 1,
	// Provider registration and catalog sealing are not complete yet.
	NotReady = 2,
	// The caller must retry with more descriptor or payload storage.
	BufferTooSmall = 3,
	// The compact component or group route is not part of this schema.
	UnknownGroup = 4,
	// A payload range or provider encoding is structurally invalid.
	MalformedPayload = 5,
	// A structurally valid payload violates NASSP subsystem rules.
	RejectedValue = 6,
	// The provider does not implement the requested supported operation.
	Unsupported = 7,
	// NASSP could not complete the requested provider operation.
	Failed = 8
};

enum class ReplicationRole : std::uint32_t { Standalone = 0, Authority = 1, Replica = 2 };
enum class ReplicationDelivery : std::uint32_t { Reliable = 0, Unreliable = 1 };
enum class ApplyPurpose : std::uint32_t { Baseline = 0, AuthoritativeUpdate = 1, RemoteInput = 2 };

// Shares one ABI pointer slot between mutually exclusive output and input operations.
// The selected Request::operation determines which member the callee may access.
union ReplicationBufferPointer
{
	void *output;
	const void *input;
};

struct ReplicationGroupDescriptor
{
	// Human-readable stable keys used by diagnostics; compact IDs remain the wire route.
	char componentKey[ReplicationKeyCapacity];
	char groupKey[ReplicationKeyCapacity];
	// Compact component route valid for the sealed schema only.
	std::uint32_t componentId;
	// Compact group route valid within componentId only.
	std::uint32_t groupId;
	// Selects ordered delivery or replaceable latest-state delivery.
	ReplicationDelivery delivery;
	// Allows replicas to capture and send this group back to the authority.
	std::uint32_t clientReplicates;
	// Sends the current value at this interval; zero disables periodic transmission.
	std::uint32_t periodicIntervalMs;
	// Captures and sends this group when its value changes.
	std::uint32_t replicateChanges;
	// Defers authority echo while a replica is actively changing this input group.
	std::uint32_t inputAuthorityHoldMs;
	// Bounds a single group payload before allocation or transport.
	std::uint32_t maximumPayloadBytes;
};

struct ApplyItem
{
	// Selects the target component in the sealed catalog.
	std::uint32_t componentId;
	// Selects the target group within componentId.
	std::uint32_t groupId;
	// Starts this item in Request::payload.
	std::uint32_t payloadOffset;
	// Bounds this item within Request::payloadSize.
	std::uint32_t payloadSize;
};

struct Request
{
	// Lets NASSP reject callers compiled against an incompatible POD layout.
	std::uint32_t size = sizeof(Request);
	// Selects the generic ABI operation.
	Operation operation = Operation::GetEntityStatus;
	// Receives the operation result through the POD callback contract.
	Result result = Result::InvalidRequest;
	// Supplies or receives the entity-wide replication role.
	ReplicationRole role = ReplicationRole::Standalone;
	// Supplies or receives the sealed catalog compatibility fingerprint.
	std::uint64_t schemaId = 0;
	// Receives a provider dirty revision when GetGroupRevision succeeds.
	std::uint64_t revision = 0;
	// Associates a capture or apply with an authority simulation frame.
	std::uint64_t serverTick = 0;
	// Estimates how old an applied authority sample is at the client.
	std::uint32_t estimatedMessageAgeMs = 0;
	// Selects a component for single-group requests.
	std::uint32_t componentId = 0;
	// Selects a component-local group for single-group requests.
	std::uint32_t groupId = 0;
	// Requests a complete baseline capture instead of an ordinary scheduled capture.
	std::uint32_t captureIsBaseline = 0;
	// Defines the semantic reason for an ApplyBatch request.
	ApplyPurpose applyPurpose = ApplyPurpose::Baseline;
	// Uses output for writable ReadCatalog descriptors and input for immutable ApplyBatch items.
	ReplicationBufferPointer items = { nullptr };
	// Bounds writable descriptor output storage.
	std::uint32_t itemCapacity = 0;
	// Supplies or receives the number of descriptor or apply items.
	std::uint32_t itemCount = 0;
	// Uses output for writable CaptureGroup bytes and input for immutable ApplyBatch bytes.
	ReplicationBufferPointer payload = { nullptr };
	// Bounds writable capture payload storage.
	std::uint32_t payloadCapacity = 0;
	// Supplies apply input length or receives captured output length.
	std::uint32_t payloadSize = 0;
};

static_assert(std::is_standard_layout<ReplicationGroupDescriptor>::value &&
	std::is_trivially_copyable<ReplicationGroupDescriptor>::value,
	"ReplicationGroupDescriptor must remain an ABI POD");
static_assert(std::is_standard_layout<ApplyItem>::value && std::is_trivially_copyable<ApplyItem>::value,
	"ApplyItem must remain an ABI POD");
static_assert(std::is_standard_layout<ReplicationBufferPointer>::value &&
	std::is_trivially_copyable<ReplicationBufferPointer>::value, "ReplicationBufferPointer must remain an ABI POD");
static_assert(std::is_standard_layout<Request>::value && std::is_trivially_copyable<Request>::value,
	"Replication Request must remain an ABI POD");
}
}
