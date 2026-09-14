#pragma once

class ReplicationHub;

// Serves the generic versioned NASSP-MP ABI without exposing NASSP C++ objects.
class NasspReplicationBridge
{
public:
	// Handles one clbkGeneric request for the receiving entity's replication hub.
	static int HandleRequest(ReplicationHub &hub, int version, void *context);

private:
	// Writes the sealed entity schema and current role into an ABI request.
	static void GetEntityStatus(ReplicationHub &hub, void *context);
	// Copies the generic replication-group catalog into the caller-provided descriptor array.
	static void ReadCatalog(ReplicationHub &hub, void *context);
	// Propagates a negotiated entity role to the vessel's replication hub.
	static void SetRole(ReplicationHub &hub, void *context);
	// Reads an optional provider dirty revision for one compact group route.
	static void GetGroupRevision(ReplicationHub &hub, void *context);
	// Captures one complete group payload into the caller-provided buffer.
	static void CaptureGroup(ReplicationHub &hub, void *context);
	// Validates and applies a complete generic ABI batch atomically.
	static void ApplyBatch(ReplicationHub &hub, void *context);
};
