#pragma once

#include "ReplicationCatalog.h"

class IReplicationProvider
{
public:
	virtual ~IReplicationProvider() = default;

	// Returns the stable key that identifies this component within one entity.
	virtual const char *ComponentKey() const = 0;
	// Declares every group and its transport-independent semantics.
	virtual ProviderResult Describe(ReplicationCatalogBuilder &catalog) const = 0;
	// Optionally returns a revision used by NASSP-MP to avoid unnecessary captures.
	virtual bool TryGetRevision(const char *groupKey, Revision &revision) const { return false; }
	// Serializes one current group state without advancing the NASSP simulation.
	virtual ProviderResult Capture(const char *groupKey, ReplicationWriter &writer, const CaptureContext &context) = 0;
	// Checks one incoming payload without changing visible or simulated state.
	virtual ProviderResult Validate(const char *groupKey, const ReplicationReader &reader, const ApplyContext &context) const { return ProviderResult::Unsupported; }
	// Applies a payload only after the hub validated the complete batch.
	virtual void Apply(const char *groupKey, const ReplicationReader &reader, const ApplyContext &context) {}
	// Notifies the provider when its owning entity changes replication role.
	virtual void OnRoleChanged(ReplicationRole role) {}
};
