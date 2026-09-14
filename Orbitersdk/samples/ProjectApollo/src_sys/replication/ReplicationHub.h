#pragma once

#include "IReplicationProvider.h"

#include <vector>

class ReplicationHub
{
public:
	enum class RegistrationResult
	{
		// The provider was retained and the catalog was rebuilt.
		Success,
		// The entity already has a provider with the same component key.
		DuplicateProvider,
		// The candidate cannot supply a non-empty component key.
		InvalidProvider,
		// The candidate's group declarations did not form a valid catalog.
		CatalogFailure,
		// Registration is closed because this entity already exposed its schema.
		CatalogSealed
	};

	struct ApplyItem
	{
		// Selects the component declared in this entity's catalog.
		ComponentId componentId = 0;
		// Selects the component-local group declared in the catalog.
		ReplicationGroupId groupId = 0;
		// Points into the caller-owned batch payload buffer.
		const void *payload = nullptr;
		// Bounds the payload range for this component group.
		std::size_t payloadSize = 0;
	};

	// Registers a non-owned provider and rebuilds the entity catalog.
	RegistrationResult Register(IReplicationProvider &provider);
	// Freezes the complete catalog before its schema is exposed to NASSP-MP.
	RegistrationResult SealCatalog();
	// Returns whether compact IDs can no longer change for this entity epoch.
	bool IsCatalogSealed() const;
	// Releases all non-owning registrations before their subsystem members are destroyed.
	void UnregisterAll();

	// Returns the current stable catalog for this entity.
	const ReplicationCatalog &Catalog() const;
	// Returns the role shared by all providers of this entity.
	ReplicationRole GetRole() const;
	// Delivers a changed role exactly once to every registered provider.
	void SetRole(ReplicationRole role);

	// Routes a capture request after confirming that both compact IDs are known.
	ProviderResult CaptureGroup(ComponentId componentId, ReplicationGroupId groupId,
		ReplicationWriter &writer, const CaptureContext &context) const;
	// Returns an optional provider revision for centrally scheduled group capture.
	bool TryGetGroupRevision(ComponentId componentId, ReplicationGroupId groupId, Revision &revision) const;
	// Validates every item without modifying any provider state.
	ProviderResult ValidateBatch(const std::vector<ApplyItem> &items,
		const ApplyContext &context) const;
	// Applies a batch only when the preceding validation accepted every item.
	ProviderResult ApplyBatch(const std::vector<ApplyItem> &items, const ApplyContext &context);

private:
	// Rebuilds only while construction-time registration remains open.
	RegistrationResult RebuildCatalog();
	// Resolves a catalog component ID to its still-registered non-owned provider.
	IReplicationProvider *FindProvider(ComponentId componentId) const;

	// Provider lifetime belongs to the owning vessel and its subsystem members.
	std::vector<IReplicationProvider *> providers;
	// Maps catalog component IDs directly to their registered providers after each rebuild.
	std::vector<IReplicationProvider *> routedProviders;
	// The catalog maps stable keys to compact per-session component and group IDs.
	ReplicationCatalog catalog;
	// Prevents a live rebuild from silently changing compact group routes.
	bool catalogSealed = false;
	// Every provider of one entity observes the same replication role.
	ReplicationRole role = ReplicationRole::Standalone;
};
