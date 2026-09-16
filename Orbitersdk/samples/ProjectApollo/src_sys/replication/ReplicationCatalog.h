#pragma once

#include "ReplicationTypes.h"

#include <cstdint>
#include <string>
#include <vector>

struct ReplicationCatalogGroup
{
	// Compact ID assigned after sorting the provider's stable group keys.
	ReplicationGroupId id = 0;
	// Owns the stable group key retained after provider catalog construction returns.
	std::string key;
	// Retains the group properties required for scheduling and schema verification.
	// The key pointer is unused after construction because this object owns key separately.
	ReplicationGroupDescriptor descriptor;
};

struct ReplicationCatalogComponent
{
	// Compact ID assigned after sorting all registered provider keys.
	ComponentId id = 0;
	// Owns the stable component key retained after provider catalog construction returns.
	std::string key;
	// Groups owned by this component, sorted by their stable descriptor keys.
	std::vector<ReplicationCatalogGroup> groups;
};

class ReplicationCatalogBuilder
{
public:
	// Adds one group declaration while a provider describes its component.
	void AddGroup(const ReplicationGroupDescriptor &descriptor);

private:
	friend class ReplicationCatalog;

	// Receives a temporary copy of each declaration during Describe().
	std::vector<ReplicationGroupDescriptor> groups;
};

// Builds stable schema identities from the ordered semantic parts of a payload contract.
class ReplicationSchemaBuilder
{
public:
	ReplicationSchemaBuilder();
	void AddString(const char *value);
	void AddUint32(std::uint32_t value);
	void AddUint64(std::uint64_t value);
	std::uint64_t SchemaId() const;

private:
	std::uint64_t schemaId;
};

class ReplicationCatalog
{
public:
	enum class BuildResult
{
		// Every provider declaration was valid and has been published.
		Success,
		// A provider cannot be addressed without a stable component key.
		EmptyComponentKey,
		// Two providers would claim the same component identity.
		DuplicateComponentKey,
		// A component cannot expose an unnamed group.
		EmptyGroupKey,
		// Two groups in one component would receive the same identity.
		DuplicateGroupKey,
		// A descriptor omits a schema identity or a finite payload bound.
		InvalidDescriptor,
		// A provider could not construct its declarations.
		ProviderFailure
	};

	// Rebuilds deterministic component and group IDs from registered providers.
	BuildResult Build(const std::vector<class IReplicationProvider *> &providers);

	// Returns components sorted by stable key, with ID equal to their array index.
	const std::vector<ReplicationCatalogComponent> &Components() const;
	// Returns the compatibility identity of all keys and wire-relevant group properties.
	std::uint64_t SchemaId() const;

private:
	// Stores only validated, deterministically ordered component declarations.
	std::vector<ReplicationCatalogComponent> components;
	// Changes whenever a key or wire-relevant descriptor property changes.
	std::uint64_t schemaId = 0;
};
