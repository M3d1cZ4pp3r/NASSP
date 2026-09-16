#include "ReplicationCatalog.h"

#include "IReplicationProvider.h"

#include <algorithm>
namespace
{
const std::uint64_t SchemaOffsetBasis = UINT64_C(14695981039346656037);
const std::uint64_t SchemaPrime = UINT64_C(1099511628211);

// Extends the schema fingerprint with one byte in a platform-independent order.
std::uint64_t HashByte(std::uint64_t hash, std::uint8_t value)
{
	return (hash ^ value) * SchemaPrime;
}

// Includes a null terminator so adjacent keys cannot produce an ambiguous hash input.
std::uint64_t HashString(std::uint64_t hash, const char *value)
{
	for (const char *character = value; *character; character++)
		hash = HashByte(hash, static_cast<std::uint8_t>(*character));

	return HashByte(hash, 0);
}

// Adds a 32-bit descriptor property in explicit little-endian byte order.
std::uint64_t HashUint32(std::uint64_t hash, std::uint32_t value)
{
	for (unsigned int index = 0; index < sizeof(value); index++)
		hash = HashByte(hash, static_cast<std::uint8_t>(value >> (index * 8)));

	return hash;
}

bool IsValidDescriptor(const ReplicationGroupDescriptor &descriptor)
{
	const bool validDelivery = descriptor.delivery == ReplicationDelivery::Reliable || descriptor.delivery == ReplicationDelivery::Unreliable;
	const bool validPrediction = descriptor.prediction == PredictionMode::Off || descriptor.prediction == PredictionMode::On;
	const bool unreliableCanRecover = descriptor.delivery != ReplicationDelivery::Unreliable || descriptor.periodicIntervalMs != 0;
	return descriptor.schemaId && descriptor.maximumPayloadBytes && validDelivery && validPrediction && unreliableCanRecover;
}
} // namespace

ReplicationSchemaBuilder::ReplicationSchemaBuilder() : schemaId(SchemaOffsetBasis) {}

void ReplicationSchemaBuilder::AddString(const char *value)
{
	schemaId = HashString(schemaId, value ? value : "");
}

void ReplicationSchemaBuilder::AddUint32(std::uint32_t value)
{
	schemaId = HashUint32(schemaId, value);
}

void ReplicationSchemaBuilder::AddUint64(std::uint64_t value)
{
	AddUint32(static_cast<std::uint32_t>(value));
	AddUint32(static_cast<std::uint32_t>(value >> 32));
}

std::uint64_t ReplicationSchemaBuilder::SchemaId() const
{
	return schemaId;
}

void ReplicationCatalogBuilder::AddGroup(const ReplicationGroupDescriptor &descriptor)
{
	// The descriptor is a small value declaration; the provider retains ownership of its key.
	groups.push_back(descriptor);
}

ReplicationCatalog::BuildResult ReplicationCatalog::Build(const std::vector<IReplicationProvider *> &providers)
{
	std::vector<ReplicationCatalogComponent> builtComponents;
	builtComponents.reserve(providers.size());

	// Ask each registered provider for its transport-independent group declarations.
	for (IReplicationProvider *provider : providers) {
		const char *componentKey = provider ? provider->ComponentKey() : nullptr;
		if (!componentKey || !componentKey[0])
			return BuildResult::EmptyComponentKey;

		ReplicationCatalogBuilder builder;
		const ProviderResult described = provider->Describe(builder);
		if (described != ProviderResult::Success)
			return BuildResult::ProviderFailure;

		ReplicationCatalogComponent component;
		component.key = componentKey;
		component.groups.reserve(builder.groups.size());
		for (const ReplicationGroupDescriptor &descriptor : builder.groups) {
			if (!descriptor.key || !descriptor.key[0])
				return BuildResult::EmptyGroupKey;
			if (!IsValidDescriptor(descriptor))
				return BuildResult::InvalidDescriptor;

			ReplicationCatalogGroup group;
			group.key = descriptor.key;
			group.descriptor = descriptor;
			group.descriptor.key = nullptr;
			component.groups.push_back(group);
		}

		// Group IDs must not depend on registration order or provider memory addresses.
		std::sort(component.groups.begin(), component.groups.end(), [](const ReplicationCatalogGroup &left, const ReplicationCatalogGroup &right) { return left.key < right.key; });
		for (std::size_t index = 1; index < component.groups.size(); index++) {
			if (component.groups[index - 1].key == component.groups[index].key)
				return BuildResult::DuplicateGroupKey;
		}

		builtComponents.push_back(std::move(component));
	}

	// Component IDs receive the same stable ordering rule as group IDs.
	std::sort(builtComponents.begin(), builtComponents.end(), [](const ReplicationCatalogComponent &left, const ReplicationCatalogComponent &right) { return left.key < right.key; });
	for (std::size_t index = 1; index < builtComponents.size(); index++) {
		if (builtComponents[index - 1].key == builtComponents[index].key)
			return BuildResult::DuplicateComponentKey;
	}

	// The schema fingerprints every wire-relevant property, not the generated IDs.
	ReplicationSchemaBuilder schema;
	for (std::size_t componentIndex = 0; componentIndex < builtComponents.size(); componentIndex++) {
		ReplicationCatalogComponent &component = builtComponents[componentIndex];
		component.id = static_cast<ComponentId>(componentIndex);
		schema.AddString(component.key.c_str());

		for (std::size_t groupIndex = 0; groupIndex < component.groups.size(); groupIndex++) {
			ReplicationCatalogGroup &group = component.groups[groupIndex];
			group.id = static_cast<ReplicationGroupId>(groupIndex);
			schema.AddString(group.key.c_str());
			schema.AddUint64(group.descriptor.schemaId);
			schema.AddUint32(static_cast<std::uint32_t>(group.descriptor.delivery));
			schema.AddUint32(static_cast<std::uint32_t>(group.descriptor.prediction));
			schema.AddUint32(group.descriptor.clientReplicates ? 1 : 0);
			schema.AddUint32(group.descriptor.replicateChanges ? 1 : 0);
			schema.AddUint32(group.descriptor.maximumPayloadBytes);
		}
	}

	// Publish only a complete, validated catalog so callers never observe a partial rebuild.
	components = std::move(builtComponents);
	schemaId = schema.SchemaId();
	return BuildResult::Success;
}

const std::vector<ReplicationCatalogComponent> &ReplicationCatalog::Components() const
{
	return components;
}

std::uint64_t ReplicationCatalog::SchemaId() const
{
	return schemaId;
}
