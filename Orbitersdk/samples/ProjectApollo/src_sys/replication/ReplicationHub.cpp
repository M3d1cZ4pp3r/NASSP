#include "ReplicationHub.h"

#include <algorithm>
#include <cstring>
#include <set>

ReplicationHub::RegistrationResult ReplicationHub::Register(IReplicationProvider &provider)
{
	// A negotiated entity schema must remain immutable for its complete epoch.
	if (catalogSealed)
		return RegistrationResult::CatalogSealed;

	const char *componentKey = provider.ComponentKey();
	if (!componentKey || !componentKey[0])
		return RegistrationResult::InvalidProvider;

	// A component key identifies exactly one provider within this entity.
	for (IReplicationProvider *registered : providers) {
		if (registered == &provider || std::strcmp(registered->ComponentKey(), componentKey) == 0)
			return RegistrationResult::DuplicateProvider;
	}

	// Keep registration atomic when a provider reports an invalid catalog declaration.
	providers.push_back(&provider);
	const RegistrationResult result = RebuildCatalog();
	if (result != RegistrationResult::Success)
		providers.pop_back();

	return result;
}

ReplicationHub::RegistrationResult ReplicationHub::SealCatalog()
{
	if (catalogSealed)
		return RegistrationResult::Success;

	// Rebuild once more so sealing always publishes the complete construction-time set.
	const RegistrationResult result = RebuildCatalog();
	if (result == RegistrationResult::Success)
		catalogSealed = true;

	return result;
}

bool ReplicationHub::IsCatalogSealed() const
{
	return catalogSealed;
}

void ReplicationHub::UnregisterAll()
{
	// The owning vessel calls this after derived subsystem members may already be destroyed.
	// Drop their non-owning addresses without invoking provider callbacks.
	providers.clear();
	routedProviders.clear();
	catalog = ReplicationCatalog();
	catalogSealed = false;
	role = ReplicationRole::Standalone;
}

ReplicationHub::RegistrationResult ReplicationHub::RebuildCatalog()
{
	const ReplicationCatalog::BuildResult buildResult = catalog.Build(providers);
	if (buildResult != ReplicationCatalog::BuildResult::Success)
		return RegistrationResult::CatalogFailure;

	// Catalog construction sorts components, so establish their direct provider routes once.
	routedProviders.clear();
	for (const ReplicationCatalogComponent &component : catalog.Components()) {
		for (IReplicationProvider *provider : providers) {
			if (std::strcmp(provider->ComponentKey(), component.key.c_str()) == 0) {
				routedProviders.push_back(provider);
				break;
			}
		}
	}
	return routedProviders.size() == catalog.Components().size() ? RegistrationResult::Success : RegistrationResult::CatalogFailure;
}

const ReplicationCatalog &ReplicationHub::Catalog() const
{
	return catalog;
}

ReplicationRole ReplicationHub::GetRole() const
{
	return role;
}

void ReplicationHub::SetRole(ReplicationRole newRole)
{
	if (role == newRole)
		return;

	// Providers clear or activate their local overrides in response to this notification.
	role = newRole;
	for (IReplicationProvider *provider : providers)
		provider->OnRoleChanged(role);
}

ProviderResult ReplicationHub::CaptureGroup(ComponentId componentId, ReplicationGroupId groupId, ReplicationWriter &writer, const CaptureContext &context) const
{
	IReplicationProvider *provider = FindProvider(componentId);
	const std::vector<ReplicationCatalogComponent> &components = catalog.Components();
	if (!provider || componentId >= components.size() || groupId >= components[componentId].groups.size())
		return ProviderResult::Unsupported;

	const ReplicationCatalogComponent &component = components[componentId];
	return provider->Capture(component.groups[groupId].key.c_str(), writer, context);
}

bool ReplicationHub::TryGetGroupRevision(ComponentId componentId, ReplicationGroupId groupId, Revision &revision) const
{
	IReplicationProvider *provider = FindProvider(componentId);
	const std::vector<ReplicationCatalogComponent> &components = catalog.Components();
	if (!provider || componentId >= components.size() || groupId >= components[componentId].groups.size())
		return false;

	const ReplicationCatalogComponent &component = components[componentId];
	return provider->TryGetRevision(component.groups[groupId].key.c_str(), revision);
}

ProviderResult ReplicationHub::ValidateBatch(const std::vector<ApplyItem> &items, const ApplyContext &context) const
{
	std::set<std::pair<ComponentId, ReplicationGroupId>> routes;
	const std::vector<ReplicationCatalogComponent> &components = catalog.Components();
	for (const ApplyItem &item : items) {
		IReplicationProvider *provider = FindProvider(item.componentId);
		if (!provider || item.componentId >= components.size() || item.groupId >= components[item.componentId].groups.size() || (!item.payload && item.payloadSize))
			return ProviderResult::Malformed;
		const std::pair<ComponentId, ReplicationGroupId> route(item.componentId, item.groupId);
		const bool routeInserted = routes.insert(route).second;
		if (!routeInserted)
			return ProviderResult::Malformed;

		// Validate first so no earlier provider can leave a partial snapshot behind.
		ReplicationReader reader(item.payload, item.payloadSize);
		const ReplicationCatalogComponent &component = components[item.componentId];
		const ProviderResult validation = provider->Validate(component.groups[item.groupId].key.c_str(), reader, context);
		if (validation != ProviderResult::Success)
			return validation;
	}

	return ProviderResult::Success;
}

ProviderResult ReplicationHub::ApplyBatch(const std::vector<ApplyItem> &items, const ApplyContext &context)
{
	const ProviderResult validation = ValidateBatch(items, context);
	if (validation != ProviderResult::Success)
		return validation;

	// A second reader starts each provider at the beginning of its already validated payload.
	const std::vector<ReplicationCatalogComponent> &components = catalog.Components();
	for (const ApplyItem &item : items) {
		IReplicationProvider *provider = FindProvider(item.componentId);
		ReplicationReader reader(item.payload, item.payloadSize);
		const ReplicationCatalogComponent &component = components[item.componentId];
		provider->Apply(component.groups[item.groupId].key.c_str(), reader, context);
	}

	return ProviderResult::Success;
}

IReplicationProvider *ReplicationHub::FindProvider(ComponentId componentId) const
{
	if (componentId >= routedProviders.size())
		return nullptr;
	return routedProviders[componentId];
}
