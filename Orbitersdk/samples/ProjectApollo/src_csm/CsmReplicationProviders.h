#pragma once

#include "replication/IReplicationProvider.h"

#include <string>
#include <vector>

class PanelSwitchItem;
class PanelSwitchScenarioHandler;

/// \brief Provides CSM panel controls through the common replication interface.
class PanelControlProvider : public IReplicationProvider
{
public:
	explicit PanelControlProvider(PanelSwitchScenarioHandler &switches);

	const char *ComponentKey() const override;
	ProviderResult Describe(ReplicationCatalogBuilder &catalog) const override;
	bool TryGetRevision(const char *groupKey, Revision &revision) const override;
	ProviderResult Capture(const char *groupKey, ReplicationWriter &writer,
		const CaptureContext &context) override;
	ProviderResult Validate(const char *groupKey, const ReplicationReader &reader,
		const ApplyContext &context) const override;
	void Apply(const char *groupKey, const ReplicationReader &reader, const ApplyContext &context) override;
	void OnRoleChanged(ReplicationRole role) override;

private:
	struct Control
	{
		PanelSwitchItem *item = nullptr;
		std::string key;
	};

	bool BuildCatalog() const;
	const std::vector<Control> &Controls(const char *groupKey) const;
	std::uint64_t GroupSchemaId(const std::vector<Control> &controls) const;
	std::uint32_t MaximumPayloadBytes(const std::vector<Control> &controls) const;
	bool WriteValues(const std::vector<Control> &controls, ReplicationWriter &writer);
	ProviderResult ReadValues(const std::vector<Control> &controls, const ReplicationReader &reader,
		const ApplyContext &context, bool apply) const;

	PanelSwitchScenarioHandler &switches;
	mutable bool catalogBuilt = false;
	mutable std::vector<Control> discreteControls;
	mutable std::vector<Control> continuousControls;
	mutable std::vector<Control> presentationControls;
	/// Revisions for the current discrete and continuous control values.
	mutable Revision discreteRevision = 1;
	mutable Revision continuousRevision = 1;
};
