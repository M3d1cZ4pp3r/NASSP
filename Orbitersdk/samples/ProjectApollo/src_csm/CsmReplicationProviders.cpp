#include "CsmReplicationProviders.h"

class SoundLib;

#include "toggleswitch.h"
#include "eps.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
const char *DiscreteGroupKey = "discrete_controls";
const char *ContinuousGroupKey = "continuous_controls";
const char *PresentationGroupKey = "presentation";

}

PanelControlProvider::PanelControlProvider(PanelSwitchScenarioHandler &switches) : switches(switches)
{
}

const char *PanelControlProvider::ComponentKey() const
{
	return "panel.controls";
}

bool PanelControlProvider::BuildCatalog() const
{
	if (catalogBuilt)
		return true;

	for (PanelSwitchItem *item = switches.GetFirstSwitch(); item; item = item->GetNextForScenario()) {
		const char *name = item->GetName();
		if (!name || !name[0] || item->GetReplicationValueType() == PanelReplicationValueType::Excluded)
			continue;

		Control control;
		control.item = item;
		control.key = name;
		const PanelReplicationValueType type = item->GetReplicationValueType();
		if (type == PanelReplicationValueType::IndicatorPresentation || type == PanelReplicationValueType::MeterPresentation)
			presentationControls.push_back(control);
		else if (type == PanelReplicationValueType::ContinuousInput) {
			item->TrackReplicationRevision(&continuousRevision);
			continuousControls.push_back(control);
		} else {
			item->TrackReplicationRevision(&discreteRevision);
			discreteControls.push_back(control);
		}
	}

	const auto compareByKey = [](const Control &left, const Control &right) { return left.key < right.key; };
	std::sort(discreteControls.begin(), discreteControls.end(), compareByKey);
	std::sort(continuousControls.begin(), continuousControls.end(), compareByKey);
	std::sort(presentationControls.begin(), presentationControls.end(), compareByKey);
	const auto hasDuplicates = [](const std::vector<Control> &controls) {
		for (std::size_t index = 1; index < controls.size(); index++) {
			if (controls[index - 1].key == controls[index].key)
				return true;
		}
		return false;
	};
	if (hasDuplicates(discreteControls) || hasDuplicates(continuousControls) || hasDuplicates(presentationControls))
		return false;
	catalogBuilt = true;
	return true;
}

bool PanelControlProvider::TryGetRevision(const char *groupKey, Revision &revision) const
{
	BuildCatalog();
	if (std::strcmp(groupKey, DiscreteGroupKey) == 0) {
		revision = discreteRevision;
		return true;
	}
	if (std::strcmp(groupKey, ContinuousGroupKey) == 0) {
		revision = continuousRevision;
		return true;
	}
	return false;
}

std::uint64_t PanelControlProvider::GroupSchemaId(const std::vector<Control> &controls) const
{
	ReplicationSchemaBuilder schema;
	schema.AddString("panel-bit-packed-v1");
	for (const Control &control : controls) {
		schema.AddString(control.key.c_str());
		schema.AddUint32(static_cast<std::uint32_t>(control.item->GetReplicationValueType()));
		schema.AddUint32(control.item->ReplicationStateBitCount());
		schema.AddUint32(control.item->MaximumReplicationState());
		schema.AddUint32(control.item->ReplicatesHeldState() ? 1 : 0);
		schema.AddUint32(control.item->ReplicatesGuardState() ? 1 : 0);
	}
	return schema.SchemaId();
}

std::uint32_t PanelControlProvider::MaximumPayloadBytes(const std::vector<Control> &controls) const
{
	std::uint32_t maximumBits = 0;
	for (const Control &control : controls)
		maximumBits += control.item->MaximumReplicationBits();
	return (maximumBits + 7) / 8;
}

ProviderResult PanelControlProvider::Describe(ReplicationCatalogBuilder &catalog) const
{
	const bool catalogAvailable = BuildCatalog();
	if (!catalogAvailable)
		return ProviderResult::Failed;

	if (!discreteControls.empty()) {
		ReplicationGroupDescriptor controls;
		controls.key = DiscreteGroupKey;
		controls.schemaId = GroupSchemaId(discreteControls);
		controls.delivery = ReplicationDelivery::Reliable;
		controls.clientReplicates = true;
		controls.replicateChanges = true;
		controls.maximumPayloadBytes = MaximumPayloadBytes(discreteControls);
		catalog.AddGroup(controls);
	}

	if (!continuousControls.empty()) {
		ReplicationGroupDescriptor controls;
		controls.key = ContinuousGroupKey;
		controls.schemaId = GroupSchemaId(continuousControls);
		controls.delivery = ReplicationDelivery::Reliable;
		controls.clientReplicates = true;
		controls.replicateChanges = true;
		controls.maximumPayloadBytes = MaximumPayloadBytes(continuousControls);
		controls.inputAuthorityHoldMs = 150;
		catalog.AddGroup(controls);
	}

	if (!presentationControls.empty()) {
		ReplicationGroupDescriptor presentation;
		presentation.key = PresentationGroupKey;
		presentation.schemaId = GroupSchemaId(presentationControls);
		presentation.delivery = ReplicationDelivery::Unreliable;
		presentation.periodicIntervalMs = 33;
		presentation.replicateChanges = false;
		presentation.maximumPayloadBytes = MaximumPayloadBytes(presentationControls);
		catalog.AddGroup(presentation);
	}

	return ProviderResult::Success;
}

const std::vector<PanelControlProvider::Control> &PanelControlProvider::Controls(const char *groupKey) const
{
	BuildCatalog();
	if (std::strcmp(groupKey, DiscreteGroupKey) == 0)
		return discreteControls;
	if (std::strcmp(groupKey, ContinuousGroupKey) == 0)
		return continuousControls;
	return presentationControls;
}

bool PanelControlProvider::WriteValues(const std::vector<Control> &controls, ReplicationWriter &writer)
{
	for (const Control &control : controls) {
		if (!control.item->WriteReplicationValue(writer))
			return false;
	}
	return true;
}

ProviderResult PanelControlProvider::Capture(const char *groupKey, ReplicationWriter &writer, const CaptureContext &)
{
	if (std::strcmp(groupKey, DiscreteGroupKey) != 0 && std::strcmp(groupKey, ContinuousGroupKey) != 0 && std::strcmp(groupKey, PresentationGroupKey) != 0)
		return ProviderResult::Unsupported;

	const bool written = WriteValues(Controls(groupKey), writer);
	return written ? ProviderResult::Success : ProviderResult::BufferTooSmall;
}

ProviderResult PanelControlProvider::ReadValues(const std::vector<Control> &controls, const ReplicationReader &reader, const ApplyContext &context, bool apply) const
{
	for (const Control &control : controls) {
		PanelReplicationValue value = 0;
		if (!control.item->ReadReplicationValue(reader, value))
			return ProviderResult::Malformed;
		if (!control.item->ValidateReplicationValue(value))
			return ProviderResult::Rejected;
		if (apply)
			control.item->ApplyReplicationValue(value, context.purpose);
	}
	const bool payloadComplete = reader.Finish();
	return payloadComplete ? ProviderResult::Success : ProviderResult::Malformed;
}

ProviderResult PanelControlProvider::Validate(const char *groupKey, const ReplicationReader &reader, const ApplyContext &context) const
{
	if (std::strcmp(groupKey, DiscreteGroupKey) != 0 && std::strcmp(groupKey, ContinuousGroupKey) != 0 && std::strcmp(groupKey, PresentationGroupKey) != 0)
		return ProviderResult::Unsupported;
	if (std::strcmp(groupKey, PresentationGroupKey) == 0 && context.purpose == ApplyPurpose::RemoteInput)
		return ProviderResult::Unsupported;
	return ReadValues(Controls(groupKey), reader, context, false);
}

void PanelControlProvider::Apply(const char *groupKey, const ReplicationReader &reader, const ApplyContext &context)
{
	ReadValues(Controls(groupKey), reader, context, true);
}

void PanelControlProvider::OnRoleChanged(ReplicationRole role)
{
	BuildCatalog();
	const bool presentationActive = role == ReplicationRole::Replica;
	for (const Control &control : presentationControls)
		control.item->SetReplicationPresentationActive(presentationActive);
}

CsmLightingProvider::CsmLightingProvider(FloodLights &leftFlood, FloodLights &rightFlood, FloodLights &lebFlood, IntegralLights &leftIntegral, IntegralLights &rightIntegral, IntegralLights &lebIntegral, NumericLights &leftNumeric, NumericLights &lebNumeric)
{
	floodLights[0] = &leftFlood;
	floodLights[1] = &rightFlood;
	floodLights[2] = &lebFlood;
	integralLights[0] = &leftIntegral;
	integralLights[1] = &rightIntegral;
	integralLights[2] = &lebIntegral;
	numericLights[0] = &leftNumeric;
	numericLights[1] = &lebNumeric;
}

const char *CsmLightingProvider::ComponentKey() const
{
	return "csm.lighting";
}

ProviderResult CsmLightingProvider::Describe(ReplicationCatalogBuilder &catalog) const
{
	ReplicationSchemaBuilder schema;
	schema.AddString("csm-interior-lighting-v1");
	schema.AddString("left_flood");
	schema.AddString("right_flood");
	schema.AddString("leb_flood");
	schema.AddString("left_integral");
	schema.AddString("right_integral");
	schema.AddString("leb_integral");
	schema.AddString("left_numeric");
	schema.AddString("leb_numeric");
	schema.AddString("float32");

	ReplicationGroupDescriptor presentation;
	presentation.key = PresentationGroupKey;
	presentation.schemaId = schema.SchemaId();
	presentation.delivery = ReplicationDelivery::Unreliable;
	presentation.periodicIntervalMs = 50;
	presentation.replicateChanges = false;
	presentation.maximumPayloadBytes = 8 * sizeof(float);
	catalog.AddGroup(presentation);
	return ProviderResult::Success;
}

ProviderResult CsmLightingProvider::Capture(const char *groupKey, ReplicationWriter &writer, const CaptureContext &)
{
	if (std::strcmp(groupKey, PresentationGroupKey) != 0)
		return ProviderResult::Unsupported;

	for (FloodLights *lights : floodLights) {
		if (!writer.WriteScalar(static_cast<float>(lights->GetCombinedOutput())))
			return ProviderResult::BufferTooSmall;
	}
	for (IntegralLights *lights : integralLights) {
		if (!writer.WriteScalar(static_cast<float>(lights->GetOutput())))
			return ProviderResult::BufferTooSmall;
	}
	for (NumericLights *lights : numericLights) {
		if (!writer.WriteScalar(static_cast<float>(lights->GetOutput())))
			return ProviderResult::BufferTooSmall;
	}
	return ProviderResult::Success;
}

ProviderResult CsmLightingProvider::Read(const ReplicationReader &reader, bool apply) const
{
	float outputs[8] = {};
	for (float &output : outputs) {
		if (!reader.ReadScalar(output))
			return ProviderResult::Malformed;
		if (!std::isfinite(output) || output < 0.0f || output > 10.0f)
			return ProviderResult::Rejected;
	}
	if (!reader.Finish())
		return ProviderResult::Malformed;

	if (apply) {
		for (int index = 0; index < 3; index++)
			floodLights[index]->SetReplicatedOutput(outputs[index]);
		for (int index = 0; index < 3; index++)
			integralLights[index]->SetReplicatedOutput(outputs[index + 3]);
		for (int index = 0; index < 2; index++)
			numericLights[index]->SetReplicatedOutput(outputs[index + 6]);
	}
	return ProviderResult::Success;
}

ProviderResult CsmLightingProvider::Validate(const char *groupKey, const ReplicationReader &reader, const ApplyContext &context) const
{
	if (std::strcmp(groupKey, PresentationGroupKey) != 0 || context.purpose == ApplyPurpose::RemoteInput)
		return ProviderResult::Unsupported;
	return Read(reader, false);
}

void CsmLightingProvider::Apply(const char *, const ReplicationReader &reader, const ApplyContext &)
{
	Read(reader, true);
}

void CsmLightingProvider::OnRoleChanged(ReplicationRole role)
{
	const bool presentationActive = role == ReplicationRole::Replica;
	for (FloodLights *lights : floodLights)
		lights->SetReplicationPresentationActive(presentationActive);
	for (IntegralLights *lights : integralLights)
		lights->SetReplicationPresentationActive(presentationActive);
	for (NumericLights *lights : numericLights)
		lights->SetReplicationPresentationActive(presentationActive);
}
