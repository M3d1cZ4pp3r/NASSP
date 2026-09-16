#include "NasspReplicationBridge.h"

#include "nasspmp_api.h"
#include "replication/ReplicationHub.h"

#include <vector>

namespace api = nasspmp_api::replication;

namespace
{
// Converts a provider validation result into a stable ABI result value.
api::Result ToApiResult(ProviderResult result)
{
	if (result == ProviderResult::Unsupported)
		return api::Result::Unsupported;
	if (result == ProviderResult::Malformed)
		return api::Result::MalformedPayload;
	if (result == ProviderResult::Rejected)
		return api::Result::RejectedValue;
	if (result == ProviderResult::BufferTooSmall)
		return api::Result::BufferTooSmall;
	if (result == ProviderResult::Failed)
		return api::Result::Failed;

	return api::Result::Ok;
}

// Rejects unknown integer values before they are converted into NASSP enums.
bool IsValidApplyPurpose(api::ApplyPurpose purpose)
{
	return purpose == api::ApplyPurpose::Baseline ||
		purpose == api::ApplyPurpose::AuthoritativeUpdate ||
		purpose == api::ApplyPurpose::RemoteInput;
}

}

int NasspReplicationBridge::HandleRequest(ReplicationHub &hub, int version, void *context)
{
	api::Request *request = static_cast<api::Request *>(context);
	if (version != api::Version || !request || request->size != sizeof(api::Request))
		return 0;

	// Every operation starts with a defined result even if a later validation fails.
	request->result = api::Result::InvalidRequest;
	switch (request->operation) {
	case api::Operation::GetEntityStatus:
		GetEntityStatus(hub, request);
		break;
	case api::Operation::ReadCatalog:
		ReadCatalog(hub, request);
		break;
	case api::Operation::SetRole:
		SetRole(hub, request);
		break;
	case api::Operation::GetGroupRevision:
		GetGroupRevision(hub, request);
		break;
	case api::Operation::CaptureGroup:
		CaptureGroup(hub, request);
		break;
	case api::Operation::ApplyBatch:
		ApplyBatch(hub, request);
		break;
	default:
		break;
	}

	return 1;
}

void NasspReplicationBridge::GetEntityStatus(ReplicationHub &hub, void *context)
{
	api::Request *request = static_cast<api::Request *>(context);
	if (!hub.IsCatalogSealed()) {
		request->result = api::Result::NotReady;
		return;
	}

	request->role = hub.GetRole();
	request->schemaId = hub.Catalog().SchemaId();
	request->result = api::Result::Ok;
}

void NasspReplicationBridge::ReadCatalog(ReplicationHub &hub, void *context)
{
	api::Request *request = static_cast<api::Request *>(context);
	if (!hub.IsCatalogSealed()) {
		request->result = api::Result::NotReady;
		return;
	}

	const std::vector<ReplicationCatalogComponent> &components = hub.Catalog().Components();
	std::uint32_t groupCount = 0;
	for (const ReplicationCatalogComponent &component : components)
		groupCount += static_cast<std::uint32_t>(component.groups.size());

	request->schemaId = hub.Catalog().SchemaId();
	request->itemCount = groupCount;
	if (request->itemCapacity < groupCount || (groupCount && !request->items.output)) {
		request->result = api::Result::BufferTooSmall;
		return;
	}

	api::ReplicationGroupDescriptor *output = static_cast<api::ReplicationGroupDescriptor *>(request->items.output);
	std::uint32_t outputIndex = 0;
	for (const ReplicationCatalogComponent &component : components) {
		for (const ReplicationCatalogGroup &group : component.groups) {
			api::ReplicationGroupDescriptor &descriptor = output[outputIndex++];
			descriptor = {};
			descriptor.componentId = component.id;
			descriptor.groupId = group.id;
			descriptor.delivery = group.descriptor.delivery;
			descriptor.prediction = group.descriptor.prediction;
			descriptor.clientReplicates = group.descriptor.clientReplicates ? 1 : 0;
			descriptor.periodicIntervalMs = group.descriptor.periodicIntervalMs;
			descriptor.replicateChanges = group.descriptor.replicateChanges ? 1 : 0;
			descriptor.inputAuthorityHoldMs = group.descriptor.inputAuthorityHoldMs;
			descriptor.maximumPayloadBytes = group.descriptor.maximumPayloadBytes;
		}
	}

	request->result = api::Result::Ok;
}

void NasspReplicationBridge::SetRole(ReplicationHub &hub, void *context)
{
	api::Request *request = static_cast<api::Request *>(context);
	if (request->role != api::ReplicationRole::Standalone && request->role != api::ReplicationRole::Authority && request->role != api::ReplicationRole::Replica) {
		request->result = api::Result::InvalidRequest;
		return;
	}

	if (!hub.IsCatalogSealed()) {
		request->result = api::Result::NotReady;
		return;
	}

	hub.SetRole(request->role);
	request->result = api::Result::Ok;
}

void NasspReplicationBridge::GetGroupRevision(ReplicationHub &hub, void *context)
{
	api::Request *request = static_cast<api::Request *>(context);
	if (!hub.IsCatalogSealed()) {
		request->result = api::Result::NotReady;
		return;
	}
	const std::vector<ReplicationCatalogComponent> &components = hub.Catalog().Components();
	if (request->componentId >= components.size() || request->groupId >= components[request->componentId].groups.size()) {
		request->result = api::Result::UnknownGroup;
		return;
	}

	Revision revision = 0;
	const bool revisionAvailable = hub.TryGetGroupRevision(request->componentId, request->groupId, revision);
	if (!revisionAvailable) {
		request->result = api::Result::Unsupported;
		return;
	}
	request->revision = revision;
	request->result = api::Result::Ok;
}

void NasspReplicationBridge::CaptureGroup(ReplicationHub &hub, void *context)
{
	api::Request *request = static_cast<api::Request *>(context);
	if (!hub.IsCatalogSealed()) {
		request->result = api::Result::NotReady;
		return;
	}
	if (!request->payload.output && request->payloadCapacity) {
		request->result = api::Result::InvalidRequest;
		return;
	}

	ReplicationWriter writer(request->payload.output, request->payloadCapacity);
	CaptureContext captureContext;
	captureContext.simulationTick = request->serverTick;
	captureContext.isBaseline = request->captureIsBaseline != 0;
	const ProviderResult capture = hub.CaptureGroup(request->componentId, request->groupId, writer, captureContext);
	if (capture == ProviderResult::Success)
		writer.Flush();
	request->payloadSize = static_cast<std::uint32_t>(writer.Size());
	if (capture == ProviderResult::Unsupported)
		request->result = api::Result::UnknownGroup;
	else
		request->result = ToApiResult(capture);
}

void NasspReplicationBridge::ApplyBatch(ReplicationHub &hub, void *context)
{
	api::Request *request = static_cast<api::Request *>(context);
	if (!hub.IsCatalogSealed()) {
		request->result = api::Result::NotReady;
		return;
	}
	if ((!request->items.input && request->itemCount) || (!request->payload.input && request->payloadSize)) {
		request->result = api::Result::InvalidRequest;
		return;
	}
	if (!IsValidApplyPurpose(request->applyPurpose)) {
		request->result = api::Result::InvalidRequest;
		return;
	}

	const api::ApplyItem *input = static_cast<const api::ApplyItem *>(request->items.input);
	std::vector<ReplicationHub::ApplyItem> items;
	items.reserve(request->itemCount);
	const std::uint8_t *payload = static_cast<const std::uint8_t *>(request->payload.input);
	for (std::uint32_t index = 0; index < request->itemCount; index++) {
		const api::ApplyItem &inputItem = input[index];
		if (inputItem.payloadOffset > request->payloadSize || inputItem.payloadSize > request->payloadSize - inputItem.payloadOffset) {
			request->result = api::Result::MalformedPayload;
			return;
		}

		ReplicationHub::ApplyItem item;
		item.componentId = inputItem.componentId;
		item.groupId = inputItem.groupId;
		item.payload = inputItem.payloadSize ? payload + inputItem.payloadOffset : nullptr;
		item.payloadSize = inputItem.payloadSize;
		items.push_back(item);
	}

	ApplyContext applyContext;
	applyContext.purpose = request->applyPurpose;
	applyContext.serverTick = request->serverTick;
	request->result = ToApiResult(hub.ApplyBatch(items, applyContext));
}
