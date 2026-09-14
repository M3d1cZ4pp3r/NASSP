#pragma once

#include "../nasspmp_api.h"

#include <type_traits>

using ComponentId = nasspmp_api::replication::ComponentId;
using ReplicationGroupId = nasspmp_api::replication::ReplicationGroupId;
using Revision = nasspmp_api::replication::Revision;
using SimulationTick = nasspmp_api::replication::SimulationTick;
using ReplicationDelivery = nasspmp_api::replication::ReplicationDelivery;
using PredictionMode = nasspmp_api::replication::PredictionMode;
using ReplicationRole = nasspmp_api::replication::ReplicationRole;
using ApplyPurpose = nasspmp_api::replication::ApplyPurpose;

enum class ProviderResult
{
	// The provider completed the requested operation.
	Success,
	// The caller must retry with a larger supplied buffer.
	BufferTooSmall,
	// The provider does not support this operation or group purpose.
	Unsupported,
	// The payload structure cannot be decoded safely.
	Malformed,
	// The decoded value violates the subsystem's semantic rules.
	Rejected,
	// The provider could not complete the operation.
	Failed
};

struct ReplicationGroupDescriptor
{
	// Stable provider-owned key used to derive the compact session group ID.
	const char *key = nullptr;
	// Content-derived identity of the provider-specific payload contract.
	std::uint64_t schemaId = 0;
	// Controls whether updates must arrive reliably or may replace older samples.
	ReplicationDelivery delivery = ReplicationDelivery::Reliable;
	// Declares whether the future client pipeline predicts between authority samples.
	PredictionMode prediction = PredictionMode::Off;
	// Allows a replica to send locally captured changes to the authority.
	bool clientReplicates = false;
	// Sends a fresh sample at this interval; zero disables periodic transmission.
	std::uint32_t periodicIntervalMs = 0;
	// Sends a sample whenever capture observes a changed payload.
	bool replicateChanges = true;
	// Prevents authority echo from fighting an in-progress local continuous input.
	std::uint32_t inputAuthorityHoldMs = 0;
	// Bounds one capture payload before any ABI or network allocation.
	std::uint32_t maximumPayloadBytes = 0;
};

struct CaptureContext
{
	// Identifies the simulator frame represented by the captured payload.
	SimulationTick simulationTick = 0;
	// Requests the complete initial state used before a replica becomes active.
	bool isBaseline = false;
};

struct ApplyContext
{
	// Distinguishes baseline restoration, authority updates and remote crew input.
	ApplyPurpose purpose = ApplyPurpose::Baseline;
	// Carries the authority simulator frame without exposing network identity.
	SimulationTick serverTick = 0;
};

class ReplicationWriter
{
public:
	// Writes a provider payload bit by bit into a buffer supplied by the caller.
	ReplicationWriter(void *buffer, std::size_t capacity);

	// Appends opaque bytes at the current bit position.
	bool Write(const void *data, std::size_t size);

	// Serializes an integral value with its native width in canonical least-significant-bit-first order.
	template<class T>
	typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, bool>::type
		WriteValue(T value)
	{
		return WriteValue(value, static_cast<unsigned int>(sizeof(T) * 8));
	}

	// Serializes only the requested low bits after verifying that the value fits.
	template<class T>
	typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, bool>::type
		WriteValue(T value,
		unsigned int bitCount)
	{
		if (!bitCount || bitCount > sizeof(T) * 8)
			return false;

		if (std::is_signed<T>::value) {
			const std::int64_t signedValue = static_cast<std::int64_t>(value);
			if (bitCount < 64) {
				const std::int64_t minimum = -(std::int64_t(1) << (bitCount - 1));
				const std::int64_t maximum = (std::int64_t(1) << (bitCount - 1)) - 1;
				if (signedValue < minimum || signedValue > maximum)
					return false;
			}
		}
		else if (bitCount < 64 && static_cast<std::uint64_t>(value) >= (std::uint64_t(1) << bitCount)) {
			return false;
		}

		typedef typename std::make_unsigned<T>::type UnsignedType;
		return WriteBits(static_cast<std::uint64_t>(static_cast<UnsignedType>(value)), bitCount);
	}
	bool WriteValue(bool value) { return WriteValue(static_cast<std::uint8_t>(value ? 1 : 0), 1); }

	// Serializes floating-point values without changing their IEEE representation.
	bool WriteValue(float value);
	bool WriteValue(double value);

	// Completes the last byte with zero padding and prevents further writes.
	void Flush();
	// Returns the number of payload bytes occupied by the written bits.
	std::size_t Size() const;
	// Returns the immutable capacity of the caller-owned buffer.
	std::size_t Capacity() const;

private:
	bool WriteBits(std::uint64_t value, unsigned int bitCount);

	// Points to the caller-owned writable payload buffer.
	std::uint8_t *buffer;
	// Prevents writes beyond the caller's buffer.
	std::size_t capacity;
	// Identifies the next payload bit, including a partially filled byte.
	std::size_t bitOffset;
	// A flushed payload is immutable and ready for transport.
	bool flushed;
};

class ReplicationReader
{
public:
	// Reads one bit-packed provider payload without taking ownership of its source buffer.
	ReplicationReader(const void *buffer, std::size_t size);

	// Copies opaque bytes from the current bit position.
	bool Read(void *destination, std::size_t size) const;

	// Deserializes an integral value written with its native width.
	template<class T>
	typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, bool>::type
		ReadValue(T &value) const
	{
		return ReadValue(value, static_cast<unsigned int>(sizeof(T) * 8));
	}

	// Deserializes a compact integral field and restores signed two's-complement values.
	template<class T>
	typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, bool>::type
		ReadValue(T &value,
		unsigned int bitCount) const
	{
		if (!bitCount || bitCount > sizeof(T) * 8)
			return false;

		std::uint64_t decoded = 0;
		if (!ReadBits(decoded, bitCount))
			return false;

		typedef typename std::make_unsigned<T>::type UnsignedType;
		UnsignedType converted = static_cast<UnsignedType>(decoded);
		if (std::is_signed<T>::value && bitCount < sizeof(T) * 8 &&
			(decoded & (std::uint64_t(1) << (bitCount - 1)))) {
			converted |= static_cast<UnsignedType>(~UnsignedType(0) << bitCount);
		}
		value = static_cast<T>(converted);
		return true;
	}
	bool ReadValue(bool &value) const
	{
		std::uint8_t encoded = 0;
		if (!ReadValue(encoded, 1))
			return false;
		value = encoded != 0;
		return true;
	}

	// Deserializes floating-point values without changing their IEEE representation.
	bool ReadValue(float &value) const;
	bool ReadValue(double &value) const;

	// Accepts only an exactly consumed payload with zero-valued final padding bits.
	bool Finish() const;

private:
	bool ReadBits(std::uint64_t &value, unsigned int bitCount) const;

	// Points to the immutable caller-owned payload buffer.
	const std::uint8_t *buffer;
	// Bounds every read operation in bytes.
	std::size_t size;
	// Identifies the next payload bit, including a partially consumed byte.
	mutable std::size_t bitOffset;
};
