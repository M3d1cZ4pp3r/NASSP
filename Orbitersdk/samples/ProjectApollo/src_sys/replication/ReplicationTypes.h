#pragma once

#include "../nasspmp_api.h"

#include <type_traits>

using ComponentId = nasspmp_api::replication::ComponentId;
using ReplicationGroupId = nasspmp_api::replication::ReplicationGroupId;
using Revision = nasspmp_api::replication::Revision;
using ReplicationDelivery = nasspmp_api::replication::ReplicationDelivery;
using ReplicationRole = nasspmp_api::replication::ReplicationRole;
using ApplyPurpose = nasspmp_api::replication::ApplyPurpose;

constexpr std::uint32_t InputAuthorityHoldMilliseconds = 150;

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
	// Allows a replica to send locally captured changes to the authority.
	bool clientReplicates = false;
	// Sends a fresh sample at this interval; zero disables periodic transmission.
	std::uint32_t periodicIntervalMs = 0;
	// Sends a sample whenever capture observes a changed payload.
	bool replicateChanges = true;
	// Prevents authority echo from fighting an in-progress local input sequence.
	std::uint32_t inputAuthorityHoldMs = 0;
	// Bounds one capture payload before any ABI or network allocation.
	std::uint32_t maximumPayloadBytes = 0;
};

// Names the timeline that authority and replica share for one session.
//
// Orbiter's simulation time counts from session start and is unrelated between machines,
// so every shared timestamp is derived from the absolute MJD instead. The authority
// announces one epoch, both sides then express instants as seconds since it. Converting a
// frame's own MJD against this epoch yields a time directly comparable with a received
// sample timestamp.
struct ReplicationTimeBase
{
	// Reference date every shared timestamp is measured against.
	double epochMjd = 0.0;
	// False until the session negotiated an epoch, no shared timestamp is usable before then.
	bool valid = false;

	// Converts an Orbiter MJD into session seconds. Both operands lie within a factor of
	// two of each other, so the subtraction itself should introduce no numerical imprecision
	double SessionTime(double mjd) const { return (mjd - epochMjd) * 86400.0; }
};

struct CaptureContext
{
	// Session time of the state being captured. During a PreStep this is the state before
	// the upcoming integration step
	double captureSessionTime = 0.0;
	// False while no session epoch exists, which makes captureSessionTime meaningless.
	bool hasSessionTime = false;
	// Requests the complete initial state used before a replica becomes active.
	bool isBaseline = false;
};

struct ApplyContext
{
	// Distinguishes baseline restoration, authority updates and remote crew input.
	ApplyPurpose purpose = ApplyPurpose::Baseline;
	// Session time at which the authority state in this payload was valid.
	double sampleSessionTime = 0.0;
	// False while no session epoch exists, which makes sampleSessionTime meaningless.
	bool hasSessionTime = false;
};

class ReplicationWriter
{
public:
	// Writes a provider payload bit by bit into a buffer supplied by the caller.
	ReplicationWriter(void *buffer, std::size_t capacity);

	// Appends opaque bytes and retains a failure for the rest of the chain.
	ReplicationWriter &WriteBytes(const void *data, std::size_t size);

	// Serializes an integral value with its native width in canonical
	// least-significant-bit-first order.
	template <class T> typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, ReplicationWriter &>::type WriteScalar(T value) { return WriteScalar(value, static_cast<unsigned int>(sizeof(T) * 8)); }

	// Serializes only the requested low bits after verifying that the value fits.
	template <class T> typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, ReplicationWriter &>::type WriteScalar(T value, unsigned int bitCount)
	{
		if (failed)
			return *this;
		if (!bitCount || bitCount > sizeof(T) * 8) {
			failed = true;
			return *this;
		}

		// Alternative is just cutting off, could be even intended, but could also lead
		// to faulty replication instead of an error if the caller has a bug
		if (std::is_signed<T>::value) {
			const std::int64_t signedValue = static_cast<std::int64_t>(value);
			if (bitCount < 64) {
				const std::int64_t minimum = -(std::int64_t(1) << (bitCount - 1));
				const std::int64_t maximum = (std::int64_t(1) << (bitCount - 1)) - 1;
				if (signedValue < minimum || signedValue > maximum) {
					failed = true;
					return *this;
				}
			}
		} else if (bitCount < 64 && static_cast<std::uint64_t>(value) >= (std::uint64_t(1) << bitCount)) {
			failed = true;
			return *this;
		}

		typedef typename std::make_unsigned<T>::type UnsignedType;
		if (!WriteBits(static_cast<std::uint64_t>(static_cast<UnsignedType>(value)), bitCount))
			failed = true;
		return *this;
	}
	ReplicationWriter &WriteScalar(bool value) { return WriteScalar(static_cast<std::uint8_t>(value ? 1 : 0), 1); }

	// Serializes floating-point values without changing their IEEE representation.
	ReplicationWriter &WriteScalar(float value);
	ReplicationWriter &WriteScalar(double value);

	// Reports whether every operation in the chain succeeded.
	explicit operator bool() const { return !failed; }

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
	// Prevents all operations after the first invalid or out-of-bounds write.
	bool failed;
};

class ReplicationReader
{
public:
	// Reads one bit-packed provider payload without taking ownership of its source buffer.
	ReplicationReader(const void *buffer, std::size_t size);

	// Copies opaque bytes and retains a failure for the rest of the chain.
	const ReplicationReader &ReadBytes(void *destination, std::size_t size) const;

	// Deserializes an integral value written with its native width.
	template <class T> typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, const ReplicationReader &>::type ReadScalar(T &value) const { return ReadScalar(value, static_cast<unsigned int>(sizeof(T) * 8)); }

	// Deserializes a compact integral field and restores signed two's-complement
	// values.
	template <class T> typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, const ReplicationReader &>::type ReadScalar(T &value, unsigned int bitCount) const
	{
		if (failed)
			return *this;
		if (!bitCount || bitCount > sizeof(T) * 8) {
			failed = true;
			return *this;
		}

		std::uint64_t decoded = 0;
		if (!ReadBits(decoded, bitCount)) {
			failed = true;
			return *this;
		}

		typedef typename std::make_unsigned<T>::type UnsignedType;
		UnsignedType converted = static_cast<UnsignedType>(decoded);
		if (std::is_signed<T>::value && bitCount < sizeof(T) * 8 && (decoded & (std::uint64_t(1) << (bitCount - 1)))) {
			converted |= static_cast<UnsignedType>(~UnsignedType(0) << bitCount);
		}
		value = static_cast<T>(converted);
		return *this;
	}
	const ReplicationReader &ReadScalar(bool &value) const
	{
		std::uint8_t encoded = 0;
		ReadScalar(encoded, 1);
		if (!failed)
			value = encoded != 0;
		return *this;
	}

	// Deserializes floating-point values without changing their IEEE
	// representation.
	const ReplicationReader &ReadScalar(float &value) const;
	const ReplicationReader &ReadScalar(double &value) const;

	// Reports whether every operation in the chain succeeded.
	explicit operator bool() const { return !failed; }

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
	// Prevents all operations after the first invalid or out-of-bounds read.
	mutable bool failed;
};
