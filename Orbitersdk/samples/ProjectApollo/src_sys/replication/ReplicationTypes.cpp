#include "ReplicationTypes.h"

#include <cstring>

ReplicationWriter::ReplicationWriter(void *buffer, std::size_t capacity) : buffer(static_cast<std::uint8_t *>(buffer)), capacity(capacity), bitOffset(0), flushed(false), failed(false) {}

ReplicationWriter &ReplicationWriter::WriteBytes(const void *data, std::size_t dataSize)
{
	if (failed)
		return *this;
	if (!data && dataSize) {
		failed = true;
		return *this;
	}

	const std::uint8_t *bytes = static_cast<const std::uint8_t *>(data);
	const std::size_t originalBitOffset = bitOffset;
	for (std::size_t index = 0; index < dataSize; index++) {
		if (!WriteBits(bytes[index], 8)) {
			bitOffset = originalBitOffset;
			failed = true;
			return *this;
		}
	}
	return *this;
}

ReplicationWriter &ReplicationWriter::WriteScalar(float value)
{
	if (failed)
		return *this;
	std::uint32_t bits = 0;
	std::memcpy(&bits, &value, sizeof(bits));
	WriteScalar(bits);
	return *this;
}

ReplicationWriter &ReplicationWriter::WriteScalar(double value)
{
	if (failed)
		return *this;
	std::uint64_t bits = 0;
	std::memcpy(&bits, &value, sizeof(bits));
	WriteScalar(bits);
	return *this;
}

bool ReplicationWriter::WriteBits(std::uint64_t value, unsigned int bitCount)
{
	if (flushed || !bitCount || bitCount > 64 || !buffer || bitCount > capacity * 8 - bitOffset)
		return false;

	const bool byteAligned = (bitOffset % 8) == 0 && (bitCount % 8) == 0;
	if (byteAligned) {
		const std::size_t byteOffset = bitOffset / 8;
		const unsigned int byteCount = bitCount / 8;
		for (unsigned int index = 0; index < byteCount; index++)
			buffer[byteOffset + index] = static_cast<std::uint8_t>(value >> (index * 8));
		bitOffset += bitCount;
		return true;
	}

	for (unsigned int index = 0; index < bitCount; index++, bitOffset++) {
		const std::size_t byteIndex = bitOffset / 8;
		const unsigned int bitIndex = static_cast<unsigned int>(bitOffset % 8);
		if (!bitIndex)
			buffer[byteIndex] = 0;
		if (value & (std::uint64_t(1) << index))
			buffer[byteIndex] |= static_cast<std::uint8_t>(1u << bitIndex);
	}
	return true;
}

void ReplicationWriter::Flush()
{
	if (failed)
		return;
	const unsigned int usedBits = static_cast<unsigned int>(bitOffset % 8);
	if (usedBits)
		buffer[bitOffset / 8] &= static_cast<std::uint8_t>((1u << usedBits) - 1);
	flushed = true;
}

std::size_t ReplicationWriter::Size() const
{
	return (bitOffset + 7) / 8;
}

std::size_t ReplicationWriter::Capacity() const
{
	return capacity;
}

ReplicationReader::ReplicationReader(const void *buffer, std::size_t size) : buffer(static_cast<const std::uint8_t *>(buffer)), size(size), bitOffset(0), failed(false) {}

const ReplicationReader &ReplicationReader::ReadBytes(void *destination, std::size_t destinationSize) const
{
	if (failed)
		return *this;
	if (!destination && destinationSize) {
		failed = true;
		return *this;
	}

	std::uint8_t *bytes = static_cast<std::uint8_t *>(destination);
	const std::size_t originalBitOffset = bitOffset;
	for (std::size_t index = 0; index < destinationSize; index++) {
		std::uint64_t byte = 0;
		if (!ReadBits(byte, 8)) {
			bitOffset = originalBitOffset;
			failed = true;
			return *this;
		}
		bytes[index] = static_cast<std::uint8_t>(byte);
	}
	return *this;
}

const ReplicationReader &ReplicationReader::ReadScalar(float &value) const
{
	if (failed)
		return *this;
	std::uint32_t bits = 0;
	ReadScalar(bits);
	if (!failed)
		std::memcpy(&value, &bits, sizeof(value));
	return *this;
}

const ReplicationReader &ReplicationReader::ReadScalar(double &value) const
{
	if (failed)
		return *this;
	std::uint64_t bits = 0;
	ReadScalar(bits);
	if (!failed)
		std::memcpy(&value, &bits, sizeof(value));
	return *this;
}

bool ReplicationReader::ReadBits(std::uint64_t &value, unsigned int bitCount) const
{
	if (!bitCount || bitCount > 64 || !buffer || bitCount > size * 8 - bitOffset)
		return false;

	const bool byteAligned = (bitOffset % 8) == 0 && (bitCount % 8) == 0;
	if (byteAligned) {
		const std::size_t byteOffset = bitOffset / 8;
		const unsigned int byteCount = bitCount / 8;
		std::uint64_t decoded = 0;
		for (unsigned int index = 0; index < byteCount; index++)
			decoded |= std::uint64_t(buffer[byteOffset + index]) << (index * 8);
		bitOffset += bitCount;
		value = decoded;
		return true;
	}

	std::uint64_t decoded = 0;
	for (unsigned int index = 0; index < bitCount; index++) {
		const std::size_t sourceBit = bitOffset + index;
		if (buffer[sourceBit / 8] & (1u << (sourceBit % 8)))
			decoded |= std::uint64_t(1) << index;
	}
	bitOffset += bitCount;
	value = decoded;
	return true;
}

bool ReplicationReader::Finish() const
{
	if (failed)
		return false;
	const std::size_t consumedBytes = (bitOffset + 7) / 8;
	if (consumedBytes != size) {
		failed = true;
		return false;
	}

	const unsigned int usedBits = static_cast<unsigned int>(bitOffset % 8);
	if (usedBits && (buffer[size - 1] & static_cast<std::uint8_t>(0xffu << usedBits))) {
		failed = true;
		return false;
	}

	return true;
}
