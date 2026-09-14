#include "ReplicationTypes.h"

#include <cstring>

ReplicationWriter::ReplicationWriter(void *buffer, std::size_t capacity) :
	buffer(static_cast<std::uint8_t *>(buffer)),
	capacity(capacity),
	bitOffset(0),
	flushed(false)
{
}

bool ReplicationWriter::Write(const void *data, std::size_t dataSize)
{
	if (!data && dataSize)
		return false;

	const std::uint8_t *bytes = static_cast<const std::uint8_t *>(data);
	const std::size_t originalBitOffset = bitOffset;
	for (std::size_t index = 0; index < dataSize; index++) {
		const bool byteWritten = WriteBits(bytes[index], 8);
		if (!byteWritten) {
			bitOffset = originalBitOffset;
			return false;
		}
	}
	return true;
}

bool ReplicationWriter::WriteValue(float value)
{
	std::uint32_t bits = 0;
	std::memcpy(&bits, &value, sizeof(bits));
	return WriteValue(bits);
}

bool ReplicationWriter::WriteValue(double value)
{
	std::uint64_t bits = 0;
	std::memcpy(&bits, &value, sizeof(bits));
	return WriteValue(bits);
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

ReplicationReader::ReplicationReader(const void *buffer, std::size_t size) :
	buffer(static_cast<const std::uint8_t *>(buffer)),
	size(size),
	bitOffset(0)
{
}

bool ReplicationReader::Read(void *destination, std::size_t destinationSize) const
{
	if (!destination && destinationSize)
		return false;

	std::uint8_t *bytes = static_cast<std::uint8_t *>(destination);
	const std::size_t originalBitOffset = bitOffset;
	for (std::size_t index = 0; index < destinationSize; index++) {
		std::uint64_t byte = 0;
		const bool byteRead = ReadBits(byte, 8);
		if (!byteRead) {
			bitOffset = originalBitOffset;
			return false;
		}
		bytes[index] = static_cast<std::uint8_t>(byte);
	}
	return true;
}

bool ReplicationReader::ReadValue(float &value) const
{
	std::uint32_t bits = 0;
	const bool bitsRead = ReadValue(bits);
	if (!bitsRead)
		return false;
	std::memcpy(&value, &bits, sizeof(value));
	return true;
}

bool ReplicationReader::ReadValue(double &value) const
{
	std::uint64_t bits = 0;
	const bool bitsRead = ReadValue(bits);
	if (!bitsRead)
		return false;
	std::memcpy(&value, &bits, sizeof(value));
	return true;
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
	const std::size_t consumedBytes = (bitOffset + 7) / 8;
	if (consumedBytes != size)
		return false;

	const unsigned int usedBits = static_cast<unsigned int>(bitOffset % 8);
	if (usedBits && (buffer[size - 1] & static_cast<std::uint8_t>(0xffu << usedBits)))
		return false;

	return true;
}
