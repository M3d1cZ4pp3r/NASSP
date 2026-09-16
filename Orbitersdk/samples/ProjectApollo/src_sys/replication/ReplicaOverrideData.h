#pragma once

template<class T>
class ReplicaOverrideData
{
public:
	T &operator*() { return value; }
	const T &operator*() const { return value; }
	T *operator->() { return &value; }
	const T *operator->() const { return &value; }

	template<class Field>
	Field OverrideIfReplica(Field T::*member, Field liveValue) const { return active ? value.*member : liveValue; }

	bool IsActive() const { return active; }
	void SetActive(bool enabled) { active = enabled; }

private:
	T value{};
	bool active = false;
};
