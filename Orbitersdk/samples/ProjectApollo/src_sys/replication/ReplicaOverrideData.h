#pragma once

#include <type_traits>

template<class T>
class ReplicaOverrideData
{
public:
	T &operator*() { return value; }
	const T &operator*() const { return value; }
	T *operator->() { return &value; }
	const T *operator->() const { return &value; }

	template<class Object, class Field>
	Field OverrideIfReplica(Field Object::*member, Field liveValue) const
	{
		static_assert(std::is_same<Object, T>::value, "Member must belong to the replicated value type");
		return active ? value.*member : liveValue;
	}

	bool IsActive() const { return active; }
	void SetActive(bool enabled) { active = enabled; }

private:
	T value{};
	bool active = false;
};
