//
// Created by thomas on 06.03.23.
//

#ifndef PX4_RUNTIMEDEFAULTSTORE_H
#define PX4_RUNTIMEDEFAULTSTORE_H

#include <stdlib.h>
#include "ParamLayer.h"

template <int N_SLOTS>
class StaticSparseLayer : public ParamLayer
{
private:
	struct Slot {
		param_t param;
		param_value_u value;
	};

	Slot _slots[N_SLOTS];
	int _next_slot = 0;


	static int _slotCompare(const void *a, const void *b)
	{
		return ((int)((Slot *)a)->param) - ((int)((Slot *)b)->param);
	}

	void _sort()
	{
		qsort(_slots, N_SLOTS, sizeof(Slot), &_slotCompare);
	}

	int _getIndex(param_t param) const
	{
		int left = 0;
		int right = N_SLOTS - 1;

		while (left <= right) {
			int mid = (left + right) / 2;

			if (_slots[mid].param == param) {
				return mid;

			} else if (_slots[mid].param < param) {
				left = mid + 1;

			} else {
				right = mid - 1;
			}
		}

		return N_SLOTS;
	}

public:

	StaticSparseLayer(ParamLayer &parent) : ParamLayer(parent)
	{
		for (int i = 0; i < N_SLOTS; i++) {
			_slots[i] = {UINT16_MAX, param_value_u{}};
		}
	}

	virtual ~StaticSparseLayer() = default;

	bool store(param_t param, param_value_u value) override
	{
		const AtomicTransaction transaction;

		if (contains(param)) {
			_slots[_getIndex(param)].value = value;

		} else if (_next_slot < N_SLOTS) {
			_slots[_next_slot++] = {param, value};

		} else {
			return false;
		}

		_sort();
		return true;
	}

	bool contains(param_t param) const override
	{
		const AtomicTransaction transaction;
		return _getIndex(param) < N_SLOTS;
	}

	param_value_u get(param_t param) const override
	{
		const AtomicTransaction transaction;
		int index = _getIndex(param);

		if (index < N_SLOTS) {
			return _slots[index].value;

		} else {
			return _parent->get(param);
		}
	}

	void reset(param_t param) override
	{
		const AtomicTransaction transaction;
		int index = _getIndex(param);

		if (index < N_SLOTS) {
			_slots[index] = {UINT16_MAX, param_value_u{}};
			_sort();
			_next_slot--;
		}
	}

	void refresh(param_t param) override
	{
		_parent->refresh(param);
	}

	int size() const override
	{
		return _next_slot;
	}

	int byteSize() const override
	{
		return N_SLOTS * sizeof(Slot);
	}

};

#endif //PX4_RUNTIMEDEFAULTSTORE_H
