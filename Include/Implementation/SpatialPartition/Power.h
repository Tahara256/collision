#pragma once

#include <vector>

#include <Util/Type.h>

namespace SpatialPartition {

/// <summary> �ݏ�v�Z�����O�ɍs���N���X </summary>
template <int32 Base>
class Power {

public:

	Power(int32 maxExponent) {
		_results.resize(maxExponent + 1);

		_results[0] = 1;

		for (auto i = int32(1); i <= maxExponent; i++)
			_results[i] = _results[i - 1] * Base;
	}

	int32 operator [] (int32 exponent) const {
		return _results[exponent];
	}

private:

	std::vector<int32> _results;

};

} // namespace SpatialPartition
