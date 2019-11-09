#pragma once

#include <Collider/Implementation/SpatialPartition/Power.h>

namespace SpatialPartition {

/// <summary> モートン符号と所属空間レベルから線形コンテナのインデックスを取得するクラス </summary>
template <int32 BranchCount>
class CalcLinearTreeIndex {

public:

	CalcLinearTreeIndex(int32 partitionLevel) : _level(partitionLevel), _pow(partitionLevel) {}

	int32 get(int32 mortonCode, int32 level) const {
		// 初項1, 公比4の等比級数の和
		return mortonCode + (_pow[level] - 1) / (BranchCount - 1);
	}

	int32 size() const {
		return (_pow[_level] * BranchCount - 1) / (BranchCount - 1);
	}

private:

	int32 _level;

	Power<BranchCount> _pow;

};

} // namespace SpatialPartition
