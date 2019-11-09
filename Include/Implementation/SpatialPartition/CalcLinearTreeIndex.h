#pragma once

#include <Collider/Implementation/SpatialPartition/Power.h>

namespace SpatialPartition {

/// <summary> ���[�g�������Ə�����ԃ��x��������`�R���e�i�̃C���f�b�N�X���擾����N���X </summary>
template <int32 BranchCount>
class CalcLinearTreeIndex {

public:

	CalcLinearTreeIndex(int32 partitionLevel) : _level(partitionLevel), _pow(partitionLevel) {}

	int32 get(int32 mortonCode, int32 level) const {
		// ����1, ����4�̓��䋉���̘a
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
