#pragma once

#include <Collider/Implementation/SpatialPartition/CalcLinearTreeIndex.h>
#include <Collider/Implementation/CollisionUtility.h>

namespace SpatialPartition {

/// <summary> ���`�R���e�i���g�����؍\�� </summary>
template <typename Node, int32 BranchCount>
class LinearTree {

public:

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="level"> ��ԕ������x��(��ԕ�����) </param>
	LinearTree(int32 level) : _indexGetter(level), _size(_indexGetter.size()) {
		_cells.resize(_size);
	}

	/// <summary> ���`�R���e�i�̃C���f�b�N�X�����Ԃ��擾���܂��B </summary>
	Node & operator [] (int32 index) {
		return _cells[index];
	}

	/// <summary> ���`�R���e�i�̃C���f�b�N�X�����Ԃ��擾���܂��B </summary>
	const Node & operator [] (int32 index) const {
		return _cells[index];
	}

	/// <summary> ���[�g�������Ƌ�Ԃ̃��x�������Ԃ��擾���܂��B </summary>
	Node & getCell(int32 mortonCode, int32 level) {
		return (*this)[_indexGetter.get(mortonCode, level)];
	}

	/// <summary> ���[�g�������Ƌ�Ԃ̃��x�������Ԃ��擾���܂��B </summary>
	const Node & getCell(int32 mortonCode, int32 level) const {
		return (*this)[_indexGetter.get(mortonCode, level)];
	}

	/// <summary> ���[�g�������Ƌ�Ԃ̃��x��������`�R���e�i�̃C���f�b�N�X���擾���܂��B </summary>
	int32 getIndex(int32 mortonCode, int32 level) const {
		return _indexGetter.get(mortonCode, level);
	}

	/// <summary> ��Ԃ̐����擾���܂��B </summary>
	int32 getSize() const {
		return _size;
	}

	/// <summary> �e�̃C���f�b�N�X����q�̃C���f�b�N�X���擾���܂��B </summary>
	/// <param name="parentIndex"> �e�̃C���f�b�N�X </param>
	/// <param name="localChildIndex"> �ǂ̎q�̃C���f�b�N�X���擾���邩 </param>
	int32 getChildIndex(int32 parentIndex, int32 localChildIndex) const {
		return parentIndex * BranchCount + 1 + localChildIndex;
	}

	/// <summary> �e�̃C���f�b�N�X���炱�̃C���f�b�N�X���擾���܂��B </summary>
	/// <param name="childIndex"> �q�̃C���f�b�N�X </param>
	int32 getParentIndex(int32 childIndex) const {
		return (childIndex - 1) / BranchCount;
	}

private:

	// ���`�R���e�i�ɕ���������Ԃ�ێ�
	std::vector<Node> _cells;

	// �����؂̃C���f�b�N�X������`�R���e�i�̃C���f�b�N�X���v�Z����N���X
	CalcLinearTreeIndex<BranchCount> _indexGetter;

	// ���`�R���e�i�̃T�C�Y
	int32 _size;

};

} // namespace SpatialPartition
