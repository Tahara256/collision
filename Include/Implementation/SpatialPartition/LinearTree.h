#pragma once

#include <Collider/Implementation/SpatialPartition/CalcLinearTreeIndex.h>
#include <Collider/Implementation/CollisionUtility.h>

namespace SpatialPartition {

/// <summary> 線形コンテナを使った木構造 </summary>
template <typename Node, int32 BranchCount>
class LinearTree {

public:

	/// <summary> コンストラクタ </summary>
	/// <param name="level"> 空間分割レベル(空間分割数) </param>
	LinearTree(int32 level) : _indexGetter(level), _size(_indexGetter.size()) {
		_cells.resize(_size);
	}

	/// <summary> 線形コンテナのインデックスから空間を取得します。 </summary>
	Node & operator [] (int32 index) {
		return _cells[index];
	}

	/// <summary> 線形コンテナのインデックスから空間を取得します。 </summary>
	const Node & operator [] (int32 index) const {
		return _cells[index];
	}

	/// <summary> モートン符号と空間のレベルから空間を取得します。 </summary>
	Node & getCell(int32 mortonCode, int32 level) {
		return (*this)[_indexGetter.get(mortonCode, level)];
	}

	/// <summary> モートン符号と空間のレベルから空間を取得します。 </summary>
	const Node & getCell(int32 mortonCode, int32 level) const {
		return (*this)[_indexGetter.get(mortonCode, level)];
	}

	/// <summary> モートン符号と空間のレベルから線形コンテナのインデックスを取得します。 </summary>
	int32 getIndex(int32 mortonCode, int32 level) const {
		return _indexGetter.get(mortonCode, level);
	}

	/// <summary> 空間の数を取得します。 </summary>
	int32 getSize() const {
		return _size;
	}

	/// <summary> 親のインデックスから子のインデックスを取得します。 </summary>
	/// <param name="parentIndex"> 親のインデックス </param>
	/// <param name="localChildIndex"> どの子のインデックスを取得するか </param>
	int32 getChildIndex(int32 parentIndex, int32 localChildIndex) const {
		return parentIndex * BranchCount + 1 + localChildIndex;
	}

	/// <summary> 親のインデックスからこのインデックスを取得します。 </summary>
	/// <param name="childIndex"> 子のインデックス </param>
	int32 getParentIndex(int32 childIndex) const {
		return (childIndex - 1) / BranchCount;
	}

private:

	// 線形コンテナに分割した空間を保持
	std::vector<Node> _cells;

	// 八分木のインデックスから線形コンテナのインデックスを計算するクラス
	CalcLinearTreeIndex<BranchCount> _indexGetter;

	// 線形コンテナのサイズ
	int32 _size;

};

} // namespace SpatialPartition
