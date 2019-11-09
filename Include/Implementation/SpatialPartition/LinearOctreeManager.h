#pragma once

#include <list>

#include <LMath.h>
#include <Collider/AABB.h>
#include <Collider/Implementation/SpatialPartition/LinearTree.h>
#include <Collider/Implementation/SpatialPartition/SpaceCell.h>

namespace SpatialPartition {

/// <summary> 線形八分木を管理するクラス </summary>
class LinearOctreeManager {

public:

	using CollisionPair = std::pair<OctreeObjectPtr, OctreeObjectPtr>;
	using CollisionPairList = std::list<CollisionPair>;
	using CollisionList = std::list<OctreeObjectPtr>;
	using CollisionStack = std::list<OctreeObjectPtr>;

	/// <summary> コンストラクタ </summary>
	/// <param name="area"> 八分木に管理させる空間を表すAABB </param>
	/// <param name="level"> 空間分割数(この回数だけ空間を八等分する) </param>
	LinearOctreeManager(const AABB & area, int32 level);

	/// <summary> 八分木で管理するオブジェクトを追加します。 </summary>
	void addObject(const OctreeObjectPtr & object);

	/// <summary> 八分木で管理するオブジェクトを削除します。 </summary>
	void removeObject(const OctreeObjectPtr & object);

	/// <summary> 八分木で管理するオブジェクトを全削除します。 </summary>
	void clearObject();

	/// <summary> 静的ではないオブジェクトを八分木に再配置します。 </summary>
	void update();

	/// <summary> 衝突している可能性のあるオブジェクトのペアのリストを取得します。 </summary>
	/// <param name="outCollisionPairList"> 出力してほしいリストへのポインタ </param>
	void getCollisionPairList(CollisionPairList * outCollisionPairList) const;

	/// <summary> AABBをキャストし、衝突している可能性のあるオブジェクトのリストを取得します。 </summary>
	/// <param name="coverAABB"> キャストするAABB </param>
	/// <param name="outCollisionList"> 衝突している可能性のあるオブジェクトを出力するリストへのポインタ </param>
	void traverseAABB(const AABB & coverAABB, CollisionList * outCollisionList) const;

	/// <summary> 八分木が管理する空間を立方体のワイヤーフレームで描画 </summary>
	void drawWireFrame(const Vector4 & color) const;

private:

	static constexpr int32 BranchCount = 8;

	void updateObject(const OctreeObjectPtr & object);

	void detachObject(const OctreeObjectPtr & object);

	/// <summary> 再帰で八分木の全ノードを巡回し、衝突する可能性のあるオブジェクトのペアをリストに詰めます。 </summary>
	/// <param name="index"> 現在のノード </param>
	/// <param name="colliderList"> 詰めるリスト </param>
	/// <param name="colliderStac"> 現在のノードと衝突している可能性のあるオブジェクトのスタック </param>
	void createCollisionPairList(int32 index, CollisionPairList * colliderList, CollisionStack * colliderStac) const;

	void createCollisionListAll(int32 index, CollisionList * outCollisionList) const;

	void createCollisionList(int32 index, CollisionList * outCollisionList) const;

	void createCollisionListChild(int32 index, CollisionList * outCollisionList) const;

	void createCollisionListParent(int32 index, CollisionList * outCollisionList) const;

	void cellSetUp(int32 cellIndex);

	LinearTree<SpaceCell, BranchCount>	_octree;
	Vector3								_unitSize;
	int32								_level;
	AABB								_area;
	int8								_maxIndex;
	std::list<OctreeObjectPtr>			_objects;

};

} // namespace SpatialPartition
