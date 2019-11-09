#pragma once

#include <list>
#include <unordered_map>
#include <functional>

#include <Util/Singleton.h>
#include <Collider/Collider/ColliderPtr.h>
#include <Collider/Collider/TerrainColliderPtr.h>
#include <Collider/CollisionFilter/CollisionFilterPtr.h>
#include <Collider/Implementation/SpatialPartition/LinearOctreeManager.h>
#include <Collider/Contact/LinecastContactList.h>
#include <Collider/CollisionFilter/CollisionLayerType.h>
#include <Collider/Collider/ColliderID.h>

class Vector4;
class Segment;
struct Vector3;

/// <summary> 衝突管理クラス </summary>
class CollisionManager : public Accessor<CollisionManager> {

public:

	/// <summary> デフォルトコンストラクタ </summary>
	CollisionManager();
	~CollisionManager();

	/// <summary> コライダーを登録します。 </summary>
	void addCollider(const ColliderPtr & collider);

	/// <summary> コライダーを削除します。 </summary>
	void removeCollider(const ColliderPtr & collider);

	/// <summary> コライダーを全削除します。 </summary>
	void clear();

	/// <summary> 衝突判定をします。 </summary>
	void update();

	/// <summary> IDからコライダーを取得します。 </summary>
	const ColliderPtr getCollider(ColliderID id) const;

	/// <summary> 登録されているコライダーをワイヤーフレームで描画します。 </summary>
	void drawWireFrame(const Vector4 & color) const;

	/// <summary> シーン上にラインをキャストします。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="outInfo"> 接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecast(const Segment & line, LinecastContact * outInfo = nullptr, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストします。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="layerMask"> 衝突判定のマスク </param>
	/// <param name="outInfo"> 接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecast(const Segment & line, CollisionLayerType layerMask, LinecastContact * outInfo = nullptr, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストし、一番近いオブジェクトとの接触情報を取得します。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="outInfo"> 一番近い接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecastClosest(const Segment & line, LinecastContact * outInfo, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストし、一番近いオブジェクトとの接触情報を取得します。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="layerMask"> 衝突判定のマスク </param>
	/// <param name="outInfo"> 一番近い接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecastClosest(const Segment & line, CollisionLayerType layerMask, LinecastContact * outInfo, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストします。全ての衝突結果を取得します。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="outInfos"> 接触した全ての接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecastAll(const Segment & line, LinecastContactList * outInfos, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストします。全ての衝突結果を取得します。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="layerMask"> 衝突判定のマスク </param>
	/// <param name="outInfos"> 接触したすべての接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecastAll(const Segment & line, CollisionLayerType layerMask, LinecastContactList * outInfos, bool ignoreTrigger = true) const;

	/// <summary> 衝突判定のフィルターを設定します。 </summary>
	void setCollisionFilter(CollisionFilterPtr && collisionFilter);

private:

	using ColliderList = std::list<ColliderPtr>;
	using ColliderTable = std::unordered_map<ColliderID, ColliderPtr>;

	bool linecast(const Segment & line, const std::function<bool(const ColliderPtr &)> & pre, LinecastContact * outInfo = nullptr) const;

	bool linecastAll(const Segment & line, const std::function<bool(const ColliderPtr &)> & pre, LinecastContactList * outInfos) const;

	/// <summary> 八分木で最適された衝突判定をします。 </summary>
	void octreeCollision();

	/// <summary> 二つのコライダーを衝突判定させます。 </summary>
	void collision(const ColliderPtr & collider1, const ColliderPtr & collider2) const;


	SpatialPartition::LinearOctreeManager	_octree;
	ColliderTable							_colliderTable;
	CollisionFilterPtr						_collisionFilter;

};
