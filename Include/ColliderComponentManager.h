#pragma once

#include <Util/Design/Singleton.h>
#include <Util/Util.h>
#include <unordered_map>
#include <Collider/Collider/ColliderID.h>
#include <Collider/CollisionFilter/CollisionLayerType.h>
#include <Collider/LinecastHit.h>
#include <Collider/LinecastHitList.h>

class ColliderComponent;
class Vector4;
class Segment;

/// <summary> コライダーコンポーネントを管理するクラス </summary>
class ColliderComponentManager : public Singleton<ColliderComponentManager> {

public:

	/// <summary> シーン上にラインをキャストします。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="outInfo"> 接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecast(const Segment & line, LinecastHit * outInfo = nullptr, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストします。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="layerMask"> 衝突判定のマスク </param>
	/// <param name="outInfo"> 接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecast(const Segment & line, CollisionLayerType layerMask, LinecastHit * outInfo = nullptr, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストし、一番近いオブジェクトとの接触情報を取得します。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="outInfo"> 一番近い接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecastClosest(const Segment & line, LinecastHit * outInfo, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストし、一番近いオブジェクトとの接触情報を取得します。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="layerMask"> 衝突判定のマスク </param>
	/// <param name="outInfo"> 一番近い接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecastClosest(const Segment & line, CollisionLayerType layerMask, LinecastHit * outInfo, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストします。全ての衝突結果を取得します。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="outInfos"> 接触した全ての接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecastAll(const Segment & line, LinecastHitList * outInfos, bool ignoreTrigger = true) const;

	/// <summary> シーン上にラインをキャストします。全ての衝突結果を取得します。 </summary>
	/// <param name="line"> キャストするライン </param>
	/// <param name="layerMask"> 衝突判定のマスク </param>
	/// <param name="outInfos"> 接触したすべての接触情報 </param>
	/// <param name="ignoreTrigger"> トリガーのコライダーを無視するか </param>
	/// <returns> コライダーに接触したか </returns>
	bool linecastAll(const Segment & line, CollisionLayerType layerMask, LinecastHitList * outInfos, bool ignoreTrigger = true) const;

	/// <summary> IDからコライダーコンポーネントを取得します。 </summary>
	RefPtr<ColliderComponent> getColliderComponent(ColliderID id) const;

	/// <summary> 各ColliderComponentを更新します。毎フレーム一回呼び出してください。 </summary>
	void update();

	/// <summary> 登録されているコライダーをワイヤーフレームで描画します。 </summary>
	void drawWireFrame(const Vector4 & color) const;

	/// <summary> コライダーコンポーネントを追加します。 </summary>
	void addColliderComponent(RefPtr<ColliderComponent> colliderComponent);

	/// <summary> コライダーコンポーネントを削除します。 </summary>
	void removeColliderComponent(RefPtr<ColliderComponent> colliderComponent);

private:

	std::unordered_map<ColliderID, RefPtr<ColliderComponent>> _table;

};
