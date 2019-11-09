#pragma once

#include <Collider/CollisionFilter/CollisionFilter.h>
#include <unordered_map>
#include <Collider/CollisionFilter/CollisionLayerType.h>

/// <summary> 衝突判定フィルターの通常の実装 </summary>
class NormalCollisionFilter : public CollisionFilter {

public:

	/// <summary> 指定されたレイヤーを持つコライダー間の衝突判定を有効にします。 </summary>
	void enableCollisionBetween(CollisionLayerType layer1, CollisionLayerType layer2);

	/// <summary> 指定されたレイヤーを持つコライダー間の衝突判定を無効にします。 </summary>
	void disableCollisionBetween(CollisionLayerType layer1, CollisionLayerType layer2);


	/*-- CollisionFilterの純粋仮想の実装 --*/

	/// <summary> 二つのレイヤーのコライダーを衝突判定するかを取得します。 </summary>
	virtual std::pair<bool, bool> filter(const ColliderPtr & layer1, const ColliderPtr & layer2) const override;

private:

	virtual bool globalFilter(CollisionLayerType layer1, CollisionLayerType layer2) const;

	// コリジョンレイヤーのマトリックス
	using FilterTable = std::unordered_map<CollisionLayerType, CollisionLayerType>;

	FilterTable _table;
};