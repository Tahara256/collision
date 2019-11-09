#pragma once

#include <Collider/Collider/ColliderPtr.h>

/// <summary> 衝突判定のフィルターインタフェース </summary>
class CollisionFilter {

public:

	/// <summary> 二つのレイヤーのコライダーを衝突判定するかを取得します。 </summary>
	virtual std::pair<bool, bool> filter(const ColliderPtr & layer1, const ColliderPtr & layer2) const = 0;

	virtual ~CollisionFilter() = default;

};
