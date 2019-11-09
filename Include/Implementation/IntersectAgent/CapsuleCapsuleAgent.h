#pragma once

#include "IntersectAgent.h"

/// <summary> カプセルとカプセルの衝突処理クラス </summary>
class CapsuleCapsuleAgent final : public IntersectAgent {

public:

	bool intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const override;

	bool intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const override;

};
