#pragma once

#include "IntersectAgent.h"

/// <summary> 球体と三角形の衝突処理クラス </summary>
class SphereTriangleAgent final : public IntersectAgent {

public:

	bool intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const override;

	bool intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const override;

};
