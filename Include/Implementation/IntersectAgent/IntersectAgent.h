#pragma once

#include "../../Collider/ColliderPtr.h"

struct Vector3;

/// <summary> 衝突判定代理クラス </summary>
class IntersectAgent {

public:

	/// <summary> 衝突判定をします。コライダーの種類毎に適切な具象クラスを使用してください。 </summary>
	/// <param name="collider1"> 一つ目のコライダー </param>
	/// <param name="collider2"> 二つ目のコライダー </param>
	virtual bool intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const = 0;

	/// <summary> トリガーの接触判定をします。コライダーの種類ごとに適切な具象クラスを使用してください。 </summary>
	/// <param name="collider1"> 一つ目のコライダー </param>
	/// <param name="collider2"> 二つ目のコライダー </param>
	virtual bool intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const = 0;


	virtual ~IntersectAgent() = default;

};
