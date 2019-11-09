#pragma once

#include "Segment.h"

class AABB;

/// <summary> カプセルクラス </summary>
class Capsule
{
public:
	/// <summary> コンストラクタ </summary>
	/// <param name="segment"> カプセルの芯の線分 </param>
	/// <param name="radius"> カプセルの半径 </param>
	Capsule(Segment const & segment, float radius);

	/// <summary> コンストラクタ </summary>
	/// <param name="halfHeight"> カプセルの半分の高さ(丸い部分も含む) </param>
	/// <param name="radius"> カプセルの半径 </param>
	Capsule(float halfHeight, float radius);

	/// <summary> カプセルの芯の線分を取得します。 </summary>
	Segment const segment() const;

	/// <summary> カプセルの半径を取得します。 </summary>
	float radius() const;

	/// <summary> カプセルの長さを取得します。 </summary>
	float height() const;

	/// <summary> カプセルの長さの半分を取得します。 </summary>
	float halfHeight() const;

	/// <summary> 自身を完全に覆うAABBを取得します。 </summary>
	AABB const coverAABB() const;

private:
	Segment _segment;
	float _radius;
};
