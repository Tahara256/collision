#pragma once

#include <LMath.h>

class AABB;

/// <summary> 球体クラス </summary>
class Sphere {

public:

	/// <summary> コンストラクタ </summary>
	/// <param name="center"> 球体の中心座標 </param>
	/// <param name="radius"> 球体の半径 </param>
	Sphere(const Vector3 & center, float radius);

	/// <summary> 球体の中心座標を取得します。 </summary>
	const Vector3 & center() const;

	/// <summary> 球体の半径を取得します。 </summary>
	float radius() const;

	/// <summary> 自身を完全に覆うAABBを取得します。 </summary>
	const AABB coverAABB() const;

private:

	Vector3 _center;
	float _radius;

};
