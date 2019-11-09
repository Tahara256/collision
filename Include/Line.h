#pragma once

#include <LMath.h>

/// <summary> 直線クラス </summary>
class Line {

public:

	/// <summary> コンストラクタ </summary>
	/// <param name="position"> 直線上の一点 </param>
	/// <param name="vector"> 方向ベクトル </param>
	Line(const Vector3 & position, const Vector3 & vector);

	/// <summary> 直線上の一点を取得します。 </summary>
	const Vector3 position() const;

	/// <summary> 方向ベクトルを取得します。 </summary>
	const Vector3 vector() const;

	/// <summary> 正規化された方向ベクトルを取得します。 </summary>
	const Vector3 unitVector() const;

	/// <summary> 直線上の任意の点を取得します。 </summary>
	const Vector3 point(float distance) const;

	/// <summary> 点を通る垂線の足を取得 </summary>
	const Vector3 perpendicularFoot(const Vector3 & point) const;

	/// <summary> 各直線のもう片方の直線に最も近い点をそれぞれ取得します。 </summary>
	void closest(const Line & other, Vector3 * outMyClosestPoint, Vector3 * outOtherClosestPoint) const;

	/// <summary> 二つの直線が平行かを取得します。 </summary>
	bool isParallel(const Line & other) const;

	/// <summary> 直線と点の距離を取得します。 </summary>
	float distance(const Vector3 & point) const;

	/// <summary> 直線と点の距離の二乗を取得します。 </summary>
	float sqrDistance(const Vector3 & point) const;

private:

	Vector3 _position;
	Vector3 _vector;

};
