#pragma once

#include <LMath.h>

class Line;
class AABB;

/// <summary> 線分クラス </summary>
class Segment {

public:

	/// <summary> コンストラクタ </summary>
	/// <param name="start"> 始点 </param>
	/// <param name="end"> 終点 </param>
	Segment(const Vector3 & start, const Vector3 & end);

	/// <summary> デフォルトコンストラクタ </summary>
	Segment() = default;

	/// <summary> 始点を取得します。 </summary>
	const Vector3 start() const;

	/// <summary> 終点を取得します。 </summary>
	const Vector3 end() const;

	/// <summary> 線分の中点を取得します。 </summary>
	const Vector3 midpoint() const;

	/// <summary> 始点から終点へのベクトルを取得します。 </summary>
	const Vector3 vector() const;

	/// <summary> 始点から中点へのベクトルを取得します。 </summary>
	const Vector3 halfVector() const;

	/// <summary> 正規化された方向ベクトルを取得します。 </summary>
	const Vector3 unitVector() const;

	/// <summary> 線分の長さを取得します。 </summary>
	float length() const;

	/// <summary> 線分の長さの二乗を取得します。 </summary>
	float sqrLength() const;

	/// <summary> 始点からの距離から線分上の点を取得します。 </summary>
	const Vector3 pointFromDistance(float distance) const;

	/// <summary> この線分を含む直線を取得します。 </summary>
	const Line asLine() const;

	/// <summary> 平行移動した線分を取得します。 </summary>
	const Segment translate(const Vector3 & vector) const;

	/// <summary> 任意の点を中心に回転させた線分を取得します。 </summary>
	const Segment rotate(const Quaternion & rotation, const Vector3 & origin) const;

	/// <summary> 原点を中心に回転させた線分を取得します。 </summary>
	const Segment rotateOrigin(const Quaternion & rotation) const;

	/// <summary> 始点を中心に回転させた線分を取得します。 </summary>
	const Segment rotateStart(const Quaternion & rotation) const;

	/// <summary> 線分同士の一番近い二点を取得します。 </summary>
	void closest(const Segment & other, Vector3 * outMyShortest, Vector3 * outOtherShortest) const;

	/// <summary> 点に最も近い線分上の座標を取得します。 </summary>
	const Vector3 closest(const Vector3 & point) const;

	/// <summary> 自身と相手の最も近い端点を取得します。 </summary>
	void closestEnd(const Segment & other, Vector3 * myEnd, Vector3 * otherEnd) const;

	/// <summary> 点を通る垂線の足を取得 </summary>
	const Vector3 perpendicularFoot(const Vector3 & point) const;

	/// <summary> 点から下ろした垂線が線分の範囲内かを取得します。 </summary>
	bool inRangePerpendicularFoot(const Vector3 & point, Vector3 * outFootPoint = nullptr) const;

	/// <summary> 二つの線分が平行かを取得します。 </summary>
	bool isParallel(const Segment & segment) const;

	/// <summary> 点と線分の距離を取得します。 </summary>
	float distance(const Vector3 & point, Vector3 * outClosestPoint) const;

	/// <summary> 点と線分の距離の二乗を取得します。 </summary>
	float sqrDistance(const Vector3 & point, Vector3 * outClosestPoint) const;

	/// <summary> 自身が平面の外側に存在するかを取得します。 </summary>
	bool isPlaneOutside(const Vector3 & planeNormal, const Vector3 & planePoint) const;

	/// <summary> 自身を完全に覆うAABBを取得します。 </summary>
	const AABB coverAABB() const;

private:

	Vector3 _start;
	Vector3 _end;

};
