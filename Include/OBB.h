#pragma once

#include <array>

#include <LMath.h>
#include <Collider/Face.h>

class Segment;
class AABB;

/// <summary> 有向バウンディングボックス </summary>
class OBB {

public:

	/// <summary> OBBの頂点数 </summary>
	static constexpr int VertexCount = 8;

	/// <summary> OBBの線分数 </summary>
	static constexpr int SegmentCount = 12;

	/// <summary> OBBの面数 </summary>
	static constexpr int FaceCount = 6;

	/// <summary> コンストラクタ </summary>
	/// <param name="center"> 中心座標 </param>
	/// <param name="size"> ローカルの各軸の幅 </param>
	/// <param name="rotation"> 回転 </param>
	OBB(const Vector3 & center, const Vector3 & size, const Quaternion & rotation);

	/// <summary> 中心座標を取得します。 </summary>
	const Vector3 & center() const;

	/// <summary> x, y, z 方向の大きさ(直径寸法)を取得します。 </summary>
	const Vector3 size() const;

	/// <summary> x, y, z 方向の半分の大きさ(半径寸法)を取得します。 </summary>
	const Vector3 & halfSize() const;

	/// <summary> 回転を取得します。 </summary>
	const Quaternion & rotation() const;

	/// <summary> 逆回転を取得します。 </summary>
	const Quaternion invRotation() const;

	/// <summary> 前方向の単位ベクトルを取得します。 </summary>
	const Vector3 forward() const;

	/// <summary> 右方向の単位ベクトルを取得します。 </summary>
	const Vector3 right() const;

	/// <summary> 上方向の単位ベクトルを取得します。 </summary>
	const Vector3 up() const;

	/// <summary> 中心から前の面へのベクトルを取得します。 </summary>
	const Vector3 toForwardFace() const;

	/// <summary> 中心から右の面へのベクトルを取得します。 </summary>
	const Vector3 toRightFace() const;

	/// <summary> 中心から上の面へのベクトルを取得します。 </summary>
	const Vector3 toUpFace() const;

	/// <summary> ワールド座標からローカル座標に変換します。 </summary>
	const Vector3 inverseTransformPoint(const Vector3 point) const;

	/// <summary> ローカル座標からワールド座標に変換します。 </summary>
	const Vector3 transformPoint(const Vector3 point) const;

	/// <summary> 点までの距離を取得します。 </summary>
	float distance(const Vector3 & point, Vector3 * outClosestPoint = nullptr) const;

	/// <summary> 点までの最短となる座標を取得します。 </summary>
	const Vector3 closestPoint(const Vector3 & point) const;

	/// <summary> 平行移動・回転していない状態の直方体で座標を押し込んだ結果を取得します。 </summary>
	const Vector3 clampLocal(const Vector3 & position) const;

	/// <summary> 点を OBB の範囲内だった場合押し出します。 </summary>
	const Vector3 extrusionPoint(const Vector3 & point) const;

	/// <summary> OBBを構成する線分を取得します。 </summary>
	const Segment segment(int axisIndex, int segIndex) const;

	/// <summary> OBBを構成する線分を取得します。 </summary>
	const Segment segment(int index) const;

	const std::array<Segment, SegmentCount> segments() const;

	/// <summary> 任意のベクトルとの内積が最も小さいOBB上の頂点を取得します。 </summary>
	const Vector3 dotMinVertex(const Vector3 & v) const;

	/// <summary> 任意のベクトルとのない席が最も小さいOBB上の頂点を取得します。 </summary>
	const Vector3 dotMaxVertex(const Vector3 & v) const;

	float dotMin(const Vector3 & v) const;

	float dotMax(const Vector3 & v) const;

	/// <summary> 任意ベクトルとの内積が最も小さい始点を持つOBBの線分を取得します。 </summary>
	const Segment dotMinSegment(int segAxis, const Vector3 & v) const;

	/// <summary> OBBを平行移動 </summary>
	const OBB translate(const Vector3 & move) const;

	/// <summary> 
	/// <para>面を取得します。</para>
	/// <para>面は法線方向から見て時計回りに定義されています。</para>
	/// </summary>
	const Face<4> face(int index) const;

	const std::array<Face<4>, FaceCount> faces() const;

	/// <summary> 頂点を取得します。 </summary>
	const Vector3 vertex(int index) const;

	/// <summary> 頂点すべてを取得します。 </summary>
	const std::array<Vector3, VertexCount> vertices() const;

	/// <summary> 自身を完全に覆うAABBを取得します。 </summary>
	const AABB coverAABB() const;

	/// <summary> 任意軸に対してOBBを投影します。 </summary>
	float project(const Vector3 & axis) const;

private:

	Vector3 _center;
	Vector3 _halfSize;
	Quaternion _rotation;

};
