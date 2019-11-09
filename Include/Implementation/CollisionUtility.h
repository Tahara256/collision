#pragma once

#include <array>

#include <LMath.h>
#include <Collider/Face.h>
#include <Collider/Segment.h>
#include <Util/Type.h>
#include <Collider/Contact/LinecastContactList.h>
#include <Collider/Collider/ColliderPtr.h>

class Segment;
class AABB;
class OBB;
class UniformGridMesh;
struct Vector3;

namespace CollisionUtility {

/// <summary> 値を[0, 1]の範囲に押し込みます。 </summary>
float saturate(float value);

/// <summary> 範囲内かを取得します。 </summary>
bool inRange(float value, float min, float max);

/// <summary> [0, 1]の範囲内かを取得します。 </summary>
bool inRange01(float value);

/// <summary> 任意軸に任意ベクトルを投影します。 </summary>
float projectVector(const Vector3 & axis, const Vector3 & v1);

/// <summary> 任意軸に任意ベクトルを投影します。 </summary>
float projectVector(const Vector3 & axis, const Vector3 & v1, const Vector3 & v2);

/// <summary> 任意軸に任意ベクトルを投影します。 </summary>
float projectVector(const Vector3 & axis, const Vector3 & v1, const Vector3 & v2, const Vector3 & v3);

/// <summary> 内積の結果の符号を取得します。 </summary>
float dotSign(const Vector3 & v1, const Vector3 & v2);

/// <summary> 平面上に点を投影します。 </summary>
const Vector3 projectPointOnPlane(const Vector3 & planeUnitNormal, const Vector3 & planePoint, const Vector3 & point);

/// <summary> ビットを二つ飛ばしにします。 </summary>
int32 bitTwoSkipping(int8 n);

/// <summary> 座標からモートン符号を取得します。 </summary>
int32 mortonCode(const Vector3 & position, const Vector3 & min, const Vector3 & unitSize, int8 max);

/// <summary> 最小レベル空間の幅を1とした座標系の点を用いてモートン符号を算出します。 </summary>
int32 mortonCode(int8 x, int8 y, int8 z);

/// <summary> 二つのモートン符号と分割レベルから共通レベルを取得します。 </summary>
int32 commonLevel3D(int32 level, int32 mortonCode1, int32 mortonCode2);

/// <summary> 二つのモートン符号と分割レベルからモートン符号と所属レベルを取得します。 </summary>
void commonMortonCodeAndBelongsLevel3D(int32 level, int32 mortonCode1, int32 mortonCode2, int32 * outCommonMortonCode, int32 * outBelongsLevel);

/// <summary> AABBからモートン符号を取得します。 </summary>
void mortonCodeAndBelongsLevel3D(const AABB & aabb, int32 level, const Vector3 & min, const Vector3 & unitSize, int8 max, int32 * outMortonCode, int32 * outBelongsLevel);

void TerrainCollisionDetail(const ColliderPtr & collider, const UniformGridMesh & mesh, Vector3 * outColliderHitPos, Vector3 * outMeshHitPos);

template <std::size_t VertexCount>
const AABB coverAABBFromFace(const Face<VertexCount> & face) {
	auto minInit = (std::numeric_limits<float>::max)();
	auto min = Vector3(minInit, minInit, minInit);
	auto maxInit = -minInit;
	auto max = Vector3(maxInit, maxInit, maxInit);
	for (auto i = 0; i < VertexCount; i++) {
		min.x = Mathf::cmin(min.x, face[i].x);
		min.y = Mathf::cmin(min.y, face[i].y);
		min.z = Mathf::cmin(min.z, face[i].z);
		max.x = Mathf::cmax(max.x, face[i].x);
		max.y = Mathf::cmax(max.y, face[i].y);
		max.z = Mathf::cmax(max.z, face[i].z);
	}
	return { min, max };
}

template <std::size_t VertexCount>
const float dotMin(const Face<VertexCount> & face, const Vector3 & v) {
	auto min = (std::numeric_limits<float>::max)();
	for (const auto & vertex : face) {
		const auto temp = Vector3::dot(v, vertex);
		if (temp > min) continue;
		min = temp;
	}
	return min;
}

template <std::size_t VertexCount>
const Vector3 dotMinVertex(const Face<VertexCount> & face, const Vector3 & v) {
	auto min = (std::numeric_limits<float>::max)();
	auto minVertex = Vector3();
	for (const auto & vertex : face) {
		const auto temp = Vector3::dot(v, vertex);
		if (temp > min) continue;
		min = temp;
		minVertex = vertex;
	}
	return minVertex;
}

template <std::size_t VertexCount>
const float dotMax(const Face<VertexCount> & face, const Vector3 & v) {
	auto max = -(std::numeric_limits<float>::max)();
	for (const auto & vertex : face) {
		const auto temp = Vector3::dot(v, vertex);
		if (temp < max) continue;
		max = temp;
	}
	return max;
}

template <std::size_t VertexCount>
const Vector3 dotMaxVertex(const Face<VertexCount> & face, const Vector3 & v) {
	auto max = -(std::numeric_limits<float>::max)();
	auto maxVertex = Vector3();
	for (const auto & vertex : face) {
		const auto temp = Vector3::dot(v, vertex);
		if (temp < max) continue;
		max = temp;
		maxVertex = vertex;
	}
	return maxVertex;
}

/// <summary> 面から辺のベクトルを取得します。 </summary>
template <std::size_t VertexCount>
const Vector3 edgeVectorFromFace(const Face<VertexCount> & face, std::size_t index) {
	return face[(index + 1) % (VertexCount)] - face[index];
}

/// <summary> 面から辺を取得します。 </summary>
template <std::size_t VertexCount>
const Segment edgeFromFace(const Face<VertexCount> & face, std::size_t index) {
	return Segment(face[index], face[index] + edgeVectorFromFace(face, index));
}

/// <summary>
/// <para>面から法線ベクトルを取得します。</para>
/// <para>取得する法線ベクトルは単位長とは限りません。</para>
/// </summary>
template <std::size_t VertexCount>
const Vector3 faceNormal(const Face<VertexCount> & face) {
	return Vector3::cross(face[1] - face[0], face[2] - face[1]);
}

/// <summary> 面から単位長の法線ベクトルを取得します。 </summary>
template <std::size_t VertexCount>
const Vector3 faceUnitNormal(const Face<VertexCount> & face) {
	return Vector3::cross(face[1] - face[0], face[2] - face[1]).normalize();
}

/// <summary>
/// <para>面に点を投影したときに、面上に点が存在するかを取得します。</para>
/// <para>存在する場合は投影した点も取得できます。</para>
/// <para>面の頂点の順番は時計回りです。</para>
/// </summary>
template <std::size_t VertexCount>
bool tryProjectPointOnFace(const Face<VertexCount> & face, const Vector3 & point, Vector3 * outProjectPoint = nullptr) {
	return tryProjectPointOnFaceFast(face, faceUnitNormal(face), point, outProjectPoint);
}

/// <summary>
/// <para>面に点を投影したときに、面上に点が存在するかを取得します。</para>
/// <para>存在する場合は投影した点も取得できます。</para>
/// <para>面の頂点の順番は時計回りです。</para>
/// </summary>
template <std::size_t VertexCount>
bool tryProjectPointOnFace(const Face<VertexCount> & face, const Vector3 & faceNormal, const Vector3 & point, Vector3 * outProjectPoint = nullptr) {
	return tryProjectPointOnFaceFast(face, faceNormal.normalize(), point, outProjectPoint);
}

/// <summary> 法線がすでに単位長の場合 </summary>
template <std::size_t VertexCount>
bool tryProjectPointOnFaceFast(const Face<VertexCount> & face, const Vector3 & faceUnitNormal, const Vector3 & point, Vector3 * outProjectPoint = nullptr) {
	const auto project = projectPointOnPlane(faceUnitNormal, face[0], point);

	for (auto i = 0; i < VertexCount; i++) {
		const auto edgeNormal = Vector3::cross(edgeVectorFromFace(face, i), faceUnitNormal);
		const auto check = Vector3::dot(edgeNormal, project - face[i]);
		if (check > 0.0f) return false;
	}

	if (outProjectPoint) *outProjectPoint = project;
	return true;
}

} // namespace CollisionUtility
