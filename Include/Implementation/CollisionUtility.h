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

/// <summary> �l��[0, 1]�͈̔͂ɉ������݂܂��B </summary>
float saturate(float value);

/// <summary> �͈͓������擾���܂��B </summary>
bool inRange(float value, float min, float max);

/// <summary> [0, 1]�͈͓̔������擾���܂��B </summary>
bool inRange01(float value);

/// <summary> �C�ӎ��ɔC�Ӄx�N�g���𓊉e���܂��B </summary>
float projectVector(const Vector3 & axis, const Vector3 & v1);

/// <summary> �C�ӎ��ɔC�Ӄx�N�g���𓊉e���܂��B </summary>
float projectVector(const Vector3 & axis, const Vector3 & v1, const Vector3 & v2);

/// <summary> �C�ӎ��ɔC�Ӄx�N�g���𓊉e���܂��B </summary>
float projectVector(const Vector3 & axis, const Vector3 & v1, const Vector3 & v2, const Vector3 & v3);

/// <summary> ���ς̌��ʂ̕������擾���܂��B </summary>
float dotSign(const Vector3 & v1, const Vector3 & v2);

/// <summary> ���ʏ�ɓ_�𓊉e���܂��B </summary>
const Vector3 projectPointOnPlane(const Vector3 & planeUnitNormal, const Vector3 & planePoint, const Vector3 & point);

/// <summary> �r�b�g����΂��ɂ��܂��B </summary>
int32 bitTwoSkipping(int8 n);

/// <summary> ���W���烂�[�g���������擾���܂��B </summary>
int32 mortonCode(const Vector3 & position, const Vector3 & min, const Vector3 & unitSize, int8 max);

/// <summary> �ŏ����x����Ԃ̕���1�Ƃ������W�n�̓_��p���ă��[�g���������Z�o���܂��B </summary>
int32 mortonCode(int8 x, int8 y, int8 z);

/// <summary> ��̃��[�g�������ƕ������x�����狤�ʃ��x�����擾���܂��B </summary>
int32 commonLevel3D(int32 level, int32 mortonCode1, int32 mortonCode2);

/// <summary> ��̃��[�g�������ƕ������x�����烂�[�g�������Ə������x�����擾���܂��B </summary>
void commonMortonCodeAndBelongsLevel3D(int32 level, int32 mortonCode1, int32 mortonCode2, int32 * outCommonMortonCode, int32 * outBelongsLevel);

/// <summary> AABB���烂�[�g���������擾���܂��B </summary>
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

/// <summary> �ʂ���ӂ̃x�N�g�����擾���܂��B </summary>
template <std::size_t VertexCount>
const Vector3 edgeVectorFromFace(const Face<VertexCount> & face, std::size_t index) {
	return face[(index + 1) % (VertexCount)] - face[index];
}

/// <summary> �ʂ���ӂ��擾���܂��B </summary>
template <std::size_t VertexCount>
const Segment edgeFromFace(const Face<VertexCount> & face, std::size_t index) {
	return Segment(face[index], face[index] + edgeVectorFromFace(face, index));
}

/// <summary>
/// <para>�ʂ���@���x�N�g�����擾���܂��B</para>
/// <para>�擾����@���x�N�g���͒P�ʒ��Ƃ͌���܂���B</para>
/// </summary>
template <std::size_t VertexCount>
const Vector3 faceNormal(const Face<VertexCount> & face) {
	return Vector3::cross(face[1] - face[0], face[2] - face[1]);
}

/// <summary> �ʂ���P�ʒ��̖@���x�N�g�����擾���܂��B </summary>
template <std::size_t VertexCount>
const Vector3 faceUnitNormal(const Face<VertexCount> & face) {
	return Vector3::cross(face[1] - face[0], face[2] - face[1]).normalize();
}

/// <summary>
/// <para>�ʂɓ_�𓊉e�����Ƃ��ɁA�ʏ�ɓ_�����݂��邩���擾���܂��B</para>
/// <para>���݂���ꍇ�͓��e�����_���擾�ł��܂��B</para>
/// <para>�ʂ̒��_�̏��Ԃ͎��v���ł��B</para>
/// </summary>
template <std::size_t VertexCount>
bool tryProjectPointOnFace(const Face<VertexCount> & face, const Vector3 & point, Vector3 * outProjectPoint = nullptr) {
	return tryProjectPointOnFaceFast(face, faceUnitNormal(face), point, outProjectPoint);
}

/// <summary>
/// <para>�ʂɓ_�𓊉e�����Ƃ��ɁA�ʏ�ɓ_�����݂��邩���擾���܂��B</para>
/// <para>���݂���ꍇ�͓��e�����_���擾�ł��܂��B</para>
/// <para>�ʂ̒��_�̏��Ԃ͎��v���ł��B</para>
/// </summary>
template <std::size_t VertexCount>
bool tryProjectPointOnFace(const Face<VertexCount> & face, const Vector3 & faceNormal, const Vector3 & point, Vector3 * outProjectPoint = nullptr) {
	return tryProjectPointOnFaceFast(face, faceNormal.normalize(), point, outProjectPoint);
}

/// <summary> �@�������łɒP�ʒ��̏ꍇ </summary>
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
