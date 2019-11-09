#include <Collider/Implementation/CollisionUtility.h>
#include <LMath.h>
#include <Collider/Segment.h>
#include <Collider/AABB.h>
#include <Collider/OBB.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/CollisionShape.h>

float CollisionUtility::saturate(float value) {
	return Mathf::clamp(value, 0.0f, 1.0f);
}

bool CollisionUtility::inRange(float value, float min, float max) {
	return min < value && value < max;
}

bool CollisionUtility::inRange01(float value) {
	return inRange(value, 0.0f, 1.0f);
}

float CollisionUtility::projectVector(const Vector3 & axis, const Vector3 & v1) {
	return Mathf::abs(Vector3::dot(axis, v1));
}

float CollisionUtility::projectVector(const Vector3 & axis, const Vector3 & v1, const Vector3 & v2) {
	return projectVector(axis, v1) + projectVector(axis, v2);
}

float CollisionUtility::projectVector(const Vector3 & axis, const Vector3 & v1, const Vector3 & v2, const Vector3 & v3) {
	return projectVector(axis, v1, v2) + projectVector(axis, v3);
}

float CollisionUtility::dotSign(const Vector3 & v1, const Vector3 & v2) {
	return Vector3::dot(v1, v2) > 0.0f ? 1.0f : -1.0f;
}

const Vector3 CollisionUtility::projectPointOnPlane(const Vector3 & planeUnitNormal, const Vector3 & planePoint, const Vector3 & point) {
	return point - Vector3::dot(planeUnitNormal, point - planePoint) * planeUnitNormal;
}

int32 CollisionUtility::bitTwoSkipping(int8 n) {
	n = (n | n << 8) & 0x0000f00f;
	n = (n | n << 4) & 0x000c30c3;
	n = (n | n << 2) & 0x00249249;
	return n;
}

int32 CollisionUtility::mortonCode(const Vector3 & position, const Vector3 & min, const Vector3 & unitSize, int8 max) {
	const auto x = Mathf::cmin(static_cast<int8>((position.x - min.x) / unitSize.x), max);
	const auto y = Mathf::cmin(static_cast<int8>((position.y - min.y) / unitSize.y), max);
	const auto z = Mathf::cmin(static_cast<int8>((position.z - min.z) / unitSize.z), max);
	return mortonCode(x, y, z);
}

int32 CollisionUtility::mortonCode(int8 x, int8 y, int8 z) {
	return bitTwoSkipping(x) | bitTwoSkipping(y) << 1 | bitTwoSkipping(z) << 2;
}

int32 CollisionUtility::commonLevel3D(int32 level, int32 mortonCode1, int32 mortonCode2) {
	auto def = mortonCode1 ^ mortonCode2;

	auto i = 0;
	auto lowLevel = level;
	while (def) {
		i++;

		if (def & 0b0111) {
			lowLevel = level - i;
		}

		def >>= 3;
	}

	return lowLevel;
}

void CollisionUtility::commonMortonCodeAndBelongsLevel3D(int32 level, int32 mortonCode1, int32 mortonCode2, int32 * outCommonMortonCode, int32 * outBelongsLevel) {
	const auto commonLevel = commonLevel3D(level, mortonCode1, mortonCode2);
	*outBelongsLevel = commonLevel;
	const auto shift = level - commonLevel;
	*outCommonMortonCode = mortonCode1 >> (shift * 3);
}

void CollisionUtility::mortonCodeAndBelongsLevel3D(const AABB & aabb, int32 level, const Vector3 & min, const Vector3 & unitSize, int8 max, int32 * outMortonCode, int32 * outBelongsLevel) {
	const auto minMortonCode = mortonCode(aabb.min, min, unitSize, max);
	const auto maxMortonCode = mortonCode(aabb.max, min, unitSize, max);

	commonMortonCodeAndBelongsLevel3D(level, minMortonCode, maxMortonCode, outMortonCode, outBelongsLevel);
}

void CollisionUtility::TerrainCollisionDetail(const ColliderPtr & collider, const UniformGridMesh & mesh, Vector3 * outColliderHitPos, Vector3 * outMeshHitPos) {
	decltype(auto) shape = collider->getCollisionShape();
	decltype(auto) curTra = collider->getTransform();
	decltype(auto) preTra = collider->getPreTransform();

	// 衝突時の座標
	auto lastHitTransform = curTra;

	// 半分の位置から二分探索
	auto ratio = 0.5f;

	// この値以下の誤差は許容する(距離の二乗)
	auto tolerance = shape->toleranceError();

	auto searchPos = curTra.position;

	for (auto i = 2;; i++) {
		const auto tempSearchPos = Vector3::lerp(preTra.position, curTra.position, ratio);

		// 距離が一定以下だったらズレを許容する
		if (Vector3::sqrDistance(tempSearchPos, searchPos) < tolerance) break;

		searchPos = tempSearchPos;
		const auto searchTra = TransformQ(searchPos, curTra.rotation, curTra.scale);

		auto searchColliderHitPos = Vector3();
		auto searchMeshHitPos = Vector3();
		const auto isCollided = shape->intersectMesh(searchTra, mesh, &searchColliderHitPos, &searchMeshHitPos);

		// 衝突したら前の時間を探す
		if (isCollided) {
			*outColliderHitPos = searchColliderHitPos;
			*outMeshHitPos = searchMeshHitPos;
			ratio -= 1.0f / (1 << i);
			// 最後に衝突を確認した場所を保持
			lastHitTransform = searchTra;
		}
		// 衝突しなかったら後の時間を探す
		else {
			ratio += 1.0f / (1 << i);
		}
	}
	*outColliderHitPos += curTra.position - lastHitTransform.position;
}
