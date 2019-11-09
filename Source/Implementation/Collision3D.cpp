#include <Collider/Implementation/Collision3D.h>
#include <algorithm>
#include <Collider/Implementation/CollisionUtility.h>
#include <Collider/Sphere.h>
#include <Collider/Capsule.h>
#include <Collider/Line.h>
#include <Collider/OBB.h>
#include <Collider/InfCylinder.h>

#include <Renderer/SceneRendererManager.h>

using namespace std;
using namespace CollisionUtility;

bool Collision3D::intersect(const Sphere & sphere1, const Sphere & sphere2) {
	auto center1 = sphere1.center();
	auto center2 = sphere2.center();
	auto radius1 = sphere1.radius();
	auto radius2 = sphere2.radius();
	auto addRad = radius1 + radius2;
	auto sqrRad = addRad * addRad;
	auto sqrDist = Vector3::sqrDistance(center1, center2);
	return sqrDist < sqrRad;
}

bool Collision3D::intersect(const Sphere & sphere1, const Sphere & sphere2, Vector3 * outSphere1HitPos, Vector3 * outSphere2HitPos) {
	auto center1 = sphere1.center();
	auto center2 = sphere2.center();
	auto radius1 = sphere1.radius();
	auto radius2 = sphere2.radius();
	auto addRad = radius1 + radius2;
	auto sqrRad = addRad * addRad;
	auto sqrDist = Vector3::sqrDistance(center1, center2);
	auto isCollided = sqrDist < sqrRad;

	if (!isCollided) return false;

	auto toSphere2 = (center2 - center1).normalize();

	*outSphere1HitPos = center1 + toSphere2 * radius1;
	*outSphere2HitPos = center2 - toSphere2 * radius2;

	return true;
}

bool Collision3D::intersect(const Sphere & sphere, const Capsule & capsule) {
	auto center = sphere.center();
	auto shortest = capsule.segment().closest(center);
	return intersect(Sphere(center, sphere.radius()), Sphere(shortest, capsule.radius()));
}

bool Collision3D::intersect(const Sphere & sphere, const Capsule & capsule, Vector3 * outSphereHitPos, Vector3 * outCapsuleHitPos) {
	auto center = sphere.center();
	auto shortest = capsule.segment().closest(center);
	return intersect(Sphere(center, sphere.radius()), Sphere(shortest, capsule.radius()), outSphereHitPos, outCapsuleHitPos);
}

bool Collision3D::intersect(const Sphere & sphere, const OBB & obb, Vector3 * outSphereHitPos, Vector3 * outOBBHitPos) {
	auto shortest = Vector3();
	auto distance = obb.distance(sphere.center(), &shortest);
	auto isCollided = distance < sphere.radius();

	if (!isCollided) return false;

	auto extrusion = obb.extrusionPoint(shortest);

	if (outSphereHitPos) *outSphereHitPos = sphere.center() + (extrusion - sphere.center()).normalize() * sphere.radius();
	if (outOBBHitPos) *outOBBHitPos = extrusion;

	return true;
}

bool Collision3D::intersect(const Sphere & sphere, const Triangle & triangle, Vector3 * outSphereHitPos, Vector3 * outTriangleHitPos) {
	decltype(auto) sphereRadius = sphere.radius();
	decltype(auto) sphereCenter = sphere.center();

	// 面と衝突判定
	auto project = Vector3();
	if (tryProjectPointOnFace(triangle, sphereCenter, &project)) {
		const auto sqrDist = (project - sphereCenter).sqrLength();
		const auto isCollide = sqrDist < sphereRadius * sphereRadius;

		if (!isCollide) return false;

		if (outSphereHitPos) *outSphereHitPos = sphereCenter + (project - sphereCenter).normalize() * sphereRadius;
		if (outTriangleHitPos)	*outTriangleHitPos = project;

		return true;
	}
	// 線分と衝突判定
	else {
		auto minSqrDist = numeric_limits<float>::max();
		auto anyCollided = false;

		for (auto i = 0; i < 3; i++) {
			decltype(auto) edge = edgeFromFace(triangle, i);
			auto closest = Vector3();
			const auto tempSqrDist = edge.sqrDistance(sphereCenter, &closest);
			const auto isCollide = tempSqrDist < sphereRadius * sphereRadius;

			if (!isCollide) continue;

			anyCollided = true;

			if (tempSqrDist > minSqrDist) continue;
			minSqrDist = tempSqrDist;

			if (outSphereHitPos) *outSphereHitPos = sphereCenter + (closest - sphereCenter).normalize() * sphereRadius;
			if (outTriangleHitPos) *outTriangleHitPos = closest;
		}
		return anyCollided;
	}
	return false;
}

bool Collision3D::intersect(const Capsule & capsule1, const Capsule & capsule2) {
	decltype(auto) seg1 = capsule1.segment();
	decltype(auto) seg2 = capsule2.segment();
	auto p1 = Vector3();
	auto p2 = Vector3();
	seg1.closest(seg2, &p1, &p2);

	auto sqrDist = Vector3::sqrDistance(p1, p2);
	auto addRad = capsule1.radius() + capsule2.radius();
	auto sqrRad = addRad * addRad;

	return sqrDist < sqrRad;
}

bool Collision3D::intersect(const Capsule & capsule1, const Capsule & capsule2, Vector3 * outCapsule1HitPos, Vector3 * outCapsule2HitPos) {
	auto radius1 = capsule1.radius();
	auto radius2 = capsule2.radius();
	auto seg1 = capsule1.segment();
	auto seg2 = capsule2.segment();
	auto p1 = Vector3();
	auto p2 = Vector3();

	seg1.closest(seg2, &p1, &p2);

	auto addRad = radius1 + radius2;
	auto sqrRad = addRad * addRad;
	auto sqrDist = Vector3::sqrDistance(p1, p2);

	if (sqrDist > sqrRad) return false;

	auto toP2 = (p2 - p1).normalize();

	*outCapsule1HitPos = p1 + toP2 * radius1;
	*outCapsule2HitPos = p2 - toP2 * radius2;

	return true;
}

bool Collision3D::intersect(const Capsule & capsule, const OBB & obb) {
	// SAT(分離軸定理)で衝突判定

	auto capsuleRadius = capsule.radius();
	auto capsuleHalfHeight = capsule.halfHeight();
	auto capsuleSegment = capsule.segment();
	auto capsuleMidpoint = capsuleSegment.midpoint();

	auto obbHalfSize = obb.halfSize();

	// 分離軸になり得るベクトル
	auto obbUnitX = obb.right();
	auto obbUnitY = obb.up();
	auto obbUnitZ = obb.forward();
	auto capsuleUnit = capsuleSegment.unitVector();
	auto obbX_x_capsule = Vector3::cross(obbUnitX, capsuleUnit);
	auto obbY_x_capsule = Vector3::cross(obbUnitY, capsuleUnit);
	auto obbZ_x_capsule = Vector3::cross(obbUnitZ, capsuleUnit);

	auto obbX = obbUnitX * obbHalfSize.x;
	auto obbY = obbUnitY * obbHalfSize.y;
	auto obbZ = obbUnitZ * obbHalfSize.z;
	auto capsuleHalf = capsuleSegment.halfVector();

	// obbの中心からカプセルの中心へのベクトル
	auto interval = capsuleMidpoint - obb.center();

	// 分離軸候補 : obbX
	auto p1 = obbX.length();
	auto p2 = projectVector(obbUnitX, capsuleHalf) + capsuleRadius;
	auto pi = projectVector(obbUnitX, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obbY
	p1 = obbY.length();
	p2 = projectVector(obbUnitY, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbUnitY, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obbZ
	p1 = obbZ.length();
	p2 = projectVector(obbUnitZ, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbUnitZ, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : capsule
	p1 = projectVector(capsuleUnit, obbX, obbY, obbZ);
	p2 = capsuleHalfHeight;
	pi = projectVector(capsuleUnit, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obbX * capsule
	p1 = projectVector(obbX_x_capsule, obbY, obbZ);
	p2 = projectVector(obbX_x_capsule, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbX_x_capsule, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obbY * capsule
	p1 = projectVector(obbY_x_capsule, obbX, obbZ);
	p2 = projectVector(obbY_x_capsule, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbY_x_capsule, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obbZ * capsule
	p1 = projectVector(obbZ_x_capsule, obbX, obbY);
	p2 = projectVector(obbZ_x_capsule, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbZ_x_capsule, interval);
	if (pi > p1 + p2) return false;

	// 分離軸が存在しないので衝突している
	return true;
}

bool Collision3D::intersect(const Capsule & capsule, const OBB & obb, Vector3 * outCapsuleExtrusion) {
	// SAT(分離軸定理)で衝突判定

	auto capsuleRadius = capsule.radius();
	auto capsuleHalfHeight = capsule.halfHeight();
	auto capsuleSegment = capsule.segment();
	auto capsuleMidpoint = capsuleSegment.midpoint();

	auto obbHalfSize = obb.halfSize();

	// 分離軸になり得るベクトル
	auto obbUnitX = obb.right();
	auto obbUnitY = obb.up();
	auto obbUnitZ = obb.forward();
	auto capsuleUnit = capsuleSegment.unitVector();
	auto obbX_x_capsule = Vector3::cross(obbUnitX, capsuleUnit).normalize();
	auto obbY_x_capsule = Vector3::cross(obbUnitY, capsuleUnit).normalize();
	auto obbZ_x_capsule = Vector3::cross(obbUnitZ, capsuleUnit).normalize();

	auto obbX = obbUnitX * obbHalfSize.x;
	auto obbY = obbUnitY * obbHalfSize.y;
	auto obbZ = obbUnitZ * obbHalfSize.z;
	auto capsuleHalf = capsuleSegment.halfVector();

	// obbの中心からカプセルの中心へのベクトル
	auto toCapsule = capsuleMidpoint - obb.center();

	auto minInterval = 0.0f;
	auto minAxis = Vector3();
	auto tempInterval = 0.0f;

	// 分離軸候補 : obbX
	auto p1 = obbX.length();
	auto p2 = projectVector(obbUnitX, capsuleHalf) + capsuleRadius;
	auto pi = projectVector(obbUnitX, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	minInterval = tempInterval;
	minAxis = obbUnitX;

	// 分離軸候補 : obbY
	p1 = obbY.length();
	p2 = projectVector(obbUnitY, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbUnitY, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obbUnitY;
	}

	// 分離軸候補 : obbZ
	p1 = obbZ.length();
	p2 = projectVector(obbUnitZ, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbUnitZ, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obbUnitZ;
	}

	// 分離軸候補 : capsule
	p1 = projectVector(capsuleUnit, obbX, obbY, obbZ);
	p2 = capsuleHalfHeight;
	pi = projectVector(capsuleUnit, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = capsuleUnit;
	}

	// 分離軸候補 : obbX * capsule
	p1 = projectVector(obbX_x_capsule, obbY, obbZ);
	p2 = projectVector(obbX_x_capsule, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbX_x_capsule, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obbX_x_capsule;
	}

	// 分離軸候補 : obbY * capsule
	p1 = projectVector(obbY_x_capsule, obbX, obbZ);
	p2 = projectVector(obbY_x_capsule, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbY_x_capsule, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obbY_x_capsule;
	}

	// 分離軸候補 : obbZ * capsule
	p1 = projectVector(obbZ_x_capsule, obbX, obbY);
	p2 = projectVector(obbZ_x_capsule, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbZ_x_capsule, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obbZ_x_capsule;
	}

	minAxis *= dotSign(minAxis, toCapsule);

	*outCapsuleExtrusion = minAxis * minInterval;

	// 分離軸が存在しないので衝突している
	return true;
}

bool Collision3D::intersect(const Capsule & capsule, const OBB & obb, Vector3 * outCapsuleHitPos, Vector3 * outOBBHitPos) {
	// SAT(分離軸定理)で衝突判定

	auto capsuleRadius = capsule.radius();
	auto capsuleHalfHeight = capsule.halfHeight();
	auto capsuleSegment = capsule.segment();
	auto capsuleMidpoint = capsuleSegment.midpoint();

	auto obbHalfSize = obb.halfSize();

	// 分離軸になり得るベクトル
	auto obbUnitX = obb.right();
	auto obbUnitY = obb.up();
	auto obbUnitZ = obb.forward();
	auto capsuleUnit = capsuleSegment.unitVector();
	auto obbX_x_capsule = Vector3::cross(obbUnitX, capsuleUnit).normalize();
	auto obbY_x_capsule = Vector3::cross(obbUnitY, capsuleUnit).normalize();
	auto obbZ_x_capsule = Vector3::cross(obbUnitZ, capsuleUnit).normalize();

	auto obbX = obbUnitX * obbHalfSize.x;
	auto obbY = obbUnitY * obbHalfSize.y;
	auto obbZ = obbUnitZ * obbHalfSize.z;
	auto capsuleHalf = capsuleSegment.halfVector();

	// obbの中心からカプセルの中心へのベクトル
	auto toCapsule = capsuleMidpoint - obb.center();

	// 分離軸候補 : obbX
	auto p1 = obbX.length();
	auto p2 = projectVector(obbUnitX, capsuleHalf) + capsuleRadius;
	auto pi = projectVector(obbUnitX, toCapsule);
	if (pi > p1 + p2) return false;
	auto tempInterval = p1 + p2 - pi;
	auto minInterval = tempInterval;
	auto minAxis = obbUnitX;

	// 分離軸候補 : obbY
	p1 = obbY.length();
	p2 = projectVector(obbUnitY, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbUnitY, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obbUnitY;
	}

	// 分離軸候補 : obbZ
	p1 = obbZ.length();
	p2 = projectVector(obbUnitZ, capsuleHalf) + capsuleRadius;
	pi = projectVector(obbUnitZ, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obbUnitZ;
	}

	// 分離軸候補 : capsule
	p1 = projectVector(capsuleUnit, obbX, obbY, obbZ);
	p2 = capsuleHalfHeight;
	pi = projectVector(capsuleUnit, toCapsule);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = capsuleUnit;
	}

	// 分離軸候補 : obbX * capsule
	if (obbX_x_capsule.sqrLength()) {
		p1 = projectVector(obbX_x_capsule, obbY, obbZ);
		p2 = projectVector(obbX_x_capsule, capsuleHalf) + capsuleRadius;
		pi = projectVector(obbX_x_capsule, toCapsule);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obbX_x_capsule;
		}
	}

	// 分離軸候補 : obbY * capsule
	if (obbY_x_capsule.sqrLength()) {
		p1 = projectVector(obbY_x_capsule, obbX, obbZ);
		p2 = projectVector(obbY_x_capsule, capsuleHalf) + capsuleRadius;
		pi = projectVector(obbY_x_capsule, toCapsule);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obbY_x_capsule;
		}
	}

	// 分離軸候補 : obbZ * capsule
	if (obbZ_x_capsule.sqrLength()) {
		p1 = projectVector(obbZ_x_capsule, obbX, obbY);
		p2 = projectVector(obbZ_x_capsule, capsuleHalf) + capsuleRadius;
		pi = projectVector(obbZ_x_capsule, toCapsule);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obbZ_x_capsule;
		}
	}

	// 分離軸候補 カプセルの辺とobbの辺
	for (auto edgeIndex = 0; edgeIndex < 12; edgeIndex++) {
		auto obbEdge = obb.segment(edgeIndex);

		auto closestCapsule = Vector3();
		auto closestOBB = Vector3();
		capsuleSegment.closest(obbEdge, &closestCapsule, &closestOBB);
		// 判定軸
		auto axis = (closestCapsule - closestOBB).normalize();
		p1 = projectVector(axis, obbX, obbY, obbZ);
		p2 = projectVector(axis, capsuleHalf) + capsuleRadius;
		pi = projectVector(axis, toCapsule);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = axis;
		}
	}

	// 衝突座標取得
	minAxis *= dotSign(minAxis, -toCapsule);

	decltype(auto) translatedOBB = obb.translate(minAxis * minInterval);

	auto minDistance = numeric_limits<float>::max();

	// capsuleの始点 boxの面
	auto s = capsuleSegment.start();
	for (auto f = 0; f < 6; f++) {
		decltype(auto) face = translatedOBB.face(f);
		auto project = Vector3();

		if (!tryProjectPointOnFace(face, s, &project)) continue;

		const auto tempDistance = Vector3::distance(project, s);

		if (tempDistance > minDistance) continue;

		minDistance = tempDistance;
		*outCapsuleHitPos = s + minAxis * capsuleRadius;
		*outOBBHitPos = project - minAxis * minInterval;
	}

	// capsuleの終点 boxの面
	auto e = capsuleSegment.end();
	for (auto f = 0; f < 6; f++) {
		decltype(auto) face = translatedOBB.face(f);
		auto project = Vector3();

		if (!tryProjectPointOnFace(face, e, &project)) continue;

		const auto tempDistance = Vector3::distance(project, e);

		if (tempDistance > minDistance) continue;

		minDistance = tempDistance;
		*outCapsuleHitPos = e + minAxis * capsuleRadius;
		*outOBBHitPos = project - minAxis * minInterval;
	}

	// capsuleの辺 boxの辺
	auto capSeg = capsuleSegment;
	for (auto s = 0; s < 12; s++) {
		auto boxSeg = translatedOBB.segment(s);
		auto closest1 = Vector3();
		auto closest2 = Vector3();
		capSeg.closest(boxSeg, &closest1, &closest2);
		auto tempDistance = Vector3::distance(closest1, closest2);
		if (tempDistance < minDistance) {
			minDistance = tempDistance;
			*outOBBHitPos = closest2 - minAxis * minInterval;
			*outCapsuleHitPos = closest1 + (closest2 - closest1).normalize() * capsuleRadius;
		}
	}

	// 分離軸が存在しないので衝突している
	return true;
}

bool Collision3D::intersect(const Capsule & capsule, const Triangle & triangle, Vector3 * outCapsuleHitPos, Vector3 * outTriangleHitPos) {
	decltype(auto) capsuleRadius = capsule.radius();
	decltype(auto) capsuleSegment = capsule.segment();

	decltype(auto) capsuleStart = capsuleSegment.start();
	decltype(auto) capsuleEnd = capsuleSegment.end();

	auto minCapsuleHitPos = Vector3();
	auto minTriangleHitPos = Vector3();

	// 線分と衝突判定
	auto minSqrDist = numeric_limits<float>::max();

	auto anyCollided = false;

	// 面と始点の衝突判定
	auto project = Vector3();
	if (tryProjectPointOnFace(triangle, capsuleStart, &project)) {
		const auto tempSqrDist = (project - capsuleStart).sqrLength();
		const auto isCollided = tempSqrDist < capsuleRadius * capsuleRadius;
		if (isCollided) {
			if (tempSqrDist < minSqrDist) {
				minSqrDist = tempSqrDist;
				minCapsuleHitPos = capsuleStart;
				minTriangleHitPos = project;
				anyCollided = true;
			}
		}
	}

	// 面と終点の衝突判定
	if (tryProjectPointOnFace(triangle, capsuleEnd, &project)) {
		const auto tempSqrDist = (project - capsuleEnd).sqrLength();
		const auto isCollided = tempSqrDist < capsuleRadius * capsuleRadius;
		if (isCollided) {
			if (tempSqrDist < minSqrDist) {
				minSqrDist = tempSqrDist;
				minCapsuleHitPos = capsuleEnd;
				minTriangleHitPos = project;
				anyCollided = true;
			}
		}
	}

	// エッジとエッジの衝突判定
	for (auto i = 0; i < 3; i++) {
		decltype(auto) edge = edgeFromFace(triangle, i);
		auto closestCapsule = Vector3();
		auto closestTriangle = Vector3();
		edge.closest(capsuleSegment, &closestTriangle, &closestCapsule);
		const auto tempSqrDist = (closestTriangle - closestCapsule).sqrLength();
		const auto isCollide = tempSqrDist < capsuleRadius * capsuleRadius;

		if (!isCollide) continue;

		if (tempSqrDist > minSqrDist) continue;

		minSqrDist = tempSqrDist;
		minCapsuleHitPos = closestCapsule;
		minTriangleHitPos = closestTriangle;
		anyCollided = true;
	}

	if (!anyCollided) return false;

	if (outCapsuleHitPos) *outCapsuleHitPos = minCapsuleHitPos + (minTriangleHitPos - minCapsuleHitPos).normalize() * capsuleRadius;
	if (outTriangleHitPos) *outTriangleHitPos = minTriangleHitPos;

	return true;
}

bool Collision3D::intersect(const OBB & obb1, const OBB & obb2) {
	// SAT(分離軸定理)で衝突判定

	// 分離軸になり得るベクトル
	auto obb1UnitX = obb1.right();
	auto obb1UnitY = obb1.up();
	auto obb1UnitZ = obb1.forward();
	auto obb2UnitX = obb2.right();
	auto obb2UnitY = obb2.up();
	auto obb2UnitZ = obb2.forward();
	auto obb1X_x_obb2X = Vector3::cross(obb1UnitX, obb2UnitX);
	auto obb1X_x_obb2Y = Vector3::cross(obb1UnitX, obb2UnitY);
	auto obb1X_x_obb2Z = Vector3::cross(obb1UnitX, obb2UnitZ);
	auto obb1Y_x_obb2X = Vector3::cross(obb1UnitY, obb2UnitX);
	auto obb1Y_x_obb2Y = Vector3::cross(obb1UnitY, obb2UnitY);
	auto obb1Y_x_obb2Z = Vector3::cross(obb1UnitY, obb2UnitZ);
	auto obb1Z_x_obb2X = Vector3::cross(obb1UnitZ, obb2UnitX);
	auto obb1Z_x_obb2Y = Vector3::cross(obb1UnitZ, obb2UnitY);
	auto obb1Z_x_obb2Z = Vector3::cross(obb1UnitZ, obb2UnitZ);

	auto obb1X = obb1UnitX * obb1.halfSize().x;
	auto obb1Y = obb1UnitY * obb1.halfSize().y;
	auto obb1Z = obb1UnitZ * obb1.halfSize().z;
	auto obb2X = obb2UnitX * obb2.halfSize().x;
	auto obb2Y = obb2UnitY * obb2.halfSize().y;
	auto obb2Z = obb2UnitZ * obb2.halfSize().z;

	// obb1の中心からobb2の中心へのベクトル
	auto interval = obb1.center() - obb2.center();

	// 分離軸候補 : obb1X
	auto p1 = obb1X.length();
	auto p2 = projectVector(obb1UnitX, obb2X, obb2Y, obb2Z);
	auto pi = projectVector(obb1UnitX, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1Y
	p1 = obb1Y.length();
	p2 = projectVector(obb1UnitY, obb2X, obb2Y, obb2Z);
	pi = projectVector(obb1UnitY, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1Z
	p1 = obb1Z.length();
	p2 = projectVector(obb1UnitZ, obb2X, obb2Y, obb2Z);
	pi = projectVector(obb1UnitZ, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb2X
	p1 = projectVector(obb2UnitX, obb1X, obb1Y, obb1Z);
	p2 = obb2X.length();
	pi = projectVector(obb2UnitX, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb2Y
	p1 = projectVector(obb2UnitY, obb1X, obb1Y, obb1Z);
	p2 = obb2Y.length();
	pi = projectVector(obb2UnitY, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb2Z
	p1 = projectVector(obb2UnitZ, obb1X, obb1Y, obb1Z);
	p2 = obb2Z.length();
	pi = projectVector(obb2UnitZ, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1X * obb2X
	p1 = projectVector(obb1X_x_obb2X, obb1Y, obb1Z);
	p2 = projectVector(obb1X_x_obb2X, obb2Y, obb2Z);
	pi = projectVector(obb1X_x_obb2X, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1X * obb2Y
	p1 = projectVector(obb1X_x_obb2Y, obb1Y, obb1Z);
	p2 = projectVector(obb1X_x_obb2Y, obb2X, obb2Z);
	pi = projectVector(obb1X_x_obb2Y, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1X * obb2Z
	p1 = projectVector(obb1X_x_obb2Z, obb1Y, obb1Z);
	p2 = projectVector(obb1X_x_obb2Z, obb2X, obb2Y);
	pi = projectVector(obb1X_x_obb2Z, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1Y * obb2X
	p1 = projectVector(obb1Y_x_obb2X, obb1X, obb1Z);
	p2 = projectVector(obb1Y_x_obb2X, obb2Y, obb2Z);
	pi = projectVector(obb1Y_x_obb2X, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1Y * obb2Y
	p1 = projectVector(obb1Y_x_obb2Y, obb1X, obb1Z);
	p2 = projectVector(obb1Y_x_obb2Y, obb2X, obb2Z);
	pi = projectVector(obb1Y_x_obb2Y, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1Y * obb2Z
	p1 = projectVector(obb1Y_x_obb2Z, obb1X, obb1Z);
	p2 = projectVector(obb1Y_x_obb2Z, obb2X, obb2Y);
	pi = projectVector(obb1Y_x_obb2Z, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1Z * obb2X
	p1 = projectVector(obb1Z_x_obb2X, obb1X, obb1Y);
	p2 = projectVector(obb1Z_x_obb2X, obb2Y, obb2Z);
	pi = projectVector(obb1Z_x_obb2X, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1Z * obb2Y
	p1 = projectVector(obb1Z_x_obb2Y, obb1X, obb1Y);
	p2 = projectVector(obb1Z_x_obb2Y, obb2X, obb2Z);
	pi = projectVector(obb1Z_x_obb2Y, interval);
	if (pi > p1 + p2) return false;

	// 分離軸候補 : obb1Z * obb2Z
	p1 = projectVector(obb1Z_x_obb2Z, obb1X, obb1Y);
	p2 = projectVector(obb1Z_x_obb2Z, obb2X, obb2Y);
	pi = projectVector(obb1Z_x_obb2Z, interval);
	if (pi > p1 + p2) return false;

	// 分離軸が存在しないので衝突している
	return true;
}

bool Collision3D::intersect(const OBB & obb1, const OBB & obb2, Vector3 * outOBB1Extrusion) {
	// SAT(分離軸定理)で衝突判定

	// 分離軸になり得るベクトル
	auto obb1UnitX = obb1.right();
	auto obb1UnitY = obb1.up();
	auto obb1UnitZ = obb1.forward();
	auto obb2UnitX = obb2.right();
	auto obb2UnitY = obb2.up();
	auto obb2UnitZ = obb2.forward();
	auto obb1X_x_obb2X = Vector3::cross(obb1UnitX, obb2UnitX).normalize();
	auto obb1X_x_obb2Y = Vector3::cross(obb1UnitX, obb2UnitY).normalize();
	auto obb1X_x_obb2Z = Vector3::cross(obb1UnitX, obb2UnitZ).normalize();
	auto obb1Y_x_obb2X = Vector3::cross(obb1UnitY, obb2UnitX).normalize();
	auto obb1Y_x_obb2Y = Vector3::cross(obb1UnitY, obb2UnitY).normalize();
	auto obb1Y_x_obb2Z = Vector3::cross(obb1UnitY, obb2UnitZ).normalize();
	auto obb1Z_x_obb2X = Vector3::cross(obb1UnitZ, obb2UnitX).normalize();
	auto obb1Z_x_obb2Y = Vector3::cross(obb1UnitZ, obb2UnitY).normalize();
	auto obb1Z_x_obb2Z = Vector3::cross(obb1UnitZ, obb2UnitZ).normalize();

	auto obb1X = obb1UnitX * obb1.halfSize().x;
	auto obb1Y = obb1UnitY * obb1.halfSize().y;
	auto obb1Z = obb1UnitZ * obb1.halfSize().z;
	auto obb2X = obb2UnitX * obb2.halfSize().x;
	auto obb2Y = obb2UnitY * obb2.halfSize().y;
	auto obb2Z = obb2UnitZ * obb2.halfSize().z;

	// obb1の中心からobb2の中心へのベクトル
	auto toOBB1 = obb1.center() - obb2.center();

	auto minInterval = 0.0f;
	auto minAxis = Vector3();
	auto tempInterval = 0.0f;

	// 分離軸候補 : obb1X
	auto p1 = obb1X.length();
	auto p2 = projectVector(obb1UnitX, obb2X, obb2Y, obb2Z);
	auto pi = projectVector(obb1UnitX, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	minInterval = tempInterval;
	minAxis = obb1UnitX;

	// 分離軸候補 : obb1Y
	p1 = obb1Y.length();
	p2 = projectVector(obb1UnitY, obb2X, obb2Y, obb2Z);
	pi = projectVector(obb1UnitY, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb1UnitY;
	}

	// 分離軸候補 : obb1Z
	p1 = obb1Z.length();
	p2 = projectVector(obb1UnitZ, obb2X, obb2Y, obb2Z);
	pi = projectVector(obb1UnitZ, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb1UnitZ;
	}

	// 分離軸候補 : obb2X
	p1 = projectVector(obb2UnitX, obb1X, obb1Y, obb1Z);
	p2 = obb2X.length();
	pi = projectVector(obb2UnitX, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb2UnitX;
	}

	// 分離軸候補 : obb2Y
	p1 = projectVector(obb2UnitY, obb1X, obb1Y, obb1Z);
	p2 = obb2Y.length();
	pi = projectVector(obb2UnitY, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb2UnitY;
	}

	// 分離軸候補 : obb2Z
	p1 = projectVector(obb2UnitZ, obb1X, obb1Y, obb1Z);
	p2 = obb2Z.length();
	pi = projectVector(obb2UnitZ, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb2UnitZ;
	}

	// 分離軸候補 : obb1X * obb2X
	if (obb1X_x_obb2X.sqrLength()) {
		p1 = projectVector(obb1X_x_obb2X, obb1Y, obb1Z);
		p2 = projectVector(obb1X_x_obb2X, obb2Y, obb2Z);
		pi = projectVector(obb1X_x_obb2X, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1X_x_obb2X;
		}
	}

	// 分離軸候補 : obb1X * obb2Y
	if (obb1X_x_obb2Y.sqrLength()) {
		p1 = projectVector(obb1X_x_obb2Y, obb1Y, obb1Z);
		p2 = projectVector(obb1X_x_obb2Y, obb2X, obb2Z);
		pi = projectVector(obb1X_x_obb2Y, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1X_x_obb2Y;
		}
	}

	// 分離軸候補 : obb1X * obb2Z
	if (obb1X_x_obb2Z.sqrLength()) {
		p1 = projectVector(obb1X_x_obb2Z, obb1Y, obb1Z);
		p2 = projectVector(obb1X_x_obb2Z, obb2X, obb2Y);
		pi = projectVector(obb1X_x_obb2Z, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1X_x_obb2Z;
		}
	}

	// 分離軸候補 : obb1Y * obb2X
	if (obb1Y_x_obb2X.sqrLength()) {
		p1 = projectVector(obb1Y_x_obb2X, obb1X, obb1Z);
		p2 = projectVector(obb1Y_x_obb2X, obb2Y, obb2Z);
		pi = projectVector(obb1Y_x_obb2X, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Y_x_obb2X;
		}
	}

	// 分離軸候補 : obb1Y * obb2Y
	if (obb1Y_x_obb2Y.sqrLength()) {
		p1 = projectVector(obb1Y_x_obb2Y, obb1X, obb1Z);
		p2 = projectVector(obb1Y_x_obb2Y, obb2X, obb2Z);
		pi = projectVector(obb1Y_x_obb2Y, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Y_x_obb2Y;
		}
	}

	// 分離軸候補 : obb1Y * obb2Z
	if (obb1Y_x_obb2Z.sqrLength()) {
		p1 = projectVector(obb1Y_x_obb2Z, obb1X, obb1Z);
		p2 = projectVector(obb1Y_x_obb2Z, obb2X, obb2Y);
		pi = projectVector(obb1Y_x_obb2Z, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Y_x_obb2Z;
		}
	}

	// 分離軸候補 : obb1Z * obb2X
	if (obb1Z_x_obb2X.sqrLength()) {
		p1 = projectVector(obb1Z_x_obb2X, obb1X, obb1Y);
		p2 = projectVector(obb1Z_x_obb2X, obb2Y, obb2Z);
		pi = projectVector(obb1Z_x_obb2X, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Z_x_obb2X;
		}
	}

	// 分離軸候補 : obb1Z * obb2Y
	if (obb1Z_x_obb2Y.sqrLength()) {
		p1 = projectVector(obb1Z_x_obb2Y, obb1X, obb1Y);
		p2 = projectVector(obb1Z_x_obb2Y, obb2X, obb2Z);
		pi = projectVector(obb1Z_x_obb2Y, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Z_x_obb2Y;
		}
	}

	// 分離軸候補 : obb1Z * obb2Z
	if (obb1Z_x_obb2Z.sqrLength()) {
		p1 = projectVector(obb1Z_x_obb2Z, obb1X, obb1Y);
		p2 = projectVector(obb1Z_x_obb2Z, obb2X, obb2Y);
		pi = projectVector(obb1Z_x_obb2Z, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Z_x_obb2Z;
		}
	}

	minAxis *= dotSign(minAxis, toOBB1);

	*outOBB1Extrusion = minAxis * minInterval;

	// 分離軸が存在しないので衝突している
	return true;
}

bool Collision3D::intersect(const OBB & obb1, const OBB & obb2, Vector3 * outOBB1HitPos, Vector3 * outOBB2HitPos) {
	// SAT(分離軸定理)で衝突判定

	// 分離軸になり得るベクトル
	auto obb1UnitX = obb1.right();
	auto obb1UnitY = obb1.up();
	auto obb1UnitZ = obb1.forward();
	auto obb2UnitX = obb2.right();
	auto obb2UnitY = obb2.up();
	auto obb2UnitZ = obb2.forward();
	auto obb1X_x_obb2X = Vector3::cross(obb1UnitX, obb2UnitX).normalize();
	auto obb1X_x_obb2Y = Vector3::cross(obb1UnitX, obb2UnitY).normalize();
	auto obb1X_x_obb2Z = Vector3::cross(obb1UnitX, obb2UnitZ).normalize();
	auto obb1Y_x_obb2X = Vector3::cross(obb1UnitY, obb2UnitX).normalize();
	auto obb1Y_x_obb2Y = Vector3::cross(obb1UnitY, obb2UnitY).normalize();
	auto obb1Y_x_obb2Z = Vector3::cross(obb1UnitY, obb2UnitZ).normalize();
	auto obb1Z_x_obb2X = Vector3::cross(obb1UnitZ, obb2UnitX).normalize();
	auto obb1Z_x_obb2Y = Vector3::cross(obb1UnitZ, obb2UnitY).normalize();
	auto obb1Z_x_obb2Z = Vector3::cross(obb1UnitZ, obb2UnitZ).normalize();

	auto obb1X = obb1UnitX * obb1.halfSize().x;
	auto obb1Y = obb1UnitY * obb1.halfSize().y;
	auto obb1Z = obb1UnitZ * obb1.halfSize().z;
	auto obb2X = obb2UnitX * obb2.halfSize().x;
	auto obb2Y = obb2UnitY * obb2.halfSize().y;
	auto obb2Z = obb2UnitZ * obb2.halfSize().z;

	// obb1の中心からobb2の中心へのベクトル
	auto toOBB1 = obb1.center() - obb2.center();


	// 分離軸候補 : obb1X
	auto p1 = obb1X.length();
	auto p2 = projectVector(obb1UnitX, obb2X, obb2Y, obb2Z);
	auto pi = projectVector(obb1UnitX, toOBB1);
	if (pi > p1 + p2) return false;
	auto minInterval = p1 + p2 - pi;
	auto minAxis = obb1UnitX;

	// 分離軸候補 : obb1Y
	p1 = obb1Y.length();
	p2 = projectVector(obb1UnitY, obb2X, obb2Y, obb2Z);
	pi = projectVector(obb1UnitY, toOBB1);
	if (pi > p1 + p2) return false;
	auto tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb1UnitY;
	}

	// 分離軸候補 : obb1Z
	p1 = obb1Z.length();
	p2 = projectVector(obb1UnitZ, obb2X, obb2Y, obb2Z);
	pi = projectVector(obb1UnitZ, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb1UnitZ;
	}

	// 分離軸候補 : obb2X
	p1 = projectVector(obb2UnitX, obb1X, obb1Y, obb1Z);
	p2 = obb2X.length();
	pi = projectVector(obb2UnitX, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb2UnitX;
	}

	// 分離軸候補 : obb2Y
	p1 = projectVector(obb2UnitY, obb1X, obb1Y, obb1Z);
	p2 = obb2Y.length();
	pi = projectVector(obb2UnitY, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb2UnitY;
	}

	// 分離軸候補 : obb2Z
	p1 = projectVector(obb2UnitZ, obb1X, obb1Y, obb1Z);
	p2 = obb2Z.length();
	pi = projectVector(obb2UnitZ, toOBB1);
	if (pi > p1 + p2) return false;
	tempInterval = p1 + p2 - pi;
	if (tempInterval < minInterval) {
		minInterval = tempInterval;
		minAxis = obb2UnitZ;
	}

	// 分離軸候補 : obb1X * obb2X
	if (obb1X_x_obb2X.sqrLength()) {
		p1 = projectVector(obb1X_x_obb2X, obb1Y, obb1Z);
		p2 = projectVector(obb1X_x_obb2X, obb2Y, obb2Z);
		pi = projectVector(obb1X_x_obb2X, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1X_x_obb2X;
		}
	}

	// 分離軸候補 : obb1X * obb2Y
	if (obb1X_x_obb2Y.sqrLength()) {
		p1 = projectVector(obb1X_x_obb2Y, obb1Y, obb1Z);
		p2 = projectVector(obb1X_x_obb2Y, obb2X, obb2Z);
		pi = projectVector(obb1X_x_obb2Y, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1X_x_obb2Y;
		}
	}

	// 分離軸候補 : obb1X * obb2Z
	if (obb1X_x_obb2Z.sqrLength()) {
		p1 = projectVector(obb1X_x_obb2Z, obb1Y, obb1Z);
		p2 = projectVector(obb1X_x_obb2Z, obb2X, obb2Y);
		pi = projectVector(obb1X_x_obb2Z, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1X_x_obb2Z;
		}
	}

	// 分離軸候補 : obb1Y * obb2X
	if (obb1Y_x_obb2X.sqrLength()) {
		p1 = projectVector(obb1Y_x_obb2X, obb1X, obb1Z);
		p2 = projectVector(obb1Y_x_obb2X, obb2Y, obb2Z);
		pi = projectVector(obb1Y_x_obb2X, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Y_x_obb2X;
		}
	}

	// 分離軸候補 : obb1Y * obb2Y
	if (obb1Y_x_obb2Y.sqrLength()) {
		p1 = projectVector(obb1Y_x_obb2Y, obb1X, obb1Z);
		p2 = projectVector(obb1Y_x_obb2Y, obb2X, obb2Z);
		pi = projectVector(obb1Y_x_obb2Y, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Y_x_obb2Y;
		}
	}

	// 分離軸候補 : obb1Y * obb2Z
	if (obb1Y_x_obb2Z.sqrLength()) {
		p1 = projectVector(obb1Y_x_obb2Z, obb1X, obb1Z);
		p2 = projectVector(obb1Y_x_obb2Z, obb2X, obb2Y);
		pi = projectVector(obb1Y_x_obb2Z, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Y_x_obb2Z;
		}
	}

	// 分離軸候補 : obb1Z * obb2X
	if (obb1Z_x_obb2X.sqrLength()) {
		p1 = projectVector(obb1Z_x_obb2X, obb1X, obb1Y);
		p2 = projectVector(obb1Z_x_obb2X, obb2Y, obb2Z);
		pi = projectVector(obb1Z_x_obb2X, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Z_x_obb2X;
		}
	}

	// 分離軸候補 : obb1Z * obb2Y
	if (obb1Z_x_obb2Y.sqrLength()) {
		p1 = projectVector(obb1Z_x_obb2Y, obb1X, obb1Y);
		p2 = projectVector(obb1Z_x_obb2Y, obb2X, obb2Z);
		pi = projectVector(obb1Z_x_obb2Y, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Z_x_obb2Y;
		}
	}

	// 分離軸候補 : obb1Z * obb2Z
	if (obb1Z_x_obb2Z.sqrLength()) {
		p1 = projectVector(obb1Z_x_obb2Z, obb1X, obb1Y);
		p2 = projectVector(obb1Z_x_obb2Z, obb2X, obb2Y);
		pi = projectVector(obb1Z_x_obb2Z, toOBB1);
		if (pi > p1 + p2) return false;
		tempInterval = p1 + p2 - pi;
		if (tempInterval < minInterval) {
			minInterval = tempInterval;
			minAxis = obb1Z_x_obb2Z;
		}
	}

	// 衝突座標取得

	// box2に向けた最短分離軸を取得
	minAxis *= dotSign(minAxis, -toOBB1);

	const auto offset = minAxis * minInterval;

	const auto box1 = obb1;
	const auto box2 = obb2.translate(offset);

	auto minSqrDist = std::numeric_limits<float>::max();

	// box1の頂点 vs box2の面

	// 分離軸への投影が最も大きいbox1の頂点のみ判定
	decltype(auto) box1ClosestVertex = box1.dotMaxVertex(minAxis);
	decltype(auto) box2Faces = box2.faces();

	for (auto f = 0; f < OBB::FaceCount; f++) {
		decltype(auto) face = box2Faces[f];

		// 頂点に表面が向いている3面のみ判定
		decltype(auto) normal = faceNormal(face);
		const auto toVertex = box1ClosestVertex - box2.center();
		const auto check = Vector3::dot(normal, toVertex);
		if (check < 0.0f) continue;

		auto project = Vector3();
		// 面内に頂点を投影できる組み合わせのみ判定
		if (!tryProjectPointOnFace(face, normal, box1ClosestVertex, &project)) continue;

		auto tempSqrDist = (project - box1ClosestVertex).sqrLength();

		if (tempSqrDist > minSqrDist) continue;

		minSqrDist = tempSqrDist;
		*outOBB1HitPos = box1ClosestVertex;
		*outOBB2HitPos = project;
	}

	// box1の面 vs box2の頂点

	decltype(auto) box1Faces = box1.faces();
	// 分離軸への投影が最も小さいbox2の頂点のみ判定
	decltype(auto) box2ClosestVertex = box2.dotMinVertex(minAxis);

	for (auto f = 0; f < OBB::FaceCount; f++) {
		decltype(auto) face = box1Faces[f];

		// 頂点に表面が向いている3面のみ判定
		decltype(auto) normal = faceNormal(face);
		const auto toVertex = box2ClosestVertex - box1.center();
		const auto check = Vector3::dot(normal, toVertex);
		if (check < 0.0f) continue;

		auto project = Vector3();
		// 面内に頂点を投影できる組み合わせのみ判定
		if (!tryProjectPointOnFace(face, normal, box2ClosestVertex, &project)) continue;

		auto tempDistance = (project - box2ClosestVertex).sqrLength();

		if (tempDistance > minSqrDist) continue;

		minSqrDist = tempDistance;
		*outOBB1HitPos = project;
		*outOBB2HitPos = box2ClosestVertex;
	}

	// box1の辺 vs box2の辺
	decltype(auto) box1Segments = box1.segments();
	decltype(auto) box2Segments = box2.segments();

	for (auto s1 = 0; s1 < OBB::SegmentCount; s1++) {
		decltype(auto) seg1 = box1Segments[s1];

		// 分離軸を法線とした平面の外側に存在する線分だけ判定
		if (!seg1.isPlaneOutside(minAxis, box1.center())) continue;

		for (auto s2 = 0; s2 < OBB::SegmentCount; s2++) {
			decltype(auto) seg2 = box2Segments[s2];

			// 分離軸の逆ベクトルを法線とした平面の外側に存在する線分だけ判定
			if (!seg2.isPlaneOutside(-minAxis, box2.center())) continue;

			auto closest1 = Vector3();
			auto closest2 = Vector3();
			seg1.closest(seg2, &closest1, &closest2);

			auto tempDistance = (closest1 - closest2).sqrLength();

			if (tempDistance > minSqrDist) continue;

			minSqrDist = tempDistance;
			*outOBB1HitPos = closest1;
			*outOBB2HitPos = closest2;
		}
	}

	*outOBB2HitPos -= offset;

	// 分離軸が存在しないので衝突している
	return true;
}

bool Collision3D::intersect(const OBB & obb, const Triangle & triangle) {
	// SAT(分離軸定理)で衝突判定

	auto triEdge1 = edgeVectorFromFace(triangle, 0);
	auto triEdge2 = edgeVectorFromFace(triangle, 1);
	auto triEdge3 = edgeVectorFromFace(triangle, 2);

	// 分離軸になり得るベクトル
	auto obbUnitX = obb.right();
	auto obbUnitY = obb.up();
	auto obbUnitZ = obb.forward();

	auto normal = faceNormal(triangle);

	auto obbX_x_edge1 = Vector3::cross(obbUnitX, triEdge1);
	auto obbX_x_edge2 = Vector3::cross(obbUnitX, triEdge2);
	auto obbX_x_edge3 = Vector3::cross(obbUnitX, triEdge3);
	auto obbY_x_edge1 = Vector3::cross(obbUnitY, triEdge1);
	auto obbY_x_edge2 = Vector3::cross(obbUnitY, triEdge2);
	auto obbY_x_edge3 = Vector3::cross(obbUnitY, triEdge3);
	auto obbZ_x_edge1 = Vector3::cross(obbUnitZ, triEdge1);
	auto obbZ_x_edge2 = Vector3::cross(obbUnitZ, triEdge2);
	auto obbZ_x_edge3 = Vector3::cross(obbUnitZ, triEdge3);

	auto obbX = obbUnitX * obb.halfSize().x;
	auto obbY = obbUnitY * obb.halfSize().y;
	auto obbZ = obbUnitZ * obb.halfSize().z;

	// 分離軸候補 : obbX
	auto minA = obb.center().x - obb.halfSize().x;
	auto maxA = obb.center().x + obb.halfSize().x;
	auto minB = dotMin(triangle, obbUnitX);
	auto maxB = dotMax(triangle, obbUnitX);
	auto d1 = minA - maxB;
	auto d2 = minB - maxA;
	if (d1 > 0.0f || d2 > 0.0f) return false;

	// 分離軸候補 : obbY
	minA = obb.center().y - obb.halfSize().y;
	maxA = obb.center().y + obb.halfSize().y;
	minB = dotMin(triangle, obbUnitY);
	maxB = dotMax(triangle, obbUnitY);
	d1 = minA - maxB;
	d2 = minB - maxA;
	if (d1 > 0.0f || d2 > 0.0f) return false;

	// 分離軸候補 : obbZ
	minA = obb.center().z - obb.halfSize().z;
	maxA = obb.center().z + obb.halfSize().z;
	minB = dotMin(triangle, obbUnitZ);
	maxB = dotMax(triangle, obbUnitZ);
	d1 = minA - maxB;
	d2 = minB - maxA;
	if (d1 > 0.0f || d2 > 0.0f) return false;

	// 分離軸候補 : triangleNormal
	minA = obb.dotMin(normal);
	maxA = obb.dotMax(normal);
	minB = Vector3::dot(triangle[0], normal);
	maxB = Vector3::dot(triangle[0], normal);
	d1 = minA - maxB;
	d2 = minB - maxA;
	if (d1 > 0.0f || d2 > 0.0f) return false;

	// 分離軸候補 : obbX_x_edge1
	if (obbX_x_edge1.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbX_x_edge1);
		maxA = obb.dotMax(obbX_x_edge1);
		minB = dotMin(triangle, obbX_x_edge1);
		maxB = dotMax(triangle, obbX_x_edge1);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
	}

	// 分離軸候補 : obbX_x_edge2
	if (obbX_x_edge2.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbX_x_edge2);
		maxA = obb.dotMax(obbX_x_edge2);
		minB = dotMin(triangle, obbX_x_edge2);
		maxB = dotMax(triangle, obbX_x_edge2);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
	}

	// 分離軸候補 : obbX_x_edge3
	if (obbX_x_edge3.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbX_x_edge3);
		maxA = obb.dotMax(obbX_x_edge3);
		minB = dotMin(triangle, obbX_x_edge3);
		maxB = dotMax(triangle, obbX_x_edge3);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
	}

	// 分離軸候補 : obbY_x_edge1
	if (obbY_x_edge1.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbY_x_edge1);
		maxA = obb.dotMax(obbY_x_edge1);
		minB = dotMin(triangle, obbY_x_edge1);
		maxB = dotMax(triangle, obbY_x_edge1);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
	}

	// 分離軸候補 : obbY_x_edge2
	if (obbY_x_edge2.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbY_x_edge2);
		maxA = obb.dotMax(obbY_x_edge2);
		minB = dotMin(triangle, obbY_x_edge2);
		maxB = dotMax(triangle, obbY_x_edge2);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
	}

	// 分離軸候補 : obbY_x_edge3
	if (obbY_x_edge3.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbY_x_edge3);
		maxA = obb.dotMax(obbY_x_edge3);
		minB = dotMin(triangle, obbY_x_edge3);
		maxB = dotMax(triangle, obbY_x_edge3);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
	}

	// 分離軸候補 : obbZ_x_edge1
	if (obbZ_x_edge1.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbZ_x_edge1);
		maxA = obb.dotMax(obbZ_x_edge1);
		minB = dotMin(triangle, obbZ_x_edge1);
		maxB = dotMax(triangle, obbZ_x_edge1);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
	}

	// 分離軸候補 : obbZ_x_edge2
	if (obbZ_x_edge2.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbZ_x_edge2);
		maxA = obb.dotMax(obbZ_x_edge2);
		minB = dotMin(triangle, obbZ_x_edge2);
		maxB = dotMax(triangle, obbZ_x_edge2);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
	}

	// 分離軸候補 : obbZ_x_edge3
	if (obbZ_x_edge3.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbZ_x_edge3);
		maxA = obb.dotMax(obbZ_x_edge3);
		minB = dotMin(triangle, obbZ_x_edge3);
		maxB = dotMax(triangle, obbZ_x_edge3);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
	}

	return true;
}

bool Collision3D::intersect(const OBB & obb, const Triangle & triangle, Vector3 * outOBBHitPos, Vector3 * outTriangleHitPos) {
	// SAT(分離軸定理)で衝突判定

	auto triEdge1 = edgeVectorFromFace(triangle, 0);
	auto triEdge2 = edgeVectorFromFace(triangle, 1);
	auto triEdge3 = edgeVectorFromFace(triangle, 2);

	// 分離軸になり得るベクトル
	auto obbUnitX = obb.right();
	auto obbUnitY = obb.up();
	auto obbUnitZ = obb.forward();

	auto normal = faceUnitNormal(triangle);

	auto obbX_x_edge1 = Vector3::cross(obbUnitX, triEdge1).normalize();
	auto obbX_x_edge2 = Vector3::cross(obbUnitX, triEdge2).normalize();
	auto obbX_x_edge3 = Vector3::cross(obbUnitX, triEdge3).normalize();
	auto obbY_x_edge1 = Vector3::cross(obbUnitY, triEdge1).normalize();
	auto obbY_x_edge2 = Vector3::cross(obbUnitY, triEdge2).normalize();
	auto obbY_x_edge3 = Vector3::cross(obbUnitY, triEdge3).normalize();
	auto obbZ_x_edge1 = Vector3::cross(obbUnitZ, triEdge1).normalize();
	auto obbZ_x_edge2 = Vector3::cross(obbUnitZ, triEdge2).normalize();
	auto obbZ_x_edge3 = Vector3::cross(obbUnitZ, triEdge3).normalize();

	auto obbX = obbUnitX * obb.halfSize().x;
	auto obbY = obbUnitY * obb.halfSize().y;
	auto obbZ = obbUnitZ * obb.halfSize().z;

	auto minAxis = Vector3::zero;
	auto minInterval = 0.0f;

	// 分離軸候補 : obbX
	auto minA = obb.center().x - obb.halfSize().x;
	auto maxA = obb.center().x + obb.halfSize().x;
	auto minB = dotMin(triangle, obbUnitX);
	auto maxB = dotMax(triangle, obbUnitX);
	auto d1 = minA - maxB;
	auto d2 = minB - maxA;
	if (d1 > 0.0f || d2 > 0.0f) return false;
	if (d1 < d2) {
		minAxis = obbUnitX;
		minInterval = -d2;
	}
	else {
		minAxis = -obbUnitX;
		minInterval = -d1;
	}

	// 分離軸候補 : obbY
	minA = obb.center().y - obb.halfSize().y;
	maxA = obb.center().y + obb.halfSize().y;
	minB = dotMin(triangle, obbUnitY);
	maxB = dotMax(triangle, obbUnitY);
	d1 = minA - maxB;
	d2 = minB - maxA;
	if (d1 > 0.0f || d2 > 0.0f) return false;
	auto temp = Mathf::abs(Mathf::cmax(d1, d2));
	if (temp < minInterval) {
		if (d1 < d2) {
			minAxis = obbUnitY;
			minInterval = -d2;
		}
		else {
			minAxis = -obbUnitY;
			minInterval = -d1;
		}
	}

	// 分離軸候補 : obbZ
	minA = obb.center().z - obb.halfSize().z;
	maxA = obb.center().z + obb.halfSize().z;
	minB = dotMin(triangle, obbUnitZ);
	maxB = dotMax(triangle, obbUnitZ);
	d1 = minA - maxB;
	d2 = minB - maxA;
	if (d1 > 0.0f || d2 > 0.0f) return false;
	temp = Mathf::abs(Mathf::cmax(d1, d2));
	if (temp < minInterval) {
		if (d1 < d2) {
			minAxis = obbUnitZ;
			minInterval = -d2;
		}
		else {
			minAxis = -obbUnitZ;
			minInterval = -d1;
		}
	}

	// 分離軸候補 : triangleNormal
	minA = obb.dotMin(normal);
	maxA = obb.dotMax(normal);
	minB = Vector3::dot(triangle[0], normal);
	maxB = Vector3::dot(triangle[0], normal);
	d1 = minA - maxB;
	d2 = minB - maxA;
	if (d1 > 0.0f || d2 > 0.0f) return false;
	temp = Mathf::abs(Mathf::cmax(d1, d2));
	if (temp < minInterval) {
		if (d1 < d2) {
			minAxis = normal;
			minInterval = -d2;
		}
		else {
			minAxis = -normal;
			minInterval = -d1;
		}
	}

	// 分離軸候補 : obbX_x_edge1
	if (obbX_x_edge1.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbX_x_edge1);
		maxA = obb.dotMax(obbX_x_edge1);
		minB = dotMin(triangle, obbX_x_edge1);
		maxB = dotMax(triangle, obbX_x_edge1);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
		temp = Mathf::abs(Mathf::cmax(d1, d2));
		if (temp < minInterval) {
			if (d1 < d2) {
				minAxis = obbX_x_edge1;
				minInterval = -d2;
			}
			else {
				minAxis = -obbX_x_edge1;
				minInterval = -d1;
			}
		}
	}

	// 分離軸候補 : obbX_x_edge2
	if (obbX_x_edge2.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbX_x_edge2);
		maxA = obb.dotMax(obbX_x_edge2);
		minB = dotMin(triangle, obbX_x_edge2);
		maxB = dotMax(triangle, obbX_x_edge2);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
		temp = Mathf::abs(Mathf::cmax(d1, d2));
		if (temp < minInterval) {
			if (d1 < d2) {
				minAxis = obbX_x_edge2;
				minInterval = -d2;
			}
			else {
				minAxis = -obbX_x_edge2;
				minInterval = -d1;
			}
		}
	}

	// 分離軸候補 : obbX_x_edge3
	if (obbX_x_edge3.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbX_x_edge3);
		maxA = obb.dotMax(obbX_x_edge3);
		minB = dotMin(triangle, obbX_x_edge3);
		maxB = dotMax(triangle, obbX_x_edge3);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
		temp = Mathf::abs(Mathf::cmax(d1, d2));
		if (temp < minInterval) {
			if (d1 < d2) {
				minAxis = obbX_x_edge3;
				minInterval = -d2;
			}
			else {
				minAxis = -obbX_x_edge3;
				minInterval = -d1;
			}
		}
	}

	// 分離軸候補 : obbY_x_edge1
	if (obbY_x_edge1.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbY_x_edge1);
		maxA = obb.dotMax(obbY_x_edge1);
		minB = dotMin(triangle, obbY_x_edge1);
		maxB = dotMax(triangle, obbY_x_edge1);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
		temp = Mathf::abs(Mathf::cmax(d1, d2));
		if (temp < minInterval) {
			if (d1 < d2) {
				minAxis = obbY_x_edge1;
				minInterval = -d2;
			}
			else {
				minAxis = -obbY_x_edge1;
				minInterval = -d1;
			}
		}
	}

	// 分離軸候補 : obbY_x_edge2
	if (obbY_x_edge2.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbY_x_edge2);
		maxA = obb.dotMax(obbY_x_edge2);
		minB = dotMin(triangle, obbY_x_edge2);
		maxB = dotMax(triangle, obbY_x_edge2);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
		temp = Mathf::abs(Mathf::cmax(d1, d2));
		if (temp < minInterval) {
			if (d1 < d2) {
				minAxis = obbY_x_edge2;
				minInterval = -d2;
			}
			else {
				minAxis = -obbY_x_edge2;
				minInterval = -d1;
			}
		}
	}

	// 分離軸候補 : obbY_x_edge3
	if (obbY_x_edge3.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbY_x_edge3);
		maxA = obb.dotMax(obbY_x_edge3);
		minB = dotMin(triangle, obbY_x_edge3);
		maxB = dotMax(triangle, obbY_x_edge3);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
		temp = Mathf::abs(Mathf::cmax(d1, d2));
		if (temp < minInterval) {
			if (d1 < d2) {
				minAxis = obbY_x_edge3;
				minInterval = -d2;
			}
			else {
				minAxis = -obbY_x_edge3;
				minInterval = -d1;
			}
		}
	}

	// 分離軸候補 : obbZ_x_edge1
	if (obbZ_x_edge1.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbZ_x_edge1);
		maxA = obb.dotMax(obbZ_x_edge1);
		minB = dotMin(triangle, obbZ_x_edge1);
		maxB = dotMax(triangle, obbZ_x_edge1);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
		temp = Mathf::abs(Mathf::cmax(d1, d2));
		if (temp < minInterval) {
			if (d1 < d2) {
				minAxis = obbZ_x_edge1;
				minInterval = -d2;
			}
			else {
				minAxis = -obbZ_x_edge1;
				minInterval = -d1;
			}
		}
	}

	// 分離軸候補 : obbZ_x_edge2
	if (obbZ_x_edge2.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbZ_x_edge2);
		maxA = obb.dotMax(obbZ_x_edge2);
		minB = dotMin(triangle, obbZ_x_edge2);
		maxB = dotMax(triangle, obbZ_x_edge2);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
		temp = Mathf::abs(Mathf::cmax(d1, d2));
		if (temp < minInterval) {
			if (d1 < d2) {
				minAxis = obbZ_x_edge2;
				minInterval = -d2;
			}
			else {
				minAxis = -obbZ_x_edge2;
				minInterval = -d1;
			}
		}
	}

	// 分離軸候補 : obbZ_x_edge3
	if (obbZ_x_edge3.sqrLength() > Mathf::Eps) {
		minA = obb.dotMin(obbZ_x_edge3);
		maxA = obb.dotMax(obbZ_x_edge3);
		minB = dotMin(triangle, obbZ_x_edge3);
		maxB = dotMax(triangle, obbZ_x_edge3);
		d1 = minA - maxB;
		d2 = minB - maxA;
		if (d1 > 0.0f || d2 > 0.0f) return false;
		temp = Mathf::abs(Mathf::cmax(d1, d2));
		if (temp < minInterval) {
			if (d1 < d2) {
				minAxis = obbZ_x_edge3;
				minInterval = -d2;
			}
			else {
				minAxis = -obbZ_x_edge3;
				minInterval = -d1;
			}
		}
	}

	// 衝突座標取得

	// box2に向けた最短分離軸を取得
	const auto offset = minAxis * minInterval;

	const auto box = obb.translate(-offset);

	auto minSqrDist = std::numeric_limits<float>::max();

	// boxの頂点 vs 三角系の面

	// 分離軸への投影が最も大きいbox1の頂点のみ判定
	decltype(auto) boxClosestVertex = box.dotMaxVertex(minAxis);

	auto project = Vector3();
	// 面内に頂点を投影できる組み合わせのみ判定
	if (tryProjectPointOnFace(triangle, normal, boxClosestVertex, &project)) {

		const auto tempSqrDist = (project - boxClosestVertex).sqrLength();

		if (tempSqrDist < minSqrDist) {
			minSqrDist = tempSqrDist;
			*outOBBHitPos = boxClosestVertex;
			*outTriangleHitPos = project;
		}
	}

	// boxの面 vs 三角形の頂点

	decltype(auto) boxFaces = box.faces();
	// 分離軸への投影が最も小さいbox2の頂点のみ判定
	decltype(auto) triangleClosestVertex = dotMinVertex(triangle, minAxis);

	for (auto f = 0; f < OBB::FaceCount; f++) {
		decltype(auto) face = boxFaces[f];

		// 頂点に表面が向いている3面のみ判定
		decltype(auto) normal = faceNormal(face);
		const auto toVertex = triangleClosestVertex - box.center();
		const auto check = Vector3::dot(normal, toVertex);
		if (check < 0.0f) continue;

		auto project = Vector3();
		// 面内に頂点を投影できる組み合わせのみ判定
		if (!tryProjectPointOnFace(face, normal, triangleClosestVertex, &project)) continue;

		auto tempDistance = (project - triangleClosestVertex).sqrLength();

		if (tempDistance > minSqrDist) continue;

		minSqrDist = tempDistance;
		*outOBBHitPos = project;
		*outTriangleHitPos = triangleClosestVertex;
	}

	// boxの辺 vs 三角形の辺
	decltype(auto) boxSegments = box.segments();

	for (auto s1 = 0; s1 < OBB::SegmentCount; s1++) {
		decltype(auto) boxSeg = boxSegments[s1];

		// 分離軸を法線とした平面の外側に存在する線分だけ判定
		if (!boxSeg.isPlaneOutside(minAxis, box.center())) continue;

		for (auto s2 = 0; s2 < 3; s2++) {
			decltype(auto) seg2 = edgeFromFace(triangle, s2);

			auto closest1 = Vector3();
			auto closest2 = Vector3();
			boxSeg.closest(seg2, &closest1, &closest2);

			auto tempDistance = (closest1 - closest2).sqrLength();

			if (tempDistance > minSqrDist) continue;

			minSqrDist = tempDistance;
			*outOBBHitPos = closest1;
			*outTriangleHitPos = closest2;
		}
	}

	*outOBBHitPos += offset;

	return true;
}

bool Collision3D::intersect(const Segment & segment, const Sphere & sphere, float * outHitDistance) {
	auto oc = segment.start() - sphere.center();
	auto a = Vector3::dot(segment.unitVector(), segment.unitVector());
	auto b = 2.0f * Vector3::dot(segment.unitVector(), oc);
	auto c = Vector3::dot(oc, oc) - std::powf(sphere.radius(), 2);

	// 判別式
	auto discriminant = b * b - 4 * a * c;

	// 解が存在するか
	if (discriminant < 0.0f) return false;

	auto rootD = std::sqrtf(discriminant);

	// 近い方の解
	auto distance = (-b - rootD) / (2.0f * a);

	// 線分の範囲外
	if (distance < 0.0f) return false;
	if (distance > segment.length()) return false;

	if (outHitDistance)	*outHitDistance = distance;
	return true;
}

bool Collision3D::intersect(const Segment & segment, const InfCylinder & cylinder, float * outHitDistance) {
	auto cylinderLine = cylinder.line();

	auto p1 = cylinderLine.position();
	auto p2 = cylinderLine.position() + cylinderLine.vector();

	auto offset = segment.start();

	p1 -= offset;
	p2 -= offset;

	auto p = p1;
	auto s = p2 - p1;
	auto v = segment.unitVector();
	auto r = cylinder.radius();

	// 各種内積値
	auto Dvv = Vector3::dot(v, v);
	auto Dsv = Vector3::dot(s, v);
	auto Dpv = Vector3::dot(p, v);
	auto Dss = Vector3::dot(s, s);
	auto Dps = Vector3::dot(p, s);
	auto Dpp = Vector3::dot(p, p);

	if (Dss == 0.0f) return false;

	auto A = Dvv - Dsv * Dsv / Dss;
	auto B = Dpv - Dps * Dsv / Dss;
	auto C = Dpp - Dps * Dps / Dss - r * r;

	if (A == 0.0f)
		return false;

	// 判別式
	auto discriminant = B * B - A * C;
	if (discriminant < 0.0f)
		return false;

	discriminant = sqrtf(discriminant);

	auto distance = (B - discriminant) / A;

	auto t = distance / segment.length();
	// レイが範囲内か
	if (t < 0.0f) return false;
	if (t > 1.0f) return false;

	if (outHitDistance) *outHitDistance = distance;

	return true;
}

bool Collision3D::intersect(const Segment & segment, const Capsule & capsule, float * outHitDistance) {
	auto capsuleSegment = capsule.segment();
	auto capsuleRadius = capsule.radius();
	auto capsuleStart = capsuleSegment.start();
	auto capsuleEnd = capsuleSegment.end();
	auto s1 = Sphere(capsuleStart, capsuleRadius);
	auto s2 = Sphere(capsuleEnd, capsuleRadius);
	auto v = capsuleSegment.vector();

	auto d = 0.0f;

	auto s1Result = intersect(segment, s1, &d);
	auto hitPos = segment.pointFromDistance(d);
	auto dot = Vector3::dot(v, hitPos - capsuleStart);

	// 球に衝突
	if (s1Result && dot < 0.0f) {
		if (outHitDistance) *outHitDistance = d;
		return true;
	}

	auto s2Result = intersect(segment, s2, &d);
	hitPos = segment.pointFromDistance(d);
	dot = Vector3::dot(-v, hitPos - capsuleEnd);

	// 球に衝突
	if (s2Result && dot < 0.0f) {
		if (outHitDistance) *outHitDistance = d;
		return true;
	}

	auto cResult = intersect(segment, InfCylinder(capsuleSegment.asLine(), capsuleRadius), &d);
	if (!cResult) return false;

	hitPos = segment.pointFromDistance(d);

	dot = Vector3::dot(v, hitPos - capsuleStart);
	if (dot < 0.0f) return false;

	dot = Vector3::dot(-v, hitPos - capsuleEnd);
	if (dot < 0.0f) return false;

	// 円柱に衝突
	if (outHitDistance) *outHitDistance = d;
	return true;
}

bool Collision3D::intersect(const Segment & segment, const OBB & obb, float * outHitDistance) {
	// OBBのローカル空間に線分を変換することで、AABBと線分の判定にする
	auto rotSeg = segment.rotate(obb.rotation().conjugate(), obb.center());

	auto center = obb.center();
	auto halfSize = obb.halfSize();
	auto length = rotSeg.length();

	auto o = rotSeg.start();
	auto d = rotSeg.unitVector();
	auto p1 = center - halfSize;
	auto p2 = center + halfSize;

	auto tInMax = std::numeric_limits<float>::lowest();
	auto tOutMin = std::numeric_limits<float>::max();

	// 各軸の到達点
	for (auto i = 0; i < 3; i++) {
		auto t1 = (p1[i] - o[i]) / d[i];
		auto t2 = (p2[i] - o[i]) / d[i];
		auto tInX = Mathf::cmin(t1, t2);
		auto tOutX = Mathf::cmax(t1, t2);
		tInMax = Mathf::cmax(tInX, tInMax);
		tOutMin = Mathf::cmin(tOutX, tOutMin);
	}

	// 出る時間が入る時間より速い場合衝突していない
	if (tOutMin - tInMax < 0.0f) return false;
	// 線分の範囲外
	if (tInMax > length) return false;
	if (tInMax < 0.0f) return false;

	if (outHitDistance) *outHitDistance = tInMax;
	return true;
}

static float det(const Vector3 & vecA, const Vector3 & vecB, const Vector3 & vecC) {
	return ((vecA.x * vecB.y * vecC.z)
			+ (vecA.y * vecB.z * vecC.x)
			+ (vecA.z * vecB.x * vecC.y)
			- (vecA.x * vecB.z * vecC.y)
			- (vecA.y * vecB.x * vecC.z)
			- (vecA.z * vecB.y * vecC.x));
}

bool Collision3D::intersect(const Segment & segment, const Triangle & triangle, float * outHitDistance) {
	const auto edge1 = triangle[1] - triangle[0];
	const auto edge2 = triangle[2] - triangle[0];

	const auto unitVector = segment.unitVector();
	const auto invUnitVector = -unitVector;

	const auto denominator = det(edge1, edge2, invUnitVector);

	if (denominator <= 0) return false;

	const auto d = segment.start() - triangle[0];

	const auto u = det(d, edge2, invUnitVector) / denominator;
	const auto v = det(edge1, d, invUnitVector) / denominator;
	if (!inRange01(u)) return false;
	if (!inRange01(v)) return false;
	if (u + v > 1.0f) return false;

	const auto t = det(edge1, edge2, d) / denominator;

	if (!inRange(t, 0.0f, segment.length())) return false;

	if (!outHitDistance) return true;

	*outHitDistance = t;

	return true;
}
