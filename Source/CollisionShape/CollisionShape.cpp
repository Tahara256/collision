#include <Collider/CollisionShape/CollisionShape.h>
#include <Collider/Implementation/UniformGridMesh.h>
#include <Collider/Contact/Contact.h>

using namespace std;

bool CollisionShape::intersectMesh(const TransformQ & transform, const UniformGridMesh & mesh) const {
	auto minXIndex = UniformGridMesh::Index();
	auto minZIndex = UniformGridMesh::Index();
	auto maxXIndex = UniformGridMesh::Index();
	auto maxZIndex = UniformGridMesh::Index();

	// 均一格子から自身を覆うAABBが触れている格子の最小添え字と最大添え字を取得する
	mesh.getMinMaxIndex(coverAABB(transform), &minXIndex, &minZIndex, &maxXIndex, &maxZIndex);

	// AABBが触れている格子に格納されているメッシュと衝突判定
	for (auto x = minXIndex; x <= maxXIndex; x++) {
		for (auto z = minZIndex; z <= maxZIndex; z++) {

			// 一つの格子に入っている三角ポリゴン二つ
			decltype(auto) trianglePair = mesh.getTrianglePair(x, z);

			// 三角ポリゴンそれぞれと衝突判定

			auto isCollided = intersectTriangle(transform, trianglePair.first);
			if (isCollided)	return true;

			isCollided = intersectTriangle(transform, trianglePair.second);
			if (isCollided) return true;
		}
	}

	return false;
}

bool CollisionShape::intersectMesh(const TransformQ & transform, const UniformGridMesh & mesh, Vector3 * outMyHitPos, Vector3 * outMeshHitPos) const {
	auto minXIndex = UniformGridMesh::Index();
	auto minZIndex = UniformGridMesh::Index();
	auto maxXIndex = UniformGridMesh::Index();
	auto maxZIndex = UniformGridMesh::Index();

	// 均一格子から自身を覆うAABBが触れている格子の最小添え字と最大添え字を取得する
	mesh.getMinMaxIndex(coverAABB(transform), &minXIndex, &minZIndex, &maxXIndex, &maxZIndex);

	auto maxSqrDist = -numeric_limits<float>::max();
	auto maxHit1 = Vector3();
	auto maxHit2 = Vector3();
	auto anyCollided = false;

	// AABBが触れている格子に格納されているメッシュと衝突判定
	for (auto x = minXIndex; x <= maxXIndex; x++) {
		for (auto z = minZIndex; z <= maxZIndex; z++) {

			// 一つの格子に入っている三角ポリゴン二つ
			decltype(auto) trianglePair = mesh.getTrianglePair(x, z);

			auto hit1 = Vector3();
			auto hit2 = Vector3();

			// 三角ポリゴンそれぞれと衝突判定

			auto isCollided = intersectTriangle(transform, trianglePair.first, &hit1, &hit2);
			if (isCollided) {
				anyCollided = true;

				const auto tempSqrDist = (hit1 - hit2).sqrLength();

				if (tempSqrDist > maxSqrDist) {
					maxSqrDist = tempSqrDist;
					maxHit1 = hit1;
					maxHit2 = hit2;
				}
			}

			isCollided = intersectTriangle(transform, trianglePair.second, &hit1, &hit2);
			if (isCollided) {
				anyCollided = true;

				const auto tempSqrDist = (hit1 - hit2).sqrLength();

				if (tempSqrDist > maxSqrDist) {
					maxSqrDist = tempSqrDist;
					maxHit1 = hit1;
					maxHit2 = hit2;
				}
			}
		}
	}

	// どれとも衝突していない
	if (!anyCollided) return false;

	auto info = Contact();

	// 衝突した場合、最もめり込んでいる三角ポリゴンのみと衝突したことにする

	*outMyHitPos = maxHit1;
	*outMeshHitPos = maxHit2;

	return true;
}
