#include <Collider/CollisionShape/CollisionShape.h>
#include <Collider/Implementation/UniformGridMesh.h>
#include <Collider/Contact/Contact.h>

using namespace std;

bool CollisionShape::intersectMesh(const TransformQ & transform, const UniformGridMesh & mesh) const {
	auto minXIndex = UniformGridMesh::Index();
	auto minZIndex = UniformGridMesh::Index();
	auto maxXIndex = UniformGridMesh::Index();
	auto maxZIndex = UniformGridMesh::Index();

	// ‹ÏˆêŠiq‚©‚ç©g‚ğ•¢‚¤AABB‚ªG‚ê‚Ä‚¢‚éŠiq‚ÌÅ¬“Y‚¦š‚ÆÅ‘å“Y‚¦š‚ğæ“¾‚·‚é
	mesh.getMinMaxIndex(coverAABB(transform), &minXIndex, &minZIndex, &maxXIndex, &maxZIndex);

	// AABB‚ªG‚ê‚Ä‚¢‚éŠiq‚ÉŠi”[‚³‚ê‚Ä‚¢‚éƒƒbƒVƒ…‚ÆÕ“Ë”»’è
	for (auto x = minXIndex; x <= maxXIndex; x++) {
		for (auto z = minZIndex; z <= maxZIndex; z++) {

			// ˆê‚Â‚ÌŠiq‚É“ü‚Á‚Ä‚¢‚éOŠpƒ|ƒŠƒSƒ““ñ‚Â
			decltype(auto) trianglePair = mesh.getTrianglePair(x, z);

			// OŠpƒ|ƒŠƒSƒ“‚»‚ê‚¼‚ê‚ÆÕ“Ë”»’è

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

	// ‹ÏˆêŠiq‚©‚ç©g‚ğ•¢‚¤AABB‚ªG‚ê‚Ä‚¢‚éŠiq‚ÌÅ¬“Y‚¦š‚ÆÅ‘å“Y‚¦š‚ğæ“¾‚·‚é
	mesh.getMinMaxIndex(coverAABB(transform), &minXIndex, &minZIndex, &maxXIndex, &maxZIndex);

	auto maxSqrDist = -numeric_limits<float>::max();
	auto maxHit1 = Vector3();
	auto maxHit2 = Vector3();
	auto anyCollided = false;

	// AABB‚ªG‚ê‚Ä‚¢‚éŠiq‚ÉŠi”[‚³‚ê‚Ä‚¢‚éƒƒbƒVƒ…‚ÆÕ“Ë”»’è
	for (auto x = minXIndex; x <= maxXIndex; x++) {
		for (auto z = minZIndex; z <= maxZIndex; z++) {

			// ˆê‚Â‚ÌŠiq‚É“ü‚Á‚Ä‚¢‚éOŠpƒ|ƒŠƒSƒ““ñ‚Â
			decltype(auto) trianglePair = mesh.getTrianglePair(x, z);

			auto hit1 = Vector3();
			auto hit2 = Vector3();

			// OŠpƒ|ƒŠƒSƒ“‚»‚ê‚¼‚ê‚ÆÕ“Ë”»’è

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

	// ‚Ç‚ê‚Æ‚àÕ“Ë‚µ‚Ä‚¢‚È‚¢
	if (!anyCollided) return false;

	auto info = Contact();

	// Õ“Ë‚µ‚½ê‡AÅ‚à‚ß‚è‚ñ‚Å‚¢‚éOŠpƒ|ƒŠƒSƒ“‚Ì‚İ‚ÆÕ“Ë‚µ‚½‚±‚Æ‚É‚·‚é

	*outMyHitPos = maxHit1;
	*outMeshHitPos = maxHit2;

	return true;
}
