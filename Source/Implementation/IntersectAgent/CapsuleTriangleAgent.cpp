#include <Collider/Implementation/IntersectAgent/CapsuleTriangleAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/CapsuleShape.h>
#include <Collider/CollisionShape/TriangleShape.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;

bool CapsuleTriangleAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// コライダーの形状を取得
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// コライダーの姿勢を取得
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// 形状をカプセルと三角形にアップキャスト
	const auto capsuleShape		= static_pointer_cast<CapsuleShape>(shape1);
	const auto triangleShape	= static_pointer_cast<TriangleShape>(shape2);

	// 衝突判定
	return Collision3D::intersect(capsuleShape->getWorldCapsule(transform1), triangleShape->getWorldTriangle(transform2), outHitPos1, outHitPos2);
}

bool CapsuleTriangleAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// コライダーの形状を取得
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// コライダーの姿勢を取得
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// 形状をカプセルと三角形にアップキャスト
	const auto capsuleShape = static_pointer_cast<CapsuleShape>(shape1);
	const auto triangleShape = static_pointer_cast<TriangleShape>(shape2);

	// 衝突判定
	return Collision3D::intersect(capsuleShape->getWorldCapsule(transform1), triangleShape->getWorldTriangle(transform2));
}
