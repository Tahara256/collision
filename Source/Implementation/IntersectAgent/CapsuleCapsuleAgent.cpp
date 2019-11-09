#include <Collider/Implementation/IntersectAgent/CapsuleCapsuleAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/CapsuleShape.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;

bool CapsuleCapsuleAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// コライダーの形状を取得
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// コライダーの姿勢を取得
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// 形状をカプセルにアップキャスト
	decltype(auto) capsuleShape1 = static_pointer_cast<CapsuleShape>(shape1);
	decltype(auto) capsuleShape2 = static_pointer_cast<CapsuleShape>(shape2);

	// 衝突判定
	return Collision3D::intersect(capsuleShape1->getWorldCapsule(transform1), capsuleShape2->getWorldCapsule(transform2), outHitPos1, outHitPos2);
}

bool CapsuleCapsuleAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// コライダーの形状を取得
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// コライダーの姿勢を取得
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// 形状をカプセルにアップキャスト
	const auto capsuleShape1 = static_pointer_cast<CapsuleShape>(shape1);
	const auto capsuleShape2 = static_pointer_cast<CapsuleShape>(shape2);

	// 衝突判定
	return Collision3D::intersect(capsuleShape1->getWorldCapsule(transform1), capsuleShape2->getWorldCapsule(transform2));
}
