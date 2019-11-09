#include <Collider/Implementation/IntersectAgent/BoxBoxAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/BoxShape.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;

bool BoxBoxAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// コライダーの形状を取得
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// コライダーの姿勢を取得
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// 形状を直方体にアップキャスト
	const auto boxShape1 = static_pointer_cast<BoxShape>(shape1);
	const auto boxShape2 = static_pointer_cast<BoxShape>(shape2);

	decltype(auto) box1 = boxShape1->getWorldBox(transform1);
	decltype(auto) box2 = boxShape2->getWorldBox(transform2);

	// 衝突判定
	return Collision3D::intersect(box1, box2, outHitPos1, outHitPos2);
}

bool BoxBoxAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// コライダーの形状を取得
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// コライダーの姿勢を取得
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// 形状を直方体にアップキャスト
	const auto boxShape1 = static_pointer_cast<BoxShape>(shape1);
	const auto boxShape2 = static_pointer_cast<BoxShape>(shape2);

	// 衝突判定
	return Collision3D::intersect(boxShape1->getWorldBox(transform1), boxShape2->getWorldBox(transform2));
}
