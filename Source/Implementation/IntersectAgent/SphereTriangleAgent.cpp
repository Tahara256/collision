#include <Collider/Implementation/IntersectAgent/SphereTriangleAgent.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/SphereShape.h>
#include <Collider/CollisionShape/TriangleShape.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;

bool SphereTriangleAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// コライダーの形状取得
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// コライダーの姿勢取得
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// 形状を球体と三角形にアップキャスト
	const auto sphereShape		= static_pointer_cast<SphereShape>(shape1);
	const auto triangleShape	= static_pointer_cast<TriangleShape>(shape2);

	// 衝突判定
	return Collision3D::intersect(sphereShape->getWorldSphere(transform1), triangleShape->getWorldTriangle(transform2), outHitPos1, outHitPos2);
}

bool SphereTriangleAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// コライダーの形状を取得
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// コライダーの姿勢を取得
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// 形状を球体と三角形にアップキャスト
	const auto sphereShape		= static_pointer_cast<SphereShape>(shape1);
	const auto triangleShape	= static_pointer_cast<TriangleShape>(shape2);

	// 衝突判定
	return Collision3D::intersect(sphereShape->getWorldSphere(transform1), triangleShape->getWorldTriangle(transform2));
}
