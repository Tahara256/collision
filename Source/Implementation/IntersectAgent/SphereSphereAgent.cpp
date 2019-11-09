#include <Collider/Implementation/IntersectAgent/SphereSphereAgent.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/SphereShape.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;

bool SphereSphereAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// �R���C�_�[�̌`��擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p���擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �`������̂ɃA�b�v�L���X�g
	const auto sphereShape1 = static_pointer_cast<SphereShape>(shape1);
	const auto sphereShape2 = static_pointer_cast<SphereShape>(shape2);

	// �Փ˔���
	return Collision3D::intersect(sphereShape1->getWorldSphere(transform1), sphereShape2->getWorldSphere(transform2), outHitPos1, outHitPos2);
}

bool SphereSphereAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// �R���C�_�[�̌`����擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p�����擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �`������̂ɃA�b�v�L���X�g
	const auto sphereShape1 = static_pointer_cast<SphereShape>(shape1);
	const auto sphereShape2 = static_pointer_cast<SphereShape>(shape2);

	// �Փ˔���
	return Collision3D::intersect(sphereShape1->getWorldSphere(transform1), sphereShape2->getWorldSphere(transform2));
}
