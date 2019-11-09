#include <Collider/Implementation/IntersectAgent/SphereCapsuleAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/SphereShape.h>
#include <Collider/CollisionShape/CapsuleShape.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;

bool SphereCapsuleAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// �R���C�_�[�̌`��擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p���擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �Փˌ`������̂ƃJ�v�Z���ɃA�b�v�L���X�g
	const auto sphereShape	= static_pointer_cast<SphereShape>(shape1);
	const auto capsuleShape = static_pointer_cast<CapsuleShape>(shape2);

	// �Փ˔���
	auto sphereHitPos	= Vector3();
	auto capsuleHitPos	= Vector3();
	return Collision3D::intersect(sphereShape->getWorldSphere(transform1), capsuleShape->getWorldCapsule(transform2), outHitPos1, outHitPos2);
}

bool SphereCapsuleAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// �R���C�_�[�̌`����擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p�����擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �`������̂ƃJ�v�Z���ɃA�b�v�L���X�g
	const auto sphereShape	= static_pointer_cast<SphereShape>(shape1);
	const auto capsuleShape = static_pointer_cast<CapsuleShape>(shape2);

	// �Փ˔���
	return Collision3D::intersect(sphereShape->getWorldSphere(transform1), capsuleShape->getWorldCapsule(transform2));
}
