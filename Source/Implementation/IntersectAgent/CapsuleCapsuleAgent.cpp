#include <Collider/Implementation/IntersectAgent/CapsuleCapsuleAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/CapsuleShape.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;

bool CapsuleCapsuleAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// �R���C�_�[�̌`����擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p�����擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �`����J�v�Z���ɃA�b�v�L���X�g
	decltype(auto) capsuleShape1 = static_pointer_cast<CapsuleShape>(shape1);
	decltype(auto) capsuleShape2 = static_pointer_cast<CapsuleShape>(shape2);

	// �Փ˔���
	return Collision3D::intersect(capsuleShape1->getWorldCapsule(transform1), capsuleShape2->getWorldCapsule(transform2), outHitPos1, outHitPos2);
}

bool CapsuleCapsuleAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// �R���C�_�[�̌`����擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p�����擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �`����J�v�Z���ɃA�b�v�L���X�g
	const auto capsuleShape1 = static_pointer_cast<CapsuleShape>(shape1);
	const auto capsuleShape2 = static_pointer_cast<CapsuleShape>(shape2);

	// �Փ˔���
	return Collision3D::intersect(capsuleShape1->getWorldCapsule(transform1), capsuleShape2->getWorldCapsule(transform2));
}
