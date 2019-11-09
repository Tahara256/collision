#include <Collider/Implementation/IntersectAgent/CapsuleTriangleAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/CapsuleShape.h>
#include <Collider/CollisionShape/TriangleShape.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;

bool CapsuleTriangleAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// �R���C�_�[�̌`����擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p�����擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �`����J�v�Z���ƎO�p�`�ɃA�b�v�L���X�g
	const auto capsuleShape		= static_pointer_cast<CapsuleShape>(shape1);
	const auto triangleShape	= static_pointer_cast<TriangleShape>(shape2);

	// �Փ˔���
	return Collision3D::intersect(capsuleShape->getWorldCapsule(transform1), triangleShape->getWorldTriangle(transform2), outHitPos1, outHitPos2);
}

bool CapsuleTriangleAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// �R���C�_�[�̌`����擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p�����擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �`����J�v�Z���ƎO�p�`�ɃA�b�v�L���X�g
	const auto capsuleShape = static_pointer_cast<CapsuleShape>(shape1);
	const auto triangleShape = static_pointer_cast<TriangleShape>(shape2);

	// �Փ˔���
	return Collision3D::intersect(capsuleShape->getWorldCapsule(transform1), triangleShape->getWorldTriangle(transform2));
}
