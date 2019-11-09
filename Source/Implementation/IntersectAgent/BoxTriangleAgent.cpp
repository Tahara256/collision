#include <Collider/Implementation/IntersectAgent/BoxTriangleAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/BoxShape.h>
#include <Collider/CollisionShape/TriangleShape.h>
#include <Collider/Implementation/Collision3D.h>

using namespace std;

bool BoxTriangleAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// �R���C�_�[�̌`����擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p�����擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �`��𒼕��̂ƎO�p�`�ɃA�b�v�L���X�g
	const auto boxShape = static_pointer_cast<BoxShape>(shape1);
	const auto triangleShape = static_pointer_cast<TriangleShape>(shape2);

	// �Փ˔���
	return Collision3D::intersect(boxShape->getWorldBox(transform1), triangleShape->getWorldTriangle(transform2), outHitPos1, outHitPos2);
}

bool BoxTriangleAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// �R���C�_�[�̌`����擾
	decltype(auto) shape1 = collider1->getCollisionShape();
	decltype(auto) shape2 = collider2->getCollisionShape();

	// �R���C�_�[�̎p�����擾
	decltype(auto) transform1 = collider1->getTransform();
	decltype(auto) transform2 = collider2->getTransform();

	// �`��𒼕��̂ƎO�p�`�ɃA�b�v�L���X�g
	const auto boxShape1 = static_pointer_cast<BoxShape>(shape1);
	const auto triangleShape = static_pointer_cast<TriangleShape>(shape2);

	// �Փ˔���
	return Collision3D::intersect(boxShape1->getWorldBox(transform1), triangleShape->getWorldTriangle(transform2));
}
