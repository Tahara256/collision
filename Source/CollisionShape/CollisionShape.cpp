#include <Collider/CollisionShape/CollisionShape.h>
#include <Collider/Implementation/UniformGridMesh.h>
#include <Collider/Contact/Contact.h>

using namespace std;

bool CollisionShape::intersectMesh(const TransformQ & transform, const UniformGridMesh & mesh) const {
	auto minXIndex = UniformGridMesh::Index();
	auto minZIndex = UniformGridMesh::Index();
	auto maxXIndex = UniformGridMesh::Index();
	auto maxZIndex = UniformGridMesh::Index();

	// �ψ�i�q���玩�g�𕢂�AABB���G��Ă���i�q�̍ŏ��Y�����ƍő�Y�������擾����
	mesh.getMinMaxIndex(coverAABB(transform), &minXIndex, &minZIndex, &maxXIndex, &maxZIndex);

	// AABB���G��Ă���i�q�Ɋi�[����Ă��郁�b�V���ƏՓ˔���
	for (auto x = minXIndex; x <= maxXIndex; x++) {
		for (auto z = minZIndex; z <= maxZIndex; z++) {

			// ��̊i�q�ɓ����Ă���O�p�|���S�����
			decltype(auto) trianglePair = mesh.getTrianglePair(x, z);

			// �O�p�|���S�����ꂼ��ƏՓ˔���

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

	// �ψ�i�q���玩�g�𕢂�AABB���G��Ă���i�q�̍ŏ��Y�����ƍő�Y�������擾����
	mesh.getMinMaxIndex(coverAABB(transform), &minXIndex, &minZIndex, &maxXIndex, &maxZIndex);

	auto maxSqrDist = -numeric_limits<float>::max();
	auto maxHit1 = Vector3();
	auto maxHit2 = Vector3();
	auto anyCollided = false;

	// AABB���G��Ă���i�q�Ɋi�[����Ă��郁�b�V���ƏՓ˔���
	for (auto x = minXIndex; x <= maxXIndex; x++) {
		for (auto z = minZIndex; z <= maxZIndex; z++) {

			// ��̊i�q�ɓ����Ă���O�p�|���S�����
			decltype(auto) trianglePair = mesh.getTrianglePair(x, z);

			auto hit1 = Vector3();
			auto hit2 = Vector3();

			// �O�p�|���S�����ꂼ��ƏՓ˔���

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

	// �ǂ�Ƃ��Փ˂��Ă��Ȃ�
	if (!anyCollided) return false;

	auto info = Contact();

	// �Փ˂����ꍇ�A�ł��߂荞��ł���O�p�|���S���݂̂ƏՓ˂������Ƃɂ���

	*outMyHitPos = maxHit1;
	*outMeshHitPos = maxHit2;

	return true;
}
