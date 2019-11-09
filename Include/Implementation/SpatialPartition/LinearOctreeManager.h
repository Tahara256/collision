#pragma once

#include <list>

#include <LMath.h>
#include <Collider/AABB.h>
#include <Collider/Implementation/SpatialPartition/LinearTree.h>
#include <Collider/Implementation/SpatialPartition/SpaceCell.h>

namespace SpatialPartition {

/// <summary> ���`�����؂��Ǘ�����N���X </summary>
class LinearOctreeManager {

public:

	using CollisionPair = std::pair<OctreeObjectPtr, OctreeObjectPtr>;
	using CollisionPairList = std::list<CollisionPair>;
	using CollisionList = std::list<OctreeObjectPtr>;
	using CollisionStack = std::list<OctreeObjectPtr>;

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="area"> �����؂ɊǗ��������Ԃ�\��AABB </param>
	/// <param name="level"> ��ԕ�����(���̉񐔂�����Ԃ𔪓�������) </param>
	LinearOctreeManager(const AABB & area, int32 level);

	/// <summary> �����؂ŊǗ�����I�u�W�F�N�g��ǉ����܂��B </summary>
	void addObject(const OctreeObjectPtr & object);

	/// <summary> �����؂ŊǗ�����I�u�W�F�N�g���폜���܂��B </summary>
	void removeObject(const OctreeObjectPtr & object);

	/// <summary> �����؂ŊǗ�����I�u�W�F�N�g��S�폜���܂��B </summary>
	void clearObject();

	/// <summary> �ÓI�ł͂Ȃ��I�u�W�F�N�g�𔪕��؂ɍĔz�u���܂��B </summary>
	void update();

	/// <summary> �Փ˂��Ă���\���̂���I�u�W�F�N�g�̃y�A�̃��X�g���擾���܂��B </summary>
	/// <param name="outCollisionPairList"> �o�͂��Ăق������X�g�ւ̃|�C���^ </param>
	void getCollisionPairList(CollisionPairList * outCollisionPairList) const;

	/// <summary> AABB���L���X�g���A�Փ˂��Ă���\���̂���I�u�W�F�N�g�̃��X�g���擾���܂��B </summary>
	/// <param name="coverAABB"> �L���X�g����AABB </param>
	/// <param name="outCollisionList"> �Փ˂��Ă���\���̂���I�u�W�F�N�g���o�͂��郊�X�g�ւ̃|�C���^ </param>
	void traverseAABB(const AABB & coverAABB, CollisionList * outCollisionList) const;

	/// <summary> �����؂��Ǘ������Ԃ𗧕��̂̃��C���[�t���[���ŕ`�� </summary>
	void drawWireFrame(const Vector4 & color) const;

private:

	static constexpr int32 BranchCount = 8;

	void updateObject(const OctreeObjectPtr & object);

	void detachObject(const OctreeObjectPtr & object);

	/// <summary> �ċA�Ŕ����؂̑S�m�[�h�����񂵁A�Փ˂���\���̂���I�u�W�F�N�g�̃y�A�����X�g�ɋl�߂܂��B </summary>
	/// <param name="index"> ���݂̃m�[�h </param>
	/// <param name="colliderList"> �l�߂郊�X�g </param>
	/// <param name="colliderStac"> ���݂̃m�[�h�ƏՓ˂��Ă���\���̂���I�u�W�F�N�g�̃X�^�b�N </param>
	void createCollisionPairList(int32 index, CollisionPairList * colliderList, CollisionStack * colliderStac) const;

	void createCollisionListAll(int32 index, CollisionList * outCollisionList) const;

	void createCollisionList(int32 index, CollisionList * outCollisionList) const;

	void createCollisionListChild(int32 index, CollisionList * outCollisionList) const;

	void createCollisionListParent(int32 index, CollisionList * outCollisionList) const;

	void cellSetUp(int32 cellIndex);

	LinearTree<SpaceCell, BranchCount>	_octree;
	Vector3								_unitSize;
	int32								_level;
	AABB								_area;
	int8								_maxIndex;
	std::list<OctreeObjectPtr>			_objects;

};

} // namespace SpatialPartition
