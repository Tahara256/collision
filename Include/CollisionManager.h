#pragma once

#include <list>
#include <unordered_map>
#include <functional>

#include <Util/Singleton.h>
#include <Collider/Collider/ColliderPtr.h>
#include <Collider/Collider/TerrainColliderPtr.h>
#include <Collider/CollisionFilter/CollisionFilterPtr.h>
#include <Collider/Implementation/SpatialPartition/LinearOctreeManager.h>
#include <Collider/Contact/LinecastContactList.h>
#include <Collider/CollisionFilter/CollisionLayerType.h>
#include <Collider/Collider/ColliderID.h>

class Vector4;
class Segment;
struct Vector3;

/// <summary> �ՓˊǗ��N���X </summary>
class CollisionManager : public Accessor<CollisionManager> {

public:

	/// <summary> �f�t�H���g�R���X�g���N�^ </summary>
	CollisionManager();
	~CollisionManager();

	/// <summary> �R���C�_�[��o�^���܂��B </summary>
	void addCollider(const ColliderPtr & collider);

	/// <summary> �R���C�_�[���폜���܂��B </summary>
	void removeCollider(const ColliderPtr & collider);

	/// <summary> �R���C�_�[��S�폜���܂��B </summary>
	void clear();

	/// <summary> �Փ˔�������܂��B </summary>
	void update();

	/// <summary> ID����R���C�_�[���擾���܂��B </summary>
	const ColliderPtr getCollider(ColliderID id) const;

	/// <summary> �o�^����Ă���R���C�_�[�����C���[�t���[���ŕ`�悵�܂��B </summary>
	void drawWireFrame(const Vector4 & color) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="outInfo"> �ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecast(const Segment & line, LinecastContact * outInfo = nullptr, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="layerMask"> �Փ˔���̃}�X�N </param>
	/// <param name="outInfo"> �ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecast(const Segment & line, CollisionLayerType layerMask, LinecastContact * outInfo = nullptr, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���A��ԋ߂��I�u�W�F�N�g�Ƃ̐ڐG�����擾���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="outInfo"> ��ԋ߂��ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecastClosest(const Segment & line, LinecastContact * outInfo, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���A��ԋ߂��I�u�W�F�N�g�Ƃ̐ڐG�����擾���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="layerMask"> �Փ˔���̃}�X�N </param>
	/// <param name="outInfo"> ��ԋ߂��ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecastClosest(const Segment & line, CollisionLayerType layerMask, LinecastContact * outInfo, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���܂��B�S�Ă̏Փˌ��ʂ��擾���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="outInfos"> �ڐG�����S�Ă̐ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecastAll(const Segment & line, LinecastContactList * outInfos, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���܂��B�S�Ă̏Փˌ��ʂ��擾���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="layerMask"> �Փ˔���̃}�X�N </param>
	/// <param name="outInfos"> �ڐG�������ׂĂ̐ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecastAll(const Segment & line, CollisionLayerType layerMask, LinecastContactList * outInfos, bool ignoreTrigger = true) const;

	/// <summary> �Փ˔���̃t�B���^�[��ݒ肵�܂��B </summary>
	void setCollisionFilter(CollisionFilterPtr && collisionFilter);

private:

	using ColliderList = std::list<ColliderPtr>;
	using ColliderTable = std::unordered_map<ColliderID, ColliderPtr>;

	bool linecast(const Segment & line, const std::function<bool(const ColliderPtr &)> & pre, LinecastContact * outInfo = nullptr) const;

	bool linecastAll(const Segment & line, const std::function<bool(const ColliderPtr &)> & pre, LinecastContactList * outInfos) const;

	/// <summary> �����؂ōœK���ꂽ�Փ˔�������܂��B </summary>
	void octreeCollision();

	/// <summary> ��̃R���C�_�[���Փ˔��肳���܂��B </summary>
	void collision(const ColliderPtr & collider1, const ColliderPtr & collider2) const;


	SpatialPartition::LinearOctreeManager	_octree;
	ColliderTable							_colliderTable;
	CollisionFilterPtr						_collisionFilter;

};
