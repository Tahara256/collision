#pragma once

#include <Util/Design/Singleton.h>
#include <Util/Util.h>
#include <unordered_map>
#include <Collider/Collider/ColliderID.h>
#include <Collider/CollisionFilter/CollisionLayerType.h>
#include <Collider/LinecastHit.h>
#include <Collider/LinecastHitList.h>

class ColliderComponent;
class Vector4;
class Segment;

/// <summary> �R���C�_�[�R���|�[�l���g���Ǘ�����N���X </summary>
class ColliderComponentManager : public Singleton<ColliderComponentManager> {

public:

	/// <summary> �V�[����Ƀ��C�����L���X�g���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="outInfo"> �ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecast(const Segment & line, LinecastHit * outInfo = nullptr, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="layerMask"> �Փ˔���̃}�X�N </param>
	/// <param name="outInfo"> �ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecast(const Segment & line, CollisionLayerType layerMask, LinecastHit * outInfo = nullptr, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���A��ԋ߂��I�u�W�F�N�g�Ƃ̐ڐG�����擾���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="outInfo"> ��ԋ߂��ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecastClosest(const Segment & line, LinecastHit * outInfo, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���A��ԋ߂��I�u�W�F�N�g�Ƃ̐ڐG�����擾���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="layerMask"> �Փ˔���̃}�X�N </param>
	/// <param name="outInfo"> ��ԋ߂��ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecastClosest(const Segment & line, CollisionLayerType layerMask, LinecastHit * outInfo, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���܂��B�S�Ă̏Փˌ��ʂ��擾���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="outInfos"> �ڐG�����S�Ă̐ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecastAll(const Segment & line, LinecastHitList * outInfos, bool ignoreTrigger = true) const;

	/// <summary> �V�[����Ƀ��C�����L���X�g���܂��B�S�Ă̏Փˌ��ʂ��擾���܂��B </summary>
	/// <param name="line"> �L���X�g���郉�C�� </param>
	/// <param name="layerMask"> �Փ˔���̃}�X�N </param>
	/// <param name="outInfos"> �ڐG�������ׂĂ̐ڐG��� </param>
	/// <param name="ignoreTrigger"> �g���K�[�̃R���C�_�[�𖳎����邩 </param>
	/// <returns> �R���C�_�[�ɐڐG������ </returns>
	bool linecastAll(const Segment & line, CollisionLayerType layerMask, LinecastHitList * outInfos, bool ignoreTrigger = true) const;

	/// <summary> ID����R���C�_�[�R���|�[�l���g���擾���܂��B </summary>
	RefPtr<ColliderComponent> getColliderComponent(ColliderID id) const;

	/// <summary> �eColliderComponent���X�V���܂��B���t���[�����Ăяo���Ă��������B </summary>
	void update();

	/// <summary> �o�^����Ă���R���C�_�[�����C���[�t���[���ŕ`�悵�܂��B </summary>
	void drawWireFrame(const Vector4 & color) const;

	/// <summary> �R���C�_�[�R���|�[�l���g��ǉ����܂��B </summary>
	void addColliderComponent(RefPtr<ColliderComponent> colliderComponent);

	/// <summary> �R���C�_�[�R���|�[�l���g���폜���܂��B </summary>
	void removeColliderComponent(RefPtr<ColliderComponent> colliderComponent);

private:

	std::unordered_map<ColliderID, RefPtr<ColliderComponent>> _table;

};
