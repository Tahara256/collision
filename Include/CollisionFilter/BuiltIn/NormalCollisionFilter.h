#pragma once

#include <Collider/CollisionFilter/CollisionFilter.h>
#include <unordered_map>
#include <Collider/CollisionFilter/CollisionLayerType.h>

/// <summary> �Փ˔���t�B���^�[�̒ʏ�̎��� </summary>
class NormalCollisionFilter : public CollisionFilter {

public:

	/// <summary> �w�肳�ꂽ���C���[�����R���C�_�[�Ԃ̏Փ˔����L���ɂ��܂��B </summary>
	void enableCollisionBetween(CollisionLayerType layer1, CollisionLayerType layer2);

	/// <summary> �w�肳�ꂽ���C���[�����R���C�_�[�Ԃ̏Փ˔���𖳌��ɂ��܂��B </summary>
	void disableCollisionBetween(CollisionLayerType layer1, CollisionLayerType layer2);


	/*-- CollisionFilter�̏������z�̎��� --*/

	/// <summary> ��̃��C���[�̃R���C�_�[���Փ˔��肷�邩���擾���܂��B </summary>
	virtual std::pair<bool, bool> filter(const ColliderPtr & layer1, const ColliderPtr & layer2) const override;

private:

	virtual bool globalFilter(CollisionLayerType layer1, CollisionLayerType layer2) const;

	// �R���W�������C���[�̃}�g���b�N�X
	using FilterTable = std::unordered_map<CollisionLayerType, CollisionLayerType>;

	FilterTable _table;
};