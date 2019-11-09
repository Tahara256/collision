#pragma once

#include <Collider/Collider/ColliderPtr.h>

/// <summary> �Փ˔���̃t�B���^�[�C���^�t�F�[�X </summary>
class CollisionFilter {

public:

	/// <summary> ��̃��C���[�̃R���C�_�[���Փ˔��肷�邩���擾���܂��B </summary>
	virtual std::pair<bool, bool> filter(const ColliderPtr & layer1, const ColliderPtr & layer2) const = 0;

	virtual ~CollisionFilter() = default;

};
