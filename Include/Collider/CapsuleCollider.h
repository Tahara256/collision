#pragma once

#include "Collider.h"

class Capsule;

/// <summary> �J�v�Z���̃R���C�_�[ </summary>
class CapsuleCollider final : public Collider
{
public:
	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="localCapsule"> ���[�J�����W�n�̃J�v�Z�� </param>
	CapsuleCollider(Capsule const & localCapsule);
};
