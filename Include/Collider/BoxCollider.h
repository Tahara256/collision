#pragma once

#include "Collider.h"

class OBB;

/// <summary> �����̂̃R���C�_�[ </summary>
class BoxCollider final : public Collider
{
public:
	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="localBox"> ���[�J�����W�n�̒����� </param>
	BoxCollider(OBB const & localBox);
};
