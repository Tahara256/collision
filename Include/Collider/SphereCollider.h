#pragma once

#include "Collider.h"

class Sphere;

/// <summary> ���̂̃R���C�_�[ </summary>
class SphereCollider final : public Collider
{
public:
	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="localSphere"> ���[�J�����W�n�̋��� </param>
	SphereCollider(Sphere const & localSphere);
};
