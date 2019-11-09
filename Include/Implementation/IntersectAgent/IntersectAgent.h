#pragma once

#include "../../Collider/ColliderPtr.h"

struct Vector3;

/// <summary> �Փ˔���㗝�N���X </summary>
class IntersectAgent {

public:

	/// <summary> �Փ˔�������܂��B�R���C�_�[�̎�ޖ��ɓK�؂ȋ�ۃN���X���g�p���Ă��������B </summary>
	/// <param name="collider1"> ��ڂ̃R���C�_�[ </param>
	/// <param name="collider2"> ��ڂ̃R���C�_�[ </param>
	virtual bool intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const = 0;

	/// <summary> �g���K�[�̐ڐG��������܂��B�R���C�_�[�̎�ނ��ƂɓK�؂ȋ�ۃN���X���g�p���Ă��������B </summary>
	/// <param name="collider1"> ��ڂ̃R���C�_�[ </param>
	/// <param name="collider2"> ��ڂ̃R���C�_�[ </param>
	virtual bool intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const = 0;


	virtual ~IntersectAgent() = default;

};
