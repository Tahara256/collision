#pragma once

#include <list>
#include <Collider/Collider/CollisionCallback.h>
#include <Collider/Collider/ColliderID.h>

class CollisionCallbackManager {

public:

	/// <summary> �R���X�g���N�^ </summary>
	CollisionCallbackManager();

	/// <summary> �R���C�_�[�Փ˒��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnCollisionStay(const OnCollisionCallback & onCollisionStay);

	/// <summary> �R���C�_�[�Փ˂��n�߂��Ƃ��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnCollisionEnter(const OnCollisionCallback & onCollisionEnter);

	/// <summary> �R���C�_�[���Փ˂���߂��Ƃ��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnCollisionExit(const OnCollisionCallback & onCollisionExit);

	/// <summary> �g���K�[�̃R���C�_�[�Փ˒��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnTriggerStay(const OnTriggerCallback & onTriggerStay);

	/// <summary> �g���K�[�̃R���C�_�[�Փ˂��n�߂��Ƃ��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnTriggerEnter(const OnTriggerCallback & onTriggerEnter);

	/// <summary> �g���K�[�̃R���C�_�[���Փ˂���߂��Ƃ��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnTriggerExit(const OnTriggerCallback & onTriggerExit);

	void notifyOnCollisionStay(const Contact & info);
	void notifyOnCollisionEnter(const Contact & info);
	void notifyOnCollisionExit(const Contact & info);

	void notifyOnTriggerStay(const TriggerContact & info);
	void notifyOnTriggerEnter(const TriggerContact & info);
	void notifyOnTriggerExit(const TriggerContact & info);

private:

	/// <summary> �R���C�_�[�̏Փˊ֘A�̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnCollision(OnCollisionCallback & dest, const OnCollisionCallback & source);

	/// <summary> �g���K�[�̃R���C�_�[�̏Փˊ֘A�̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnTrigger(OnTriggerCallback & dest, const OnTriggerCallback & source);

	OnCollisionCallback		_onCollisionStay;
	OnCollisionCallback		_onCollisionEnter;
	OnCollisionCallback		_onCollisionExit;

	OnTriggerCallback		_onTriggerStay;
	OnTriggerCallback		_onTriggerEnter;
	OnTriggerCallback		_onTriggerExit;

};
