#pragma once

#include <Collider/Collider/ColliderID.h>

/// <summary> �g���K�[�̐ڐG���N���X </summary>
class TriggerContact {

public:

	/// <summary> �Փˑ����ID���擾���܂��B </summary>
	ColliderID getOtherID() const;

	/// <summary> �Փˑ����ID��ݒ肵�܂��B </summary>
	void setOtherID(ColliderID otherID);

private:

	ColliderID _otherID;

};
