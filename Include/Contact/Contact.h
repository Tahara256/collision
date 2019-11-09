#pragma once

#include <LMath.h>
#include <Collider/Collider/ColliderID.h>

/// <summary> �ڐG���N���X </summary>
class Contact {

public:

	/// <summary> �Փˈʒu���擾���܂��B </summary>
	const Vector3 getPosition() const;

	/// <summary> �����o���x�N�g�����擾���܂��B </summary>
	const Vector3 getExtrusionVector() const;

	/// <summary> �Փˑ����ID���擾���܂��B </summary>
	ColliderID getOtherID() const;

	/// <summary> �Փˈʒu��ݒ肵�܂��B </summary>
	void setPosition(const Vector3 & hitPosition);

	/// <summary> �����o���x�N�g����ݒ肵�܂��B </summary>
	void setExtrusionVector(const Vector3 & extrusionVector);

	/// <summary> �Փˑ����ID��ݒ肵�܂��B </summary>
	void setOtherID(ColliderID otherID);

private:

	Vector3		_hitPosition;
	Vector3		_extrusionVector;
	ColliderID	_otherID;

};
