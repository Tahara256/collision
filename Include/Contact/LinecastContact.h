#pragma once

#include <LMath.h>
#include <Collider/Collider/ColliderID.h>

/// <summary> ���C���L���X�g�ڐG���N���X </summary>
class LinecastContact {

public:

	/// <summary> �Փˈʒu���擾���܂��B </summary>
	const Vector3 getPosition() const;

	/// <summary> ���C���̎n�_����Փˈʒu�܂ł̋������擾���܂��B </summary>
	float getDistance() const;

	/// <summary> �Փˑ����ID���擾���܂��B </summary>
	ColliderID getOtherID() const;

	/// <summary> �Փˈʒu��ݒ肵�܂��B </summary>
	void setPosition(const Vector3 & position);

	/// <summary> ������ݒ肵�܂��B </summary>
	void setDistance(float distance);

	/// <summary> �Փˑ����ID��ݒ肵�܂��B </summary>
	void setOtherID(ColliderID otherID);

	LinecastContact();

private:

	Vector3		_position;
	float		_distance;
	ColliderID	_otherID;

};
