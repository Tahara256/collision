#pragma once

#include <LMath.h>

/// <summary> �����N���X </summary>
class Line {

public:

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="position"> ������̈�_ </param>
	/// <param name="vector"> �����x�N�g�� </param>
	Line(const Vector3 & position, const Vector3 & vector);

	/// <summary> ������̈�_���擾���܂��B </summary>
	const Vector3 position() const;

	/// <summary> �����x�N�g�����擾���܂��B </summary>
	const Vector3 vector() const;

	/// <summary> ���K�����ꂽ�����x�N�g�����擾���܂��B </summary>
	const Vector3 unitVector() const;

	/// <summary> ������̔C�ӂ̓_���擾���܂��B </summary>
	const Vector3 point(float distance) const;

	/// <summary> �_��ʂ鐂���̑����擾 </summary>
	const Vector3 perpendicularFoot(const Vector3 & point) const;

	/// <summary> �e�����̂����Е��̒����ɍł��߂��_�����ꂼ��擾���܂��B </summary>
	void closest(const Line & other, Vector3 * outMyClosestPoint, Vector3 * outOtherClosestPoint) const;

	/// <summary> ��̒��������s�����擾���܂��B </summary>
	bool isParallel(const Line & other) const;

	/// <summary> �����Ɠ_�̋������擾���܂��B </summary>
	float distance(const Vector3 & point) const;

	/// <summary> �����Ɠ_�̋����̓����擾���܂��B </summary>
	float sqrDistance(const Vector3 & point) const;

private:

	Vector3 _position;
	Vector3 _vector;

};
