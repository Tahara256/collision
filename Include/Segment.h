#pragma once

#include <LMath.h>

class Line;
class AABB;

/// <summary> �����N���X </summary>
class Segment {

public:

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="start"> �n�_ </param>
	/// <param name="end"> �I�_ </param>
	Segment(const Vector3 & start, const Vector3 & end);

	/// <summary> �f�t�H���g�R���X�g���N�^ </summary>
	Segment() = default;

	/// <summary> �n�_���擾���܂��B </summary>
	const Vector3 start() const;

	/// <summary> �I�_���擾���܂��B </summary>
	const Vector3 end() const;

	/// <summary> �����̒��_���擾���܂��B </summary>
	const Vector3 midpoint() const;

	/// <summary> �n�_����I�_�ւ̃x�N�g�����擾���܂��B </summary>
	const Vector3 vector() const;

	/// <summary> �n�_���璆�_�ւ̃x�N�g�����擾���܂��B </summary>
	const Vector3 halfVector() const;

	/// <summary> ���K�����ꂽ�����x�N�g�����擾���܂��B </summary>
	const Vector3 unitVector() const;

	/// <summary> �����̒������擾���܂��B </summary>
	float length() const;

	/// <summary> �����̒����̓����擾���܂��B </summary>
	float sqrLength() const;

	/// <summary> �n�_����̋������������̓_���擾���܂��B </summary>
	const Vector3 pointFromDistance(float distance) const;

	/// <summary> ���̐������܂ޒ������擾���܂��B </summary>
	const Line asLine() const;

	/// <summary> ���s�ړ������������擾���܂��B </summary>
	const Segment translate(const Vector3 & vector) const;

	/// <summary> �C�ӂ̓_�𒆐S�ɉ�]�������������擾���܂��B </summary>
	const Segment rotate(const Quaternion & rotation, const Vector3 & origin) const;

	/// <summary> ���_�𒆐S�ɉ�]�������������擾���܂��B </summary>
	const Segment rotateOrigin(const Quaternion & rotation) const;

	/// <summary> �n�_�𒆐S�ɉ�]�������������擾���܂��B </summary>
	const Segment rotateStart(const Quaternion & rotation) const;

	/// <summary> �������m�̈�ԋ߂���_���擾���܂��B </summary>
	void closest(const Segment & other, Vector3 * outMyShortest, Vector3 * outOtherShortest) const;

	/// <summary> �_�ɍł��߂�������̍��W���擾���܂��B </summary>
	const Vector3 closest(const Vector3 & point) const;

	/// <summary> ���g�Ƒ���̍ł��߂��[�_���擾���܂��B </summary>
	void closestEnd(const Segment & other, Vector3 * myEnd, Vector3 * otherEnd) const;

	/// <summary> �_��ʂ鐂���̑����擾 </summary>
	const Vector3 perpendicularFoot(const Vector3 & point) const;

	/// <summary> �_���牺�낵�������������͈͓̔������擾���܂��B </summary>
	bool inRangePerpendicularFoot(const Vector3 & point, Vector3 * outFootPoint = nullptr) const;

	/// <summary> ��̐��������s�����擾���܂��B </summary>
	bool isParallel(const Segment & segment) const;

	/// <summary> �_�Ɛ����̋������擾���܂��B </summary>
	float distance(const Vector3 & point, Vector3 * outClosestPoint) const;

	/// <summary> �_�Ɛ����̋����̓����擾���܂��B </summary>
	float sqrDistance(const Vector3 & point, Vector3 * outClosestPoint) const;

	/// <summary> ���g�����ʂ̊O���ɑ��݂��邩���擾���܂��B </summary>
	bool isPlaneOutside(const Vector3 & planeNormal, const Vector3 & planePoint) const;

	/// <summary> ���g�����S�ɕ���AABB���擾���܂��B </summary>
	const AABB coverAABB() const;

private:

	Vector3 _start;
	Vector3 _end;

};
