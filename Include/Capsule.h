#pragma once

#include "Segment.h"

class AABB;

/// <summary> �J�v�Z���N���X </summary>
class Capsule
{
public:
	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="segment"> �J�v�Z���̐c�̐��� </param>
	/// <param name="radius"> �J�v�Z���̔��a </param>
	Capsule(Segment const & segment, float radius);

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="halfHeight"> �J�v�Z���̔����̍���(�ۂ��������܂�) </param>
	/// <param name="radius"> �J�v�Z���̔��a </param>
	Capsule(float halfHeight, float radius);

	/// <summary> �J�v�Z���̐c�̐������擾���܂��B </summary>
	Segment const segment() const;

	/// <summary> �J�v�Z���̔��a���擾���܂��B </summary>
	float radius() const;

	/// <summary> �J�v�Z���̒������擾���܂��B </summary>
	float height() const;

	/// <summary> �J�v�Z���̒����̔������擾���܂��B </summary>
	float halfHeight() const;

	/// <summary> ���g�����S�ɕ���AABB���擾���܂��B </summary>
	AABB const coverAABB() const;

private:
	Segment _segment;
	float _radius;
};
