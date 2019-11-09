#pragma once

#include <LMath.h>

class AABB;

/// <summary> ���̃N���X </summary>
class Sphere {

public:

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="center"> ���̂̒��S���W </param>
	/// <param name="radius"> ���̂̔��a </param>
	Sphere(const Vector3 & center, float radius);

	/// <summary> ���̂̒��S���W���擾���܂��B </summary>
	const Vector3 & center() const;

	/// <summary> ���̂̔��a���擾���܂��B </summary>
	float radius() const;

	/// <summary> ���g�����S�ɕ���AABB���擾���܂��B </summary>
	const AABB coverAABB() const;

private:

	Vector3 _center;
	float _radius;

};
