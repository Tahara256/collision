#pragma once

#include "Line.h"

/// <summary> �����~���N���X </summary>
class InfCylinder
{
public:
	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="segment"> �~���̐c�̒��� </param>
	/// <param name="radius"> �~���̔��a </param>
	InfCylinder(Line const & line, float radius);

	/// <summary> �~���̐c�̒������擾���܂��B </summary>
	Line const line() const;

	/// <summary> �~���̔��a���擾���܂��B </summary>
	float radius() const;

private:
	Line _line;
	float _radius;
};
