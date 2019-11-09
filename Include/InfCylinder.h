#pragma once

#include "Line.h"

/// <summary> –³ŒÀ‰~’ŒƒNƒ‰ƒX </summary>
class InfCylinder
{
public:
	/// <summary> ƒRƒ“ƒXƒgƒ‰ƒNƒ^ </summary>
	/// <param name="segment"> ‰~’Œ‚Ìc‚Ì’¼ü </param>
	/// <param name="radius"> ‰~’Œ‚Ì”¼Œa </param>
	InfCylinder(Line const & line, float radius);

	/// <summary> ‰~’Œ‚Ìc‚Ì’¼ü‚ğæ“¾‚µ‚Ü‚·B </summary>
	Line const line() const;

	/// <summary> ‰~’Œ‚Ì”¼Œa‚ğæ“¾‚µ‚Ü‚·B </summary>
	float radius() const;

private:
	Line _line;
	float _radius;
};
