#include <Collider/InfCylinder.h>

InfCylinder::InfCylinder(Line const & line, float radius) :
	_line(line),
	_radius(radius)
{
}

Line const InfCylinder::line() const
{
	return _line;
}

float InfCylinder::radius() const
{
	return _radius;
}
