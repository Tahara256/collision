#include <Collider/Capsule.h>
#include <Collider/AABB.h>

Capsule::Capsule(Segment const & segment, float radius) :
	_segment(segment), _radius(radius)
{
}

Capsule::Capsule(float halfHeight, float radius) :
	_segment(Segment(-Vector3::up * Mathf::abs(halfHeight - radius), Vector3::up * Mathf::abs(halfHeight - radius))), _radius(radius)
{
}

Segment const Capsule::segment() const
{
	return _segment;
}

float Capsule::radius() const
{
	return _radius;
}

float Capsule::height() const
{
	return _segment.length() + _radius + _radius;
}

float Capsule::halfHeight() const
{
	return height() / 2.0f;
}

AABB const Capsule::coverAABB() const
{
	auto segAABB = segment().coverAABB();
	auto rad = radius();
	return { segAABB.min - Vector3::one * rad, segAABB.max + Vector3::one * rad };
}
