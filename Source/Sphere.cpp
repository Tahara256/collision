#include <Collider/Sphere.h>
#include <Collider/AABB.h>

Sphere::Sphere(const Vector3 & center, float radius) :
	_center(center),
	_radius(radius) {
}

const Vector3 & Sphere::center() const {
	return _center;
}

float Sphere::radius() const {
	return _radius;
}

const AABB Sphere::coverAABB() const {
	decltype(auto) c = center();
	const auto r = radius();
	const auto offset = Vector3::one * r;
	return { c - offset, c + offset };
}
