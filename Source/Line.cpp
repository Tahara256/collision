#include <Collider/Line.h>

Line::Line(const Vector3 & position, const Vector3 & vector) :
	_position(position),
	_vector(vector) {
}

const Vector3 Line::position() const {
	return _position;
}

const Vector3 Line::vector() const {
	return _vector;
}

const Vector3 Line::unitVector() const {
	return vector().normalize();
}

const Vector3 Line::point(float distance) const {
	return position() + unitVector() * distance;
}

const Vector3 Line::perpendicularFoot(const Vector3 & p) const {
	const auto toPoint = p - position();
	decltype(auto) unit = unitVector();
	const auto distance = Vector3::dot(unit, toPoint);
	return position() + unit * distance;
}

void Line::closest(const Line & other, Vector3 * outMyClosestPoint, Vector3 * outOtherClosestPoint) const {
	decltype(auto) v1 = vector();
	decltype(auto) v2 = other.vector();
	auto crossSqrLength = Vector3::cross(v1, v2).sqrLength();

	// ïΩçsÇæÇ¡ÇΩèÍçá
	if (crossSqrLength < Mathf::Eps) {
		*outMyClosestPoint = position();
		*outOtherClosestPoint = other.position();
	}

	decltype(auto) uv1 = unitVector();
	decltype(auto) uv2 = other.unitVector();

	decltype(auto) p1 = position();
	decltype(auto) p2 = other.position();

	const auto p1p2 = p2 - p1;

	const auto d1 = Vector3::dot(p1p2, uv1);
	const auto dv = Vector3::dot(uv1, uv2);
	const auto d2 = Vector3::dot(p1p2, uv2);

	const auto t1 = (d1 - d2 * dv) / (1.0f - dv * dv);
	const auto t2 = (d2 - d1 * dv) / (dv * dv - 1.0f);

	if (outMyClosestPoint) {
		const auto q1 = p1 + t1 * uv1;
		*outMyClosestPoint = q1;
	}
	if (outOtherClosestPoint) {
		const auto q2 = p2 + t2 * uv2;
		*outOtherClosestPoint = q2;
	}
}

bool Line::isParallel(const Line & other) const {
	decltype(auto) v1 = vector();
	decltype(auto) v2 = other.vector();
	const auto crossSqrLength = Vector3::cross(v1, v2).sqrLength();
	// ïΩçsÇæÇ¡ÇΩèÍçá
	return crossSqrLength < Mathf::Eps;
}

float Line::distance(const Vector3 & point) const {
	return Vector3::distance(perpendicularFoot(point), point);
}

float Line::sqrDistance(const Vector3 & point) const {
	return Vector3::sqrDistance(perpendicularFoot(point), point);
}
