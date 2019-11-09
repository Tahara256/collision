#include <Collider/CollisionShape/TriangleShape.h>
#include <Collider/AABB.h>
#include <Collider/CollisionShape/CollisionShapeType.h>
#include <Collider/Implementation/CollisionUtility.h>
#include <Renderer/SceneRendererManager.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Contact/LinecastContact.h>

using namespace CollisionUtility;

TriangleShape::TriangleShape(const Triangle & localTriangle) :
	_baseTriangle(localTriangle) {
}

const AABB TriangleShape::coverAABB(TransformQ const & transform) const {
	return coverAABBFromFace(getWorldTriangle(transform));
}

CollisionShapeType TriangleShape::getType() const {
	return CollisionShapeType::Triangle;
}

bool TriangleShape::linecast(const TransformQ & transform, const Segment & line, LinecastContact * outInfo) const {
	auto distance = 0.0f;
	const auto isCollided = Collision3D::intersect(line, getWorldTriangle(transform), &distance);
	if (!isCollided) return false;
	if (!outInfo) return true;
	outInfo->setDistance(distance);
	outInfo->setPosition(line.pointFromDistance(distance));
	return true;
}

void TriangleShape::drawWireFrame(const TransformQ & transform, const Vector4 & color) const {
	auto triangle = getWorldTriangle(transform);
	for (auto i = 0; i < 3; i++) {
		decltype(auto) edge = edgeFromFace(triangle, i);
		SceneRendererManager::debugDrawLine(edge.start(), edge.end(), color);
	}
}

const Triangle TriangleShape::getWorldTriangle(const TransformQ & transform) const {
	const auto rot = transform.rotation;
	const auto pos = transform.position;
	decltype(auto) triangle = getLocalTriangle();

	const auto v0 = Quaternion::rotVector(rot, triangle[0]) + pos;
	const auto v1 = Quaternion::rotVector(rot, triangle[1]) + pos;
	const auto v2 = Quaternion::rotVector(rot, triangle[2]) + pos;

	return { v0, v1, v2 };
}

const Triangle & TriangleShape::getLocalTriangle() const {
	return _baseTriangle;
}

void TriangleShape::setLocalTriangle(const Triangle & localTriangle) {
	_baseTriangle = localTriangle;
}
