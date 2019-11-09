#include <Collider/CollisionShape/BoxShape.h>
#include <Renderer/SceneRendererManager.h>
#include <Collider/AABB.h>
#include <Collider/CollisionShape/CollisionShapeType.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Contact/LinecastContact.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Segment.h>

BoxShape::BoxShape(const OBB & localBox) :
	_baseBox(localBox) {
}

AABB const BoxShape::coverAABB(const TransformQ & transform) const {
	return getWorldBox(transform).coverAABB();
}

CollisionShapeType BoxShape::getType() const {
	return CollisionShapeType::Box;
}

bool BoxShape::linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo) const {
	auto distance = 0.0f;
	const auto isCollided = Collision3D::intersect(segment, getWorldBox(transform), &distance);
	if (!isCollided) return false;
	if (!outInfo) return true;
	outInfo->setDistance(distance);
	outInfo->setPosition(segment.pointFromDistance(distance));
	return true;
}

bool BoxShape::intersectTriangle(const TransformQ & transform, const Triangle & triangle) const {
	return Collision3D::intersect(getWorldBox(transform), triangle);
}

bool BoxShape::intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const {
	return Collision3D::intersect(getWorldBox(transform), triangle, outMyHitPos, outTriangleHitPos);
}

void BoxShape::drawWireFrame(const TransformQ & transform, const Vector4 & color) const {
	decltype(auto) box = getWorldBox(transform);
	SceneRendererManager::debugDrawBox(box.center(), box.size(), box.rotation(), color);
}

const OBB BoxShape::getWorldBox(const TransformQ & transform) const {
	return { transform.position + Quaternion::rotVector(transform.rotation, _baseBox.center()), _baseBox.size(), transform.rotation * _baseBox.rotation() };
}

const OBB & BoxShape::getLocalBox() const {
	return _baseBox;
}

void BoxShape::setLocalBox(const OBB & localBox) {
	_baseBox = localBox;
}
