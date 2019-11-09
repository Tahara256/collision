#include <Collider/CollisionShape/SphereShape.h>
#include <Renderer/SceneRendererManager.h>
#include <Collider/CollisionShape/CollisionShapeType.h>
#include <Collider/AABB.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Contact/LinecastContact.h>
#include <Collider/Segment.h>
#include <Collider/Contact/Contact.h>

using namespace std;

SphereShape::SphereShape(Sphere const & localSphere) :
	_baseSphere(localSphere) {
}

AABB const SphereShape::coverAABB(TransformQ const & transform) const {
	auto sphere = getWorldSphere(transform);
	return sphere.coverAABB();
}

CollisionShapeType SphereShape::getType() const {
	return CollisionShapeType::Sphere;
}

bool SphereShape::linecast(TransformQ const & transform, Segment const & line, LinecastContact * outInfo) const {
	auto distance = 0.0f;
	decltype(auto) sphere = getWorldSphere(transform);
	const auto isCollided = Collision3D::intersect(line, sphere, &distance);
	if (!isCollided) return false;
	if (!outInfo) return true;
	outInfo->setDistance(distance);
	outInfo->setPosition(line.pointFromDistance(distance));
	return isCollided;
}

bool SphereShape::intersectTriangle(const TransformQ & transform, const Triangle & triangle) const {
	return Collision3D::intersect(getWorldSphere(transform), triangle);
}

bool SphereShape::intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const {
	return Collision3D::intersect(getWorldSphere(transform), triangle, outMyHitPos, outTriangleHitPos);
}

void SphereShape::drawWireFrame(TransformQ const & transform, Vector4 const & color) const {
	auto sphere = getWorldSphere(transform);
	SceneRendererManager::debugDrawSphere(sphere.center(), sphere.radius(), color);
}

Sphere const SphereShape::getWorldSphere(TransformQ const & transform) const {
	return { Quaternion::rotVector(transform.rotation, _baseSphere.center()) + transform.position, _baseSphere.radius() };
}

Sphere const & SphereShape::getLocalSphere() const {
	return _baseSphere;
}

void SphereShape::setLocalSphere(Sphere const & localSphere) {
	_baseSphere = localSphere;
}
