#include <Collider/CollisionShape/CapsuleShape.h>
#include <Renderer/SceneRendererManager.h>
#include <Collider/CollisionShape/CollisionShapeType.h>
#include <Collider/AABB.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Contact/LinecastContact.h>
#include <Collider/Capsule.h>

static void drawLine(const Segment & line, const Vector4 & color) {
	SceneRendererManager::debugDrawLine(line.start(), line.end(), color);
}

CapsuleShape::CapsuleShape(const Capsule & localCapsule) :
	_baseCapsule(localCapsule),
	_tolerance(0.0f) {
	setLocalCapsule(localCapsule);
}

const AABB CapsuleShape::coverAABB(const TransformQ & transform) const {
	return getWorldCapsule(transform).coverAABB();
}

CollisionShapeType CapsuleShape::getType() const {
	return CollisionShapeType::Capsule;
}

bool CapsuleShape::linecast(const TransformQ & transform, const Segment & line, LinecastContact * outInfo) const {
	auto distance = 0.0f;
	decltype(auto) sphere = getWorldCapsule(transform);
	const auto isCollided = Collision3D::intersect(line, sphere, &distance);
	if (!isCollided) return false;
	if (!outInfo) return true;
	outInfo->setDistance(distance);
	outInfo->setPosition(line.pointFromDistance(distance));
	return isCollided;
}

bool CapsuleShape::intersectTriangle(const TransformQ & transform, const Triangle & triangle) const {
	return Collision3D::intersect(getWorldCapsule(transform), triangle);
}

bool CapsuleShape::intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const {
	return Collision3D::intersect(getWorldCapsule(transform), triangle, outMyHitPos, outTriangleHitPos);
}

void CapsuleShape::drawWireFrame(const TransformQ & transform, const Vector4 & color) const {
	decltype(auto) capsule = getWorldCapsule(transform);
	decltype(auto) centerLine = capsule.segment();
	decltype(auto) p1 = centerLine.start();
	decltype(auto) p2 = centerLine.end();
	decltype(auto) radius = capsule.radius();

	decltype(auto) vector = centerLine.unitVector();
	auto hori = Vector3::cross(vector, Vector3::up);

	if (hori.sqrLength() < Mathf::Eps)
		hori = Vector3::cross(vector, Vector3::right);
	hori = hori.normalize();

	// ––’[‚ð‹…‚Å•\Œ»
	SceneRendererManager::debugDrawSphere(p1, radius, color);
	SceneRendererManager::debugDrawSphere(p2, radius, color);

	// ‹…“¯Žm‚ðü‚Å‚Â‚È‚®
	constexpr auto lineCount = 10;
	for (auto i = 0; i < lineCount; i++) {
		const auto angle = 360.0f * Mathf::degToRad * i / lineCount;
		const auto rot = Quaternion(vector, angle);
		const auto dif = Quaternion::rotVector(rot, hori);
		decltype(auto) line = centerLine.translate(dif * radius);
		drawLine(line, color);
	}
}

const Capsule CapsuleShape::getWorldCapsule(const TransformQ & transform) const {
	const auto rot = transform.rotation;
	const auto tra = transform.position;

	decltype(auto) baseSegment = _baseCapsule.segment();
	const auto rotSegment = Segment(Quaternion::rotVector(rot, baseSegment.start()), Quaternion::rotVector(rot, baseSegment.end()));
	decltype(auto) traSegment = rotSegment.translate(tra);

	const auto capsule = Capsule(traSegment, _baseCapsule.radius());

	return capsule;
}

const Capsule & CapsuleShape::getLocalCapsule() const {
	return _baseCapsule;
}

void CapsuleShape::setLocalCapsule(const Capsule & localCapsule) {
	_baseCapsule = localCapsule;
	_tolerance = _baseCapsule.radius() / 40.0f;
}
