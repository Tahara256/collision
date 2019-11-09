#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/CollisionShape.h>
#include <Collider/AABB.h>
#include <Collider/Collider/ColliderIDManager.h>
#include <Collider/Contact/LinecastContact.h>

using namespace std;

Collider::Collider(const CollisionShapePtr & collisionShape) :
	_collisionShape(collisionShape),
	_collisionLayer(0),
	_collisionMask(0),
	_id(ColliderIDManager::ins().issuanceID()),
	_isActive(true),
	_isStatic(false),
	_isTrigger(false) {
}

void Collider::setCollisionShape(const CollisionShapePtr & collisionShape) {
	_collisionShape = collisionShape;
}

void Collider::setTransform(const TransformQ & transform) {
	_transform = transform;
}

void Collider::setPreTransform(const TransformQ & preTransform) {
	_preTransform = preTransform;
}

void Collider::setCollisionLayer(CollisionLayerType collisionLayer) {
	_collisionLayer = collisionLayer;
}

void Collider::setCollisionMask(CollisionLayerType collisionMask) {
	_collisionMask = collisionMask;
}

void Collider::addOnCollisionStay(const OnCollisionCallback & onCollisionStay) {
	_callbackManager.addOnCollisionStay(onCollisionStay);
}

void Collider::addOnCollisionEnter(const OnCollisionCallback & onCollisionEnter) {
	_callbackManager.addOnCollisionEnter(onCollisionEnter);
}

void Collider::addOnCollisionExit(const OnCollisionCallback & onCollisionExit) {
	_callbackManager.addOnCollisionExit(onCollisionExit);
}

void Collider::addOnTriggerStay(const OnTriggerCallback & onTriggerStay) {
	_callbackManager.addOnTriggerStay(onTriggerStay);
}

void Collider::addOnTriggerEnter(const OnTriggerCallback & onTriggerEnter) {
	_callbackManager.addOnTriggerEnter(onTriggerEnter);
}

void Collider::addOnTriggerExit(const OnTriggerCallback & onTriggerExit) {
	_callbackManager.addOnTriggerExit(onTriggerExit);
}

const CollisionShapePtr & Collider::getCollisionShape() const {
	return _collisionShape;
}

const TransformQ & Collider::getTransform() const {
	return _transform;
}

const TransformQ & Collider::getPreTransform() const {
	return _preTransform;
}

const AABB Collider::coverAABB() const {
	return _collisionShape->coverAABB(_transform);
}

CollisionShapeType Collider::getCollisionShapeType() const {
	return _collisionShape->getType();
}

CollisionLayerType Collider::getCollisionLayer() const {
	return _collisionLayer;
}

CollisionLayerType Collider::getCollisionMask() const {
	return _collisionMask;
}

uint32 Collider::getID() const {
	return _id;
}

bool Collider::getIsActive() const {
	return _isActive;
}

bool Collider::getIsStatic() const {
	return _isStatic;
}

bool Collider::getIsTrigger() const {
	return _isTrigger;
}

void Collider::setIsActive(bool isActive) {
	if (_isActive == isActive) return;
	_isActive = isActive;
}

void Collider::setIsStatic(bool isStatic) {
	_isStatic = isStatic;
}

void Collider::setIsTrigger(bool isTrigger) {
	_isTrigger = isTrigger;
}

void Collider::drawWireFrame(const Vector4 & color) const {
	if (isAttached())
		_collisionShape->drawWireFrame(_transform, color);
	else
		_collisionShape->drawWireFrame(_transform, Vector4(1, 0, 1, color.w));
}

bool Collider::linecast(const Segment & line, LinecastContact * outInfo) const {
	const auto isCollided = getCollisionShape()->linecast(getTransform(), line, outInfo);
	if (!isCollided) return false;
	if (!outInfo) return true;
	outInfo->setOtherID(getID());
	return true;
}

bool Collider::intersectMesh(const UniformGridMesh & mesh) const {
	return getCollisionShape()->intersectMesh(getTransform(), mesh);
}

bool Collider::intersectMesh(const UniformGridMesh & mesh, Vector3 * outMyHitPos, Vector3 * outMeshHitPos) const {
	return getCollisionShape()->intersectMesh(getTransform(), mesh, outMyHitPos, outMeshHitPos);
}

void Collider::notifyOnCollisionStay(const Contact & info) {
	_callbackManager.notifyOnCollisionStay(info);
}

void Collider::notifyOnCollisionEnter(const Contact & info) {
	_callbackManager.notifyOnCollisionEnter(info);
}

void Collider::notifyOnCollisionExit(const Contact & info) {
	_callbackManager.notifyOnCollisionExit(info);
}

void Collider::notifyOnTriggerStay(const TriggerContact & info) {
	_callbackManager.notifyOnTriggerStay(info);
}

void Collider::notifyOnTriggerEnter(const TriggerContact & info) {
	_callbackManager.notifyOnTriggerEnter(info);
}

void Collider::notifyOnTriggerExit(const TriggerContact & info) {
	_callbackManager.notifyOnTriggerExit(info);
}
