#include <Collider/Collider/CollisionCallbackManager.h>
#include <algorithm>
#include <Collider/Contact/Contact.h>
#include <Collider/Contact/TriggerContact.h>

CollisionCallbackManager::CollisionCallbackManager() :
	_onCollisionStay([](const Contact &) {}),
	_onCollisionEnter([](const Contact &) {}),
	_onCollisionExit([](const Contact &) {}),
	_onTriggerStay([](const TriggerContact &) {}),
	_onTriggerEnter([](const TriggerContact &) {}),
	_onTriggerExit([](const TriggerContact &) {}) {
}

void CollisionCallbackManager::addOnCollisionStay(const OnCollisionCallback & onCollisionStay) {
	addOnCollision(_onCollisionStay, onCollisionStay);
}

void CollisionCallbackManager::addOnCollisionEnter(const OnCollisionCallback & onCollisionEnter) {
	addOnCollision(_onCollisionEnter, onCollisionEnter);
}

void CollisionCallbackManager::addOnCollisionExit(const OnCollisionCallback & onCollisionExit) {
	addOnCollision(_onCollisionExit, onCollisionExit);
}

void CollisionCallbackManager::addOnTriggerStay(const OnTriggerCallback & onTriggerStay) {
	addOnTrigger(_onTriggerStay, onTriggerStay);
}

void CollisionCallbackManager::addOnTriggerEnter(const OnTriggerCallback & onTriggerEnter) {
	addOnTrigger(_onTriggerEnter, onTriggerEnter);
}

void CollisionCallbackManager::addOnTriggerExit(const OnTriggerCallback & onTriggerExit) {
	addOnTrigger(_onTriggerExit, onTriggerExit);
}

void CollisionCallbackManager::notifyOnCollisionStay(const Contact & info) {
	_onCollisionStay(info);
}

void CollisionCallbackManager::notifyOnCollisionEnter(const Contact & info) {
	_onCollisionEnter(info);
}

void CollisionCallbackManager::notifyOnCollisionExit(const Contact & info) {
	_onCollisionExit(info);
}

void CollisionCallbackManager::notifyOnTriggerStay(const TriggerContact & info) {
	_onTriggerStay(info);
}

void CollisionCallbackManager::notifyOnTriggerEnter(const TriggerContact & info) {
	_onTriggerEnter(info);
}

void CollisionCallbackManager::notifyOnTriggerExit(const TriggerContact & info) {
	_onTriggerExit(info);
}

void CollisionCallbackManager::addOnCollision(OnCollisionCallback & dest, const OnCollisionCallback & source) {
	const auto & current = dest;
	dest = [current, source](const Contact & info) { current(info); source(info); };
}

void CollisionCallbackManager::addOnTrigger(OnTriggerCallback & dest, const OnTriggerCallback & source) {
	const auto & current = dest;
	dest = [current, source](const TriggerContact & info) { current(info); source(info); };
}
