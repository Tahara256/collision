#include <Collider/ColliderComponentManager.h>
#include <ActorComponent/ColliderComponent.h>
#include <Collider/CollisionManager.h>

bool ColliderComponentManager::linecast(const Segment & line, LinecastHit * outInfo, bool ignoreTrigger) const {
	if (!outInfo) return CollisionManager::instance().linecast(line, nullptr, ignoreTrigger);
	auto contact = LinecastContact();
	auto isCollided = CollisionManager::instance().linecast(line, &contact, ignoreTrigger);
	outInfo->setContact(contact);
	return isCollided;
}

bool ColliderComponentManager::linecast(const Segment & line, CollisionLayerType layerMask, LinecastHit * outInfo, bool ignoreTrigger) const {
	if (!outInfo) return CollisionManager::instance().linecast(line, layerMask, nullptr, ignoreTrigger);
	auto contact = LinecastContact();
	auto isCollided = CollisionManager::instance().linecast(line, layerMask, &contact, ignoreTrigger);
	outInfo->setContact(contact);
	return isCollided;
}

bool ColliderComponentManager::linecastClosest(const Segment & line, LinecastHit * outInfo, bool ignoreTrigger) const {
	if (!outInfo) return CollisionManager::instance().linecastClosest(line, nullptr, ignoreTrigger);
	auto contact = LinecastContact();
	auto isCollided = CollisionManager::instance().linecastClosest(line, &contact, ignoreTrigger);
	outInfo->setContact(contact);
	return isCollided;
}

bool ColliderComponentManager::linecastClosest(const Segment & line, CollisionLayerType layerMask, LinecastHit * outInfo, bool ignoreTrigger) const {
	if (!outInfo) return CollisionManager::instance().linecastClosest(line, layerMask, nullptr, ignoreTrigger);
	auto contact = LinecastContact();
	auto isCollided = CollisionManager::instance().linecastClosest(line, layerMask, &contact, ignoreTrigger);
	outInfo->setContact(contact);
	return isCollided;
}

bool ColliderComponentManager::linecastAll(const Segment & line, LinecastHitList * outInfos, bool ignoreTrigger) const {
	auto contactList = LinecastContactList();
	auto isCollided = CollisionManager::instance().linecastAll(line, &contactList, ignoreTrigger);
	for (auto const & contact : contactList) {
		outInfos->emplace_back(contact);
	}
	return isCollided;
}

bool ColliderComponentManager::linecastAll(const Segment & line, CollisionLayerType layerMask, LinecastHitList * outInfos, bool ignoreTrigger) const {
	auto contactList = LinecastContactList();
	auto isCollided = CollisionManager::instance().linecastAll(line, layerMask, &contactList, ignoreTrigger);
	for (auto const & contact : contactList) {
		outInfos->emplace_back(contact);
	}
	return isCollided;
}

RefPtr<ColliderComponent> ColliderComponentManager::getColliderComponent(ColliderID id) const {
	return _table.at(id);
}

void ColliderComponentManager::update() {
	for (const auto & pair : _table)
		pair.second->preUpdateCollider();

	CollisionManager::instance().update();

	for (const auto & pair : _table)
		pair.second->postUpdateCollider();
}

void ColliderComponentManager::drawWireFrame(const Vector4 & color) const {
	CollisionManager::instance().drawWireFrame(color);
}

void ColliderComponentManager::addColliderComponent(RefPtr<ColliderComponent> colliderComponent) {
	_table[colliderComponent->getColliderID()] = colliderComponent;
}

void ColliderComponentManager::removeColliderComponent(RefPtr<ColliderComponent> colliderComponent) {
	_table.erase(colliderComponent->getColliderID());
}
