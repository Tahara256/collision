#include <Collider/Collision.h>
#include <Collider/ColliderComponentManager.h>

const RefPtr<ColliderComponent> Collision::getOther() const {
	const auto otherId = getContact().getOtherID();
	return ColliderComponentManager::ins().getColliderComponent(otherId);
}

const Contact Collision::getContact() const {
	return _contact;
}

Collision::Collision(const Contact & contact) : _contact(contact) {}
