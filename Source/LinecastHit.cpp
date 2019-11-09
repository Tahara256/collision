#include <Collider/LinecastHit.h>
#include <Collider/ColliderComponentManager.h>

const RefPtr<ColliderComponent> LinecastHit::getOther() const {
	const auto otherId = getContact().getOtherID();
	return ColliderComponentManager::ins().getColliderComponent(otherId);
}

const LinecastContact LinecastHit::getContact() const {
	return _contact;
}

LinecastHit::LinecastHit(const LinecastContact & contact) : _contact(contact) {
}

void LinecastHit::setContact(const LinecastContact & contact) {
	_contact = contact;
}
