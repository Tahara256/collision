#include <Collider/Contact/TriggerContact.h>

ColliderID TriggerContact::getOtherID() const {
	return _otherID;
}

void TriggerContact::setOtherID(ColliderID otherID) {
	_otherID = otherID;
}
