#include <Collider/Contact/Contact.h>

void Contact::setPosition(const Vector3 & hitPosition) {
	_hitPosition = hitPosition;
}

void Contact::setExtrusionVector(const Vector3 & extrusionVector) {
	_extrusionVector = extrusionVector;
}

void Contact::setOtherID(ColliderID otherID) {
	_otherID = otherID;
}

const Vector3 Contact::getPosition() const {
	return _hitPosition;
}

const Vector3 Contact::getExtrusionVector() const {
	return _extrusionVector;
}

ColliderID Contact::getOtherID() const {
	return _otherID;
}
