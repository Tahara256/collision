#include <Collider/Contact/LinecastContact.h>

const Vector3 LinecastContact::getPosition() const {
	return _position;
}

float LinecastContact::getDistance() const {
	return _distance;
}

uint32 LinecastContact::getOtherID() const {
	return _otherID;
}

void LinecastContact::setPosition(const Vector3 & position) {
	_position = position;
}

void LinecastContact::setDistance(float distance) {
	_distance = distance;
}

void LinecastContact::setOtherID(ColliderID otherID) {
	_otherID = otherID;
}

LinecastContact::LinecastContact() : _position(Vector3::zero), _distance(0.0f), _otherID(0) {
}
