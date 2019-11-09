#include <Collider/Collider/ColliderIDManager.h>

ColliderID ColliderIDManager::issuanceID() {
	auto id = 0;
	decltype(_idList.count(id)) count = 1;
	// ‹ó‚¢‚Ä‚¢‚éID‚ð’T‚·
	while (count) {
		id = _issuanceCount++;
		count = _idList.count(id);
	}
	_idList.insert(id);
	return id;
}

void ColliderIDManager::remove(ColliderID id) {
	_idList.erase(id);
}
