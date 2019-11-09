#include <Collider/Implementation/CollisionLogManager.h>
#include <algorithm>
#include <Collider/CollisionManager.h>
#include <Collider/Collider/Collider.h>
#include <Collider/Contact/Contact.h>
#include <Collider/Contact/TriggerContact.h>

bool CollisionLogManager::alreadyCollision(ColliderID id1, ColliderID id2) {
	auto beItr = std::begin(_preCollisionLog);
	auto enItr = std::end(_preCollisionLog);

	auto const result = std::find_if(beItr, enItr, [id1, id2](CollisionLog pair) {
		if (pair.collider1ID == id1 && pair.collider2ID == id2) return true;
		if (pair.collider1ID == id2 && pair.collider2ID == id1) return true;
		return false;
	});
	return result != enItr;
}

bool CollisionLogManager::alreadyTrigger(ColliderID id1, ColliderID id2) {
	auto beItr = std::begin(_preTriggerLog);
	auto enItr = std::end(_preTriggerLog);

	auto const result = std::find_if(beItr, enItr, [id1, id2](CollisionLog pair) {
		if (pair.collider1ID == id1 && pair.collider2ID == id2) return true;
		if (pair.collider1ID == id2 && pair.collider2ID == id1) return true;
		return false;
	});
	return result != enItr;
}

void CollisionLogManager::collision(ColliderID id1, ColliderID id2, bool needsCollider1Callback, bool needsCollider2Callback) {
	_curCollisionLog.emplace_back(id1, id2, needsCollider1Callback, needsCollider2Callback);
}

void CollisionLogManager::trigger(ColliderID id1, ColliderID id2, bool needsCollider1Callback, bool needsCollider2Callback) {
	_curTriggerLog.emplace_back(id1, id2, needsCollider1Callback, needsCollider2Callback);
}

void CollisionLogManager::notifyExitCallbackIfNeeded() {
	notifyCollisionExitCallbackIfNeeded();
	notifyTriggerExitCallbackIfNeeded();
}

void CollisionLogManager::updateLog() {
	_preCollisionLog.swap(_curCollisionLog);
	_curCollisionLog.clear();
	_preTriggerLog.swap(_curTriggerLog);
	_curTriggerLog.clear();
}

void CollisionLogManager::notifyCollisionExitCallbackIfNeeded() {
	for (auto log : _preCollisionLog) {

		auto beItr = std::begin(_curCollisionLog);
		auto enItr = std::end(_curCollisionLog);

		auto const result = std::find_if(beItr, enItr, [log](CollisionLog curPair) {
			if (log.collider1ID == curPair.collider1ID && log.collider2ID == curPair.collider2ID) return true;
			if (log.collider1ID == curPair.collider2ID && log.collider2ID == curPair.collider1ID) return true;
			return false;
		});
		// ˆê’v‚·‚é‚à‚Ì‚ª‚ ‚Á‚½‚çÕ“Ë‚µ‘±‚¯‚Ä‚¢‚é‚±‚Æ‚É‚È‚é
		if (enItr != result) continue;

		// ˆê’v‚·‚é‚à‚Ì‚ª‚È‚©‚Á‚½‚çÕ“Ë‚ªI‚í‚Á‚½‚±‚Æ‚É‚È‚é
		auto first = CollisionManager::instance().getCollider(log.collider1ID);
		auto second = CollisionManager::instance().getCollider(log.collider2ID);

		if (first && log.needsCollider1Callback) {
			auto info1 = Contact();
			info1.setOtherID(log.collider2ID);
			first->notifyOnCollisionExit(info1);
		}

		if (second && log.needsCollider2Callback) {
			auto info2 = Contact();
			info2.setOtherID(log.collider1ID);
			second->notifyOnCollisionExit(info2);
		}
	}
}

void CollisionLogManager::notifyTriggerExitCallbackIfNeeded() {
	for (auto log : _preTriggerLog) {

		auto beItr = std::begin(_curTriggerLog);
		auto enItr = std::end(_curTriggerLog);

		auto const result = std::find_if(beItr, enItr, [log](CollisionLog curPair) {
			if (log.collider1ID == curPair.collider1ID && log.collider2ID == curPair.collider2ID) return true;
			if (log.collider1ID == curPair.collider2ID && log.collider2ID == curPair.collider1ID) return true;
			return false;
		});
		// ˆê’v‚·‚é‚à‚Ì‚ª‚ ‚Á‚½‚çÕ“Ë‚µ‘±‚¯‚Ä‚¢‚é‚±‚Æ‚É‚È‚é
		if (enItr != result) continue;

		// ˆê’v‚·‚é‚à‚Ì‚ª‚È‚©‚Á‚½‚çÕ“Ë‚ªI‚í‚Á‚½‚±‚Æ‚É‚È‚é
		auto first = CollisionManager::instance().getCollider(log.collider1ID);
		auto second = CollisionManager::instance().getCollider(log.collider2ID);

		if (first && log.needsCollider1Callback) {
			auto info1 = TriggerContact();
			info1.setOtherID(log.collider2ID);
			first->notifyOnTriggerExit(info1);
		}

		if (second && log.needsCollider2Callback) {
			auto info2 = TriggerContact();
			info2.setOtherID(log.collider1ID);
			second->notifyOnTriggerExit(info2);
		}
	}
}
