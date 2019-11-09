#pragma once

#include <Collider/Collider/ColliderID.h>
#include <Util/Design/Singleton.h>
#include <list>

struct CollisionLog {
	ColliderID collider1ID;
	ColliderID collider2ID;
	bool needsCollider1Callback;
	bool needsCollider2Callback;

	CollisionLog(ColliderID id1, ColliderID id2, bool needsCollider1Callback, bool needsCollider2Callback) :
		collider1ID(id1), collider2ID(id2), needsCollider1Callback(needsCollider1Callback), needsCollider2Callback(needsCollider2Callback) {

	}
};

class CollisionLogManager : public Singleton<CollisionLogManager> {

public:

	bool alreadyCollision(ColliderID id1, ColliderID id2);
	bool alreadyTrigger(ColliderID id1, ColliderID id2);

	void collision(ColliderID id1, ColliderID id2, bool needsCollider1Callback, bool needsCollider2Callback);
	void trigger(ColliderID id1, ColliderID id2, bool needsCollider1Callback, bool needsCollider2Callback);

	void notifyExitCallbackIfNeeded();

	void updateLog();

private:

	void notifyCollisionExitCallbackIfNeeded();
	void notifyTriggerExitCallbackIfNeeded();

	//using IDPair = std::pair<ColliderID, ColliderID>;
	using IDPairList = std::list<CollisionLog>;

	IDPairList _preCollisionLog;
	IDPairList _curCollisionLog;

	IDPairList _preTriggerLog;
	IDPairList _curTriggerLog;

};
