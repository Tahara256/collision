#include <Collider/Implementation/CollisionAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/Implementation/CollisionDispatcher.h>
#include <Collider/Contact/Contact.h>
#include <Collider/Contact/TriggerContact.h>
#include <Collider/Implementation/CollisionLogManager.h>

static void collisionBlock(const ColliderPtr & collider1, const ColliderPtr & collider2, bool passCollider1, bool passCollider2) {
	auto hitPos1 = Vector3();
	auto hitPos2 = Vector3();
	const auto isCollided = CollisionDispatcher::intersectBlock(collider1, collider2, &hitPos1, &hitPos2);

	if (!isCollided) return;

	// 衝突情報
	auto info1 = Contact();
	auto info2 = Contact();

	// ID
	info1.setOtherID(collider2->getID());
	info2.setOtherID(collider1->getID());

	// 衝突座標
	info1.setPosition(hitPos1);
	info2.setPosition(hitPos2);

	// 押し出しベクトル
	info1.setExtrusionVector(hitPos2 - hitPos1);
	info2.setExtrusionVector(hitPos1 - hitPos2);

	// 衝突イベントコール
	const auto alreadyHit = CollisionLogManager::ins().alreadyCollision(collider1->getID(), collider2->getID());
	if (!alreadyHit) {
		if (passCollider1) collider1->notifyOnCollisionEnter(info1);
		if (passCollider2) collider2->notifyOnCollisionEnter(info2);
	}

	if (passCollider1) collider1->notifyOnCollisionStay(info1);
	if (passCollider2) collider2->notifyOnCollisionStay(info2);

	CollisionLogManager::ins().collision(collider1->getID(), collider2->getID(), passCollider1, passCollider2);
}

static void collisionTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2, bool passCollider1, bool passCollider2) {
	const auto isCollided = CollisionDispatcher::intersectTrigger(collider1, collider2);

	if (!isCollided) return;

	// 衝突情報
	auto info1 = TriggerContact();
	auto info2 = TriggerContact();

	// ID
	info1.setOtherID(collider2->getID());
	info2.setOtherID(collider1->getID());

	// 衝突イベントコール
	const auto alreadyHit = CollisionLogManager::ins().alreadyTrigger(collider1->getID(), collider2->getID());
	if (!alreadyHit) {
		if (passCollider1) collider1->notifyOnTriggerEnter(info1);
		if (passCollider2) collider2->notifyOnTriggerEnter(info2);
	}

	if (passCollider1) collider1->notifyOnTriggerStay(info1);
	if (passCollider2) collider2->notifyOnTriggerStay(info2);

	CollisionLogManager::ins().trigger(collider1->getID(), collider2->getID(), passCollider1, passCollider2);
}


void CollisionAgent::collision(const ColliderPtr & collider1, const ColliderPtr & collider2, bool passCollider1, bool passCollider2) {
	const auto isTrigger1 = collider1->getIsTrigger();
	const auto isTrigger2 = collider2->getIsTrigger();

	if (isTrigger1 || isTrigger2)
		collisionTrigger(collider1, collider2, passCollider1, passCollider2);
	else
		collisionBlock(collider1, collider2, passCollider1, passCollider2);
}
