#include "NormalCollisionFilter.h"
#include "CollisionLayer.h"
#include <Collider/Collider/Collider.h>

void NormalCollisionFilter::enableCollisionBetween(CollisionLayerType layer1, CollisionLayerType layer2) {
	_table[layer1] |= layer2;
	_table[layer2] |= layer1;
}

void NormalCollisionFilter::disableCollisionBetween(CollisionLayerType layer1, CollisionLayerType layer2) {
	_table[layer1] &= ~layer2;
	_table[layer2] &= ~layer1;
}

std::pair<bool, bool> NormalCollisionFilter::filter(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	const auto notMove = collider1->getIsStatic() && collider2->getIsStatic();
	// 両方動かないオブジェクトは通さない
	if (notMove) return { false, false };

	const auto layer1 = collider1->getCollisionLayer();
	const auto layer2 = collider2->getCollisionLayer();

	const auto mask1 = collider1->getCollisionMask();
	const auto mask2 = collider2->getCollisionMask();

	const auto passGlobal = globalFilter(layer1, layer2);
	const auto passCollider1 = mask1 ? (mask1 & layer2) : passGlobal;
	const auto passCollider2 = mask2 ? (mask2 & layer1) : passGlobal;

	return { passCollider1, passCollider2 };
}

bool NormalCollisionFilter::globalFilter(CollisionLayerType layer1, CollisionLayerType layer2) const {
	// デフォルト値だったら必ず通す
	if (!layer1) return true;
	if (!layer2) return true;
	// テーブルに設定されていなかったので通さない
	if (_table.count(layer1) == 0) return false;
	// テーブルを参照して衝突判定が有効かを返す
	return _table.at(layer1) & layer2;
}
