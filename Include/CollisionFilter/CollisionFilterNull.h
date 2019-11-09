#pragma once

#include <Collider/CollisionFilter/CollisionFilter.h>

/// <summary> 全くフィルタリングしない実装 </summary>
class CollisionFilterNull : public CollisionFilter {

public:

	virtual std::pair<bool, bool> filter(const ColliderPtr & collider1, const ColliderPtr & collider2) const override;

};
