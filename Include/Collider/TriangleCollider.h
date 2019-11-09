#pragma once

#include <Collider/Collider/Collider.h>
#include <Collider/Face.h>

class TriangleCollider : public Collider {

public:

	TriangleCollider(const Triangle & triangle);

};
