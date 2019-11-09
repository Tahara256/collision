#include <Collider/Collider/TriangleCollider.h>
#include <Collider/CollisionShape/TriangleShape.h>

using namespace std;

TriangleCollider::TriangleCollider(const Triangle & localTriangle) :
	Collider(make_shared<TriangleShape>(localTriangle)) {
}
