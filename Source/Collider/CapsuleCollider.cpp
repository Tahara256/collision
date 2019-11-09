#include <Collider/Collider/CapsuleCollider.h>
#include <Collider/CollisionShape/CapsuleShape.h>

using namespace std;

CapsuleCollider::CapsuleCollider(Capsule const & localCapsule) :
	Collider(make_shared<CapsuleShape>(localCapsule))
{
}
