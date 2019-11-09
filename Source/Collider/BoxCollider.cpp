#include <Collider/Collider/BoxCollider.h>
#include <Collider/CollisionShape/BoxShape.h>

using namespace std;

BoxCollider::BoxCollider(OBB const & localBox) :
	Collider(make_shared<BoxShape>(localBox))
{
}
