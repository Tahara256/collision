#include <Collider/Collider/SphereCollider.h>
#include <Collider/CollisionShape/SphereShape.h>

using namespace std;

SphereCollider::SphereCollider(Sphere const & localSphere) :
	Collider(make_shared<SphereShape>(localSphere))
{
}
