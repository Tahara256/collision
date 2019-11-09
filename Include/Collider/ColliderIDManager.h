#pragma once

#include <set>

#include <Util/Design/Singleton.h>
#include <Util/Type.h>
#include <Collider/Collider/ColliderID.h>

/// <summary> コライダーのIDを管理するクラス </summary>
class ColliderIDManager : public Singleton<ColliderIDManager> {

public:

	/// <summary> コライダーのIDを発行します。 </summary>
	ColliderID issuanceID();

	/// <summary> コライダーのIDを返却します。 </summary>
	void remove(ColliderID id);

private:

	ColliderID _issuanceCount;

	std::set<ColliderID> _idList;

};
