#pragma once

#include <set>

#include <Util/Design/Singleton.h>
#include <Util/Type.h>
#include <Collider/Collider/ColliderID.h>

/// <summary> �R���C�_�[��ID���Ǘ�����N���X </summary>
class ColliderIDManager : public Singleton<ColliderIDManager> {

public:

	/// <summary> �R���C�_�[��ID�𔭍s���܂��B </summary>
	ColliderID issuanceID();

	/// <summary> �R���C�_�[��ID��ԋp���܂��B </summary>
	void remove(ColliderID id);

private:

	ColliderID _issuanceCount;

	std::set<ColliderID> _idList;

};
