#pragma once

#include <Collider/Collider/ColliderID.h>

/// <summary> トリガーの接触情報クラス </summary>
class TriggerContact {

public:

	/// <summary> 衝突相手のIDを取得します。 </summary>
	ColliderID getOtherID() const;

	/// <summary> 衝突相手のIDを設定します。 </summary>
	void setOtherID(ColliderID otherID);

private:

	ColliderID _otherID;

};
