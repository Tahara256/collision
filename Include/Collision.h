#pragma once

#include <Collider/Contact/Contact.h>
#include <Util/RefPtr.h>

class ColliderComponent;

/// <summary> 衝突情報クラス </summary>
class Collision {

public:

	/// <summary> 衝突相手を取得します。 </summary>
	const RefPtr<ColliderComponent> getOther() const;

	/// <summary> 接触情報を取得します。 </summary>
	const Contact getContact() const;

	/// <summary> コンストラクタ </summary>
	/// <param name="contact"> 接触情報 </param>
	explicit Collision(const Contact & contact);

private:

	Contact _contact;

};
