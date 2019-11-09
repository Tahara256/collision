#pragma once

#include <Collider/Contact/LinecastContact.h>
#include <Util/RefPtr.h>

class ColliderComponent;

/// <summary> ラインキャスト接触情報ラップクラス </summary>
class LinecastHit
{

public:

	/// <summary> 衝突相手を取得します。 </summary>
	const RefPtr<ColliderComponent> getOther() const;

	/// <summary> 線分接触情報を取得します。 </summary>
	const LinecastContact getContact() const;

	/// <summary> コンストラクタ </summary>
	/// <param name="contact"> 線分接触情報 </param>
	LinecastHit(const LinecastContact & contact);

	/// <summary> 線分接触情報を設定します。 </summary>
	void setContact(const LinecastContact & contact);

private:

	LinecastContact _contact;

};
