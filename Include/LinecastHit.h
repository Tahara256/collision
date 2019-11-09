#pragma once

#include <Collider/Contact/LinecastContact.h>
#include <Util/RefPtr.h>

class ColliderComponent;

/// <summary> ���C���L���X�g�ڐG��񃉃b�v�N���X </summary>
class LinecastHit
{

public:

	/// <summary> �Փˑ�����擾���܂��B </summary>
	const RefPtr<ColliderComponent> getOther() const;

	/// <summary> �����ڐG�����擾���܂��B </summary>
	const LinecastContact getContact() const;

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="contact"> �����ڐG��� </param>
	LinecastHit(const LinecastContact & contact);

	/// <summary> �����ڐG����ݒ肵�܂��B </summary>
	void setContact(const LinecastContact & contact);

private:

	LinecastContact _contact;

};
