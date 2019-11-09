#pragma once

#include <functional>

class Contact;
class TriggerContact;

using OnCollisionCallback = std::function<void(const Contact &)>;
using OnTriggerCallback = std::function<void(const TriggerContact &)>;
