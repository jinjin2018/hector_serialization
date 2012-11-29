//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_SERIALIZATION_CONTEXT_H
#define HECTOR_SERIALIZATION_CONTEXT_H

#include <vector>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

namespace hector_serialization {

class Context {
public:
  struct ElementBase {
    typedef boost::shared_ptr<ElementBase> Ptr;
    typedef boost::shared_ptr<const ElementBase> ConstPtr;

    virtual bool operator==(const ElementBase& other) const = 0;
    virtual bool operator!=(const ElementBase& other) const = 0;
    virtual Ptr clone() const = 0;

    virtual std::string identifier() const = 0;
    virtual std::string to_string() const { return identifier(); }
  };

  template <typename Derived>
  struct Element : public ElementBase
  {
    bool operator==(const ElementBase& other) const {
      const Derived *other_derived = dynamic_cast<const Derived *>(&other);
      return other_derived && isEqual(*other_derived);
    }
    bool operator!=(const ElementBase& other) const { return !(*this == other); }
    ElementBase::Ptr clone() const { return ElementBase::Ptr(new Derived(*static_cast<const Derived *>(this))); }

    virtual bool isEqual(const Derived& other) const = 0;
  };

  template <typename T>
  class TypedElement : public Element<TypedElement<T> > {
  public:
    typedef T value_type;
    T value;

    TypedElement() : value() {}
    TypedElement(const T& value) : value(value) {}

    operator T&() { return value; }

    virtual bool isEqual(const TypedElement<T>& other) const {
      return (other.value == this->value);
    }

    virtual std::string identifier() const {
      return boost::lexical_cast<std::string>(value);
    }

    virtual std::string to_string() const {
      return boost::lexical_cast<std::string>(value);
    }
  };

public:
  Context() {}
  Context(const Context &other) { add(other); }
  Context(const ElementBase &element) { add(element); }
  Context(const Context& other1, const Context& other2) { add(other1); add(other2); }
  virtual ~Context() {}

  Context& add(const ElementBase &element) {
    elements_.push_back(element.clone());
    return *this;
  }

  Context& remove(const ElementBase &element) {
    ContextList::iterator it = elements_.begin();
    while(it != elements_.end()) if (**it == element) it = elements_.erase(it); else ++it;
    return *this;
  }

  Context& add(const Context& other) {
    for(ContextList::const_iterator it = other.elements_.begin(); it != other.elements_.end(); ++it) add(**it);
    return *this;
  }

  bool operator==(const Context& other) const {
    if (elements_.size() != other.elements_.size()) return false;
    ContextList::const_iterator it1 = elements_.begin();
    ContextList::const_iterator it2 = other.elements_.begin();

    for( ; it1 != elements_.end() && it2 != other.elements_.end(); ++it1, ++it2) {
      if (**it1 != **it2) return false;
    }
    return true;
  }

  template <typename ElementType>
  boost::shared_ptr<ElementType const> get() const {
    boost::shared_ptr<ElementType const> result;
    for(ContextList::const_iterator it = elements_.begin(); it != elements_.end(); ++it) {
      result = boost::shared_dynamic_cast<ElementType const>(*it);
      if (result) return result;
    }
    return result;
  }

  ElementBase::ConstPtr operator[](std::size_t i) {
    return elements_[i];
  }

  std::string identifier() const {
    if (elements_.empty()) return std::string();
    return elements_.back()->identifier();
  }

  std::string to_string() const {
    if (elements_.empty()) return std::string();
    return elements_.back()->to_string();
  }

private:
  typedef std::vector<ElementBase::ConstPtr> ContextList;
  ContextList elements_;
};

static inline Context operator+(const Context& context1, const Context& context2) {
  return Context(context1, context2);
}

static inline Context operator+(const Context& context, const Context::ElementBase& element) {
  return Context(context).add(element);
}

}

#endif // HECTOR_SERIALIZATION_CONTEXT_H
