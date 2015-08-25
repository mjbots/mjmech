// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <map>

#include <boost/signals2/signal.hpp>

#include "telemetry_archive.h"

namespace legtool {
/// Maintain a local publish-subscribe model used for writing data to
/// log files or publishing them over the network.
///
/// Data is serialized using TelemetryWriteArchive, using the format
/// found in telemetry_format.h.
class TelemetryRegistry : boost::noncopyable {
 public:
  /// Register a serializable object, and return a function object
  /// which when called will disseminate serialized versions of the
  /// object to any observers.
  template <typename Serializable>
  std::function<void (const Serializable*)>
  Register(const std::string& record_name,
           boost::signals2::signal<void (const Serializable)>* = 0) {
    auto* ptr = new Concrete<Serializable>();
    auto result =
        std::bind(&Concrete<Serializable>::Invoke, ptr,
                  std::placeholders::_1);
    RegisterImpl(record_name, std::move(std::unique_ptr<Base>(ptr)));
    return result;
  };

  /// Register a record, and return a callable which when invoked
  /// requires pre-serialized data.
  std::function<void (const std::string&)>
  RegisterPreSerialized(const std::string& schema,
                        const std::string& record_name) {
    auto* ptr = new PreSerialized(schema);
    auto result =
        std::bind(&PreSerialized::Invoke, ptr,
                  std::placeholders::_1);
    RegisterImpl(record_name, std::move(std::unique_ptr<Base>(ptr)));
    return result;
  }

  struct RecordProperties {
    std::string name;
    std::string schema;
    boost::signals2::signal<
      void (const std::string&)>* serialized_data_signal = nullptr;
  };

  typedef std::function<void (const RecordProperties&)> SchemaHandler;

  /// Indicate that the caller wants to be informed when new records
  /// have been registered.  If records have already been registered
  /// at the time of call, the handler will be invoked with all
  /// records currently present.
  void ObserveSchema(SchemaHandler handler);

  /// If the named record has been registered, return a signal that
  /// emits serialized data anytime that record is emitted.
  boost::signals2::signal<
    void (const std::string&)>* GetSerializedSignal(const std::string& name);

  /// If the named signal of the appropriate type has already been
  /// registered, return a correctly typed signal for it.
  template <typename T>
  boost::signals2::signal<void (const T*)>*
  GetConcreteSignal(const std::string& name) {
    auto it = records_.find(name);
    if (it == records_.end()) { return nullptr; }
    Concrete<T>* concrete = dynamic_cast<Concrete<T>*>(it->second.get());
    if (!concrete) { return nullptr; }
    return concrete->concrete_signal();
  }

 private:
  class Base {
   public:
    Base() {}
    virtual ~Base() {}

    boost::signals2::signal<
      void (const std::string&)>* signal() { return &signal_; }
    const std::string& schema() const { return schema_; }

   protected:
    boost::signals2::signal<void (const std::string&)> signal_;
    std::string schema_;
  };

  template <typename Serializable>
  class Concrete : public Base {
   public:
    Concrete() {
      schema_ = archive_.schema();
    }

    void Invoke(const Serializable* object) {
      concrete_signal_(object);
      signal_(archive_.Serialize(object));
    }

    boost::signals2::signal<void (const Serializable*)>* concrete_signal() {
      return &concrete_signal_;
    }

   private:
    TelemetryWriteArchive<Serializable> archive_;
    boost::signals2::signal<void (const Serializable*)> concrete_signal_;
  };

  class PreSerialized : public Base {
   public:
    PreSerialized(const std::string& schema) {
      schema_ = schema;
    }

    void Invoke(const std::string& data) {
      signal_(data);
    }
  };

  void RegisterImpl(const std::string& name, std::unique_ptr<Base>&& record);

  std::map<std::string, std::unique_ptr<Base> > records_;
  std::list<SchemaHandler> observers_;
};

}
