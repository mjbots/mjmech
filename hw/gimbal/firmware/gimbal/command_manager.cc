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

#include "command_manager.h"

#include <assert.h>

#include "async_read.h"
#include "lock_manager.h"
#include "named_registry.h"
#include "tokenizer.h"

class CommandManager::Impl {
 public:
  Impl(Pool& pool, AsyncStream& stream, LockManager& lock_manager)
      : pool_(pool), stream_(stream), lock_manager_(lock_manager) {
  }

  void StartRead() {
    read_until_context_.stream = &stream_;
    read_until_context_.buffer = gsl::string_span(line_buffer_);
    read_until_context_.delimiters = "\r\n";
    read_until_context_.callback =
        [this](int error, int size) {
      this->HandleRead(error, size);
    };

    AsyncReadUntil(read_until_context_);
  }

  void HandleRead(int error, int size) {
    if (error) {
      // Well, not much we can do, but ignore everything until we get
      // another newline and try again.

      // TODO jpieper: Once we have an error system, log this error.

      read_until_context_.stream = &stream_;
      read_until_context_.buffer = gsl::string_span(line_buffer_);
      read_until_context_.delimiters = "\r\n";
      read_until_context_.callback = [this](int error, int size) {
        this->StartRead();
      };
      AsyncIgnoreUntil(read_until_context_);
      return;
    }

    HandleCommand(size);

    StartRead();
  }

  void HandleCommand(int size) {
    // If we haven't managed to emit our last message yet, just ignore
    // this command entirely. :(
    if (lock_manager_.locked(LockManager::kCommandManager)) { return; }

    // Make our command, minus whatever the delimeter was that ended
    // it.
    const gsl::cstring_span line(line_buffer_, size - 1);

    Tokenizer tokenizer(line, " ");
    auto cmd = tokenizer.next();
    if (cmd.size() == 0) { return; }

    auto* element = registry_.FindOrCreate(cmd, Registry::kFindOnly);

    CommandFunction command;
    if (element == nullptr) {
      command = [this](const gsl::cstring_span& line, Response response) {
        this->UnknownGroup(line, response.callback);
      };
    } else {
      command = element->ptr->command_function;
    }

    current_command_ = command;
    auto args = tokenizer.remaining();

    // Clear out anything that was previously in our arguments, then
    // fill it in with our new stuff.
    std::memset(arguments_, 0, sizeof(arguments_));
    std::memcpy(arguments_, args.data(), args.size());
    group_arguments_ = gsl::cstring_span(arguments_, args.size());

    // We're done with line_buffer_ now, so clear it out to make
    // debugging easier.
    std::memset(line_buffer_, 0, sizeof(line_buffer_));

    lock_manager_.Lock(
        LockManager::kCommandManager,
        [this](LockManager::ReleaseCallback release) {
          auto callback = this->current_command_;
          this->current_command_ = CommandFunction();
          auto args = this->group_arguments_;
          this->group_arguments_ = gsl::cstring_span();

          Response context{&this->stream_, release};
          callback(args, context);
        });
  }

  void UnknownGroup(const gsl::cstring_span&, ErrorCallback cbk) {
    AsyncWrite(stream_, gsl::ensure_z("unknown command\r\n"), cbk);
  }

  struct Item {
    CommandFunction command_function;
  };

  Pool& pool_;
  AsyncStream& stream_;
  LockManager& lock_manager_;

  typedef NamedRegistryBase<Item, 4> Registry;
  Registry registry_;
  ErrorCallback start_callback_;
  char line_buffer_[100] = {};
  char arguments_[100] = {};
  bool started_ = false;

  gsl::cstring_span group_arguments_;
  CommandFunction current_command_;

  AsyncReadUntilContext read_until_context_;
};

CommandManager::CommandManager(
    Pool& pool, AsyncStream& stream, LockManager& lock_manager)
    : impl_(&pool, pool, stream, lock_manager) {}

CommandManager::~CommandManager() {}

void CommandManager::Register(const gsl::cstring_span& name,
                              CommandFunction command_function) {
  auto* element = impl_->registry_.FindOrCreate(
      name, Impl::Registry::kAllowCreate);
  PoolPtr<Impl::Item> item(&impl_->pool_);
  element->ptr = item.get();
  element->ptr->command_function = command_function;
}

void CommandManager::AsyncStart(ErrorCallback callback) {
  assert(!impl_->start_callback_.valid());
  impl_->start_callback_ = callback;

  impl_->StartRead();
}

void CommandManager::Poll() {
  if (!impl_->started_) {
    impl_->started_ = true;
    auto callback = impl_->start_callback_;
    impl_->start_callback_ = ErrorCallback();
    callback(0);
  }
}
