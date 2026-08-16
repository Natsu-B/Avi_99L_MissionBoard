#include "protocol/command_cache.hpp"

#include <algorithm>

namespace protocol {
namespace {
bool sameRequest(const GenericCommandRequest &left,
                 const GenericCommandRequest &right) {
  return left.transaction_id == right.transaction_id &&
         left.command == right.command && left.arguments == right.arguments;
}
}

CommandCache::LookupResult
CommandCache::lookup(const GenericCommandRequest &request) const {
  const auto iterator = std::find_if(entries_.begin(), entries_.end(),
                                     [&](const Entry &entry) {
    return entry.valid && entry.request.transaction_id == request.transaction_id;
  });
  if (iterator == entries_.end())
    return {};
  if (!sameRequest(iterator->request, request))
    return {Lookup::conflict,
            {request.transaction_id, request.command, CommandPhase::rejected,
             CommandReason::protocol_error, 0}};
  return {Lookup::replay, iterator->result};
}

void CommandCache::rememberAccepted(const GenericCommandRequest &request) {
  Entry *slot = nullptr;
  for (auto &entry : entries_) {
    if (!entry.valid) {
      slot = &entry;
      break;
    }
  }
  if (slot == nullptr) {
    slot = &*std::min_element(entries_.begin(), entries_.end(),
                              [](const Entry &a, const Entry &b) {
                                return a.age < b.age;
                              });
  }
  *slot = {true, ++age_, request,
           {request.transaction_id, request.command, CommandPhase::accepted,
            CommandReason::none, 0}};
}

void CommandCache::finish(const CommandResult &result) {
  for (auto &entry : entries_) {
    if (entry.valid && entry.request.transaction_id == result.transaction_id) {
      entry.result = result;
      entry.age = ++age_;
      return;
    }
  }
}

} // namespace protocol
