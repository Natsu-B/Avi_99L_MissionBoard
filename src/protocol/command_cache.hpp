#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "protocol/wire.hpp"

namespace protocol {

class CommandCache {
public:
  enum class Lookup : uint8_t { miss, replay, conflict };
  struct LookupResult {
    Lookup kind{Lookup::miss};
    CommandResult result{};
  };

  [[nodiscard]] LookupResult lookup(const GenericCommandRequest &request) const;
  void rememberAccepted(const GenericCommandRequest &request);
  void finish(const CommandResult &result);

private:
  struct Entry {
    bool valid{};
    uint32_t age{};
    GenericCommandRequest request{};
    CommandResult result{};
  };
  static constexpr std::size_t kCapacity = 16;
  std::array<Entry, kCapacity> entries_{};
  uint32_t age_{};
};

} // namespace protocol
