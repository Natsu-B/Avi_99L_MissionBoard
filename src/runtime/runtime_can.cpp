#include "runtime/runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>

#include "esp_timer.h"
#include "freertos/task.h"
#include "protocol/wire.hpp"

namespace runtime {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
bool allZero(const std::array<uint8_t, 6> &args) {
  return std::all_of(args.begin(), args.end(),
                     [](uint8_t value) { return value == 0; });
}
uint64_t elapsed(const mission::Snapshot &snapshot, uint64_t now_us) {
  if (!snapshot.liftoff_valid || now_us < snapshot.liftoff_us) return 0;
  return now_us - snapshot.liftoff_us;
}
} // namespace

void Runtime::canTask() {
  uint8_t status_sequence = 0;
  uint8_t kinematics_sequence = 0;
  uint8_t control_sequence = 0;
  uint8_t control_roll_sequence = 0;
  uint8_t lps_sequence = 0;
  uint8_t airspeed_sequence = 0;
  uint8_t last_reference_event_sequence = 0;
  uint64_t next_status = 0;
  uint64_t next_kinematics = 0;
  uint64_t next_control = 0;
  uint64_t next_control_roll = 0;
  uint64_t next_lps = 0;
  uint64_t next_airspeed = 0;
  TickType_t wake = xTaskGetTickCount();
  uint64_t next_can_retry = 0;
  uint64_t next_can_status = 0;

  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    if (!can_.initialized() && now >= next_can_retry) {
      (void)initializeCan();
      next_can_retry = now + 1'000'000ULL;
    }
    if (can_.initialized() && now >= next_can_status) {
      CANCREATE::Status status{};
      if (can_.getStatus(status) == ESP_OK &&
          status.state == CANCREATE::State::bus_off)
        (void)can_.recover(avi::Timeout::milliseconds(10));
      next_can_status = now + 100'000ULL;
    }
    CANCREATE::Frame raw{};
    while (can_.read(raw, avi::Timeout::noWait()) == ESP_OK) {
      protocol::CanFrame frame{};
      frame.identifier = raw.identifier;
      frame.data_length = raw.data_length;
      frame.extended = raw.extended;
      frame.remote = raw.remote;
      std::copy(std::begin(raw.data), std::end(raw.data), frame.data.begin());

      protocol::GenericCommandRequest request{};
      if (protocol::decodeGenericCommand(frame, request)) {
        if (request.transaction_id == 0 || !allZero(request.arguments)) {
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::rejected,
               protocol::CommandReason::invalid_argument, 0}));
          continue;
        }
        const auto lookup = command_cache_.lookup(request);
        if (lookup.kind != protocol::CommandCache::Lookup::miss) {
          sendCanFrame(protocol::encode(lookup.result));
          continue;
        }

        const auto code = static_cast<protocol::CommandCode>(request.command);
        const auto phase = state_.snapshot().phase;
        if (code == protocol::CommandCode::start_sequence) {
          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));
          const bool ok = state_.startSequence();
          const protocol::CommandResult final{
              request.transaction_id, request.command,
              ok ? protocol::CommandPhase::completed
                 : protocol::CommandPhase::failed,
              ok ? protocol::CommandReason::none
                 : protocol::CommandReason::invalid_state,
              0};
          command_cache_.finish(final);
          sendCanFrame(protocol::encode(final));
          continue;
        }

        const bool is_fin = code == protocol::CommandCode::fin_free ||
                            code == protocol::CommandCode::fin_hold_current;
        const bool is_para = code == protocol::CommandCode::para_open ||
                             code == protocol::CommandCode::para_close;
        if ((is_fin || is_para) && phase != mission::Phase::command_receive) {
          const protocol::CommandResult rejected{
              request.transaction_id, request.command,
              protocol::CommandPhase::rejected,
              protocol::CommandReason::invalid_state, 0};
          command_cache_.rememberAccepted(request);
          command_cache_.finish(rejected);
          sendCanFrame(protocol::encode(rejected));
          continue;
        }

        if (is_fin) {
          bool expected = false;
          if (!fin_command_pending_.compare_exchange_strong(expected, true)) {
            const protocol::CommandResult rejected{
                request.transaction_id, request.command,
                protocol::CommandPhase::rejected,
                protocol::CommandReason::busy, 0};
            command_cache_.rememberAccepted(request);
            command_cache_.finish(rejected);
            sendCanFrame(protocol::encode(rejected));
            continue;
          }
          const ActuatorCommand command{request.transaction_id, request.command};
          if (xQueueSend(fin_command_queue_, &command, 0) != pdTRUE) {
            fin_command_pending_.store(false);
            sendCanFrame(protocol::encode(
                {request.transaction_id, request.command,
                 protocol::CommandPhase::rejected,
                 protocol::CommandReason::busy, 0}));
            continue;
          }
          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));
          continue;
        }

        if (is_para) {
          bool expected = false;
          if (!para_command_pending_.compare_exchange_strong(expected, true)) {
            const protocol::CommandResult rejected{
                request.transaction_id, request.command,
                protocol::CommandPhase::rejected,
                protocol::CommandReason::busy, 0};
            command_cache_.rememberAccepted(request);
            command_cache_.finish(rejected);
            sendCanFrame(protocol::encode(rejected));
            continue;
          }
          const ActuatorCommand command{request.transaction_id, request.command};
          if (xQueueSend(para_command_queue_, &command, 0) != pdTRUE) {
            para_command_pending_.store(false);
            sendCanFrame(protocol::encode(
                {request.transaction_id, request.command,
                 protocol::CommandPhase::rejected,
                 protocol::CommandReason::busy, 0}));
            continue;
          }
          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));
          continue;
        }

        const protocol::CommandResult rejected{
            request.transaction_id, request.command,
            protocol::CommandPhase::rejected,
            protocol::CommandReason::not_supported, 0};
        command_cache_.rememberAccepted(request);
        command_cache_.finish(rejected);
        sendCanFrame(protocol::encode(rejected));
        continue;
      }

      uint8_t emergency_transaction = 0;
      if (protocol::decodeEmergency(frame, emergency_transaction)) {
        protocol::GenericCommandRequest replay_key{};
        replay_key.transaction_id = emergency_transaction;
        replay_key.command = static_cast<uint8_t>(
            protocol::CommandCode::liftoff_emergency_result);
        const auto lookup = command_cache_.lookup(replay_key);
        if (lookup.kind != protocol::CommandCache::Lookup::miss) {
          sendCanFrame(protocol::encode(lookup.result));
          continue;
        }
        if (emergency_transaction != 0)
          command_cache_.rememberAccepted(replay_key);
        const bool accepted = emergency_transaction != 0 &&
                              state_.liftoffEmergencyRollback();
        const protocol::CommandResult result{
            emergency_transaction,
            static_cast<uint8_t>(protocol::CommandCode::liftoff_emergency_result),
            accepted ? protocol::CommandPhase::completed
                     : protocol::CommandPhase::rejected,
            accepted ? protocol::CommandReason::none
                     : (emergency_transaction == 0
                            ? protocol::CommandReason::invalid_argument
                            : protocol::CommandReason::invalid_state),
            0};
        if (emergency_transaction != 0)
         ÛÛ[X[™ØØXÚWË™š[š\Ú
™\İ[
NÂˆÙ[™Ø[‘œ˜[YJ›İØÛÛ™[˜ÛÙJ™\İ[
JNÂˆBˆB‚ˆ›İØÛÛÛÛ[X[™™\İ[\Ú×Ü™\İ[ßNÂˆÚ[H
]Y]YT™XÙZ]™J™\İ[Ü]Y]YWË	\Ú×Ü™\İ[
HOH•QJHÂˆÛÛ[X[™ØØXÚWË™š[š\Ú
\Ú×Ü™\İ[
NÂˆÙ[™Ø[‘œ˜[YJ›İØÛÛ™[˜ÛÙJ\Ú×Ü™\İ[
JNÂˆB‚ˆYˆ
›İÈH™^ÚÚ[™[X]XÜÊHÂˆÛÛœİ]]Èš[ˆHš[—Ë[[Y]J
NÂˆÛÛœİ›ÛÛ™Y™\™[˜ÙWİ˜[YBˆÛÛ›ÛÜ™Y™\™[˜ÙWİ˜[YË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JNÂˆ›İØÛÛ’Ú[™[X]XÜÈY\ÜØYÙ^ßNÂˆY\ÜØYÙKœÙ\]Y[˜ÙHHÚ[™[X]XÜ×ÜÙ\]Y[˜ÙJÊÎÂˆY\ÜØYÙKœ›ÛÜ˜]ÈH›İØÛÛ™[˜ÛÙT›Û
ˆÛÛ›ÛÜ›ÛÙ]šX][Û—Ü˜YË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JH
ˆÔ˜YÑYËˆ™Y™\™[˜ÙWİ˜[Y
NÂˆY\ÜØYÙKœ›ÛÜ˜]WÜ˜]ÈH›İØÛÛ™[˜ÛÙT›Û˜]JˆŞ\›×Ü›ÛÜ˜]WÙ×Ë›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JKˆ[]Wİ˜[YË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JJNÂˆY\ÜØYÙK™š[—Ø[™ÛWÜ˜]ÈH›İØÛÛ™[˜ÛÙQš[[™ÛJˆš[‹˜[™ÛWÙYËš[‹™[˜ÛÙ\—İ˜[Y	‰ˆš[‹™\›×İ˜[Y
NÂˆY\ÜØYÙK™š[—Ü˜]WÜ˜]ÈBˆ›İØÛÛ™[˜ÛÙQš[”˜]Jš[‹œ˜]WÙY×ÜËš[‹œ˜]Wİ˜[Y
NÂˆÙ[™Ø[‘œ˜[YJ›İØÛÛ™[˜ÛÙJY\ÜØYÙJJNÂˆ™^ÚÚ[™[X]XÜÈH›İÈ
ÈL	ÌSÂˆB‚ˆYˆ
›İÈH™^ØÛÛ›Û
HÂˆÛÛœİ]]ÈÛ˜\ÚİHİ]WËœÛ˜\Úİ

NÂˆÛÛœİ›ÛÛ›YÚİ[YWİ˜[YHÛ˜\Úİ›YÙ™—İ˜[YÂˆ›İØÛÛÛÛ›Û[[Y]HY\ÜØYÙ^ßNÂˆY\ÜØYÙKœÙ\]Y[˜ÙHHÛÛ›ÛÜÙ\]Y[˜ÙJÊÎÂˆY\ÜØYÙKœ™\]Y\İYİÜœ]YWÜ˜]ÈH›İØÛÛ™[˜ÛÙT™\]Y\İYÜœ]YJˆ™\]Y\İYØÛÛ›ÛİÜœ]YWÛ›WË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JKˆÛÛ›ÛØXİ]™WË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JJNÂˆY\ÜØYÙK™›YÚÙ[\ÙYÜ˜]ÈH›İØÛÛ™[˜ÛÙQ›YÚ[\ÙY
ˆİ]X×ØØ\İİX›OŠ[\ÙY
Û˜\Úİ›İÊJH
ˆKŒKM‹ˆ›YÚİ[YWİ˜[Y
NÂˆÙ[™Ø[‘œ˜[YJ›İØÛÛ™[˜ÛÙJY\ÜØYÙJJNÂˆ™^ØÛÛ›ÛH›İÈ
ÈL	ÌSÂˆB‚ˆYˆ
›İÈH™^Üİ]\ÊHÂˆÛÛœİ]]ÈÛ˜\ÚİHİ]WËœÛ˜\Úİ

NÂˆÛÛœİ]]Èš[ˆHš[—Ë[[Y]J
NÂˆÛÛœİ]]È\˜HH\˜WË[[Y]J
NÂˆ›İØÛÛ“Z\ÜÚ[Û”İ]\ÈY\ÜØYÙ^ßNÂˆY\ÜØYÙKœÙ\]Y[˜ÙHHİ]\×ÜÙ\]Y[˜ÙJÊÎÂˆY\ÜØYÙKœİ]HHÚ\™Tİ]J
NÂˆY\ÜØYÙK™›YÚÜİ]\ÈHİ]X×ØØ\İZ[M—İŠˆ
[]Wİ˜[YË›ØY

HÈUHˆJHˆ
×İ˜[YË›ØY

HÈ•HˆJHˆ
š[‹™\›×İ˜[YÈHˆJH
\˜Kœ™XYHÈHˆJHˆ
Û˜\Úİ™\Ş[Y[Üİ\YÈM•HˆJHˆ
Û˜\ÚİœİÙ\—Øİ]Ù™ˆÈÌ•HˆJJNÂˆY\ÜØYÙK™š[—Û[ÙHBˆš[‹œİ]HOHXİX]ÜœÎ‘š[”İ]N™\›×ÚÛˆÈ›İØÛÛ‘š[“[ÙN™\›×ÚÛˆˆš[‹œİ]HOHXİX]ÜœÎ‘š[”İ]Nœ›ÛØÛÛ›ÛˆÈ›İØÛÛ‘š[“[ÙNœ›ÛØÛÛ›Ûˆˆš[‹œİ]HOHXİX]ÜœÎ‘š[”İ]N™œ™YHÈ›İØÛÛ‘š[“[ÙN™œ™YBˆˆ›İØÛÛ‘š[“[ÙN[šÛ›İÛÂˆY\ÜØYÙKœ\˜WÛ[ÙHH\˜K›[ÙNÂˆY\ÜØYÙKœ\˜XÚ]WØ[™ÛWÜ˜]ÈBˆ›İØÛÛ™[˜ÛÙT\˜XÚ]P[™ÛJ\˜KœÜÚ][Û—ÙYË\˜KœÜÚ][Û—İ˜[Y
NÂˆÙ[™Ø[‘œ˜[YJ›İØÛÛ™[˜ÛÙJY\ÜØYÙJJNÂˆ™^Üİ]\ÈH›İÈ
ÈL	ÌSÂˆB‚ˆYˆ
›İÈH™^ÛÊHÂˆ›İØÛÛ“Õ[[Y]HY\ÜØYÙ^ßNÂˆY\ÜØYÙKœÙ\]Y[˜ÙHH×ÜÙ\]Y[˜ÙJÊÎÂˆÛÛœİ›ÛÛ˜[YH×İ˜[YË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JNÂˆY\ÜØYÙKœ™\Üİ\™WÜ˜]ÈH›İØÛÛ™[˜ÛÙSÔ™\Üİ\™Jˆ×Ü™\Üİ\™WÚWË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JK˜[Y
NÂˆY\ÜØYÙK[\\˜]\™WÜ˜]ÈH›İØÛÛ™[˜ÛÙSÕ[\\˜]\™Jˆ×İ[\\˜]\™WØ×Ë›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JK˜[Y
NÂˆÙ[™Ø[‘œ˜[YJ›İØÛÛ™[˜ÛÙJY\ÜØYÙJJNÂˆ™^ÛÈH›İÈ
È	ÌSÂˆB‚ˆYˆ
›İÈH™^ØZ\œÜYY
HÂˆ›İØÛÛZ\œÜYY[[Y]HY\ÜØYÙ^ßNÂˆY\ÜØYÙKœÙ\]Y[˜ÙHHZ\œÜYYÜÙ\]Y[˜ÙJÊÎÂˆY\ÜØYÙK˜Z\œÜYYÜ˜]ÈH›İØÛÛ™[˜ÛÙPZ\œÜYY
ˆZ\œÜYYÛ\×Ë›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JKˆZ\œÜYYİ˜[YË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JJNÂˆÙ[™Ø[‘œ˜[YJ›İØÛÛ™[˜ÛÙJY\ÜØYÙJJNÂˆ™^ØZ\œÜYYH›İÈ
ÈL	ÌSÂˆB‚ˆYˆ
›İÈH™^ØÛÛ›ÛÜ›Û
HÂˆÛÛœİ›ÛÛ™Y™\™[˜ÙWİ˜[YBˆÛÛ›ÛÜ™Y™\™[˜ÙWİ˜[YË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JNÂˆÛÛœİ›ÛÛXİ]™HHÛÛ›ÛØXİ]™WË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JNÂˆÛÛœİZ[İ]™[ÜÙ\]Y[˜ÙHBˆ™Y™\™[˜ÙWØØ\\™WÙ]™[ÜÙ\]Y[˜ÙWË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JNÂˆ›İØÛÛÛÛ›Û›Û[[Y]UŒˆY\ÜØYÙ^ßNÂˆY\ÜØYÙKœÙ\]Y[˜ÙHHÛÛ›ÛÜ›ÛÜÙ\]Y[˜ÙJÊÎÂˆY\ÜØYÙK˜ÛÛ›ÛÜ›ÛÜ™Y™\™[˜ÙWİ[Ü˜\YÜ˜]ÈBˆ›İØÛÛ™[˜ÛÙT›Û
Œ™Y™\™[˜ÙWİ˜[Y
NÂˆY\ÜØYÙKœ›ÛÙ]šX][Û—İ[Ü˜\YÜ˜]ÈH›İØÛÛ™[˜ÛÙT›Û
ˆÛÛ›ÛÜ›ÛÙ]šX][Û—Ü˜YË›ØY
İ›Y[[ÜWÛÜ™\—ØXÜ]Z\™JH
ˆÔ˜YÑYËˆ™Y™\™[˜ÙWİ˜[Y
NÂˆYˆ
™Y™\™[˜ÙWİ˜[Y
BˆY\ÜØYÙK™›YÜÈH›İØÛÛÛÛ›Û›Û[[Y]UŒœ™Y™\™[˜ÙWİ˜[YÂˆYˆ
Xİ]™JBˆY\ÜØYÙK™›YÜÈH›İØÛÛÛÛ›Û›Û[[Y]UŒ˜ÛÛ›ÛØXİ]™NÂˆYˆ
]™[ÜÙ\]Y[˜ÙHOH\İÜ™Y™\™[˜ÙWÙ]™[ÜÙ\]Y[˜ÙJHÂˆY\ÜØYÙK™›YÜÈH›İØÛÛÛÛ›Û›Û[[Y]UŒ‚ˆ™Y™\™[˜ÙWØØ\\™YÜÚ[˜ÙWÜ™]š[İ\×Ùœ˜[YNÂˆ\İÜ™Y™\™[˜ÙWÙ]™[ÜÙ\]Y[˜ÙHH]™[ÜÙ\]Y[˜ÙNÂˆBˆY\ÜØYÙKœ™Y™\™[˜ÙWØØ\\™WÙ]™[ÜÙ\]Y[˜ÙHH]™[ÜÙ\]Y[˜ÙNÂˆÙ[™Ø[‘œ˜[YJ›İØÛÛ™[˜ÛÙJY\ÜØYÙJJNÂˆ™^ØÛÛ›ÛÜ›ÛH›İÈ
ÈL	ÌSÂˆB‚ˆ•\ÚÑ[^U[[
	ØZÙKT×Õ×ÕPÒÔÊJJNÂˆBŸB‚ŸHËÈ˜[Y\ÜXÙH[[YB