import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = __dirname;
const SRC = path.join(ROOT, "HTS_V400_Dispatcher.cpp");
const BACKUP = path.join(ROOT, "HTS_V400_Dispatcher.cpp.pre_split_backup");
/** Monolithic source of truth for line slices (created from SRC on first run). */
const CANONICAL = BACKUP;

function stripStaticDetail(s) {
  return s
    .split(/\r?\n/)
    .map((line) => {
      const t = line.trimStart();
      if (t.startsWith("static constexpr "))
        return line.replace("static constexpr ", "constexpr ", 1);
      if (t.startsWith("static inline void "))
        return line.replace("static inline void ", "inline void ", 1);
      if (t.startsWith("static inline int64_t "))
        return line.replace("static inline int64_t ", "inline int64_t ", 1);
      if (t.startsWith("static void "))
        return line.replace("static void ", "inline void ", 1);
      if (t.startsWith("static int64_t "))
        return line.replace("static int64_t ", "inline int64_t ", 1);
      if (t.startsWith("static constexpr int "))
        return line.replace("static constexpr int ", "constexpr int ", 1);
      if (t.startsWith("static inline int16_t "))
        return line.replace("static inline int16_t ", "inline int16_t ", 1);
      return line;
    })
    .join("\n");
}

function main() {
  if (!fs.existsSync(CANONICAL)) {
    if (!fs.existsSync(SRC)) throw new Error("missing " + SRC + " and " + CANONICAL);
    fs.copyFileSync(SRC, CANONICAL);
  }
  const text = fs.readFileSync(CANONICAL, "utf8");
  const lines = text.split(/\r?\n/).map((l) => l + "\n");

  const L = (a, b) => lines.slice(a - 1, b).join("");

  let detailCore = stripStaticDetail(L(98, 193));
  detailCore = detailCore.replace(
    "constexpr int8_t k_w63[64] = {",
    "inline constexpr int8_t k_w63[64] = {",
    1
  );
  let detailMid = stripStaticDetail(L(233, 402));
  detailMid = detailMid.replace(
    "constexpr uint8_t kV5_PreambleSeq[4]",
    "inline constexpr uint8_t kV5_PreambleSeq[4]",
    1
  );
  const detailTail = stripStaticDetail(L(631, 708));

  const internal_hpp =
    `#ifndef HTS_V400_DISPATCHER_INTERNAL_HPP
#define HTS_V400_DISPATCHER_INTERNAL_HPP

#include "HTS_V400_Dispatcher.hpp"
#include <climits>
#include <cstdint>
#include <cstddef>

#if !defined(HTS_TARGET_AMI) && !defined(HTS_TARGET_PSLTE)
#  define HTS_TARGET_PSLTE
#endif
#if defined(HTS_TARGET_AMI) && defined(HTS_TARGET_PSLTE)
#  error "HTS_TARGET_AMI 와 HTS_TARGET_PSLTE 는 동시 정의 불가"
#endif
#if defined(HTS_TARGET_AMI)
#  error "HTS_TARGET_AMI 32-chip 경로는 2026-04-18 T6 붕괴 (43/11200) 로 비활성. 원인 규명 전 빌드 금지."
#endif

namespace ProtectedEngine {

constexpr uint8_t k_W63_FWHT_ROW_NATURAL = 63u;

#if defined(HTS_PHASE0_WALSH_BANK)
extern uint8_t k_W63_FWHT_ROW;
#endif

struct V400HarqCcmChase {
    int32_t harq_Q[FEC_HARQ::NSYM64][FEC_HARQ::C64];
};
struct V400HarqCcmIr {
    int16_t chip_I[FEC_HARQ::NSYM64][FEC_HARQ::C64];
    int16_t chip_Q[FEC_HARQ::NSYM64][FEC_HARQ::C64];
    FEC_HARQ::IR_RxState ir_state;
};
union V400HarqCcm {
    V400HarqCcmChase chase;
    V400HarqCcmIr ir;
};

alignas(64) extern uint8_t g_v400_sym_scratch[FEC_HARQ::NSYM64];
alignas(64) extern HTS_CCM_SECTION V400HarqCcm g_harq_ccm_union;
alignas(64) extern uint64_t g_sic_bits[FEC_HARQ::NSYM64];

extern const int16_t k_walsh_dummy_iq_[64];

extern "C" {
extern volatile int g_hts_ir_diag_chip0;
extern volatile int g_hts_ir_diag_feed_idx;
}

namespace detail {
` +
    detailCore +
    detailMid +
    detailTail +
    `} // namespace detail

} // namespace ProtectedEngine

#endif // HTS_V400_DISPATCHER_INTERNAL_HPP
`;

  const calcFn = L(210, 229).replace(
    "static uint8_t calc_kw63",
    "uint8_t calc_kw63",
    1
  );

  const core_cpp =
    `// =============================================================================
// HTS_V400_Dispatcher_Core.cpp — ctor / FHSS / 설정 / CCM·스크래치 정의
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#include "HTS_Holo_LPI.h"
#include "HTS_RF_Metrics.h"
#include "HTS_Secure_Memory.h"
#include <cstdio>
#include <cstring>
extern "C" volatile int g_hts_ir_diag_chip0 = 0;
extern "C" volatile int g_hts_ir_diag_feed_idx = -1;
extern "C" void Mock_RF_Synth_Set_Channel(uint8_t channel) noexcept {
    const unsigned ch = static_cast<unsigned>(channel) & 0x7Fu;
    std::printf("[Mock_RF_Synth] ch=%u\\n", ch);
}
namespace ProtectedEngine {
using namespace detail;
alignas(64) uint8_t g_v400_sym_scratch[FEC_HARQ::NSYM64];
` +
    L(66, 67) +
    `alignas(64) HTS_CCM_SECTION V400HarqCcm g_harq_ccm_union{};
` +
    L(84, 97).replace(
      "alignas(64) static uint64_t g_sic_bits",
      "alignas(64) uint64_t g_sic_bits",
      1
    ) +
    `#if defined(HTS_PHASE0_WALSH_BANK)
uint8_t k_W63_FWHT_ROW = 255u;
namespace {
` +
    calcFn +
    `} // namespace
#endif
` +
    L(747, 931) +
    L(980, 1285) +
    `} // namespace ProtectedEngine
`;

  const decode_cpp =
    `// =============================================================================
// HTS_V400_Dispatcher_Decode.cpp — Walsh decode / blackhole
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#include <climits>
#include <cstdint>
namespace ProtectedEngine {
using namespace detail;
alignas(64) extern const int16_t k_walsh_dummy_iq_[64] = {};
` +
    L(407, 630) +
    L(709, 740) +
    `} // namespace ProtectedEngine
`;

  const payload_cpp =
    `// =============================================================================
// HTS_V400_Dispatcher_Payload.cpp — on_sym / try_decode / HARQ / hdr / retx
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#include "HTS_Secure_Memory.h"
#if defined(HTS_HARQ_DIAG)
#include "HTS_HARQ_Diag.hpp"
#endif
#if defined(HTS_AMP_DIAG)
#include "HTS_Amp_Diag.hpp"
#endif
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstddef>
namespace ProtectedEngine {
using namespace detail;
` +
    L(932, 979) +
    L(1286, 1810) +
    L(4103, 4175) +
    `} // namespace ProtectedEngine
`;

  const tx_cpp =
    `// =============================================================================
// HTS_V400_Dispatcher_TX.cpp — Build_Packet / Build_Retx
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#include "HTS_Holo_LPI.h"
#include "HTS_Secure_Memory.h"
#include <cstdint>
#include <cstddef>
namespace ProtectedEngine {
using namespace detail;
` +
    L(1811, 2233) +
    `} // namespace ProtectedEngine
`;

  const sync_cpp =
    `// =============================================================================
// HTS_V400_Dispatcher_Sync_PSLTE.cpp — Feed_Chip / phase0 / MF
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#include "HTS_Secure_Memory.h"
#if defined(HTS_SYNC_USE_MATCHED_FILTER)
#include "HTS_Dynamic_Config.h"
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#endif
#if defined(HTS_SYNC_DIAG)
#include "HTS_Sync_Diag.hpp"
#endif
#if defined(HTS_WALSH_ROW_DIAG)
#include "HTS_Walsh_Row_Diag.hpp"
#endif
#if defined(HTS_AMP_DIAG)
#include "HTS_Amp_Diag.hpp"
#endif
#include <climits>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstddef>
namespace ProtectedEngine {
using namespace detail;
` +
    L(2234, 4102) +
    L(4177, 4449) +
    `} // namespace ProtectedEngine
`;

  fs.writeFileSync(path.join(ROOT, "HTS_V400_Dispatcher_Internal.hpp"), internal_hpp);
  fs.writeFileSync(path.join(ROOT, "HTS_V400_Dispatcher_Core.cpp"), core_cpp);
  fs.writeFileSync(path.join(ROOT, "HTS_V400_Dispatcher_Decode.cpp"), decode_cpp);
  fs.writeFileSync(path.join(ROOT, "HTS_V400_Dispatcher_Payload.cpp"), payload_cpp);
  fs.writeFileSync(path.join(ROOT, "HTS_V400_Dispatcher_TX.cpp"), tx_cpp);
  fs.writeFileSync(path.join(ROOT, "HTS_V400_Dispatcher_Sync_PSLTE.cpp"), sync_cpp);

  console.log("OK from", path.basename(CANONICAL));
}

main();
