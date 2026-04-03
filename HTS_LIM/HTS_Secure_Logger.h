// =========================================================================
// HTS_Secure_Logger.h
// 보안 감사 로거 — 이벤트 로깅 + CRC32 로그 무결성
// Target: STM32F407 (Cortex-M4)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  보안 감사 이벤트를 CRC32 무결성 지문 포함 포맷으로 기록
//
//  [사용법]
//   SecureLogger::logSecurityEvent("SESSION_OPEN", "PUF seed injected");
//
//  [출력 포맷]
//   [AUDIT@0xTICK] TYPE | DETAIL | CRC:0xHEX
//
//  [호출 제약]
//   ✓ 메인 루프 / ISR / 기타 — 스핀·mutex·PRIMASK 없음. 슬롯은 fetch_add 후 &(N-1) 마스크로 인덱스(UDIV 없음).
//   슬롯 상태 0/1/2(Empty/Writing/Ready)·CAS·레이트리밋 시 빈 슬롯도 Ready로 커밋(소비자 스톨 방지).
//   레이트: atomic<uint32_t> XOR 패킹(tick>>16·crc16)·단조 CAS. CRC 해시는 접두 제외 페이로드만. 드롭 시 슬롯 secureWipe.
//   pollDebuggerHardwareOrFault()는 AntiDebugManager::pollHardwareOrFault()에 위임 — 실제 감시는 SysTick/스케줄 Tick 등 주기 경로.
//   고정 링(alignas(4), extern "C" 게터·used). N 초과 시 순환 덮임.
//
//  [M-3] HTS_MILITARY_GRADE_EW 정의 시 logSecurityEvent 묵살
//       (HTS_Hardware_Init fputc EMCON과 동일 Zero-Emission 정책)
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif
/// 안티디버그 자폭 전용: 감사 링 SRAM 소거 후 Terminal_Fault (구현: noinline+used+배리어로 LTO 보강).
void SecureLogger_WipeRingAndFault(void);
/// LTO/DCE 방지 및 UART/스토리지 소비자용: 감사 링 저장소(슬롯×256바이트, 행렬 순서).
const char* SecureLogger_GetAuditRingBase(void);
size_t SecureLogger_GetAuditRingSlotCount(void);
size_t SecureLogger_GetAuditLineBytes(void);
uint32_t SecureLogger_GetAuditDropCount(void);
/// 0=Empty, 1=Writing, 2=Ready(소비 가능·빈 줄은 플래시 생략). 읽은 뒤 ClearAuditSlotReady로 0 복귀.
uint8_t SecureLogger_GetAuditSlotReady(size_t slot_index);
void SecureLogger_ClearAuditSlotReady(size_t slot_index);
#ifdef __cplusplus
}
#endif

/// 보드/드라이버에서 강한 심볼로 재정의 가능(GCC/Clang: weak 기본 정의). UART 폴링 TX·플래시 기록 등.
#ifdef __cplusplus
extern "C" {
#endif
void HTS_AuditLog_SyncDrainLine(const char* line, size_t len) noexcept;
#ifdef __cplusplus
}
#endif

namespace ProtectedEngine {

    /// @brief 보안 감사 로거 (정적 유틸리티, 힙 할당 0회)
    class SecureLogger {
    public:
        /// @brief 보안 감사 이벤트 기록
        /// @param eventType  이벤트 분류 ("SESSION_OPEN", "POST_START" 등)
        /// @param details    상세 설명 (const char* — 힙 할당 금지)
        /// @note  ARM: stdout/semihosting 비의존 고정 버퍼 경로 + CRC32 지문
        static void logSecurityEvent(
            const char* eventType,
            const char* details) noexcept;

        /// AntiDebugManager::pollHardwareOrFault() 위임(호환). 양산: HTS_Anti_Debug 직접 호출 권장.
        static void pollDebuggerHardwareOrFault() noexcept;

        /// 트랩/리셋 직전: Ready 감사 슬롯을 HTS_AuditLog_SyncDrainLine(보드 구현)으로 동기 전달. 미구현 시 no-op.
        static void flushAuditRingForTrap() noexcept;

        // 정적 전용 — 인스턴스화 차단 (6종)
        SecureLogger() = delete;
        ~SecureLogger() = delete;
        SecureLogger(const SecureLogger&) = delete;
        SecureLogger& operator=(const SecureLogger&) = delete;
        SecureLogger(SecureLogger&&) = delete;
        SecureLogger& operator=(SecureLogger&&) = delete;
    };

} // namespace ProtectedEngine