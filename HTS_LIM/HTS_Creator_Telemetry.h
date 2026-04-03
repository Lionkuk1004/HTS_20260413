// =========================================================================
// HTS_Creator_Telemetry.h
// 개발 모드 텔레메트리 (릴리즈 빌드 시 Zero-Cost 소거)
// Target: STM32F407 (Cortex-M4)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  개발/디버깅 중 모듈별 동작 상태를 실시간 추적합니다.
//  릴리즈 빌드에서는 함수 본문이 비워져 LTO가 호출 자체를 소거합니다.
//  → Flash 0바이트, 사이클 0회 (Zero-Cost Abstraction)
//
//  [활성화]
//   전처리기 정의: -D_HTS_CREATOR_MODE
//   STM32 양산: 정의 시 컴파일 에러 (#error 차단)
//   A55 양산:   정의 시 컴파일 에러 (HTS_ALLOW_TELEMETRY_AARCH64 병기 필요)
//   PC 개발:    자유 사용
//
//  [사용법]
//   HTS_Telemetry::Log("PHY_TX", "Spreading complete", chip_count);
//   HTS_Telemetry::Log("SESSION", "Gateway opened");
//   // 릴리즈: 위 호출이 컴파일 후 0바이트 → 성능 영향 없음
//
// ─────────────────────────────────────────────────────────────────────────
// =========================================================================
#pragma once

#include <cstdint>

namespace ProtectedEngine {

    class HTS_Telemetry {
    public:
        // =================================================================
        //  Log — 모듈별 동작 이벤트 기록
        //
        //  [파라미터]
        //  module_name: 모듈 식별자 (예: "PHY_TX", "SESSION") — nullptr 안전
        //  action:      동작 설명 (예: "initialized", "key rotated") — nullptr 안전
        //  value:       선택적 32비트 값 (0이면 출력 생략)
        //
        //  [릴리즈 빌드]
        //  _HTS_CREATOR_MODE 미정의 → 빈 함수 → LTO가 호출 자체를 소거
        //  → 코드 크기 영향 0바이트, 런타임 영향 0사이클
        // =================================================================
        static void Log(const char* module_name,
            const char* action,
            uint32_t value = 0) noexcept;
    };

} // namespace ProtectedEngine