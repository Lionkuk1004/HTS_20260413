// =========================================================================
// HTS_Secure_Memory_Manager.hpp
// SecureMemory RAII 스코프 가드 (헤더 전용)
// Target: STM32F407 (Cortex-M4)
//
// [설계]
//  외주·애플리케이션 계층에는 Scoped_Region 만 노출.
//  lockMemory / secureWipe 수동 페어는 본 클래스에 두지 않음(MPU 미해제·MemManage Fault 오용 방지).
//  코어/프레임워크 저수준 직접 호출이 필요하면 SecureMemory.h 를 사용.
//
#pragma once
// ─────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────
//  [필수] 민감 버퍼는 Secure_Memory_Manager::Scoped_Region 만 사용.
//        조기 return 경로에서도 소멸자가 secureWipe 호출 → MPU 해제 + 파쇄.
//  [주의] HTS_SECURE_MPU_LOCK_REGION 빌드에서 lockMemory 는 MPU RBAR/RASR 실제 프로그램.
//        고빈도 루프·ISR 에서 Scoped_Region 남용 금지.
//  [보안] Scoped_Region 복사/이동 금지.
// ─────────────────────────────────────────────────────────

#include "HTS_Secure_Memory.h"
#include <cstdint>
#include <cstddef>

namespace ProtectedEngine {

    /// @brief RAII 전용 — SecureMemory 잠금/소거를 스코프에 바인딩
    class Secure_Memory_Manager {
    public:
        /// @brief 생성 시 lockMemory, 소멸 시 secureWipe(MPU 해제·파쇄).
        class Scoped_Region final {
        public:
            explicit Scoped_Region(void* ptr, size_t size) noexcept
                : ptr_(ptr), size_(size) {
                if (ptr_ != nullptr && size_ != 0u) {
                    SecureMemory::lockMemory(ptr_, size_);
                }
            }

            ~Scoped_Region() noexcept {
                if (ptr_ != nullptr && size_ != 0u) {
                    SecureMemory::secureWipe(ptr_, size_);
                }
            }

            Scoped_Region(const Scoped_Region&) = delete;
            Scoped_Region& operator=(const Scoped_Region&) = delete;
            Scoped_Region(Scoped_Region&&) = delete;
            Scoped_Region& operator=(Scoped_Region&&) = delete;

        private:
            void* ptr_;
            size_t size_;
        };

        Secure_Memory_Manager() = delete;
        ~Secure_Memory_Manager() = delete;
        Secure_Memory_Manager(const Secure_Memory_Manager&) = delete;
        Secure_Memory_Manager& operator=(const Secure_Memory_Manager&) = delete;
        Secure_Memory_Manager(Secure_Memory_Manager&&) = delete;
        Secure_Memory_Manager& operator=(Secure_Memory_Manager&&) = delete;
    };

} // namespace ProtectedEngine
