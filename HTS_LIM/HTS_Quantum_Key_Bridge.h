// =========================================================================
// HTS_Quantum_Key_Bridge.h
// PQC 양자 내성 키 브릿지 (AES-CTR 세션 ID 도출)
// Target: STM32F407 (Cortex-M4)
// =========================================================================
#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>     // Inject_Quantum_Entropy 파라미터 (PENDING: raw API 전환 후 제거)

namespace ProtectedEngine {

    class Quantum_Key_Bridge {
    private:
        uint64_t quantum_master_seed[4] = { 0 };
        bool is_pqc_established = false;
        // 인스턴스별 독립 카운터
        uint64_t sync_counter = 0;

    public:
        Quantum_Key_Bridge() noexcept = default;

        // 소멸자: 256비트 마스터 키 보안 소거
        ~Quantum_Key_Bridge() noexcept;

        // 마스터 키 복제 원천 차단
        Quantum_Key_Bridge(const Quantum_Key_Bridge&) = delete;
        Quantum_Key_Bridge& operator=(const Quantum_Key_Bridge&) = delete;
        Quantum_Key_Bridge(Quantum_Key_Bridge&&) = delete;
        Quantum_Key_Bridge& operator=(Quantum_Key_Bridge&&) = delete;

        void Inject_Quantum_Entropy(
            const std::vector<uint8_t>& pqc_material) noexcept;

        [[nodiscard]]
        uint64_t Derive_Quantum_Session_ID() noexcept;

        void Synchronize_CTR_State(uint64_t& out_session_id) noexcept;
    };

} // namespace ProtectedEngine