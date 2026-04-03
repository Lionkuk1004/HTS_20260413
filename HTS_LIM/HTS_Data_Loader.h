// =========================================================================
// HTS_Data_Loader.h
// 대용량 파일 I/O 및 텐서 변환 (A55 Linux / PC 전용)
// Target: 통합콘솔 (A55 Linux) / PC (STM32 베어메탈 제외)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [주의] 이 모듈은 std::ifstream/ofstream을 사용합니다.
//         STM32F407에는 파일시스템이 없으므로 Cortex-M 빌드에서 제외됩니다.
//         통합콘솔(A55 Linux)은 정상 동작합니다.
//         펌웨어 스토리지는 HTS_Storage_Adapter를 사용하십시오.
//
//  [사용법]
//   1. Data_Loader loader;  → 64MB 메모리 풀 1회 할당
//      → OOM 시 memory_pool 비어있음 → 모든 함수 false 반환
//   2. loader.Initialize(pqc_key);  → PQC 키 브릿지
//   3. loader.Load_File_As_Tensor(path, tensor); → 파일 → 텐서
//   4. loader.Process_Big_Data(in, out); → 64MB 청크 스트리밍
//
//  [보안 설계]
//   memory_pool: 소멸자에서 64MB 전체 보안 소거 (DCE 안전)
//   복사/이동: = delete (64MB pool 복제 원천 차단)
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

// STM32 (Cortex-M) 빌드 차단 — 베어메탈에 파일시스템 없음
// A55 (aarch64) Linux는 차단 대상 아님
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Data_Loader는 A55 Linux/PC 전용입니다. STM32 베어메탈 빌드에서 제외하십시오."
#endif

#include <vector>
#include <string>
#include <cstdint>
#include <cstddef>
#include "HTS_Storage_Interface.h"

namespace ProtectedEngine {

    class Data_Loader {
    public:
        /// @brief 생성자 — 64MB 메모리 풀 1회 할당
        /// @note  OOM 시 memory_pool 비어있음 → 모든 함수 false 반환
        Data_Loader() noexcept;

        /// @brief 소멸자 — memory_pool 64MB 보안 소거
        ~Data_Loader() noexcept;

        /// 64MB pool + Storage_Interface 복제 원천 차단
        Data_Loader(const Data_Loader&) = delete;
        Data_Loader& operator=(const Data_Loader&) = delete;
        Data_Loader(Data_Loader&&) = delete;
        Data_Loader& operator=(Data_Loader&&) = delete;

        /// @brief PQC 키 브릿지 초기화
        void Initialize(const std::vector<uint8_t>& pqc_key) noexcept;

        /// @brief 파일 → 텐서 (하위 호환 래퍼)
        [[nodiscard]]
        std::vector<uint32_t> Load_File_As_Tensor(
            const std::string& path) noexcept;

        /// @brief 파일 → 텐서 (Zero-Allocation 다이렉트 — 권장 API)
        /// @param path        파일 경로
        /// @param out_tensor  출력 텐서 (사전 할당 재활용)
        /// @return true=성공, false=파일 오류/OOM
        [[nodiscard]]
        bool Load_File_As_Tensor(const std::string& path,
            std::vector<uint32_t>& out_tensor) noexcept;

        /// @brief 텐서 → 파일 저장
        /// @param path           출력 파일 경로
        /// @param tensor         텐서 데이터
        /// @param original_size  원본 바이트 수 (tensor.size()×4 이하)
        /// @return true=성공
        [[nodiscard]]
        bool Save_Tensor_As_File(const std::string& path,
            const std::vector<uint32_t>& tensor,
            size_t original_size) noexcept;

        /// @brief 대용량 스트리밍 보호 공정 (64MB 청크 단위)
        /// @return true=전체 처리 성공
        [[nodiscard]]
        bool Process_Big_Data(const std::string& input_path,
            const std::string& output_path) noexcept;

        /// @brief 파일 크기 조회 (바이트)
        [[nodiscard]]
        size_t Get_File_Size(const std::string& path) noexcept;

    private:
        Storage_Interface storage_io;
        static const size_t CHUNK_SIZE =
            (64u * 1024u * 1024u) / sizeof(uint32_t);
        std::vector<uint32_t> memory_pool;
    };

} // namespace ProtectedEngine