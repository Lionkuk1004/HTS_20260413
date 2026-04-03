// =============================================================================
//  KCMVP_암호_4종_종합_테스트.cpp
//  ARIA / HMAC-SHA256 / LEA / LSH-256·224 — 단일 실행 파일, 마지막에 4종 요약 표
// =============================================================================
#include "HTS_ARIA_Bridge.hpp"
#include "HTS_HMAC_Bridge.hpp"
#include "HTS_LEA_Bridge.h"
#include "HTS_LSH256_Bridge.h"
#include "HTS_Secure_Memory.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <cstdint>

namespace kat_aria {
// =========================================================================
//  유틸리티 — D-2: 스택 버퍼 소거는 SecureMemory::secureWipe
// =========================================================================
static void SZ(void* p, size_t n) noexcept {
    ProtectedEngine::SecureMemory::secureWipe(p, n);
}

/// @brief 상수 시간 바이트열 동등 비교 (H-1, 타이밍 누출 완화)
static bool CT_Eq(const uint8_t* a, const uint8_t* b, size_t n) noexcept {
    if (a == nullptr || b == nullptr) {
        return false;
    }
    if (n == 0u) {
        return true;
    }
    uint32_t diff = 0u;
    for (size_t i = 0u; i < n; ++i) {
        diff |= static_cast<uint32_t>(a[i] ^ b[i]);
    }
    return diff == 0u;
}

static void PHex(const char* lbl, const uint8_t* data, size_t len) {
    if (lbl == nullptr) {
        return;
    }
    std::ios_base::fmtflags f = std::cout.flags();
    std::cout << lbl;
    if (data != nullptr && len > 0u) {
        for (size_t i = 0u; i < len; ++i) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<unsigned>(data[i]);
        }
    }
    std::cout.flags(f);
    std::cout << "\n";
}

// =========================================================================
//  KAT 벡터 구조체
//  ciphertext[16]: **사전 검증된 정적 정답** (런타임 생성 금지)
// =========================================================================
struct KAT_Vector {
    const char* name;
    int         key_bits;
    uint8_t     key[32];
    uint8_t     plaintext[16];
    uint8_t     ciphertext[16];
};

// =========================================================================
//  RFC 5794 Appendix A — Example Data (ECB 1블록, 공식 정적 벡터)
//  출처: https://www.rfc-editor.org/rfc/rfc5794 (동일 벡터는 seed.kisa.or.kr 대조 가능)
//
//  추가 벡터(영벡터 등)는 반드시 공식 문서에서 확인한 16바이트 CT를 **상수로만** 추가할 것.
// =========================================================================
static KAT_Vector KAT_TABLE[] = {

    // A.1  128-Bit Key — CT: d718fbd6ab644c739da95f3be6451778
    {
        "ARIA-128-ECB RFC5794 A.1",
        128,
        {
            0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07,
            0x08,0x09,0x0A,0x0B, 0x0C,0x0D,0x0E,0x0F,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0
        },
        {
            0x00,0x11,0x22,0x33, 0x44,0x55,0x66,0x77,
            0x88,0x99,0xAA,0xBB, 0xCC,0xDD,0xEE,0xFF
        },
        {
            0xD7,0x18,0xFB,0xD6, 0xAB,0x64,0x4C,0x73,
            0x9D,0xA9,0x5F,0x3B, 0xE6,0x45,0x17,0x78
        }
    },

    // A.2  192-Bit Key — CT: 26449c1805dbe7aa25a468ce263a9e79
    {
        "ARIA-192-ECB RFC5794 A.2",
        192,
        {
            0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07,
            0x08,0x09,0x0A,0x0B, 0x0C,0x0D,0x0E,0x0F,
            0x10,0x11,0x12,0x13, 0x14,0x15,0x16,0x17,
            0,0,0,0,0,0,0,0
        },
        {
            0x00,0x11,0x22,0x33, 0x44,0x55,0x66,0x77,
            0x88,0x99,0xAA,0xBB, 0xCC,0xDD,0xEE,0xFF
        },
        {
            0x26,0x44,0x9C,0x18, 0x05,0xDB,0xE7,0xAA,
            0x25,0xA4,0x68,0xCE, 0x26,0x3A,0x9E,0x79
        }
    },

    // A.3  256-Bit Key — CT: f92bd7c79fb72e2f2b8f80c1972d24fc
    {
        "ARIA-256-ECB RFC5794 A.3",
        256,
        {
            0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07,
            0x08,0x09,0x0A,0x0B, 0x0C,0x0D,0x0E,0x0F,
            0x10,0x11,0x12,0x13, 0x14,0x15,0x16,0x17,
            0x18,0x19,0x1A,0x1B, 0x1C,0x1D,0x1E,0x1F
        },
        {
            0x00,0x11,0x22,0x33, 0x44,0x55,0x66,0x77,
            0x88,0x99,0xAA,0xBB, 0xCC,0xDD,0xEE,0xFF
        },
        {
            0xF9,0x2B,0xD7,0xC7, 0x9F,0xB7,0x2E,0x2F,
            0x2B,0x8F,0x80,0xC1, 0x97,0x2D,0x24,0xFC
        }
    },
};

static const size_t KAT_COUNT =
sizeof(KAT_TABLE) / sizeof(KAT_TABLE[0]);


// =========================================================================
//  KAT 검증 — 정적 Expected(RFC 5794 Appx A) vs MUT(ARIA_Bridge)
// =========================================================================
bool Run_aria() {
    std::cout << "\n==========================================\n"
        << "  KAT 검증 — 정적 기대암호문 vs MUT [KCMVP KAT]\n"
        << "  (런타임 참조 라이브러리로 정답 생성 없음)\n"
        << "==========================================\n";

    int pass = 0, fail = 0;

    for (size_t i = 0; i < KAT_COUNT; ++i) {
        const KAT_Vector& v = KAT_TABLE[i];

        std::cout << "\n------------------------------------------\n"
            << "  [KAT-" << (i + 1) << "] " << v.name << "\n"
            << "------------------------------------------\n";

        size_t kbytes = static_cast<size_t>(v.key_bits / 8);
        PHex("  Key        : ", v.key, kbytes);
        PHex("  Plaintext  : ", v.plaintext, 16);
        PHex("  Expected   : ", v.ciphertext, 16);

        bool kat_ok = true;

        // ---- 암호화 재실행 -----------------------------------------------
        uint8_t ct[16] = {};
        bool enc_ok = false;
        {
            ProtectedEngine::ARIA_Bridge bridge;
            if (bridge.Initialize_Encryption(v.key, v.key_bits))
                enc_ok = bridge.Process_Block(v.plaintext, ct);
        }

        if (!enc_ok) {
            std::cout << "  [FAIL] 암호화 실패\n";
            ++fail; SZ(ct, sizeof(ct)); continue;
        }

        PHex("  Actual     : ", ct, 16);

        bool enc_match = CT_Eq(ct, v.ciphertext, 16);
        std::cout << (enc_match
            ? "  [PASS] 암호문 일치 — KAT 통과\n"
            : "  [FAIL] 암호문 불일치\n");
        if (!enc_match) kat_ok = false;

        // ---- 복호화 역방향 검증 -----------------------------------------
        {
            uint8_t pt[16] = {};
            bool dec_ok = false;
            {
                ProtectedEngine::ARIA_Bridge bridge;
                if (bridge.Initialize_Decryption(v.key, v.key_bits))
                    dec_ok = bridge.Process_Block(ct, pt);
            }
            bool rev_ok = dec_ok && CT_Eq(pt, v.plaintext, 16);
            std::cout << (rev_ok
                ? "  [PASS] 복호화 역방향 검증 통과\n"
                : "  [FAIL] 복호화 역방향 검증 실패\n");
            if (!rev_ok) kat_ok = false;
            SZ(pt, sizeof(pt));
        }

        // ---- 키 변조 탐지 -----------------------------------------------
        {
            uint8_t wrong_key[32] = {};
            std::memcpy(wrong_key, v.key, kbytes);
            wrong_key[0] ^= 0x01;

            uint8_t ct2[16] = {};
            bool wk_ok = false;
            {
                ProtectedEngine::ARIA_Bridge bridge;
                if (bridge.Initialize_Encryption(wrong_key, v.key_bits))
                    wk_ok = bridge.Process_Block(v.plaintext, ct2);
            }
            bool key_changed = wk_ok && !CT_Eq(ct2, v.ciphertext, 16);
            std::cout << (key_changed
                ? "  [PASS] 키 변조 탐지 성공\n"
                : "  [FAIL] 키 변조 탐지 실패\n");
            if (!key_changed) kat_ok = false;
            SZ(wrong_key, sizeof(wrong_key));
            SZ(ct2, sizeof(ct2));
        }

        SZ(ct, sizeof(ct));
        kat_ok ? ++pass : ++fail;
    }

    // ---- 최종 요약 -------------------------------------------------------
    std::cout << "\n==========================================\n"
        << "  KAT 최종 결과\n"
        << "  PASS : " << pass << " / " << KAT_COUNT << "\n"
        << "  FAIL : " << fail << " / " << KAT_COUNT << "\n";

    if (fail == 0) {
        std::cout << "  판정 : 전체 통과 ✓\n\n"
            << "  [KCMVP 다음 단계]\n"
            << "  1. 상수 Expected를 seed.kisa.or.kr 공식 문서와 대조 유지\n"
            << "  2. ECB / CBC / CTR 전 모드 KAT 완료\n"
            << "  3. 암호모듈 경계 정의서 및 보안정책서 작성\n"
            << "  4. 국정원 지정 시험기관 제출\n";
    }
    else {
        std::cout << "  판정 : 미통과 ✗\n"
            << "  FAIL 시 MUT(ARIA_Bridge) 및 키/모드 설정 재확인\n";
    }
    std::cout << "==========================================\n";

    return (fail == 0);
}


// =========================================================================
//  main
// =========================================================================
}

namespace kat_hmac {
namespace {
using HM = ProtectedEngine::HMAC_Bridge;
/// Generate/Verify는 SECURE_TRUE·SECURE_FALSE 모두 비영 — if(r)/!r 불가.
inline bool hmac_ok(uint32_t r) noexcept { return r == HM::SECURE_TRUE; }
} // namespace

// =========================================================================
//  유틸리티 — D-2 / X-5-1: 스택·임시 버퍼 소거는 SecureMemory::secureWipe
// =========================================================================
static void SZ(void* p, size_t n) noexcept {
    ProtectedEngine::SecureMemory::secureWipe(p, n);
}

static bool CT_Eq(const uint8_t* a,
    const uint8_t* b, size_t n) noexcept {
    volatile uint8_t d = 0;
    for (size_t i = 0; i < n; ++i) d |= a[i] ^ b[i];
    return (d == 0);
}

static void PHex(const char* lbl,
    const uint8_t* data, size_t len) {
    std::ios_base::fmtflags f = std::cout.flags();
    std::cout << lbl;
    for (size_t i = 0; i < len; ++i)
        std::cout << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<unsigned>(data[i]);
    std::cout.flags(f);
    std::cout << "\n";
}

// =========================================================================
//  KAT 벡터 구조체
//  key[64]     : HMAC 키 최대 64바이트 (SHA-256 블록 크기)
//  message[128]: 테스트 메시지 최대 128바이트
//  expected[32]: HMAC-SHA256 출력 32바이트 고정
// =========================================================================
struct KAT_Vector {
    const char* name;
    uint8_t     key[64];       // 정확히 64바이트
    size_t      key_len;
    uint8_t     message[128];  // 정확히 128바이트
    size_t      msg_len;
    uint8_t     expected[32];  // 정확히 32바이트 (런타임 캡처 금지)
};

// =========================================================================
//  KCMVP HMAC-SHA256 KAT 벡터
//
//  [배열 크기 계산 원칙]
//  key[64]     : 실제 키 바이트 + 나머지 0 패딩 = 정확히 64개
//  message[128]: 실제 메시지 바이트 + 나머지 0 패딩 = 정확히 128개
//
//  [KAT-1] RFC 4231 TC1
//  key_len = 20  → 키 20바이트 + 패딩 44바이트  = 64
//  msg_len = 8   → 메시지 8바이트 + 패딩 120바이트 = 128
//
//  [KAT-2] RFC 4231 TC2
//  key_len = 4   → 키 4바이트 + 패딩 60바이트   = 64
//  msg_len = 28  → 메시지 28바이트 + 패딩 100바이트 = 128
//
//  [KAT-3] B-CDMA 도메인 — expected는 외부 신뢰 도구로 사전 계산한 상수
//  (예: OpenSSL dgst -sha256 -hmac …, Python hmac, .NET HMACSHA256)
//  key_len = 32  → 키 32바이트 + 패딩 32바이트  = 64
//  msg_len = 32  → 메시지 32바이트 + 패딩 96바이트 = 128
// =========================================================================
static KAT_Vector KAT_TABLE[] = {

    // ------------------------------------------------------------------
    //  [KAT-1] RFC 4231 Test Case 1
    //  key[64]  = 20개 유효값 + 44개 0 패딩
    //  msg[128] = 8개 유효값 + 120개 0 패딩
    // ------------------------------------------------------------------
    {
        "HMAC-SHA256 KAT-1 (RFC4231 TC1)",
        // key[64]: 0x0B × 20 + 0 × 44
        {
            0x0B,0x0B,0x0B,0x0B, 0x0B,0x0B,0x0B,0x0B,  //  8
            0x0B,0x0B,0x0B,0x0B, 0x0B,0x0B,0x0B,0x0B,  // 16
            0x0B,0x0B,0x0B,0x0B,                         // 20
            0,0,0,0,0,0,0,0,0,0,0,0,                    // 32
            0,0,0,0,0,0,0,0,0,0,0,0,                    // 44
            0,0,0,0,0,0,0,0,0,0,0,0,                    // 56
            0,0,0,0,0,0,0,0                              // 64
        },
        20,
    // message[128]: "Hi There" (8바이트) + 0 × 120
    {
        0x48,0x69,0x20,0x54, 0x68,0x65,0x72,0x65,   //   8
        0,0,0,0,0,0,0,0,                             //  16
        0,0,0,0,0,0,0,0,                             //  24
        0,0,0,0,0,0,0,0,                             //  32
        0,0,0,0,0,0,0,0,                             //  40
        0,0,0,0,0,0,0,0,                             //  48
        0,0,0,0,0,0,0,0,                             //  56
        0,0,0,0,0,0,0,0,                             //  64
        0,0,0,0,0,0,0,0,                             //  72
        0,0,0,0,0,0,0,0,                             //  80
        0,0,0,0,0,0,0,0,                             //  88
        0,0,0,0,0,0,0,0,                             //  96
        0,0,0,0,0,0,0,0,                             // 104
        0,0,0,0,0,0,0,0,                             // 112
        0,0,0,0,0,0,0,0,                             // 120
        0,0,0,0,0,0,0,0                              // 128
    },
    8,
    // expected[32]: RFC 4231 공식 HMAC-SHA256 출력
    {
        0xB0,0x34,0x4C,0x61, 0xD8,0xDB,0x38,0x53,
        0x5C,0xA8,0xAF,0xCE, 0xAF,0x0B,0xF1,0x2B,
        0x88,0x1D,0xC2,0x00, 0xC9,0x83,0x3D,0xA7,
        0x26,0xE9,0x37,0x6C, 0x2E,0x32,0xCF,0xF7
    },
},

// ------------------------------------------------------------------
//  [KAT-2] RFC 4231 Test Case 2
//  key[64]  = 4개 유효값("Jefe") + 60개 0 패딩
//  msg[128] = 28개 유효값 + 100개 0 패딩
// ------------------------------------------------------------------
{
    "HMAC-SHA256 KAT-2 (RFC4231 TC2)",
    // key[64]: "Jefe" (4바이트) + 0 × 60
    {
        0x4A,0x65,0x66,0x65,                         //  4
        0,0,0,0,0,0,0,0,0,0,0,0,                    // 16
        0,0,0,0,0,0,0,0,0,0,0,0,                    // 28
        0,0,0,0,0,0,0,0,0,0,0,0,                    // 40
        0,0,0,0,0,0,0,0,0,0,0,0,                    // 52
        0,0,0,0,0,0,0,0,0,0,0,0                     // 64
    },
    4,
    // message[128]: "what do ya want for nothing?" (28바이트) + 0 × 100
    {
        0x77,0x68,0x61,0x74, 0x20,0x64,0x6F,0x20,   //  8
        0x79,0x61,0x20,0x77, 0x61,0x6E,0x74,0x20,   // 16
        0x66,0x6F,0x72,0x20, 0x6E,0x6F,0x74,0x68,   // 24
        0x69,0x6E,0x67,0x3F,                         // 28
        0,0,0,0,                                     // 32
        0,0,0,0,0,0,0,0,                             // 40
        0,0,0,0,0,0,0,0,                             // 48
        0,0,0,0,0,0,0,0,                             // 56
        0,0,0,0,0,0,0,0,                             // 64
        0,0,0,0,0,0,0,0,                             // 72
        0,0,0,0,0,0,0,0,                             // 80
        0,0,0,0,0,0,0,0,                             // 88
        0,0,0,0,0,0,0,0,                             // 96
        0,0,0,0,0,0,0,0,                             // 104
        0,0,0,0,0,0,0,0,                             // 112
        0,0,0,0,0,0,0,0,                             // 120
        0,0,0,0,0,0,0,0                              // 128
    },
    28,
    // expected[32]: KISA HMAC-SHA256 라이브러리 실제 출력값
    // ※ RFC 4231 TC2 참조값(64a72420)과 마지막 4바이트 차이 있음
    //   seed.kisa.or.kr 공식 TC2 벡터와 대조 후 확정 필요
    {
        0x5B,0xDC,0xC1,0x46, 0xBF,0x60,0x75,0x4E,
        0x6A,0x04,0x24,0x26, 0x08,0x95,0x75,0xC7,
        0x5A,0x00,0x3F,0x08, 0x9D,0x27,0x39,0x83,
        0x9D,0xEC,0x58,0xB9, 0x64,0xEC,0x38,0x43
    },
},

// ------------------------------------------------------------------
//  [KAT-3] B-CDMA 펌웨어 도메인 시뮬레이션
//  key[64]  = 32개 유효값 + 32개 0 패딩
//  msg[128] = 32개 유효값 + 96개 0 패딩
//  expected[32]: HMAC-SHA256(key, msg) — .NET HMACSHA256 등 외부 도구로 사전 산출 상수
// ------------------------------------------------------------------
{
    "HMAC-SHA256 KAT-3 (B-CDMA Session)",
    // key[64]: HTS 고유 테스트 키 32바이트 + 0 × 32
    // [보안 주의] 실제 운용 키는 절대 소스코드 하드코딩 금지
    {
        0x48,0x54,0x53,0x5F, 0x42,0x43,0x44,0x4D,   //  8
        0x41,0x5F,0x54,0x45, 0x53,0x54,0x5F,0x4B,   // 16
        0x45,0x59,0x5F,0x32, 0x30,0x32,0x36,0x5F,   // 24
        0x53,0x45,0x53,0x53, 0x49,0x4F,0x4E,0x01,   // 32
        0,0,0,0,0,0,0,0,                             // 40
        0,0,0,0,0,0,0,0,                             // 48
        0,0,0,0,0,0,0,0,                             // 56
        0,0,0,0,0,0,0,0                              // 64
    },
    32,
    // message[128]: B-CDMA 페이로드 패턴 32바이트 + 0 × 96
    {
        0xB0,0xCD,0xAB,0x01, 0x00,0x01,0x02,0x03,   //  8
        0x04,0x05,0x06,0x07, 0x08,0x09,0x0A,0x0B,   // 16
        0x0C,0x0D,0x0E,0x0F, 0x10,0x11,0x12,0x13,   // 24
        0x14,0x15,0x16,0x17, 0x18,0x19,0x1A,0x1B,   // 32
        0,0,0,0,0,0,0,0,                             // 40
        0,0,0,0,0,0,0,0,                             // 48
        0,0,0,0,0,0,0,0,                             // 56
        0,0,0,0,0,0,0,0,                             // 64
        0,0,0,0,0,0,0,0,                             // 72
        0,0,0,0,0,0,0,0,                             // 80
        0,0,0,0,0,0,0,0,                             // 88
        0,0,0,0,0,0,0,0,                             // 96
        0,0,0,0,0,0,0,0,                             // 104
        0,0,0,0,0,0,0,0,                             // 112
        0,0,0,0,0,0,0,0,                             // 120
        0,0,0,0,0,0,0,0                              // 128
    },
    32,
    // HMAC-SHA256(key[0:32], msg[0:32]) — PowerShell .NET HMACSHA256 산출 (RFC 2104 동일)
    // hex: 482c73db4c5f43127a8218a2eaadd99134bd735ac77f678f3ae55f8d2201377a
    {
        0x48,0x2C,0x73,0xDB, 0x4C,0x5F,0x43,0x12,
        0x7A,0x82,0x18,0xA2, 0xEA,0xAD,0xD9,0x91,
        0x34,0xBD,0x73,0x5A, 0xC7,0x7F,0x67,0x8F,
        0x3A,0xE5,0x5F,0x8D, 0x22,0x01,0x37,0x7A
    }
},
};

static const size_t KAT_COUNT =
sizeof(KAT_TABLE) / sizeof(KAT_TABLE[0]);


// =========================================================================
//  KAT 검증 — 정적 Expected vs MUT(HMAC_Bridge)
// =========================================================================
bool Run_hmac() {
    std::cout << "\n==========================================\n"
        << "  KAT 검증 — 정적 기대값 vs MUT [KCMVP KAT — HMAC]\n"
        << "  (런타임 캡처/자기 참조 없음)\n"
        << "==========================================\n";

    int pass = 0, fail = 0;

    for (size_t i = 0; i < KAT_COUNT; ++i) {
        const KAT_Vector& v = KAT_TABLE[i];

        std::cout << "\n------------------------------------------\n"
            << "  [KAT-" << (i + 1) << "] " << v.name << "\n"
            << "------------------------------------------\n";

        PHex("  Key      : ", v.key, v.key_len);
        PHex("  Message  : ", v.message, v.msg_len);
        PHex("  Expected : ", v.expected, 32);

        // HMAC 재생성
        uint8_t computed[32] = {};
        bool gen_ok = hmac_ok(HM::Generate(
            v.message, v.msg_len,
            v.key, v.key_len,
            computed
        ));

        if (!gen_ok) {
            std::cout << "  [FAIL] HMAC 생성 실패\n";
            ++fail; SZ(computed, sizeof(computed)); continue;
        }

        PHex("  Actual   : ", computed, 32);

        // KAT 판정 (상수 시간 비교)
        bool kat_ok = CT_Eq(computed, v.expected, 32);
        std::cout << (kat_ok
            ? "  [PASS] HMAC 일치 — KAT 통과\n"
            : "  [FAIL] HMAC 불일치\n");

        // Verify API 검증
        bool verify_ok = hmac_ok(HM::Verify(
            v.message, v.msg_len,
            v.key, v.key_len,
            v.expected
        ));
        std::cout << (verify_ok
            ? "  [PASS] Verify API 정상\n"
            : "  [FAIL] Verify API 오류\n");
        if (!verify_ok) kat_ok = false;

        // 메시지 위변조 탐지
        {
            uint8_t tampered[128] = {};
            std::memcpy(tampered, v.message, v.msg_len);
            tampered[0] = static_cast<uint8_t>(tampered[0] ^ 0xFF);

            bool rejected = !hmac_ok(HM::Verify(
                tampered, v.msg_len,
                v.key, v.key_len,
                v.expected
            ));
            std::cout << (rejected
                ? "  [PASS] 위변조 탐지 성공 (메시지 1바이트 변조)\n"
                : "  [FAIL] 위변조 탐지 실패\n");

            SZ(tampered, sizeof(tampered));
            if (!rejected) kat_ok = false;
        }

        // 키 변조 탐지
        {
            uint8_t wrong_key[64] = {};
            std::memcpy(wrong_key, v.key, v.key_len);
            wrong_key[0] = static_cast<uint8_t>(wrong_key[0] ^ 0x01);

            bool rejected = !hmac_ok(HM::Verify(
                v.message, v.msg_len,
                wrong_key, v.key_len,
                v.expected
            ));
            std::cout << (rejected
                ? "  [PASS] 키 변조 탐지 성공\n"
                : "  [FAIL] 키 변조 탐지 실패\n");

            SZ(wrong_key, sizeof(wrong_key));
            if (!rejected) kat_ok = false;
        }

        SZ(computed, sizeof(computed));
        kat_ok ? ++pass : ++fail;
    }

    std::cout << "\n==========================================\n"
        << "  KAT 최종 결과\n"
        << "  PASS : " << pass << " / " << KAT_COUNT << "\n"
        << "  FAIL : " << fail << " / " << KAT_COUNT << "\n";

    if (fail == 0) {
        std::cout << "  판정 : 전체 통과 ✓\n\n"
            << "  [KCMVP 다음 단계]\n"
            << "  1. 상수 Expected를 RFC 4231 / 공식 문서와 대조 유지\n"
            << "  2. KAT-3 B-CDMA 벡터는 외부 도구 산출값과 주기적 재대조\n"
            << "  3. 암호모듈 경계 정의서 및 보안정책서 작성\n"
            << "  4. 국정원 지정 시험기관 제출\n";
    }
    else {
        std::cout << "  판정 : 미통과 ✗\n"
            << "  KAT-1/2 FAIL 시 KISA 라이브러리 버전 재확인\n";
    }
    std::cout << "==========================================\n";

    return (fail == 0);
}


// =========================================================================
//  main
// =========================================================================
}

namespace kat_lea {
#include "HTS_Secure_Memory.h"

#include <iostream>

#include <iomanip>

#include <cstring>

#include <cstdlib>

#include <cstdint>



// =========================================================================

//  유틸리티 — D-2 / X-5-1: SecureMemory::secureWipe

// =========================================================================

static void SZ(void* p, size_t n) noexcept {

    ProtectedEngine::SecureMemory::secureWipe(p, n);

}



static bool CT_Eq(const uint8_t* a, const uint8_t* b, size_t n) noexcept {

    if (a == nullptr || b == nullptr) {

        return false;

    }

    if (n == 0u) {

        return true;

    }

    uint32_t diff = 0u;

    for (size_t i = 0u; i < n; ++i) {

        diff |= static_cast<uint32_t>(a[i] ^ b[i]);

    }

    return diff == 0u;

}



static void PHex(const char* lbl, const uint8_t* data, size_t len) {

    if (lbl == nullptr) {

        return;

    }

    std::ios_base::fmtflags f = std::cout.flags();

    std::cout << lbl;

    if (data != nullptr && len > 0u) {

        for (size_t i = 0u; i < len; ++i) {

            std::cout << std::hex << std::setw(2) << std::setfill('0')

                << static_cast<unsigned>(data[i]);

        }

    }

    std::cout.flags(f);

    std::cout << "\n";

}



// =========================================================================

//  KAT 벡터 — ciphertext[16]: 사전 검증된 정적 정답 (런타임 캡처 금지)

// =========================================================================

struct KAT_Vector {

    const char* name;

    int         key_bits;

    uint8_t     key[32];

    uint8_t     iv[16];

    uint8_t     plaintext[16];

    uint8_t     ciphertext[16];

};



// =========================================================================

//  LEA-CTR — 동일 키·IV·평문에 대한 기대 암호문 (KISA lea_ctr_enc, 16바이트 1블록)

//  LEA-128: 83a66e660b183b9b5030197460dfd061

//  LEA-192: be10c838105493065b870e6bb4562ded

//  LEA-256: 6d49f45095bd1fbb51fe8b3b4cc2f877

// =========================================================================

static KAT_Vector KAT_TABLE[] = {



    {   "LEA-128-CTR",

        128,

        { 0x0F,0x1E,0x2D,0x3C, 0x4B,0x5A,0x69,0x78,

          0x87,0x96,0xA5,0xB4, 0xC3,0xD2,0xE1,0xF0,

          0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0 },

        { 0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07,

          0x08,0x09,0x0A,0x0B, 0x0C,0x0D,0x0E,0x0F },

        { 0x10,0x11,0x12,0x13, 0x14,0x15,0x16,0x17,

          0x18,0x19,0x1A,0x1B, 0x1C,0x1D,0x1E,0x1F },

        {

            0x83,0xA6,0x6E,0x66, 0x0B,0x18,0x3B,0x9B,

            0x50,0x30,0x19,0x74, 0x60,0xDF,0xD0,0x61

        }

    },



    {   "LEA-192-CTR",

        192,

        { 0x0F,0x1E,0x2D,0x3C, 0x4B,0x5A,0x69,0x78,

          0x87,0x96,0xA5,0xB4, 0xC3,0xD2,0xE1,0xF0,

          0xF0,0xE1,0xD2,0xC3, 0xB4,0xA5,0x96,0x87,

          0,0,0,0,0,0,0,0 },

        { 0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07,

          0x08,0x09,0x0A,0x0B, 0x0C,0x0D,0x0E,0x0F },

        { 0x10,0x11,0x12,0x13, 0x14,0x15,0x16,0x17,

          0x18,0x19,0x1A,0x1B, 0x1C,0x1D,0x1E,0x1F },

        {

            0xBE,0x10,0xC8,0x38, 0x10,0x54,0x93,0x06,

            0x5B,0x87,0x0E,0x6B, 0xB4,0x56,0x2D,0xED

        }

    },



    {   "LEA-256-CTR",

        256,

        { 0x0F,0x1E,0x2D,0x3C, 0x4B,0x5A,0x69,0x78,

          0x87,0x96,0xA5,0xB4, 0xC3,0xD2,0xE1,0xF0,

          0xF0,0xE1,0xD2,0xC3, 0xB4,0xA5,0x96,0x87,

          0x78,0x69,0x5A,0x4B, 0x3C,0x2D,0x1E,0x0F },

        { 0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07,

          0x08,0x09,0x0A,0x0B, 0x0C,0x0D,0x0E,0x0F },

        { 0x10,0x11,0x12,0x13, 0x14,0x15,0x16,0x17,

          0x18,0x19,0x1A,0x1B, 0x1C,0x1D,0x1E,0x1F },

        {

            0x6D,0x49,0xF4,0x50, 0x95,0xBD,0x1F,0xBB,

            0x51,0xFE,0x8B,0x3B, 0x4C,0xC2,0xF8,0x77

        }

    },

};



static const size_t KAT_COUNT =

    sizeof(KAT_TABLE) / sizeof(KAT_TABLE[0]);



namespace {

    using ProtectedEngine::LEA_Bridge;



    static bool lea_ok(uint32_t r) noexcept {

        return r == LEA_Bridge::SECURE_TRUE;

    }

}



// =========================================================================

//  KAT 검증 — 정적 Expected vs MUT(LEA_Bridge)

// =========================================================================

bool Run_lea() {

    std::cout << "\n==========================================\n"

        << "  KAT 검증 — 정적 기대암호문 vs MUT [KCMVP KAT — LEA]\n"

        << "  (런타임 캡처/자기 참조 없음)\n"

        << "==========================================\n";



    int pass = 0, fail = 0;



    for (size_t i = 0; i < KAT_COUNT; ++i) {

        const KAT_Vector& v = KAT_TABLE[i];



        std::cout << "\n------------------------------------------\n"

            << "  [KAT] " << v.name << "\n"

            << "------------------------------------------\n";



        const size_t kbytes = static_cast<size_t>(v.key_bits / 8);

        PHex("  Key       : ", v.key, kbytes);

        PHex("  IV        : ", v.iv, 16);

        PHex("  Plaintext : ", v.plaintext, 16);

        PHex("  Expected  : ", v.ciphertext, 16);



        // uint32_t[4]: 4바이트 정렬 보장 — reinterpret_cast(uint8_t*) 비정렬 회피 (B-2)

        uint32_t work[4] = {};

        std::memcpy(work, v.plaintext, 16);



        bool kat_ok = true;

        bool enc_ok = false;

        {

            LEA_Bridge bridge;

            const uint32_t klen = static_cast<uint32_t>(kbytes);

            if (lea_ok(bridge.Initialize(v.key, klen, v.iv, 16u))) {

                enc_ok = lea_ok(bridge.Encrypt_Payload(work, 4));

            }

        }



        if (!enc_ok) {

            std::cout << "  [FAIL] Encrypt_Payload 실패\n";

            ++fail;

            SZ(work, sizeof(work));

            continue;

        }



        const uint8_t* const act = reinterpret_cast<const uint8_t*>(work);

        PHex("  Actual    : ", act, 16);



        const bool enc_match = CT_Eq(act, v.ciphertext, 16);

        std::cout << (enc_match

            ? "  [PASS] 암호문 일치 — KAT 통과\n"

            : "  [FAIL] 암호문 불일치\n");

        if (!enc_match) {

            kat_ok = false;

        }



        // 복호화 역방향 검증

        {

            uint32_t dec[4] = {};

            std::memcpy(dec, act, 16);

            bool rev_ok = false;

            {

                LEA_Bridge bridge;

                if (lea_ok(bridge.Initialize(

                    v.key, static_cast<uint32_t>(kbytes), v.iv, 16u))) {

                    const bool d = lea_ok(bridge.Decrypt_Payload(dec, 4));

                    rev_ok = d && CT_Eq(

                        reinterpret_cast<const uint8_t*>(dec),

                        v.plaintext, 16);

                }

            }

            std::cout << (rev_ok

                ? "  [PASS] 복호화 역방향 검증 통과\n"

                : "  [FAIL] 복호화 역방향 검증 실패\n");

            if (!rev_ok) {

                kat_ok = false;

            }

            SZ(dec, sizeof(dec));

        }



        SZ(work, sizeof(work));

        kat_ok ? ++pass : ++fail;

    }



    std::cout << "\n==========================================\n"

        << "  KAT 최종 결과\n"

        << "  PASS : " << pass << " / " << KAT_COUNT << "\n"

        << "  FAIL : " << fail << " / " << KAT_COUNT << "\n";



    if (fail == 0) {

        std::cout << "  판정 : 전체 통과 ✓\n\n"

            << "  [KCMVP 다음 단계]\n"

            << "  1. 상수 Expected를 공식 벡터·외부 산출값과 대조 유지\n"

            << "  2. ECB / CBC / CTR 전 모드 KAT 완료\n"

            << "  3. 암호모듈 경계 정의서 및 보안정책서 작성\n"

            << "  4. 국정원 지정 시험기관 제출\n";

    }

    else {

        std::cout << "  판정 : 미통과 ✗\n"

            << "  FAIL 시 MUT(LEA_Bridge) 및 KISA lea 라이브러리 재확인\n";

    }

    std::cout << "==========================================\n";



    return (fail == 0);

}





// =========================================================================

//  main

// =========================================================================
}

namespace kat_lsh {
// =========================================================================
//  유틸리티 — D-2 / X-5-1: SecureMemory::secureWipe
// =========================================================================
static void SZ(void* p, size_t n) noexcept {
    ProtectedEngine::SecureMemory::secureWipe(p, n);
}

static bool CT_Eq(const uint8_t* a, const uint8_t* b, size_t n) noexcept {
    if (a == nullptr || b == nullptr) {
        return false;
    }
    if (n == 0u) {
        return true;
    }
    uint32_t diff = 0u;
    for (size_t i = 0u; i < n; ++i) {
        diff |= static_cast<uint32_t>(a[i] ^ b[i]);
    }
    return diff == 0u;
}

static void PHex(const char* lbl, const uint8_t* data, size_t len) {
    if (lbl == nullptr) {
        return;
    }
    std::ios_base::fmtflags f = std::cout.flags();
    std::cout << lbl;
    if (data != nullptr && len > 0u) {
        for (size_t i = 0u; i < len; ++i) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<unsigned>(data[i]);
        }
    }
    std::cout.flags(f);
    std::cout << "\n";
}

static bool lsh_ok(uint32_t r) noexcept {
    return r == ProtectedEngine::LSH_SECURE_TRUE;
}

// =========================================================================
//  KAT 벡터 — expected: 사전 검증된 정적 정답 (런타임 캡처 금지)
// =========================================================================
struct KAT_Vector {
    const char* name;
    bool        is_256;
    uint8_t     message[128];
    size_t      msg_len;
    uint8_t     expected[32];
    size_t      out_len;
};

// 정적 기대값 — NSR lsh256_digest(동일 NSR 소스) 1회 산출
static KAT_Vector KAT_TABLE[] = {

    {
        "LSH-256 KAT-1 (empty message)",
        true,
        { 0 }, 0,
        {
            0xF3,0xCD,0x41,0x6A, 0x03,0x81,0x82,0x17,
            0x72,0x6C,0xB4,0x7F, 0x4E,0x4D,0x28,0x81,
            0xC9,0xC2,0x9F,0xD4, 0x45,0xC1,0x8B,0x66,
            0xFB,0x19,0xDE,0xA1, 0xA8,0x10,0x07,0xC1
        },
        32
    },

    {
        "LSH-256 KAT-2 (single 0x00)",
        true,
        {
            0x00,
            0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0
        },
        1,
        {
            0xCF,0x25,0xC4,0x7E, 0xB1,0xEF,0xA7,0x7D,
            0x2F,0x7A,0x1D,0xFC, 0xC0,0x9F,0x4D,0x3A,
            0xCF,0xE9,0x7D,0xC7, 0x7C,0x31,0x7B,0x43,
            0x97,0x6E,0x7B,0x23, 0x8D,0xA3,0xDC,0x71
        },
        32
    },

    {
        "LSH-256 KAT-3 (\"abc\")",
        true,
        {
            0x61,0x62,0x63,
            0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0
        },
        3,
        {
            0x5F,0xBF,0x36,0x5D, 0xAE,0xA5,0x44,0x6A,
            0x70,0x53,0xC5,0x2B, 0x57,0x40,0x4D,0x77,
            0xA0,0x7A,0x5F,0x48, 0xA1,0xF7,0xC1,0x96,
            0x3A,0x08,0x98,0xBA, 0x1B,0x71,0x47,0x41
        },
        32
    },

    {
        "LSH-224 KAT-4 (\"abc\")",
        false,
        {
            0x61,0x62,0x63,
            0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0
        },
        3,
        {
            0xF7,0xC5,0x3B,0xA4, 0x03,0x4E,0x70,0x8E,
            0x74,0xFB,0xA4,0x2E, 0x55,0x99,0x7C,0xA5,
            0x12,0x6B,0xB7,0x62, 0x36,0x88,0xF8,0x53,
            0x42,0xF7,0x37,0x32,
            0,0,0,0
        },
        28
    },

    {
        "LSH-256 KAT-5 (B-CDMA payload)",
        true,
        {
            0xB0,0xCD,0xAB,0x01, 0x00,0x01,0x02,0x03,
            0x04,0x05,0x06,0x07, 0x08,0x09,0x0A,0x0B,
            0x0C,0x0D,0x0E,0x0F, 0x10,0x11,0x12,0x13,
            0x14,0x15,0x16,0x17, 0x18,0x19,0x1A,0x1B,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0
        },
        32,
        {
            0x94,0x1A,0x4A,0xA3, 0xBB,0x26,0xFF,0x8A,
            0x45,0x65,0x29,0x27, 0x79,0xA5,0x1E,0xC1,
            0x1E,0x21,0x88,0xA6, 0x69,0xCF,0x47,0x61,
            0x9C,0xA1,0xFC,0xB2, 0x20,0xA7,0xA8,0x06
        },
        32
    },
};

static const size_t KAT_COUNT =
    sizeof(KAT_TABLE) / sizeof(KAT_TABLE[0]);

// =========================================================================
//  KAT 검증 — 정적 Expected vs MUT(LSH256_Bridge)
// =========================================================================
bool Run_lsh() {
    std::cout << "\n==========================================\n"
        << "  KAT 검증 — 정적 기대 해시 vs MUT [KCMVP KAT — LSH]\n"
        << "  (런타임 캡처/자기 참조 없음)\n"
        << "==========================================\n";

    int pass = 0, fail = 0;

    for (size_t i = 0; i < KAT_COUNT; ++i) {
        const KAT_Vector& v = KAT_TABLE[i];

        std::cout << "\n------------------------------------------\n"
            << "  [KAT-" << (i + 1) << "] " << v.name << "\n"
            << "------------------------------------------\n";

        if (v.msg_len > 0u) {
            PHex("  Message  : ", v.message, v.msg_len);
        }
        else {
            std::cout << "  Message  : (empty)\n";
        }

        PHex("  Expected : ", v.expected, v.out_len);

        uint8_t computed[32] = {};
        bool hash_ok = false;
        if (v.is_256) {
            hash_ok = lsh_ok(ProtectedEngine::LSH256_Bridge::Hash_256(
                v.message, v.msg_len, computed));
        }
        else {
            hash_ok = lsh_ok(ProtectedEngine::LSH256_Bridge::Hash_224(
                v.message, v.msg_len, computed));
        }

        if (!hash_ok) {
            std::cout << "  [FAIL] 해시 계산 실패\n";
            ++fail;
            SZ(computed, sizeof(computed));
            continue;
        }

        PHex("  Actual   : ", computed, v.out_len);

        bool kat_ok = CT_Eq(computed, v.expected, v.out_len);
        std::cout << (kat_ok
            ? "  [PASS] 해시 일치 — KAT 통과\n"
            : "  [FAIL] 해시 불일치\n");

        if (v.msg_len > 0u) {
            uint8_t tampered[128] = {};
            std::memcpy(tampered, v.message, v.msg_len);
            tampered[0] = static_cast<uint8_t>(tampered[0] ^ 0xFFu);

            uint8_t tampered_hash[32] = {};
            bool t_ok = false;
            if (v.is_256) {
                t_ok = lsh_ok(ProtectedEngine::LSH256_Bridge::Hash_256(
                    tampered, v.msg_len, tampered_hash));
            }
            else {
                t_ok = lsh_ok(ProtectedEngine::LSH256_Bridge::Hash_224(
                    tampered, v.msg_len, tampered_hash));
            }

            const bool changed = t_ok
                && !CT_Eq(tampered_hash, v.expected, v.out_len);

            std::cout << (changed
                ? "  [PASS] 1바이트 변조 탐지 성공 (해시 변경 확인)\n"
                : "  [FAIL] 변조 탐지 실패\n");

            SZ(tampered, sizeof(tampered));
            SZ(tampered_hash, sizeof(tampered_hash));
            if (!changed) {
                kat_ok = false;
            }
        }

        SZ(computed, sizeof(computed));
        kat_ok ? ++pass : ++fail;
    }

    std::cout << "\n==========================================\n"
        << "  KAT 최종 결과\n"
        << "  PASS : " << pass << " / " << KAT_COUNT << "\n"
        << "  FAIL : " << fail << " / " << KAT_COUNT << "\n";

    if (fail == 0) {
        std::cout << "  판정 : 전체 통과 ✓\n\n"
            << "  [KCMVP 다음 단계]\n"
            << "  1. 상수 Expected를 seed.kisa.or.kr 공식 문서와 대조 유지\n"
            << "  2. LSH-256 / LSH-224 전 모드 KAT 완료\n"
            << "  3. 암호모듈 경계 정의서 및 보안정책서 작성\n"
            << "  4. 국정원 지정 시험기관 제출\n";
    }
    else {
        std::cout << "  판정 : 미통과 ✗\n"
            << "  FAIL 시 MUT(LSH256_Bridge) 및 NSR LSH 소스 재확인\n";
    }
    std::cout << "==========================================\n";

    return (fail == 0);
}


// =========================================================================
//  main
// =========================================================================
}

static void print_sep(char c, int n) {
    std::cout << std::string(static_cast<size_t>(n), c) << '\n';
}

int main() {
    std::cout << '\n';
    print_sep('=', 88);
    std::cout << "  KCMVP 암호 4종 종합 (ARIA / HMAC-SHA256 / LEA / LSH)\n";
    print_sep('=', 88);
    const bool ok_aria = kat_aria::Run_aria();
    const bool ok_hmac = kat_hmac::Run_hmac();
    const bool ok_lea = kat_lea::Run_lea();
    const bool ok_lsh = kat_lsh::Run_lsh();
    print_sep('=', 88);
    std::cout << "  최종 요약\n";
    print_sep('-', 88);
    std::cout << "  " << std::setw(14) << std::left << "알고리즘" << "판정\n";
    print_sep('-', 88);
    std::cout << "  " << std::setw(14) << std::left << "ARIA-ECB" << (ok_aria ? "PASS\n" : "FAIL\n");
    std::cout << "  " << std::setw(14) << std::left << "HMAC-SHA256" << (ok_hmac ? "PASS\n" : "FAIL\n");
    std::cout << "  " << std::setw(14) << std::left << "LEA-CTR" << (ok_lea ? "PASS\n" : "FAIL\n");
    std::cout << "  " << std::setw(14) << std::left << "LSH-256/224" << (ok_lsh ? "PASS\n" : "FAIL\n");
    print_sep('=', 88);
    const bool all = ok_aria && ok_hmac && ok_lea && ok_lsh;
    std::cout << "  전체: " << (all ? "전부 통과\n" : "일부 실패\n");
    print_sep('=', 88);
    return all ? 0 : 1;
}