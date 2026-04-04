/* HTS_LSH256_Bridge.cpp GCC-ARM 대체 TU — nullptr→함수포인터 static_cast 만 회피 (코어 원본 비수정) */
#include "HTS_LSH256_Bridge.h"
#include "HTS_Secure_Memory.h"
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <limits>

extern "C" {
#include "lsh256.h"
}

namespace ProtectedEngine {

    static_assert(
        LSH256_DIGEST_BYTES == static_cast<size_t>(LSH256_HASH_VAL_MAX_BYTE_LEN),
        "LSH256 digest length must match NSR LSH256_HASH_VAL_MAX_BYTE_LEN");
    static_assert(LSH224_DIGEST_BYTES == 28u,
        "LSH-224 output is 224 bits (28 bytes)");

    static void Secure_Zero_LSH(void* ptr, size_t size) noexcept
    {
        SecureMemory::secureWipe(ptr, size);
    }

    static uint32_t Do_Hash(
        lsh_type       algtype,
        const uint8_t* data,
        size_t         data_len,
        uint8_t* output,
        size_t         output_len,
        void (*feed_callback)(void)) noexcept
    {

        if (output == nullptr) {
            return LSH_SECURE_FALSE;
        }
        if (data_len > 0u && data == nullptr) {
            Secure_Zero_LSH(output, output_len);
            return LSH_SECURE_FALSE;
        }

        constexpr size_t MAX_BYTE_LEN =
            std::numeric_limits<size_t>::max() / 8u;
        if (data_len > MAX_BYTE_LEN) {
            Secure_Zero_LSH(output, output_len);
            return LSH_SECURE_FALSE;
        }

        LSH256_Context ctx;
        Secure_Zero_LSH(&ctx, sizeof(ctx));

        lsh_err err = lsh256_init(&ctx, algtype);
        if (err != LSH_SUCCESS) {
            Secure_Zero_LSH(&ctx, sizeof(ctx));
            Secure_Zero_LSH(output, output_len);
            return LSH_SECURE_FALSE;
        }

        if (data_len > 0u) {
            if (feed_callback == nullptr) {
                const size_t databitlen =
                    data_len * static_cast<size_t>(8u);
                err = lsh256_update(&ctx,
                    reinterpret_cast<const lsh_u8*>(data),
                    databitlen);
            } else {
                static constexpr size_t kUpdateChunkBytes = 64u * 1024u;
                size_t off = 0u;
                while (off < data_len && err == LSH_SUCCESS) {
                    size_t n = data_len - off;
                    if (n > kUpdateChunkBytes) {
                        n = kUpdateChunkBytes;
                    }
                    const size_t databitlen =
                        n * static_cast<size_t>(8u);
                    err = lsh256_update(&ctx,
                        reinterpret_cast<const lsh_u8*>(data + off),
                        databitlen);
                    off += n;
                    if (err == LSH_SUCCESS) {
                        feed_callback();
                    }
                }
            }
            if (err != LSH_SUCCESS) {
                Secure_Zero_LSH(&ctx, sizeof(ctx));
                Secure_Zero_LSH(output, output_len);
                return LSH_SECURE_FALSE;
            }
        }

        err = lsh256_final(&ctx, reinterpret_cast<lsh_u8*>(output));

        Secure_Zero_LSH(&ctx, sizeof(ctx));

        if (err != LSH_SUCCESS) {
            Secure_Zero_LSH(output, output_len);
            return LSH_SECURE_FALSE;
        }

        return LSH_SECURE_TRUE;
    }

    uint32_t LSH256_Bridge::Hash_256(
        const uint8_t* data,
        size_t         data_len,
        uint8_t* output_32) noexcept
    {

        if (output_32 == nullptr) {
            return LSH_SECURE_FALSE;
        }

        /* lsh_def.h 가 C 호환으로 #define nullptr ((void*)0) — 함수포인터 인자에 쓰면 변환 오류 */
        return Do_Hash(
            LSH_TYPE_256_256,
            data, data_len,
            output_32, LSH256_DIGEST_BYTES,
            reinterpret_cast<void (*)(void)>(0));
    }

    uint32_t LSH256_Bridge::Hash_256_WithPeriodicCallback(
        const uint8_t* data,
        size_t         data_len,
        uint8_t*       output_32,
        void (*callback)(void)) noexcept
    {

        if (output_32 == nullptr) {
            return LSH_SECURE_FALSE;
        }
        if (callback == nullptr) {
            return Hash_256(data, data_len, output_32);
        }
        return Do_Hash(
            LSH_TYPE_256_256,
            data, data_len,
            output_32, LSH256_DIGEST_BYTES,
            callback);
    }

    uint32_t LSH256_Bridge::Hash_224(
        const uint8_t* data,
        size_t         data_len,
        uint8_t* output_28) noexcept
    {

        if (output_28 == nullptr) {
            return LSH_SECURE_FALSE;
        }

        return Do_Hash(
            LSH_TYPE_256_224,
            data, data_len,
            output_28, LSH224_DIGEST_BYTES,
            reinterpret_cast<void (*)(void)>(0));
    }

} // namespace ProtectedEngine
