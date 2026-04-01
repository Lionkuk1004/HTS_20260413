/// @file  HTS_IoT_Codec.cpp
/// @brief HTS IoT Codec -- Implementation
/// @note  ARM only. Pure ASCII. No PC/server code.
/// @author Lim Young-jun
/// @copyright INNOViD 2026. All rights reserved.

#include "HTS_IoT_Codec.h"

namespace ProtectedEngine {
    namespace {
        struct IoT_Codec_Busy_Guard final {
            std::atomic_flag* flag;
            uint32_t locked;
            explicit IoT_Codec_Busy_Guard(std::atomic_flag& f) noexcept
                : flag(&f), locked(HTS_IoT_Codec::SECURE_FALSE) {
                locked = (!flag->test_and_set(std::memory_order_acq_rel))
                    ? HTS_IoT_Codec::SECURE_TRUE
                    : HTS_IoT_Codec::SECURE_FALSE;
            }
            ~IoT_Codec_Busy_Guard() noexcept {
                if (locked == HTS_IoT_Codec::SECURE_TRUE) {
                    flag->clear(std::memory_order_release);
                }
            }
            IoT_Codec_Busy_Guard(const IoT_Codec_Busy_Guard&) = delete;
            IoT_Codec_Busy_Guard& operator=(const IoT_Codec_Busy_Guard&) = delete;
        };
    }

    // ============================================================
    //  Constructor
    // ============================================================

    HTS_IoT_Codec::HTS_IoT_Codec() noexcept
        : build_pos_(0u)
        , tlv_count_(0u)
        , frame_active_(false)
    {
        for (uint32_t i = 0u; i < IOT_MAX_FRAME_SIZE; ++i) {
            build_buf_[i] = 0u;
        }
    }

    // ============================================================
    //  Begin_Frame
    // ============================================================

    void HTS_IoT_Codec::Begin_Frame(IoT_MsgType type, uint32_t device_id,
        uint32_t timestamp) noexcept
    {
        IoT_Codec_Busy_Guard guard(op_busy_);
        if (guard.locked != HTS_IoT_Codec::SECURE_TRUE) { return; }

        // Reset build state
        build_pos_ = 0u;
        tlv_count_ = 0u;
        frame_active_ = true;

        // Reserve header space: MSG_TYPE(1) + DEVICE_ID(4) + TIMESTAMP(4) + TLV_COUNT(1)
        if (IOT_FRAME_HEADER_SIZE > IOT_MAX_FRAME_SIZE) {
            frame_active_ = false;
            return;
        }

        build_buf_[0] = static_cast<uint8_t>(type);
        Write_U32(&build_buf_[1], device_id);
        Write_U32(&build_buf_[5], timestamp);
        build_buf_[9] = 0u;  // TLV count placeholder (patched in Finalize)

        build_pos_ = static_cast<uint16_t>(IOT_FRAME_HEADER_SIZE);
    }

    // ============================================================
    //  Add TLV Items
    // ============================================================

    uint32_t HTS_IoT_Codec::Add_U8(SensorType sensor, uint8_t value) noexcept
    {
        return Add_Raw(sensor, &value, 1u);
    }

    uint32_t HTS_IoT_Codec::Add_U16(SensorType sensor, uint16_t value) noexcept
    {
        uint8_t buf[2];
        Write_U16(buf, value);
        return Add_Raw(sensor, buf, 2u);
    }

    uint32_t HTS_IoT_Codec::Add_U32(SensorType sensor, uint32_t value) noexcept
    {
        uint8_t buf[4];
        Write_U32(buf, value);
        return Add_Raw(sensor, buf, 4u);
    }

    uint32_t HTS_IoT_Codec::Add_Raw(SensorType sensor, const uint8_t* data,
        uint8_t data_len) noexcept
    {
        IoT_Codec_Busy_Guard guard(op_busy_);
        if (guard.locked != HTS_IoT_Codec::SECURE_TRUE) { return SECURE_FALSE; }

        if (!frame_active_) { return SECURE_FALSE; }
        if (data == nullptr && data_len > 0u) { return SECURE_FALSE; }
        if (data_len > IOT_TLV_MAX_VALUE_SIZE) { return SECURE_FALSE; }
        if (tlv_count_ >= IOT_MAX_TLV_COUNT) { return SECURE_FALSE; }

        // Check remaining space: TLV header + value + CRC trailer must fit
        const uint32_t needed = IOT_TLV_HEADER_SIZE + static_cast<uint32_t>(data_len);
        const uint32_t remaining = IOT_MAX_FRAME_SIZE - static_cast<uint32_t>(build_pos_);
        if (needed + IOT_FRAME_CRC_SIZE > remaining) { return SECURE_FALSE; }

        // Write TLV: [SENSOR_TYPE(1)][LENGTH(1)][VALUE(data_len)]
        build_buf_[build_pos_] = static_cast<uint8_t>(sensor);
        build_buf_[build_pos_ + 1u] = data_len;
        for (uint8_t i = 0u; i < data_len; ++i) {
            build_buf_[build_pos_ + IOT_TLV_HEADER_SIZE + i] = data[i];
        }

        build_pos_ = static_cast<uint16_t>(
            static_cast<uint32_t>(build_pos_) + needed);
        tlv_count_++;
        return SECURE_TRUE;
    }

    // ============================================================
    //  Finalize
    // ============================================================

    uint32_t HTS_IoT_Codec::Finalize(uint8_t* out_buf, uint16_t out_buf_size,
        uint16_t& out_len) noexcept
    {
        IoT_Codec_Busy_Guard guard(op_busy_);
        if (guard.locked != HTS_IoT_Codec::SECURE_TRUE) { out_len = 0u; return SECURE_FALSE; }

        out_len = 0u;
        if (!frame_active_) { return SECURE_FALSE; }
        if (out_buf == nullptr) { return SECURE_FALSE; }

        // Patch TLV count into header
        build_buf_[9] = tlv_count_;

        // Append CRC-16 over entire frame (header + TLV chain)
        const uint32_t data_region = static_cast<uint32_t>(build_pos_);
        if (data_region + IOT_FRAME_CRC_SIZE > IOT_MAX_FRAME_SIZE) {
            frame_active_ = false;
            return SECURE_FALSE;
        }

        const uint16_t crc = IPC_Compute_CRC16(build_buf_, data_region);
        Write_U16(&build_buf_[data_region], crc);

        const uint16_t total = static_cast<uint16_t>(data_region + IOT_FRAME_CRC_SIZE);
        if (total > out_buf_size) { return SECURE_FALSE; }

        // Copy to output
        for (uint16_t i = 0u; i < total; ++i) {
            out_buf[i] = build_buf_[i];
        }
        out_len = total;

        // Reset for next frame
        frame_active_ = false;
        return SECURE_TRUE;
    }

    // ============================================================
    //  Parse
    // ============================================================

    uint32_t HTS_IoT_Codec::Parse(const uint8_t* wire_buf, uint16_t wire_len,
        IoT_Frame_Header& out_header,
        IoT_TLV_Item* out_items, uint8_t max_items,
        uint8_t& out_item_count) const noexcept
    {
        out_item_count = 0u;

        if (wire_buf == nullptr) { return SECURE_FALSE; }
        if (wire_len < IOT_FRAME_HEADER_SIZE + IOT_FRAME_CRC_SIZE) { return SECURE_FALSE; }

        // CRC validation (over all bytes except trailing CRC)
        const uint32_t data_region = static_cast<uint32_t>(wire_len) - IOT_FRAME_CRC_SIZE;
        const uint16_t computed_crc = IPC_Compute_CRC16(wire_buf, data_region);
        const uint16_t received_crc = Read_U16(&wire_buf[data_region]);
        if (computed_crc != received_crc) { return SECURE_FALSE; }

        // Parse header
        out_header.msg_type = static_cast<IoT_MsgType>(wire_buf[0]);
        out_header.reserved = 0u;
        out_header.device_id = Read_U32(&wire_buf[1]);
        out_header.timestamp_sec = Read_U32(&wire_buf[5]);
        out_header.tlv_count = wire_buf[9];
        out_header.pad_ = 0u;

        // Validate TLV count bounds
        if (out_header.tlv_count > IOT_MAX_TLV_COUNT) { return SECURE_FALSE; }
        if (out_header.tlv_count > max_items) { return SECURE_FALSE; }

        // Parse TLV chain
        uint32_t offset = IOT_FRAME_HEADER_SIZE;
        uint8_t  parsed = 0u;

        while ((parsed < out_header.tlv_count) &&
            (offset <= data_region) &&
            ((data_region - offset) >= IOT_TLV_HEADER_SIZE))
        {
            const SensorType stype = static_cast<SensorType>(wire_buf[offset]);
            const uint8_t    vlen = wire_buf[offset + 1u];

            // Validate value length
            if (vlen > IOT_TLV_MAX_VALUE_SIZE) { return SECURE_FALSE; }
            // Overflow-safe: subtract from known upper bound, not add to offset
            const uint32_t needed = IOT_TLV_HEADER_SIZE + static_cast<uint32_t>(vlen);
            if ((data_region - offset) < needed) {
                return SECURE_FALSE;
            }

            // Cross-check with type registry (optional: strict mode)
            const uint8_t expected_size = IoT_Sensor_Value_Size(stype);
            if (expected_size != 0u && vlen != expected_size) {
                return SECURE_FALSE;  // Type/length mismatch -- corrupted or malicious
            }

            // Store if caller has space
            if (parsed < max_items && out_items != nullptr) {
                out_items[parsed].sensor_type = stype;
                out_items[parsed].value_len = vlen;
                for (uint8_t j = 0u; j < vlen; ++j) {
                    out_items[parsed].value[j] = wire_buf[offset + IOT_TLV_HEADER_SIZE + j];
                }
                // Zero remaining value bytes
                for (uint8_t j = vlen; j < IOT_TLV_MAX_VALUE_SIZE; ++j) {
                    out_items[parsed].value[j] = 0u;
                }
                out_items[parsed].padding[0] = 0u;
                out_items[parsed].padding[1] = 0u;
            }

            offset += needed;
            parsed++;
        }

        // Verify we consumed expected number of TLVs
        if (parsed != out_header.tlv_count) { return SECURE_FALSE; }

        out_item_count = parsed;
        return SECURE_TRUE;
    }

    // ============================================================
    //  Accessors
    // ============================================================

    uint8_t HTS_IoT_Codec::Get_TLV_Count() const noexcept
    {
        IoT_Codec_Busy_Guard guard(op_busy_);
        if (guard.locked != HTS_IoT_Codec::SECURE_TRUE) { return 0u; }
        return tlv_count_;
    }

    uint16_t HTS_IoT_Codec::Get_Used_Bytes() const noexcept
    {
        IoT_Codec_Busy_Guard guard(op_busy_);
        if (guard.locked != HTS_IoT_Codec::SECURE_TRUE) { return 0u; }
        return build_pos_;
    }

} // namespace ProtectedEngine