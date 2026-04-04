// HTS/3DPOWNS — 베어메탈 부트스트랩 (코어 HTS_LIM 소스 비변경)
#include "HTS_HAL.h"

#include "HTS_Hardware_Init.h"
#include "HTS_Priority_Scheduler.h"
#include "HTS_Security_Session.h"

#include <cstdint>

namespace {

using ProtectedEngine::CipherAlgorithm;
using ProtectedEngine::EnqueueResult;
using ProtectedEngine::Hardware_Init_Manager;
using ProtectedEngine::HTS_Priority_Scheduler;
using ProtectedEngine::HTS_Security_Session;
using ProtectedEngine::MacAlgorithm;
using ProtectedEngine::PacketPriority;

} // namespace

int main()
{
    Hardware_Init_Manager::Initialize_System();
    HTS_HAL_Cycles_Init();

    HTS_Priority_Scheduler sched;
    alignas(8) uint8_t sos[HTS_Priority_Scheduler::MAX_PACKET_DATA] = {
        'P', '0', '_', 'S', 'O', 'S', 0, 0};
    const EnqueueResult er = sched.Enqueue(
        PacketPriority::SOS,
        sos,
        7u,
        0u);
    (void)er;

    alignas(8) uint8_t enc_key[32] = {};
    alignas(8) uint8_t mac_key[32] = {};
    alignas(8) uint8_t iv[16] = {};
    HTS_HAL_TRNG_Fill(enc_key, sizeof(enc_key));
    HTS_HAL_TRNG_Fill(mac_key, sizeof(mac_key));
    HTS_HAL_TRNG_Fill(iv, sizeof(iv));

    HTS_Security_Session session;
    const bool ok_init = session.Initialize(
        CipherAlgorithm::LEA_256_CTR,
        MacAlgorithm::HMAC_SHA256,
        enc_key,
        mac_key,
        iv);
    (void)ok_init;

    alignas(8) uint8_t pt[16] = {
        'D', 'U', 'M', 'M', 'Y', '_', 'P', '0',
        '_', 'P', 'A', 'Y', 'L', 'O', 'A', 'D'};
    alignas(8) uint8_t ct[sizeof(pt)] = {};
    alignas(8) uint8_t mac_tag[32] = {};
    const bool ok_prot = session.Protect_Payload(pt, sizeof(pt), ct, mac_tag);
    (void)ok_prot;

    while (true) {
        Hardware_Init_Manager::Kick_Watchdog();
    }
}
