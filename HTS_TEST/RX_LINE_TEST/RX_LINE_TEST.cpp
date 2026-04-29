// RX_LINE_TEST.cpp Phase 1 CFO sweep — see HTS_RX_LINE_TEST_v2.md
#include "HTS_RX_Random.hpp"
#include "HTS_RX_Channel.hpp"
#include "HTS_RX_Scenarios.hpp"
#include "HTS_RX_Metrics.hpp"
#include <cstdio>
#include <cstring>

int main(int argc, char** argv) {
    uint32_t base_seed = 0xC0FFEEu;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            base_seed = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 0));
        }
    }
    std::printf("=== HTS RX_LINE_TEST Phase 1 (CFO sweep) ===\n");
    std::printf("base_seed=0x%08X\n", static_cast<unsigned>(base_seed));

    std::printf("\n--- AMI 200 kcps ---\n");
    hts::rx_test::run_scenario(hts::rx_test::kScenarioCfoSweepAmi, base_seed);

    std::printf("\n--- PSLTE 1 Mcps ---\n");
    hts::rx_test::run_scenario(hts::rx_test::kScenarioCfoSweepPslte,
                               base_seed);

    return 0;
}

#include "../../HTS_LIM/HTS_Secure_Memory.cpp"
#include "../../HTS_LIM/HTS_Polar_Codec.cpp"
#include "../../HTS_LIM/HTS_FEC_HARQ.cpp"
#include "../../HTS_LIM/HTS_Holo_LPI.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Permuter.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Core.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Payload.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_TX.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Decode.cpp"
