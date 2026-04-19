// =============================================================================
/// @file  HTS_Harq_Matrix_Main.cpp
/// @brief Phase 6 HARQ Matrix 메인 실행기
/// @target PC 시뮬 (HTS_Jammer_STD 프로젝트, Win32/x64)
///
/// [매트릭스]
///   fec_path × mode × channel × intensity × N_MC(50) = 총 ~13,000 trials
///
/// [출력]
///   HARQ_Matrix_Results.csv  (260 행, BOM 포함 UTF-8)
///
/// [실행]
///   HTS_Jammer_STD.exe
///   (작업 디렉터리에 HARQ_Matrix_Results.csv 생성)
// =============================================================================
#include "HTS_Jammer_STD.hpp"
#include "HTS_MC_Runner.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
// ── 매트릭스 정의 ─────────────────────────────────────────────
namespace {
struct ChannelSpec {
    HTS_Jammer_STD::ChannelType type;
    std::vector<double> intensities;
};
std::vector<ChannelSpec> Build_Channel_Matrix() {
    using Ch = HTS_Jammer_STD::ChannelType;
    std::vector<ChannelSpec> m;
    m.push_back({Ch::Clean, {0.0}});
    m.push_back({Ch::AWGN, {-20.0, -25.0, -30.0, -35.0, -40.0, -45.0, -50.0}});
    m.push_back({Ch::Barrage,
                 {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0}});
    m.push_back(
        {Ch::CW, {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0}});
    m.push_back({Ch::Pulse, {10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0}});
    m.push_back({Ch::MultiTone,
                 {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0}});
    m.push_back({Ch::Swept,
                 {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0}});
    m.push_back({Ch::Partial_Barrage,
                 {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0}});
    return m;
}
const char *FecName(bool use_ir) { return use_ir ? "ir_harq" : "chase"; }
const char *ModeName(bool is_voice) { return is_voice ? "VOICE" : "DATA"; }
// Cell 고유 seed 도출 (재현성)
uint64_t Derive_Cell_Seed(uint64_t base, bool use_ir, bool is_voice, int ch_idx,
                          double intensity) {
    uint64_t s = base;
    s ^= static_cast<uint64_t>(use_ir) * 0x0123456789ABCDEFull;
    s ^= static_cast<uint64_t>(is_voice) * 0xFEDCBA9876543210ull;
    s += static_cast<uint64_t>(ch_idx) * 0x100000000ull;
    s += static_cast<uint64_t>(std::llround(intensity * 100.0)) * 0x10000ull;
    // Mix (Wang hash)
    s = (s ^ (s >> 30)) * 0xBF58476D1CE4E5B9ull;
    s = (s ^ (s >> 27)) * 0x94D049BB133111EBull;
    s = s ^ (s >> 31);
    return s;
}
} // anonymous namespace
// ── main ──────────────────────────────────────────────────────
int main(int argc, char **argv) {
    // ── 파라미터 ──────────────────────────────────────────────
    constexpr uint64_t BASE_SEED = 0xDEADBEEFCAFEBABEull;
    constexpr int N_MC = 50;
    constexpr int MAX_HARQ_ROUNDS = 8;
    // 빠른 시험용: argv[1] == "--smoke" 면 N_MC=5 로 축소
    int n_mc = N_MC;
    bool smoke = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--smoke") {
            smoke = true;
            n_mc = 5;
        }
    }
    // ── CSV 출력 준비 ─────────────────────────────────────────
    const char *csv_path =
        smoke ? "HARQ_Matrix_Smoke.csv" : "HARQ_Matrix_Results.csv";
    std::FILE *fout = std::fopen(csv_path, "wb");
    if (fout == nullptr) {
        std::fprintf(stderr, "ERROR: cannot open %s\n", csv_path);
        return 1;
    }
    // UTF-8 BOM (Excel 한글 깨짐 방지)
    const unsigned char bom[3] = {0xEF, 0xBB, 0xBF};
    std::fwrite(bom, 1, 3, fout);
    std::fprintf(fout,
                 "fec_path,mode,channel,intensity_unit,intensity,"
                 "crc_ok,total,crc_rate,crc_ci_low,crc_ci_high,"
                 "ber,total_harq_rounds,avg_harq_per_block,max_harq_rounds,"
                 "saturation_rate,base_seed\n");
    std::fflush(fout);
    // ── 매트릭스 순회 ─────────────────────────────────────────
    const auto matrix = Build_Channel_Matrix();
    int total_cells = 0;
    for (const auto &cs : matrix)
        total_cells += static_cast<int>(cs.intensities.size());
    total_cells *= 4; // fec(2) × mode(2)
    int cell_idx = 0;
    const auto t_start = std::chrono::steady_clock::now();
    std::printf("=== Phase 6 HARQ Matrix ===\n");
    std::printf("N_MC=%d, MAX_HARQ_ROUNDS=%d, total_cells=%d\n", n_mc,
                MAX_HARQ_ROUNDS, total_cells);
    std::printf("Expected total trials: %d\n", n_mc * total_cells);
    std::printf("Output: %s\n\n", csv_path);
    std::fflush(stdout);
    for (int f = 0; f < 2; ++f) { // fec_path: chase(0) / ir_harq(1)
        const bool use_ir = (f == 1);
        for (int m = 0; m < 2; ++m) { // mode: DATA(0) / VOICE(1)
            const bool is_voice = (m == 1);
            int ch_idx = 0;
            for (const auto &cs : matrix) {
                for (const double intensity : cs.intensities) {
                    ++cell_idx;
                    const uint64_t cell_seed = Derive_Cell_Seed(
                        BASE_SEED, use_ir, is_voice, ch_idx, intensity);
                    const auto t0 = std::chrono::steady_clock::now();
                    HTS_MC_Runner::CellResult r = HTS_MC_Runner::Run_Cell(
                        use_ir, is_voice, cs.type, intensity, n_mc, cell_seed,
                        MAX_HARQ_ROUNDS);
                    const auto t1 = std::chrono::steady_clock::now();
                    const double elapsed_ms =
                        std::chrono::duration<double, std::milli>(t1 - t0)
                            .count();
                    // CSV 한 줄
                    std::fprintf(fout,
                                 "%s,%s,%s,%s,%.2f,"
                                 "%d,%d,%.6f,%.6f,%.6f,"
                                 "%.6f,%d,%.4f,%d,"
                                 "%.6f,0x%016llx\n",
                                 FecName(use_ir), ModeName(is_voice),
                                 HTS_Jammer_STD::Channel_Name(cs.type),
                                 HTS_Jammer_STD::Channel_Unit(cs.type),
                                 intensity, r.crc_ok_count, r.total, r.crc_rate,
                                 r.ci_low, r.ci_high, r.ber,
                                 r.total_harq_rounds, r.avg_harq_per_block,
                                 r.max_harq_rounds, r.saturation_rate,
                                 static_cast<unsigned long long>(r.base_seed));
                    std::fflush(fout);
                    // 진행 로그
                    std::printf("[%4d/%4d] %-7s %-5s %-16s %7.2f | "
                                "crc=%2d/%2d (%5.1f%%) CI[%.2f,%.2f] "
                                "ber=%.4f avg_harq=%.2f sat=%.4f  %.0f ms\n",
                                cell_idx, total_cells, FecName(use_ir),
                                ModeName(is_voice),
                                HTS_Jammer_STD::Channel_Name(cs.type),
                                intensity, r.crc_ok_count, r.total,
                                r.crc_rate * 100.0, r.ci_low, r.ci_high, r.ber,
                                r.avg_harq_per_block, r.saturation_rate,
                                elapsed_ms);
                    std::fflush(stdout);
                }
                ++ch_idx;
            }
        }
    }
    const auto t_end = std::chrono::steady_clock::now();
    const double total_sec =
        std::chrono::duration<double>(t_end - t_start).count();
    std::fclose(fout);
    std::printf("\n=== DONE ===\n");
    std::printf("Total cells: %d\n", cell_idx);
    std::printf("Total trials: %d\n", cell_idx * n_mc);
    std::printf("Elapsed: %.1f sec (%.2f min)\n", total_sec, total_sec / 60.0);
    std::printf("CSV: %s\n", csv_path);
    return 0;
}
