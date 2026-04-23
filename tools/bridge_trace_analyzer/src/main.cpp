#include <algorithm>
#include <array>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>
#include <sys/wait.h>

namespace {

enum class InputFormat {
    Auto,
    Vcd,
    Fst,
};

struct Options {
    std::string input_path;
    InputFormat format = InputFormat::Auto;
    std::size_t max_mismatches = 50;
    std::size_t progress_interval = 10000;
    bool shadow_memory = true;
    bool verbose = false;
    bool show_notes = false;
};

enum class SignalKey : std::size_t {
    Clk,
    HSelExt,
    HAddr,
    HWData,
    HWStrb,
    HWrite,
    HSize,
    HBurst,
    HTrans,
    HReady,
    HReadyExt,
    HRespExt,
    HRDataExt,
    AwAddr,
    AwLen,
    AwSize,
    AwBurst,
    AwValid,
    AwReady,
    WData,
    WStrb,
    WLast,
    WValid,
    WReady,
    BResp,
    BValid,
    BReady,
    ArAddr,
    ArLen,
    ArSize,
    ArBurst,
    ArValid,
    ArReady,
    RData,
    RResp,
    RLast,
    RValid,
    RReady,
    Count
};

constexpr std::size_t kSignalCount = static_cast<std::size_t>(SignalKey::Count);

struct SignalValue {
    bool known = false;
    std::uint64_t value = 0;
    std::string bits;
};

struct VarDecl {
    std::string code;
    std::string full_name;
    int width = 0;
};

struct SignalSpec {
    SignalKey key;
    const char* pretty_name;
    const char* suffix;
};

constexpr std::array<SignalSpec, kSignalCount> kSignalSpecs{{
    {SignalKey::Clk,       "clk",              "clk"},
    {SignalKey::HSelExt,   "HSELEXT",          "HSELEXT"},
    {SignalKey::HAddr,     "HADDR",            "HADDR [55:0]"},
    {SignalKey::HWData,    "HWDATA",           "HWDATA [63:0]"},
    {SignalKey::HWStrb,    "HWSTRB",           "HWSTRB [7:0]"},
    {SignalKey::HWrite,    "HWRITE",           "HWRITE"},
    {SignalKey::HSize,     "HSIZE",            "HSIZE [2:0]"},
    {SignalKey::HBurst,    "HBURST",           "HBURST [2:0]"},
    {SignalKey::HTrans,    "HTRANS",           "HTRANS [1:0]"},
    {SignalKey::HReady,    "HREADY",           "HREADY"},
    {SignalKey::HReadyExt, "HREADYEXT",        "HREADYEXT"},
    {SignalKey::HRespExt,  "HRESPEXT",         "HRESPEXT"},
    {SignalKey::HRDataExt, "HRDATAEXT",        "HRDATAEXT [63:0]"},
    {SignalKey::AwAddr,    "m_axi_awaddr",     "m_axi_awaddr [31:0]"},
    {SignalKey::AwLen,     "m_axi_awlen",      "m_axi_awlen [7:0]"},
    {SignalKey::AwSize,    "m_axi_awsize",     "m_axi_awsize [2:0]"},
    {SignalKey::AwBurst,   "m_axi_awburst",    "m_axi_awburst [1:0]"},
    {SignalKey::AwValid,   "m_axi_awvalid",    "m_axi_awvalid"},
    {SignalKey::AwReady,   "m_axi_awready",    "m_axi_awready"},
    {SignalKey::WData,     "m_axi_wdata",      "m_axi_wdata [63:0]"},
    {SignalKey::WStrb,     "m_axi_wstrb",      "m_axi_wstrb [7:0]"},
    {SignalKey::WLast,     "m_axi_wlast",      "m_axi_wlast"},
    {SignalKey::WValid,    "m_axi_wvalid",     "m_axi_wvalid"},
    {SignalKey::WReady,    "m_axi_wready",     "m_axi_wready"},
    {SignalKey::BResp,     "m_axi_bresp",      "m_axi_bresp [1:0]"},
    {SignalKey::BValid,    "m_axi_bvalid",     "m_axi_bvalid"},
    {SignalKey::BReady,    "m_axi_bready",     "m_axi_bready"},
    {SignalKey::ArAddr,    "m_axi_araddr",     "m_axi_araddr [31:0]"},
    {SignalKey::ArLen,     "m_axi_arlen",      "m_axi_arlen [7:0]"},
    {SignalKey::ArSize,    "m_axi_arsize",     "m_axi_arsize [2:0]"},
    {SignalKey::ArBurst,   "m_axi_arburst",    "m_axi_arburst [1:0]"},
    {SignalKey::ArValid,   "m_axi_arvalid",    "m_axi_arvalid"},
    {SignalKey::ArReady,   "m_axi_arready",    "m_axi_arready"},
    {SignalKey::RData,     "m_axi_rdata",      "m_axi_rdata [63:0]"},
    {SignalKey::RResp,     "m_axi_rresp",      "m_axi_rresp [1:0]"},
    {SignalKey::RLast,     "m_axi_rlast",      "m_axi_rlast"},
    {SignalKey::RValid,    "m_axi_rvalid",     "m_axi_rvalid"},
    {SignalKey::RReady,    "m_axi_rready",     "m_axi_rready"},
}};

struct SignalBinding {
    std::string code;
    std::string full_name;
    int width = 0;
    bool bound = false;
};

struct AhbWriteBeat {
    std::uint64_t time = 0;
    std::uint64_t addr = 0;
    std::uint64_t data = 0;
    std::uint8_t strb = 0;
    std::uint8_t size = 0;
};

struct AhbReadBeat {
    std::uint64_t time = 0;
    std::uint64_t addr = 0;
    std::uint64_t data = 0;
    std::uint8_t size = 0;
};

struct AxiReadBeat {
    std::uint64_t time = 0;
    std::uint64_t addr = 0;
    std::uint64_t data = 0;
    std::uint8_t size = 0;
};

struct AhbAddrPhase {
    std::uint64_t time = 0;
    std::uint64_t addr = 0;
    std::uint8_t size = 0;
    std::uint8_t burst = 0;
    std::uint8_t htrans = 0;
    bool is_write = false;
};

struct AxiBurstDesc {
    std::uint64_t time = 0;
    std::uint64_t addr = 0;
    std::uint8_t len = 0;
    std::uint8_t size = 0;
    std::uint8_t burst = 0;
};

struct ActiveBurst {
    AxiBurstDesc desc;
    std::uint32_t beat_index = 0;
};

struct AhbBurstDesc {
    std::uint64_t time = 0;
    std::uint64_t start_addr = 0;
    std::uint8_t beats = 0;
    std::uint8_t size = 0;
    std::uint8_t burst = 0;
    bool is_write = false;
};

struct ActiveAhbBurst {
    AhbBurstDesc desc;
    std::uint32_t beats_seen = 0;
    std::uint64_t last_addr = 0;
};

struct Stats {
    std::uint64_t posedges = 0;
    std::uint64_t ahb_writes = 0;
    std::uint64_t ahb_reads = 0;
    std::uint64_t axi_aw = 0;
    std::uint64_t axi_w = 0;
    std::uint64_t axi_b = 0;
    std::uint64_t axi_ar = 0;
    std::uint64_t axi_r = 0;
    std::uint64_t write_mismatches = 0;
    std::uint64_t read_mismatches = 0;
    std::uint64_t shadow_mismatches = 0;
    std::uint64_t burst_mismatches = 0;
    std::uint64_t warnings = 0;
    std::uint64_t notes = 0;
    std::uint64_t resync_events = 0;
};

std::string format_hex(std::uint64_t value, unsigned width_bits = 64) {
    const unsigned width_nibbles = (width_bits + 3U) / 4U;
    std::ostringstream oss;
    oss << "0x" << std::hex << std::setfill('0') << std::setw(static_cast<int>(width_nibbles))
        << value;
    return oss.str();
}

std::string trim(std::string_view sv) {
    while (!sv.empty() && (sv.front() == ' ' || sv.front() == '\t' || sv.front() == '\r' || sv.front() == '\n')) {
        sv.remove_prefix(1);
    }
    while (!sv.empty() && (sv.back() == ' ' || sv.back() == '\t' || sv.back() == '\r' || sv.back() == '\n')) {
        sv.remove_suffix(1);
    }
    return std::string(sv);
}

std::vector<std::string> split_ws(const std::string& line) {
    std::istringstream iss(line);
    std::vector<std::string> out;
    std::string tok;
    while (iss >> tok) {
        out.push_back(tok);
    }
    return out;
}

std::string shell_escape(const std::string& s) {
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') {
            out += "'\\''";
        } else {
            out += c;
        }
    }
    out += "'";
    return out;
}

InputFormat deduce_format(const Options& options) {
    if (options.format != InputFormat::Auto) {
        return options.format;
    }
    if (options.input_path.size() >= 4 &&
        options.input_path.substr(options.input_path.size() - 4) == ".fst") {
        return InputFormat::Fst;
    }
    return InputFormat::Vcd;
}

class LineReader {
  public:
    virtual ~LineReader() = default;
    virtual bool getline(std::string& out) = 0;
};

class FileLineReader final : public LineReader {
  public:
    explicit FileLineReader(const std::string& path) {
        fp_ = std::fopen(path.c_str(), "r");
        if (!fp_) {
            throw std::runtime_error("failed to open input file: " + path + ": " + std::strerror(errno));
        }
    }

    ~FileLineReader() override {
        if (fp_) {
            std::fclose(fp_);
        }
    }

    bool getline(std::string& out) override {
        char* line = nullptr;
        std::size_t cap = 0;
        const ssize_t rc = ::getline(&line, &cap, fp_);
        if (rc < 0) {
            std::free(line);
            return false;
        }
        out.assign(line, static_cast<std::size_t>(rc));
        std::free(line);
        return true;
    }

  private:
    FILE* fp_ = nullptr;
};

class PipeLineReader final : public LineReader {
  public:
    explicit PipeLineReader(const std::string& command) : command_(command) {
        fp_ = ::popen(command_.c_str(), "r");
        if (!fp_) {
            throw std::runtime_error("failed to launch pipe command: " + command_);
        }
    }

    ~PipeLineReader() override {
        close_pipe(false);
    }

    bool getline(std::string& out) override {
        char* line = nullptr;
        std::size_t cap = 0;
        const ssize_t rc = ::getline(&line, &cap, fp_);
        if (rc < 0) {
            std::free(line);
            close_pipe(true);
            return false;
        }
        out.assign(line, static_cast<std::size_t>(rc));
        std::free(line);
        return true;
    }

  private:
    void close_pipe(bool throw_on_failure) {
        if (!fp_) {
            return;
        }
        FILE* fp = fp_;
        fp_ = nullptr;
        const int rc = ::pclose(fp);
        if (throw_on_failure && rc != 0) {
            std::ostringstream oss;
            oss << "failed to convert FST input with fst2vcd";
            if (WIFEXITED(rc)) {
                oss << " (exit " << WEXITSTATUS(rc) << ")";
            }
            oss << ". The FST may be unreadable or incomplete.";
            throw std::runtime_error(oss.str());
        }
    }

    std::string command_;
    FILE* fp_ = nullptr;
};

class Analyzer {
  public:
    explicit Analyzer(const Options& options)
        : options_(options), next_progress_report_(options.progress_interval) {}

    void bind_signal(SignalKey key, const SignalBinding& binding) {
        bindings_[index_of(key)] = binding;
    }

    void on_posedge(std::uint64_t time, const std::array<SignalValue, kSignalCount>& state) {
        stats_.posedges++;
        handle_ahb_side(time, state);
        handle_axi_side(time, state);
    }

    void print_summary() const {
        std::cout
            << "Summary:\n"
            << "  posedges:          " << stats_.posedges << "\n"
            << "  ahb writes:        " << stats_.ahb_writes << "\n"
            << "  ahb reads:         " << stats_.ahb_reads << "\n"
            << "  axi aw:            " << stats_.axi_aw << "\n"
            << "  axi w:             " << stats_.axi_w << "\n"
            << "  axi b:             " << stats_.axi_b << "\n"
            << "  axi ar:            " << stats_.axi_ar << "\n"
            << "  axi r:             " << stats_.axi_r << "\n"
            << "  write mismatches:  " << stats_.write_mismatches << "\n"
            << "  read mismatches:   " << stats_.read_mismatches << "\n"
            << "  shadow mismatches: " << stats_.shadow_mismatches << "\n"
            << "  burst mismatches:  " << stats_.burst_mismatches << "\n"
            << "  warnings:          " << stats_.warnings << "\n"
            << "  notes:             " << stats_.notes << "\n"
            << "  resync events:     " << stats_.resync_events << "\n"
            << "  pending ahb writes:" << expected_ahb_writes_.size() << "\n"
            << "  pending axi reads: " << completed_axi_reads_.size() << "\n"
            << "  pending aw bursts: " << aw_queue_.size() << "\n"
            << "  pending ar bursts: " << ar_queue_.size() << "\n";
    }

  private:
    static constexpr std::uint64_t kUnknown64 = ~0ULL;

    static std::size_t index_of(SignalKey key) {
        return static_cast<std::size_t>(key);
    }

    bool bit(const std::array<SignalValue, kSignalCount>& state, SignalKey key) const {
        const auto& v = state[index_of(key)];
        return v.known && ((v.value & 1ULL) != 0ULL);
    }

    std::optional<std::uint64_t> value(const std::array<SignalValue, kSignalCount>& state,
                                       SignalKey key) const {
        const auto& v = state[index_of(key)];
        if (!v.known) {
            return std::nullopt;
        }
        return v.value;
    }

    static std::uint64_t beat_addr(const ActiveBurst& burst) {
        const std::uint64_t beat_bytes = 1ULL << burst.desc.size;
        return burst.desc.addr + beat_bytes * burst.beat_index;
    }

    void warn(std::string message) {
        stats_.warnings++;
        if (stats_.warnings <= options_.max_mismatches) {
            std::cout << "[warn] " << message << "\n";
        }
    }

    void note(std::string message) {
        stats_.notes++;
        if (options_.show_notes && stats_.notes <= options_.max_mismatches) {
            std::cout << "[note] " << message << "\n";
        }
    }

    void resync_note(std::string message) {
        stats_.resync_events++;
        note(std::move(message));
    }

    void bridge_write_mismatch(std::string message) {
        stats_.write_mismatches++;
        if (stats_.write_mismatches <= options_.max_mismatches) {
            std::cout << "[bridge-write-mismatch] " << message << "\n";
        }
    }

    void bridge_read_mismatch(std::string message) {
        stats_.read_mismatches++;
        if (stats_.read_mismatches <= options_.max_mismatches) {
            std::cout << "[bridge-read-mismatch] " << message << "\n";
        }
    }

    void shadow_mismatch(std::string message) {
        stats_.shadow_mismatches++;
        if (stats_.shadow_mismatches <= options_.max_mismatches) {
            std::cout << "[shadow-read-mismatch] " << message << "\n";
        }
    }

    void burst_mismatch(std::string message) {
        stats_.burst_mismatches++;
        if (stats_.burst_mismatches <= options_.max_mismatches) {
            std::cout << "[burst-mismatch] " << message << "\n";
        }
    }

    static const char* ahb_burst_name(std::uint8_t burst) {
        switch (burst & 0x7U) {
            case 0: return "SINGLE";
            case 1: return "INCR";
            case 2: return "WRAP4";
            case 3: return "INCR4";
            case 4: return "WRAP8";
            case 5: return "INCR8";
            case 6: return "WRAP16";
            case 7: return "INCR16";
            default: return "?";
        }
    }

    static const char* axi_burst_name(std::uint8_t burst) {
        switch (burst & 0x3U) {
            case 0: return "FIXED";
            case 1: return "INCR";
            case 2: return "WRAP";
            default: return "RESERVED";
        }
    }

    static std::uint8_t ahb_expected_beats(std::uint8_t burst) {
        switch (burst & 0x7U) {
            case 0: return 1;
            case 2:
            case 3: return 4;
            case 4:
            case 5: return 8;
            case 6:
            case 7: return 16;
            case 1:
            default: return 0;
        }
    }

    static bool ahb_burst_is_wrap(std::uint8_t burst) {
        switch (burst & 0x7U) {
            case 2:
            case 4:
            case 6: return true;
            default: return false;
        }
    }

    static std::uint64_t next_ahb_addr(const ActiveAhbBurst& burst) {
        const std::uint64_t beat_bytes = 1ULL << burst.desc.size;
        if (!ahb_burst_is_wrap(burst.desc.burst)) {
            return burst.last_addr + beat_bytes;
        }
        const std::uint64_t beats = burst.desc.beats;
        if (beats == 0) {
            return burst.last_addr + beat_bytes;
        }
        const std::uint64_t wrap_bytes = beat_bytes * beats;
        const std::uint64_t base = burst.desc.start_addr & ~(wrap_bytes - 1ULL);
        const std::uint64_t offset = (burst.last_addr + beat_bytes - base) % wrap_bytes;
        return base + offset;
    }

    void compare_burst_desc(const AhbBurstDesc& ahb, const AxiBurstDesc& axi, const char* side) {
        if (ahb.start_addr != (axi.addr & 0xffffffffULL)) {
            burst_mismatch(std::string(side) + " burst start address mismatch at time=" +
                           std::to_string(axi.time) +
                           " ahb_addr=" + format_hex(ahb.start_addr, 32) +
                           " axi_addr=" + format_hex(axi.addr & 0xffffffffULL, 32));
        }
        if (ahb.size != axi.size) {
            burst_mismatch(std::string(side) + " burst size mismatch at time=" +
                           std::to_string(axi.time) +
                           " ahb_size=" + std::to_string(ahb.size) +
                           " axi_size=" + std::to_string(axi.size));
        }
        if (ahb.beats != 0 && ahb.beats != static_cast<std::uint8_t>(axi.len + 1U)) {
            burst_mismatch(std::string(side) + " burst length mismatch at time=" +
                           std::to_string(axi.time) +
                           " ahb_burst=" + ahb_burst_name(ahb.burst) +
                           " ahb_beats=" + std::to_string(ahb.beats) +
                           " axi_len=" + std::to_string(axi.len));
        }

        const bool ahb_wrap = ahb_burst_is_wrap(ahb.burst);
        const std::uint8_t expected_axi_burst = ahb_wrap ? 2U : 1U;
        if (axi.burst != expected_axi_burst) {
            burst_mismatch(std::string(side) + " burst type mismatch at time=" +
                           std::to_string(axi.time) +
                           " ahb_burst=" + ahb_burst_name(ahb.burst) +
                           " axi_burst=" + axi_burst_name(axi.burst));
        }
    }

    bool maybe_resync_read_to_addr(const AhbReadBeat& ahb) {
        if (completed_axi_reads_.size() < 2) {
            return false;
        }
        constexpr std::size_t kMaxDrop = 64;
        const std::size_t limit = std::min(completed_axi_reads_.size(), kMaxDrop + 1U);
        std::size_t drop_count = 0;
        for (std::size_t i = 1; i < limit; ++i) {
            if (completed_axi_reads_[i].addr == ahb.addr) {
                drop_count = i;
                break;
            }
        }
        if (drop_count == 0) {
            return false;
        }
        resync_note("resynchronized read stream by dropping " +
                    std::to_string(drop_count) +
                    " stale AXI read beat(s) at time=" +
                    std::to_string(ahb.time) +
                    " addr=" + format_hex(ahb.addr, 32));
        for (std::size_t i = 0; i < drop_count; ++i) {
            completed_axi_reads_.pop_front();
        }
        return true;
    }

    bool maybe_resync_write_one_beat(std::uint64_t time, std::uint64_t addr) {
        if (expected_ahb_writes_.size() < 2) {
            return false;
        }
        constexpr std::size_t kMaxDrop = 64;
        const std::size_t limit = std::min(expected_ahb_writes_.size(), kMaxDrop + 1U);
        std::size_t drop_count = 0;
        for (std::size_t i = 1; i < limit; ++i) {
            if (expected_ahb_writes_[i].addr == addr) {
                drop_count = i;
                break;
            }
        }
        if (drop_count == 0) {
            return false;
        }
        resync_note("resynchronized write stream by dropping " +
                    std::to_string(drop_count) +
                    " stale AHB write beat(s) at time=" +
                    std::to_string(time) +
                    " addr=" + format_hex(addr, 32));
        for (std::size_t i = 0; i < drop_count; ++i) {
            expected_ahb_writes_.pop_front();
        }
        return true;
    }

    void revise_pending_ahb_burst_length(const AhbBurstDesc& desc,
                                         std::uint32_t actual_beats) {
        auto& queue = desc.is_write ? pending_ahb_write_bursts_ : pending_ahb_read_bursts_;
        for (auto it = queue.rbegin(); it != queue.rend(); ++it) {
            if (it->time == desc.time &&
                it->start_addr == desc.start_addr &&
                it->is_write == desc.is_write) {
                it->beats = static_cast<std::uint8_t>(std::min<std::uint32_t>(actual_beats, 255U));
                return;
            }
        }
    }

    void finish_ahb_burst_early(std::optional<ActiveAhbBurst>& active,
                                std::uint64_t time,
                                const char* reason) {
        if (!active) {
            return;
        }
        if (active->desc.beats != 0 &&
            active->beats_seen > 0 &&
            active->beats_seen < active->desc.beats) {
            revise_pending_ahb_burst_length(active->desc, active->beats_seen);
            resync_note(std::string(active->desc.is_write ? "write" : "read") +
                        " AHB " + ahb_burst_name(active->desc.burst) +
                        " burst ended early at time=" + std::to_string(time) +
                        " reason=" + reason +
                        " beats_seen=" + std::to_string(active->beats_seen) +
                        " declared_beats=" + std::to_string(active->desc.beats));
        }
        active.reset();
    }

    void finish_all_ahb_bursts_early(std::uint64_t time, const char* reason) {
        finish_ahb_burst_early(active_ahb_write_burst_, time, reason);
        finish_ahb_burst_early(active_ahb_read_burst_, time, reason);
    }

    void lose_read_sync(std::uint64_t time, const std::string& reason) {
        warn("lost read synchronization at time=" + std::to_string(time) + ": " + reason);
        read_burst_synced_ = false;
        completed_axi_reads_.clear();
        ar_queue_.clear();
        active_read_.reset();
        pending_ahb_read_bursts_.clear();
        active_ahb_read_burst_.reset();
    }

    void lose_write_sync(std::uint64_t time, const std::string& reason) {
        warn("lost write synchronization at time=" + std::to_string(time) + ": " + reason);
        write_burst_synced_ = false;
        expected_ahb_writes_.clear();
        aw_queue_.clear();
        active_write_.reset();
        pending_ahb_write_bursts_.clear();
        active_ahb_write_burst_.reset();
    }

    void maybe_report_progress(std::uint64_t time) {
        if (options_.progress_interval == 0) {
            return;
        }
        const std::uint64_t total_transactions =
            stats_.ahb_writes + stats_.ahb_reads + stats_.axi_w + stats_.axi_r;
        if (total_transactions >= next_progress_report_) {
            std::cout << "[progress] time=" << time
                      << " tx=" << total_transactions
                      << " ahb_w=" << stats_.ahb_writes
                      << " ahb_r=" << stats_.ahb_reads
                      << " axi_w=" << stats_.axi_w
                      << " axi_r=" << stats_.axi_r
                      << " wr_mis=" << stats_.write_mismatches
                      << " rd_mis=" << stats_.read_mismatches
                      << " shadow_mis=" << stats_.shadow_mismatches
                      << "\n";
            next_progress_report_ += options_.progress_interval;
        }
    }

    void handle_ahb_side(std::uint64_t time, const std::array<SignalValue, kSignalCount>& state) {
        const bool data_ready = bit(state, SignalKey::HReadyExt);
        const bool addr_ready = bit(state, SignalKey::HReady);
        if (data_ready && pending_ahb_addr_phase_) {
            const auto& pending = *pending_ahb_addr_phase_;
            if (pending.is_write) {
                stats_.ahb_writes++;
                const auto data = value(state, SignalKey::HWData).value_or(0);
                const auto strb = static_cast<std::uint8_t>(value(state, SignalKey::HWStrb).value_or(0));
                expected_ahb_writes_.push_back(
                    AhbWriteBeat{time, pending.addr & 0xffffffffULL, data, strb, pending.size});
            } else {
                stats_.ahb_reads++;
                const auto data = value(state, SignalKey::HRDataExt).value_or(0);
                consume_ahb_read_completion(
                    AhbReadBeat{time, pending.addr & 0xffffffffULL, data, pending.size});
                if (options_.shadow_memory) {
                    check_shadow_read("AHB", time, pending.addr & 0xffffffffULL, data);
                }
            }
            maybe_report_progress(time);
            pending_ahb_addr_phase_.reset();
        }

        const bool selected = bit(state, SignalKey::HSelExt);
        const auto htrans = value(state, SignalKey::HTrans).value_or(0);
        if (addr_ready && (!selected || ((htrans & 0x3ULL) == 0x0ULL))) {
            finish_all_ahb_bursts_early(time, selected ? "IDLE" : "HSEL=0");
        }
        const bool active_transfer = selected && addr_ready && ((htrans & 0x2ULL) != 0ULL);
        if (active_transfer) {
            const auto addr = value(state, SignalKey::HAddr).value_or(0);
            const auto size = static_cast<std::uint8_t>(value(state, SignalKey::HSize).value_or(0));
            const auto burst = static_cast<std::uint8_t>(value(state, SignalKey::HBurst).value_or(0));
            const bool is_write = bit(state, SignalKey::HWrite);
            const bool nonseq = (htrans & 0x3ULL) == 0x2ULL;
            const bool seq = (htrans & 0x3ULL) == 0x3ULL;

            if (nonseq) {
                finish_all_ahb_bursts_early(time, "NONSEQ restart");
                ActiveAhbBurst new_burst;
                new_burst.desc = AhbBurstDesc{
                    time,
                    addr & 0xffffffffULL,
                    ahb_expected_beats(burst),
                    size,
                    burst,
                    is_write
                };
                new_burst.beats_seen = 1;
                new_burst.last_addr = addr & 0xffffffffULL;
                if (is_write) {
                    active_ahb_write_burst_ = new_burst;
                    pending_ahb_write_bursts_.push_back(new_burst.desc);
                    seen_any_ahb_write_burst_ = true;
                } else {
                    active_ahb_read_burst_ = new_burst;
                    pending_ahb_read_bursts_.push_back(new_burst.desc);
                    seen_any_ahb_read_burst_ = true;
                }
                if (new_burst.desc.beats == 1) {
                    if (is_write) {
                        active_ahb_write_burst_.reset();
                    } else {
                        active_ahb_read_burst_.reset();
                    }
                }
            } else if (seq) {
                auto& active = is_write ? active_ahb_write_burst_ : active_ahb_read_burst_;
                if (!active) {
                    const bool seen_any = is_write ? seen_any_ahb_write_burst_ : seen_any_ahb_read_burst_;
                    if (seen_any) {
                        note(std::string("AHB SEQ beat without tracked NONSEQ at time=") +
                             std::to_string(time));
                    }
                } else {
                    const auto expected_addr = next_ahb_addr(*active);
                    const auto actual_addr = addr & 0xffffffffULL;
                    if (expected_addr != actual_addr) {
                        burst_mismatch(std::string(is_write ? "write" : "read") +
                                       " AHB burst address progression mismatch at time=" +
                                       std::to_string(time) +
                                       " expected_addr=" + format_hex(expected_addr, 32) +
                                       " actual_addr=" + format_hex(actual_addr, 32) +
                                       " burst=" + ahb_burst_name(active->desc.burst));
                    }
                    active->beats_seen++;
                    active->last_addr = actual_addr;
                    if (active->desc.beats != 0 && active->beats_seen >= active->desc.beats) {
                        active.reset();
                    }
                }
            }

            pending_ahb_addr_phase_ = AhbAddrPhase{time, addr, size, burst,
                                                   static_cast<std::uint8_t>(htrans), is_write};
        }
    }

    void handle_axi_side(std::uint64_t time, const std::array<SignalValue, kSignalCount>& state) {
        if (bit(state, SignalKey::AwValid) && bit(state, SignalKey::AwReady)) {
            stats_.axi_aw++;
            AxiBurstDesc axi{
                time,
                value(state, SignalKey::AwAddr).value_or(0),
                static_cast<std::uint8_t>(value(state, SignalKey::AwLen).value_or(0)),
                static_cast<std::uint8_t>(value(state, SignalKey::AwSize).value_or(0)),
                static_cast<std::uint8_t>(value(state, SignalKey::AwBurst).value_or(0)),
            };
            aw_queue_.push_back(axi);
            if (!pending_ahb_write_bursts_.empty()) {
                compare_burst_desc(pending_ahb_write_bursts_.front(), axi, "write");
                pending_ahb_write_bursts_.pop_front();
                write_burst_synced_ = true;
            }
        }

        if (bit(state, SignalKey::ArValid) && bit(state, SignalKey::ArReady)) {
            stats_.axi_ar++;
            AxiBurstDesc axi{
                time,
                value(state, SignalKey::ArAddr).value_or(0),
                static_cast<std::uint8_t>(value(state, SignalKey::ArLen).value_or(0)),
                static_cast<std::uint8_t>(value(state, SignalKey::ArSize).value_or(0)),
                static_cast<std::uint8_t>(value(state, SignalKey::ArBurst).value_or(0)),
            };
            ar_queue_.push_back(axi);
            if (!pending_ahb_read_bursts_.empty()) {
                compare_burst_desc(pending_ahb_read_bursts_.front(), axi, "read");
                pending_ahb_read_bursts_.pop_front();
                read_burst_synced_ = true;
            }
        }

        if (bit(state, SignalKey::WValid) && bit(state, SignalKey::WReady)) {
            stats_.axi_w++;
            if (!active_write_ && !aw_queue_.empty()) {
                active_write_ = ActiveBurst{aw_queue_.front(), 0};
                aw_queue_.pop_front();
            }
            if (!active_write_) {
                if (write_burst_synced_) {
                    bridge_write_mismatch("AXI W beat without a pending AW at time=" + std::to_string(time));
                }
            } else {
                consume_axi_write(time, state);
            }
            maybe_report_progress(time);
        }

        if (bit(state, SignalKey::BValid) && bit(state, SignalKey::BReady)) {
            stats_.axi_b++;
            const auto bresp = value(state, SignalKey::BResp).value_or(0);
            if (bresp != 0) {
                warn("AXI write response BRESP=" + format_hex(bresp, 2) +
                     " at time=" + std::to_string(time));
            }
        }

        if (bit(state, SignalKey::RValid) && bit(state, SignalKey::RReady)) {
            stats_.axi_r++;
            const auto rresp = value(state, SignalKey::RResp).value_or(0);
            if (rresp != 0) {
                warn("AXI read response RRESP=" + format_hex(rresp, 2) +
                     " at time=" + std::to_string(time));
            }
            if (!active_read_ && !ar_queue_.empty()) {
                active_read_ = ActiveBurst{ar_queue_.front(), 0};
                ar_queue_.pop_front();
            }
            if (!active_read_) {
                if (read_burst_synced_) {
                    bridge_read_mismatch("AXI R beat without a pending AR at time=" + std::to_string(time));
                }
            } else {
                consume_axi_read(time, state);
            }
            maybe_report_progress(time);
        }
    }

    void consume_axi_write(std::uint64_t time, const std::array<SignalValue, kSignalCount>& state) {
        const auto data = value(state, SignalKey::WData).value_or(0);
        const auto strb = static_cast<std::uint8_t>(value(state, SignalKey::WStrb).value_or(0));
        const auto addr = beat_addr(*active_write_) & 0xffffffffULL;

        if (expected_ahb_writes_.empty()) {
            if (write_burst_synced_) {
                lose_write_sync(time, "AXI W beat without prior AHB write beat at addr=" +
                                      format_hex(addr, 32));
            }
            return;
        } else {
            AhbWriteBeat expected = expected_ahb_writes_.front();
            if (expected.addr != addr && maybe_resync_write_one_beat(time, addr)) {
                expected = expected_ahb_writes_.front();
            }
            expected_ahb_writes_.pop_front();

            if (expected.addr != addr) {
                if (write_burst_synced_) {
                    lose_write_sync(time, "write address mismatch ahb_addr=" +
                                          format_hex(expected.addr, 32) +
                                          " axi_addr=" + format_hex(addr, 32));
                }
                return;
            }
            if (expected.strb != strb) {
                bridge_write_mismatch("write strobe mismatch at time=" + std::to_string(time) +
                                      " ahb_strb=" + format_hex(expected.strb, 8) +
                                      " axi_strb=" + format_hex(strb, 8));
            }
            if (expected.data != data) {
                bridge_write_mismatch("write data mismatch at time=" + std::to_string(time) +
                                      " addr=" + format_hex(addr, 32) +
                                      " ahb_data=" + format_hex(expected.data, 64) +
                                      " axi_data=" + format_hex(data, 64));
            }
        }

        if (options_.shadow_memory) {
            apply_shadow_write(addr, data, strb);
        }

        const bool wlast = bit(state, SignalKey::WLast);
        active_write_->beat_index++;
        if (wlast &&
            active_write_->beat_index != static_cast<std::uint32_t>(active_write_->desc.len) + 1U) {
            burst_mismatch("AXI write burst ended with WLAST at unexpected beat count time=" +
                           std::to_string(time) +
                           " beats_seen=" + std::to_string(active_write_->beat_index) +
                           " expected_beats=" + std::to_string(static_cast<std::uint32_t>(active_write_->desc.len) + 1U));
        }
        if (wlast) {
            active_write_.reset();
        }
    }

    void consume_axi_read(std::uint64_t time, const std::array<SignalValue, kSignalCount>& state) {
        const auto data = value(state, SignalKey::RData).value_or(0);
        const auto addr = beat_addr(*active_read_) & 0xffffffffULL;
        completed_axi_reads_.push_back(AxiReadBeat{time, addr, data, active_read_->desc.size});

        if (options_.shadow_memory) {
            check_shadow_read("AXI", time, addr, data);
        }

        const bool rlast = bit(state, SignalKey::RLast);
        active_read_->beat_index++;
        if (rlast &&
            active_read_->beat_index != static_cast<std::uint32_t>(active_read_->desc.len) + 1U) {
            burst_mismatch("AXI read burst ended with RLAST at unexpected beat count time=" +
                           std::to_string(time) +
                           " beats_seen=" + std::to_string(active_read_->beat_index) +
                           " expected_beats=" + std::to_string(static_cast<std::uint32_t>(active_read_->desc.len) + 1U));
        }
        if (rlast) {
            active_read_.reset();
        }
    }

    void consume_ahb_read_completion(const AhbReadBeat& ahb) {
        if (completed_axi_reads_.empty()) {
            if (read_burst_synced_) {
                lose_read_sync(ahb.time, "AHB read completion without prior AXI R beat at addr=" +
                                         format_hex(ahb.addr, 32));
            }
            return;
        }

        AxiReadBeat axi = completed_axi_reads_.front();
        if (axi.addr != ahb.addr && maybe_resync_read_to_addr(ahb)) {
            axi = completed_axi_reads_.front();
        }
        completed_axi_reads_.pop_front();

        if (ahb.addr != axi.addr) {
            if (read_burst_synced_) {
                lose_read_sync(ahb.time, "read address mismatch ahb_addr=" +
                                         format_hex(ahb.addr, 32) +
                                         " axi_addr=" + format_hex(axi.addr, 32));
            }
            return;
        }
        if (ahb.data != axi.data) {
            bridge_read_mismatch("read data mismatch at time=" + std::to_string(ahb.time) +
                                 " addr=" + format_hex(ahb.addr, 32) +
                                 " ahb_data=" + format_hex(ahb.data, 64) +
                                 " axi_data=" + format_hex(axi.data, 64));
        }
    }

    void apply_shadow_write(std::uint64_t addr, std::uint64_t data, std::uint8_t strb) {
        for (unsigned i = 0; i < 8; ++i) {
            if ((strb >> i) & 0x1U) {
                shadow_memory_[addr + i] = static_cast<std::uint8_t>((data >> (8U * i)) & 0xffU);
            }
        }
    }

    void check_shadow_read(const char* side, std::uint64_t time, std::uint64_t addr, std::uint64_t data) {
        std::uint64_t expected = 0;
        for (unsigned i = 0; i < 8; ++i) {
            const auto it = shadow_memory_.find(addr + i);
            if (it == shadow_memory_.end()) {
                return;
            }
            expected |= static_cast<std::uint64_t>(it->second) << (8U * i);
        }

        if (expected != data) {
            shadow_mismatch(std::string(side) + " shadow mismatch at time=" + std::to_string(time) +
                            " addr=" + format_hex(addr, 32) +
                            " expected=" + format_hex(expected, 64) +
                            " actual=" + format_hex(data, 64));
        }
    }

    Options options_;
    Stats stats_;
    std::array<SignalBinding, kSignalCount> bindings_{};
    std::deque<AhbWriteBeat> expected_ahb_writes_;
    std::deque<AxiReadBeat> completed_axi_reads_;
    std::optional<AhbAddrPhase> pending_ahb_addr_phase_;
    std::deque<AhbBurstDesc> pending_ahb_write_bursts_;
    std::deque<AhbBurstDesc> pending_ahb_read_bursts_;
    std::deque<AxiBurstDesc> aw_queue_;
    std::deque<AxiBurstDesc> ar_queue_;
    std::optional<ActiveBurst> active_write_;
    std::optional<ActiveBurst> active_read_;
    std::optional<ActiveAhbBurst> active_ahb_write_burst_;
    std::optional<ActiveAhbBurst> active_ahb_read_burst_;
    std::unordered_map<std::uint64_t, std::uint8_t> shadow_memory_;
    bool seen_any_ahb_write_burst_ = false;
    bool seen_any_ahb_read_burst_ = false;
    bool write_burst_synced_ = false;
    bool read_burst_synced_ = false;
    std::uint64_t next_progress_report_;
};

class VcdProcessor {
  public:
    VcdProcessor(LineReader& reader, const Options& options)
        : reader_(reader), options_(options), analyzer_(options) {}

    void run() {
        parse_header();
        if (options_.verbose) {
            dump_bindings();
        }
        parse_body();
        analyzer_.print_summary();
    }

  private:
    static bool ends_with(const std::string& s, const std::string& suffix) {
        return s.size() >= suffix.size() &&
               s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    void parse_header() {
        std::vector<std::string> scope_stack;
        std::string line;
        while (reader_.getline(line)) {
            const std::string stripped = trim(line);
            if (stripped.empty()) {
                continue;
            }

            if (stripped.rfind("$scope", 0) == 0) {
                const auto tokens = split_ws(stripped);
                if (tokens.size() >= 3) {
                    scope_stack.push_back(tokens[2]);
                }
                continue;
            }

            if (stripped == "$upscope $end") {
                if (!scope_stack.empty()) {
                    scope_stack.pop_back();
                }
                continue;
            }

            if (stripped.rfind("$var", 0) == 0) {
                const auto tokens = split_ws(stripped);
                if (tokens.size() >= 6) {
                    VarDecl var;
                    var.width = std::stoi(tokens[2]);
                    var.code = tokens[3];
                    std::ostringstream ref;
                    for (std::size_t i = 4; i + 1 < tokens.size(); ++i) {
                        if (i != 4) {
                            ref << ' ';
                        }
                        ref << tokens[i];
                    }
                    std::ostringstream full;
                    for (std::size_t i = 0; i < scope_stack.size(); ++i) {
                        if (i != 0) {
                            full << '.';
                        }
                        full << scope_stack[i];
                    }
                    if (!scope_stack.empty()) {
                        full << '.';
                    }
                    full << ref.str();
                    var.full_name = full.str();
                    vars_.push_back(std::move(var));
                }
                continue;
            }

            if (stripped == "$enddefinitions $end") {
                break;
            }
        }

        resolve_signals();
    }

    void resolve_signals() {
        for (const auto& spec : kSignalSpecs) {
            const VarDecl* best = nullptr;
            int best_score = -1;
            for (const auto& var : vars_) {
                int score = signal_match_score(var.full_name, spec.suffix);
                if (score > best_score) {
                    best = &var;
                    best_score = score;
                } else if (score == best_score && best && var.full_name.size() < best->full_name.size()) {
                    best = &var;
                }
            }

            if (best && best_score > 0) {
                SignalBinding binding;
                binding.code = best->code;
                binding.full_name = best->full_name;
                binding.width = best->width;
                binding.bound = true;
                bindings_[static_cast<std::size_t>(spec.key)] = binding;
                code_to_keys_[best->code].push_back(spec.key);
                analyzer_.bind_signal(spec.key, binding);
            }
        }

        require_binding(SignalKey::Clk);
        require_binding(SignalKey::HSelExt);
        require_binding(SignalKey::HAddr);
        require_binding(SignalKey::HWData);
        require_binding(SignalKey::HWStrb);
        require_binding(SignalKey::HWrite);
        require_binding(SignalKey::HSize);
        require_binding(SignalKey::HBurst);
        require_binding(SignalKey::HTrans);
        require_binding(SignalKey::HReady);
        require_binding(SignalKey::HReadyExt);
        require_binding(SignalKey::HRDataExt);
        require_binding(SignalKey::AwAddr);
        require_binding(SignalKey::AwLen);
        require_binding(SignalKey::AwSize);
        require_binding(SignalKey::AwBurst);
        require_binding(SignalKey::AwValid);
        require_binding(SignalKey::AwReady);
        require_binding(SignalKey::WData);
        require_binding(SignalKey::WStrb);
        require_binding(SignalKey::WLast);
        require_binding(SignalKey::WValid);
        require_binding(SignalKey::WReady);
        require_binding(SignalKey::BValid);
        require_binding(SignalKey::BReady);
        require_binding(SignalKey::ArAddr);
        require_binding(SignalKey::ArLen);
        require_binding(SignalKey::ArSize);
        require_binding(SignalKey::ArBurst);
        require_binding(SignalKey::ArValid);
        require_binding(SignalKey::ArReady);
        require_binding(SignalKey::RData);
        require_binding(SignalKey::RLast);
        require_binding(SignalKey::RValid);
        require_binding(SignalKey::RReady);
    }

    static int signal_match_score(const std::string& full_name, const std::string& suffix) {
        if (full_name == suffix) {
            return 3;
        }
        const std::string top_level = "testbench_bridge." + suffix;
        if (full_name == top_level) {
            return 4;
        }
        if (ends_with(full_name, "." + suffix)) {
            return 2;
        }
        return 0;
    }

    void require_binding(SignalKey key) const {
        const auto& binding = bindings_[static_cast<std::size_t>(key)];
        if (!binding.bound) {
            const auto& spec = kSignalSpecs[static_cast<std::size_t>(key)];
            throw std::runtime_error("missing required signal in trace: " + std::string(spec.suffix));
        }
    }

    static std::optional<SignalValue> parse_scalar_change(char ch) {
        SignalValue out;
        switch (ch) {
            case '0':
                out.known = true;
                out.value = 0;
                out.bits = "0";
                return out;
            case '1':
                out.known = true;
                out.value = 1;
                out.bits = "1";
                return out;
            default:
                out.known = false;
                out.bits = std::string(1, ch);
                return out;
        }
    }

    static std::optional<SignalValue> parse_vector_change(const std::string& bits) {
        SignalValue out;
        out.bits = bits;
        if (bits.find_first_not_of("01") != std::string::npos) {
            out.known = false;
            return out;
        }
        out.known = true;
        std::uint64_t value = 0;
        for (char c : bits) {
            value = (value << 1U) | static_cast<std::uint64_t>(c == '1');
        }
        out.value = value;
        return out;
    }

    void apply_change(const std::string& code, const SignalValue& value) {
        const auto it = code_to_keys_.find(code);
        if (it == code_to_keys_.end()) {
            return;
        }
        for (SignalKey key : it->second) {
            state_[static_cast<std::size_t>(key)] = value;
        }
    }

    void finalize_timestamp() {
        if (!have_timestamp_) {
            return;
        }
        const auto clk_before = prev_state_[static_cast<std::size_t>(SignalKey::Clk)];
        const auto clk_after = state_[static_cast<std::size_t>(SignalKey::Clk)];
        if (clk_before.known && clk_after.known && clk_before.value == 0 && clk_after.value == 1) {
            analyzer_.on_posedge(current_time_, state_);
        }
        prev_state_ = state_;
    }

    void parse_body() {
        std::string line;
        while (reader_.getline(line)) {
            const std::string stripped = trim(line);
            if (stripped.empty()) {
                continue;
            }
            if (stripped[0] == '#') {
                finalize_timestamp();
                current_time_ = std::strtoull(stripped.c_str() + 1, nullptr, 10);
                have_timestamp_ = true;
                continue;
            }
            if (stripped[0] == 'b' || stripped[0] == 'B') {
                const auto tokens = split_ws(stripped);
                if (tokens.size() == 2) {
                    auto parsed = parse_vector_change(tokens[0].substr(1));
                    if (parsed) {
                        apply_change(tokens[1], *parsed);
                    }
                }
                continue;
            }
            if ((stripped[0] == '0' || stripped[0] == '1' || stripped[0] == 'x' || stripped[0] == 'z') &&
                stripped.size() >= 2) {
                auto parsed = parse_scalar_change(stripped[0]);
                if (parsed) {
                    apply_change(stripped.substr(1), *parsed);
                }
            }
        }
        finalize_timestamp();
    }

    void dump_bindings() const {
        std::cout << "Resolved signal bindings:\n";
        for (const auto& spec : kSignalSpecs) {
            const auto& binding = bindings_[static_cast<std::size_t>(spec.key)];
            if (binding.bound) {
                std::cout << "  " << spec.pretty_name << " -> " << binding.full_name
                          << " (code " << binding.code << ")\n";
            }
        }
    }

    LineReader& reader_;
    Options options_;
    Analyzer analyzer_;
    std::vector<VarDecl> vars_;
    std::array<SignalBinding, kSignalCount> bindings_{};
    std::unordered_map<std::string, std::vector<SignalKey>> code_to_keys_;
    std::array<SignalValue, kSignalCount> state_{};
    std::array<SignalValue, kSignalCount> prev_state_{};
    std::uint64_t current_time_ = 0;
    bool have_timestamp_ = false;
};

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " --input <trace.{fst|vcd}> [options]\n"
        << "Options:\n"
        << "  --format <auto|fst|vcd>\n"
        << "  --max-mismatches <n>\n"
        << "  --progress-interval <n>\n"
        << "  --no-shadow-memory\n"
        << "  --show-notes\n"
        << "  --verbose\n";
}

Options parse_args(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            options.input_path = argv[++i];
        } else if (arg == "--format" && i + 1 < argc) {
            const std::string fmt = argv[++i];
            if (fmt == "auto") {
                options.format = InputFormat::Auto;
            } else if (fmt == "fst") {
                options.format = InputFormat::Fst;
            } else if (fmt == "vcd") {
                options.format = InputFormat::Vcd;
            } else {
                throw std::runtime_error("unknown format: " + fmt);
            }
        } else if (arg == "--max-mismatches" && i + 1 < argc) {
            options.max_mismatches = std::strtoull(argv[++i], nullptr, 10);
        } else if (arg == "--progress-interval" && i + 1 < argc) {
            options.progress_interval = std::strtoull(argv[++i], nullptr, 10);
        } else if (arg == "--no-shadow-memory") {
            options.shadow_memory = false;
        } else if (arg == "--show-notes") {
            options.show_notes = true;
        } else if (arg == "--verbose") {
            options.verbose = true;
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (options.input_path.empty()) {
        throw std::runtime_error("missing --input");
    }
    return options;
}

std::unique_ptr<LineReader> open_reader(const Options& options) {
    const auto format = deduce_format(options);
    if (format == InputFormat::Vcd) {
        return std::make_unique<FileLineReader>(options.input_path);
    }
    const std::string cmd = "fst2vcd " + shell_escape(options.input_path);
    return std::make_unique<PipeLineReader>(cmd);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_args(argc, argv);
        auto reader = open_reader(options);
        VcdProcessor processor(*reader, options);
        processor.run();
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "error: " << ex.what() << "\n";
        print_usage(argv[0]);
        return 1;
    }
}
