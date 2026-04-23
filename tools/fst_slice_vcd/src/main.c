#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fstapi.h"

typedef struct {
    const char *input_path;
    const char *output_path;
    uint64_t start_time;
    uint64_t end_time;
    int enable_vcd_extensions;
} Options;

static void print_usage(const char *argv0) {
    fprintf(stderr,
            "Usage: %s --input <trace.fst> --output <slice.vcd> --start <time> --end <time> [--vcd-extensions]\n",
            argv0);
}

static int parse_u64(const char *text, uint64_t *out) {
    char *endptr = NULL;
    unsigned long long value;

    if (!text || !*text) {
        return 0;
    }

    value = strtoull(text, &endptr, 10);
    if (!endptr || *endptr != '\0') {
        return 0;
    }

    *out = (uint64_t)value;
    return 1;
}

static int parse_args(int argc, char **argv, Options *opts) {
    int i;

    memset(opts, 0, sizeof(*opts));
    for (i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--input") && (i + 1) < argc) {
            opts->input_path = argv[++i];
        } else if (!strcmp(argv[i], "--output") && (i + 1) < argc) {
            opts->output_path = argv[++i];
        } else if (!strcmp(argv[i], "--start") && (i + 1) < argc) {
            if (!parse_u64(argv[++i], &opts->start_time)) {
                fprintf(stderr, "error: invalid --start value\n");
                return 0;
            }
        } else if (!strcmp(argv[i], "--end") && (i + 1) < argc) {
            if (!parse_u64(argv[++i], &opts->end_time)) {
                fprintf(stderr, "error: invalid --end value\n");
                return 0;
            }
        } else if (!strcmp(argv[i], "--vcd-extensions")) {
            opts->enable_vcd_extensions = 1;
        } else {
            fprintf(stderr, "error: unknown argument: %s\n", argv[i]);
            return 0;
        }
    }

    if (!opts->input_path || !opts->output_path) {
        fprintf(stderr, "error: --input and --output are required\n");
        return 0;
    }
    if (opts->end_time < opts->start_time) {
        fprintf(stderr, "error: --end must be >= --start\n");
        return 0;
    }

    return 1;
}

int main(int argc, char **argv) {
    Options opts;
    fstReaderContext *reader = NULL;
    FILE *out = NULL;
    int rc = 1;
    uint64_t src_start;
    uint64_t src_end;

    if (!parse_args(argc, argv, &opts)) {
        print_usage(argv[0]);
        return 1;
    }

    reader = fstReaderOpen(opts.input_path);
    if (!reader) {
        fprintf(stderr, "error: could not open input FST: %s\n", opts.input_path);
        goto done;
    }

    src_start = fstReaderGetStartTime(reader);
    src_end = fstReaderGetEndTime(reader);
    if (opts.start_time > src_end || opts.end_time < src_start) {
        fprintf(stderr,
                "error: requested range [%" PRIu64 ", %" PRIu64 "] is outside trace range [%" PRIu64 ", %" PRIu64 "]\n",
                opts.start_time,
                opts.end_time,
                src_start,
                src_end);
        goto done;
    }

    out = fopen(opts.output_path, "wb");
    if (!out) {
        fprintf(stderr, "error: could not open output VCD: %s\n", opts.output_path);
        goto done;
    }

    fstReaderSetVcdExtensions(reader, opts.enable_vcd_extensions);
    fstReaderSetLimitTimeRange(reader, opts.start_time, opts.end_time);

    if (!fstReaderProcessHier(reader, out)) {
        fprintf(stderr, "error: failed to emit hierarchy/header\n");
        goto done;
    }

    fstReaderSetFacProcessMaskAll(reader);

    if (!fstReaderIterBlocks(reader, NULL, NULL, out)) {
        fprintf(stderr, "error: failed while iterating value-change blocks\n");
        goto done;
    }

    if (ferror(out)) {
        fprintf(stderr, "error: write failed for output VCD: %s\n", opts.output_path);
        goto done;
    }

    rc = 0;

done:
    if (out) {
        fclose(out);
    }
    if (reader) {
        fstReaderClose(reader);
    }
    return rc;
}
