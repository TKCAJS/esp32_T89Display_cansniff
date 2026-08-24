Import("env")
import shutil, os, re

libdeps = os.path.join(env["PROJECT_LIBDEPS_DIR"], env["PIOENV"])

# Copy lv_conf.h next to the lvgl folder where LVGL expects it
src  = os.path.join(env["PROJECT_DIR"], "include", "lv_conf.h")
dest = os.path.join(libdeps, "lv_conf.h")
if os.path.isfile(src):
    shutil.copy(src, dest)
    print("lv_conf.h -> libdeps/")

# Patch lvgl_panel_st7262_par.c:
#  - remove struct members that were removed in newer ESP-IDF
#    (on_frame_trans_done, user_ctx, flags.relax_on_idle)
#  - JITTER-T89-FIX3: add DRAM bounce buffers (2x 10 lines). Without them the
#    RGB peripheral DMA-reads the framebuffer straight out of PSRAM; any
#    bandwidth stall underruns the panel FIFO and the picture shifts sideways
#    until the next full flush resyncs it (visible as random jitter on
#    otherwise-static screens). See ESP-IDF RGB LCD docs, "bounce buffer mode".
panel_file = os.path.join(libdeps, "esp32_smartdisplay", "src", "lvgl_panel_st7262_par.c")
if os.path.isfile(panel_file):
    with open(panel_file, "r") as f:
        src = f.read()
    patched = src
    patched = re.sub(r'\s*\.on_frame_trans_done\s*=\s*\w+,\n', '\n', patched)
    patched = re.sub(r'\s*\.user_ctx\s*=\s*\w+,\n', '\n', patched)
    patched = re.sub(r'\s*\.relax_on_idle\s*=\s*\w+,', '', patched)
    patched = re.sub(r'log_d\("rgb_panel_config:.*?\);', 'log_d("rgb_panel_config initialized");', patched, flags=re.DOTALL)

    # Drop any previously inserted FIX3 line first so re-running (or changing
    # the line count) can't leave a stale duplicate initializer behind.
    patched = re.sub(r'[ \t]*\.bounce_buffer_size_px =[^\n]*JITTER-T89-FIX3[^\n]*\n', '', patched)
    patched, n3 = re.subn(
        r'(\.clk_src = ST7262_PANEL_CONFIG_CLK_SRC,\n)',
        r'\1        .bounce_buffer_size_px = DISPLAY_WIDTH * 20, /* JITTER-T89-FIX3 */\n',
        patched)
    if n3:
        print("lvgl_panel_st7262_par.c: Fix 3 (RGB bounce buffers) applied")
    elif "JITTER-T89-FIX3" not in patched:
        print("WARNING: lvgl_panel_st7262_par.c Fix 3 anchor not found - upstream changed?")

    if patched != src:
        with open(panel_file, "w") as f:
            f.write(patched)
        print("lvgl_panel_st7262_par.c patched for current ESP-IDF API")

# Patch esp32_smartdisplay_dma.c: two display-flush performance fixes (T89).
#  Fix 1 — drop the per-chunk vTaskDelay(1) in smartdisplay_dma_transfer_chunk.
#          A full-screen flush is ~6 chunks; the delay added ~6ms of pure sleep
#          to every full repaint for no benefit (it just memcpys into the fb).
#  Fix 2 — let smartdisplay_dma_copy_to_buffer pass a PSRAM source straight to
#          the panel instead of bounce-copying through the internal scratch
#          buffer. The LVGL draw buffer is MALLOC_CAP_SPIRAM, which
#          esp_ptr_dma_capable() rejects -> 2x PSRAM bandwidth per pixel.
# Done as a pre-build patch (not a fork/vendor) so the project keeps tracking
# upstream lib_deps; it re-runs every build and survives a .pio wipe. Idempotent.
dma_file = os.path.join(libdeps, "esp32_smartdisplay", "src", "esp32_smartdisplay_dma.c")
if os.path.isfile(dma_file):
    with open(dma_file, "r") as f:
        src = f.read()
    patched = src

    patched, n1 = re.subn(
        r'[ \t]*// Small delay to prevent overwhelming the system\n[ \t]*vTaskDelay\(1\);\n',
        '        // PERF-T89-FIX1: removed per-chunk vTaskDelay(1)\n',
        patched)
    if n1:
        print("esp32_smartdisplay_dma.c: Fix 1 (per-chunk vTaskDelay) applied")
    elif "PERF-T89-FIX1" not in patched:
        print("WARNING: esp32_smartdisplay_dma.c Fix 1 anchor not found - upstream changed?")

    patched, n2 = re.subn(
        r'if \(esp_ptr_dma_capable\(src\)\)',
        'if (esp_ptr_dma_capable(src) || esp_ptr_external_ram(src)) /* PERF-T89-FIX2 */',
        patched)
    if n2:
        print("esp32_smartdisplay_dma.c: Fix 2 (PSRAM passthrough) applied")
    elif "PERF-T89-FIX2" not in patched:
        print("WARNING: esp32_smartdisplay_dma.c Fix 2 anchor not found - upstream changed?")

    if patched != src:
        with open(dma_file, "w") as f:
            f.write(patched)
