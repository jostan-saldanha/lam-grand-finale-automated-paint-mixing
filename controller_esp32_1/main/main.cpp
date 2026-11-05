// LAM Research Challenge 2026 - Stage 3 - Day 1 (Industrial Paint Mixing)
// ESP32 #1 (U2) = CONTROLLER NODE (Native ESP-IDF v5.x)
//   Drives 4 pumps (Cytron MDD10A via LEDC PWM), 4-6 solenoid valves via GPIO,
//   reads start switch and float switches, receives flow-pulse totals from ESP32 #2,
//   and executes the automated Mustard batch recipe (Yellow 80%, Blue 10%, Red 10%).

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"

static const char *TAG = "CONTROLLER";

static LineCfg     cfg[NUM_LINES];
static volatile bool gBusy = false;
static volatile bool gAbort = false;
static volatile bool gStartReq = false;
static const char*   gFault = NULL;

static uint32_t cnt[5] = {0, 0, 0, 0, 0};     // pulse totals from ESP32 #2
static uint32_t gFsw = 0;
static uint32_t lastFrameMs = 0;
static uint32_t goodFrames = 0;
static uint32_t badFrames = 0;
static bool     linkSeen = false;

static float pumpPct[5]  = {0, 0, 0, 0, 0};
static bool  valveOn[6]  = {false, false, false, false, false, false};

static double   calN[NUM_LINES] = {0, 0, 0, 0};
static double   calV[NUM_LINES] = {0, 0, 0, 0};
static int      calCount[NUM_LINES] = {0, 0, 0, 0};
static int      lastRunLine = -1;
static uint32_t lastRunPulses = 0;
static bool     gStreamTelemetry = true;

static inline uint32_t millis() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// ---------------------------------------------------------------------------
// Hardware Outputs: Valves & Pumps
// ---------------------------------------------------------------------------
static void setValve(int idx, bool on) {
    if (idx < 0 || idx > 5) return;
    if (idx == LINE_SV[L_FLUSH]) on = false; // Hard-disable flush solenoid (SV4)
    valveOn[idx] = on;
    if (PIN_SV[idx] >= 0) {
        gpio_set_level((gpio_num_t)PIN_SV[idx], on ? 1 : 0);
    }
}

static void allValvesOff() {
    for (int i = 0; i < 6; i++) setValve(i, false);
}

static void setPumpPct(int ch, float pct) {
    if (ch < 0 || ch > 4) return;
    if (ch == LINE_PUMP[L_FLUSH]) pct = 0.0f; // Hard-disable flush pump (Motor Driver 2 M2 / PWM_4)
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    pumpPct[ch] = pct;

    if (PIN_PWM[ch] >= 0) {
        uint32_t duty = (uint32_t)(pct * 255.0f / 100.0f + 0.5f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
    }
}

static void allOff() {
    for (int i = 0; i < 5; i++) setPumpPct(i, 0.0f);
    allValvesOff();
}

static void fault(const char* msg) {
    if (gFault) return;
    gFault = msg;
    gAbort = true;
    allOff();
    printf("\n[FAULT] %s\n", msg);
}

static bool waitMs(uint32_t ms) {
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (gAbort) return false;
        uint8_t ch;
        int len = uart_read_bytes(UART_NUM_0, &ch, 1, 0);
        if (len > 0) {
            if (ch == 'x' || ch == 'X' || ch == 27 || ch == 3) {
                gAbort = true;
                allOff();
                printf("\n[EMERGENCY STOP] '%c' received during dose -> Aborted!\n", ch);
                return false;
            }
        }
        // Delay at least 1 FreeRTOS tick so IDLE task runs and resets Task Watchdog (TWDT)
        vTaskDelay(pdMS_TO_TICKS(10) > 0 ? pdMS_TO_TICKS(10) : 1);
    }
    return !gAbort;
}

static bool rampPump(int ch, float from, float to, uint32_t ms) {
    uint32_t steps = ms / 20;
    if (steps < 1) steps = 1;
    for (uint32_t i = 1; i <= steps; i++) {
        setPumpPct(ch, from + (to - from) * (float)i / (float)steps);
        if (!waitMs(20)) return false;
    }
    return true;
}

static bool hwReady() {
    bool ok = true;
    for (int i = 0; i < NUM_LINES; i++) {
        if (!LINE_ENABLED[i]) continue; // Skip disabled lines (Flush)
        if (PIN_PWM[LINE_PUMP[i]] < 0) { printf("[PINS] PWM_%d not set\n", LINE_PUMP[i] + 1); ok = false; }
        if (PIN_SV[LINE_SV[i]] < 0)    { printf("[PINS] SV%d not set\n", LINE_SV[i] + 1); ok = false; }
    }
    if (!linkSeen || millis() - lastFrameMs > LINK_TIMEOUT_MS) {
        printf("[LINK] no data from ESP32 #2\n");
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------------
// Dosing Engine
// ---------------------------------------------------------------------------
static bool beginBusy() {
    if (!hwReady()) return false;
    gBusy = true;
    gAbort = false;
    gFault = NULL;
    return true;
}

static void endBusy(bool ok) {
    allOff();
    gBusy = false;
    if (!ok) {
        printf("%s\n", gFault ? "[STOPPED] fault, all outputs off" : "[STOPPED] aborted by operator, all outputs off");
    }
}

static bool doseLine(int line, float targetMl, bool precise, DoseResult &r) {
    if (line == L_FLUSH || !LINE_ENABLED[line]) {
        printf("[ERROR] Line %s is disabled (flush tank is not used)\n", LINE_NAME[line]);
        return false;
    }
    const LineCfg &c = cfg[line];
    const int ft = LINE_FT[line], pump = LINE_PUMP[line], sv = LINE_SV[line];
    r = DoseResult();
    r.targetMl = targetMl;
    printf("[DOSE] %s -> Mixing Tank: target %.1f mL (K=%.3f p/mL, fast %.0f%%, slow %.0f%%, lead %.1f mL)\n",
           LINE_NAME[line], targetMl, c.kPpm, c.dutyFast, c.dutySlow, precise ? c.leadMl : 0.0f);

    allValvesOff();
    setValve(sv, true);
    if (!waitMs(SV_OPEN_LEAD_MS)) return false;

    const uint32_t base = cnt[ft];
    const uint32_t t0 = millis();
    auto delivered = [&]() -> float { return (float)(uint32_t)(cnt[ft] - base) / c.kPpm; };

    float slowZone = fmaxf(targetMl * c.slowFrac, SLOW_MIN_ML);
    if (slowZone > targetMl * 0.5f) slowZone = targetMl * 0.5f;
    const float switchMl = targetMl - slowZone;
    float stopMl = targetMl - (precise ? c.leadMl : 0.0f);
    if (stopMl < switchMl) stopMl = switchMl;

    uint32_t lastPrint = millis(), lastProgMs = millis();
    float lastProgMl = 0.0f, lastPrintMl = 0.0f;

    auto tick = [&](int phase) -> bool {
        if (!waitMs(5)) return false;
        const uint32_t now = millis();
        const float d = delivered();
        if (d >= lastProgMl + 0.3f) { lastProgMl = d; lastProgMs = now; }
        else if (now - lastProgMs > NOFLOW_FAULT_MS) {
            fault("no flow: tank empty, air lock, or pump/sensor not working");
            return false;
        }
        if (now - t0 > DOSE_TIMEOUT_MS) { fault("dose timeout"); return false; }
        if (now - lastPrint >= 200) {
            const float flow = (d - lastPrintMl) / ((now - lastPrint) / 1000.0f) * 0.06f;   // mL/s -> L/min
            lastPrint = now;
            lastPrintMl = d;
            printf("/*%.2f,%d,%d,%.1f,%.1f,%.0f,%.2f,%lu,%lu,%lu,%lu*/\n",
                   now / 1000.0f, line, phase, targetMl, d, pumpPct[pump], flow,
                   (unsigned long)cnt[0], (unsigned long)cnt[1], (unsigned long)cnt[2], (unsigned long)cnt[3]);
        }
        return true;
    };

    // Fast Phase
    if (stopMl > switchMl) {
        if (!rampPump(pump, 0.0f, c.dutyFast, START_RAMP_MS)) return false;
        while (delivered() < switchMl) {
            if (!tick(1)) return false;
        }
    }

    // Slow Phase
    setPumpPct(pump, c.dutySlow);
    uint32_t rT = millis();
    float rMl = delivered(), rate = 0.0f;
    while (delivered() < stopMl) {
        if (!tick(2)) return false;
        const uint32_t now = millis();
        if (now - rT >= 250) {
            const float d = delivered();
            rate = (d - rMl) / ((now - rT) / 1000.0f);
            rT = now;
            rMl = d;
        }
    }

    // Shut off pump; keep valve open to count coasting pulses
    setPumpPct(pump, 0.0f);
    r.stopMl = delivered();
    if (!waitMs(SETTLE_MS)) return false;
    float d = delivered();

    // Top-up bursts (from below only)
    if (precise) {
        for (int k = 0; k < TOPUP_MAX && rate > 0.5f; k++) {
            const float rem = targetMl - d;
            if (rem < TOPUP_TRIGGER_ML) break;
            uint32_t burst = (uint32_t)(TOPUP_AIM * rem / rate * 1000.0f);
            if (burst < TOPUP_MIN_BURST_MS) break;
            if (burst > TOPUP_MAX_BURST_MS) burst = TOPUP_MAX_BURST_MS;
            setPumpPct(pump, c.dutySlow);
            if (!waitMs(burst)) return false;
            setPumpPct(pump, 0.0f);
            if (!waitMs(TOPUP_SETTLE_MS)) return false;
            d = delivered();
            printf("[TOPUP] %d: burst %lu ms -> %.1f mL\n", k + 1, (unsigned long)burst, d);
        }
    }

    setValve(sv, false);
    r.deliveredMl = d;
    r.coastMl = d - r.stopMl;
    r.pulses = (uint32_t)(cnt[ft] - base);
    r.secs = (millis() - t0) / 1000.0f;
    lastRunLine = line;
    lastRunPulses = r.pulses;
    printf("[DONE] %s: target %.1f mL, counted %.1f mL (%+.2f%%), pulses %lu, coast %.1f mL, %.1f s\n",
           LINE_NAME[line], targetMl, d, targetMl > 0 ? (d - targetMl) / targetMl * 100.0f : 0.0f,
           (unsigned long)r.pulses, r.coastMl, r.secs);
    return true;
}

// ---------------------------------------------------------------------------
// Parallel Multi-Line Closed-Loop Dosing Engine
// ---------------------------------------------------------------------------
struct ParallelState {
    int line;
    int pump;
    int sv;
    int ft;
    float targetMl;
    float switchMl;
    float stopMl;
    float deliveredMl;
    uint32_t base;
    uint32_t stopMs;
    uint32_t lastProgMs;
    float lastProgMl;
    int phase; // 1 = Fast, 2 = Slow, 3 = Coast/Settle, 4 = Done
    bool done;
    DoseResult res;
};

static bool doseParallel(const int lines[], const float targets[], int nLines, DoseResult results[]) {
    ParallelState st[NUM_LINES];
    int activeCount = 0;

    for (int i = 0; i < nLines; i++) {
        int l = lines[i];
        if (l == L_FLUSH || !LINE_ENABLED[l]) {
            printf("[PARALLEL] Skipping disabled line %s\n", LINE_NAME[l]);
            continue;
        }
        st[activeCount].line = l;
        st[activeCount].pump = LINE_PUMP[l];
        st[activeCount].sv   = LINE_SV[l];
        st[activeCount].ft   = LINE_FT[l];
        st[activeCount].targetMl = targets[i];
        const LineCfg &c = cfg[l];

        float slowZone = fmaxf(targets[i] * c.slowFrac, SLOW_MIN_ML);
        if (slowZone > targets[i] * 0.5f) slowZone = targets[i] * 0.5f;
        st[activeCount].switchMl = targets[i] - slowZone;
        st[activeCount].stopMl   = targets[i] - (c.leadMl > 0 ? c.leadMl : 0.0f);
        if (st[activeCount].stopMl < st[activeCount].switchMl) st[activeCount].stopMl = st[activeCount].switchMl;

        st[activeCount].deliveredMl = 0.0f;
        st[activeCount].phase = 1;
        st[activeCount].done = false;
        st[activeCount].res = DoseResult();
        st[activeCount].res.targetMl = targets[i];

        printf("[PARALLEL] %s -> Target: %.1f mL (fast=%.0f%%, slow=%.0f%%, lead=%.1f mL)\n",
               LINE_NAME[l], targets[i], c.dutyFast, c.dutySlow, c.leadMl);
        activeCount++;
    }

    if (activeCount == 0) return true;

    // 1. Open all active solenoid valves together
    for (int i = 0; i < activeCount; i++) {
        setValve(st[i].sv, true);
    }
    if (!waitMs(SV_OPEN_LEAD_MS)) return false;

    // 2. Capture baseline pulse counters
    uint32_t t0 = millis();
    for (int i = 0; i < activeCount; i++) {
        st[i].base = cnt[st[i].ft];
        st[i].lastProgMs = t0;
        st[i].lastProgMl = 0.0f;
    }

    // 3. Ramp all active pumps up together
    uint32_t rampSteps = START_RAMP_MS / 20;
    if (rampSteps < 1) rampSteps = 1;
    for (uint32_t step = 1; step <= rampSteps; step++) {
        for (int i = 0; i < activeCount; i++) {
            float fastDuty = cfg[st[i].line].dutyFast;
            setPumpPct(st[i].pump, fastDuty * (float)step / (float)rampSteps);
        }
        if (!waitMs(20)) return false;
    }

    // 4. Parallel closed-loop dosing execution
    uint32_t lastPrint = millis(), lastStatus = millis();
    int running = activeCount;

    while (running > 0) {
        if (!waitMs(5)) return false;
        uint32_t now = millis();

        for (int i = 0; i < activeCount; i++) {
            if (st[i].done) continue;
            int l = st[i].line;
            const LineCfg &c = cfg[l];

            float d = (float)(uint32_t)(cnt[st[i].ft] - st[i].base) / c.kPpm;
            st[i].deliveredMl = d;

            // No-flow timeout check (while pump running)
            if (st[i].phase == 1 || st[i].phase == 2) {
                if (d >= st[i].lastProgMl + 0.3f) {
                    st[i].lastProgMl = d;
                    st[i].lastProgMs = now;
                } else if (now - st[i].lastProgMs > NOFLOW_FAULT_MS) {
                    char msg[64];
                    snprintf(msg, sizeof(msg), "no flow on %s: check tank, pump, or sensor", LINE_NAME[l]);
                    fault(msg);
                    return false;
                }
            }

            // Phase 1 -> Phase 2 (Fast -> Slow)
            if (st[i].phase == 1 && d >= st[i].switchMl) {
                st[i].phase = 2;
                setPumpPct(st[i].pump, c.dutySlow);
            }

            // Phase 2 -> Phase 3 (Pump OFF immediately when threshold is reached)
            if ((st[i].phase == 1 || st[i].phase == 2) && d >= st[i].stopMl) {
                setPumpPct(st[i].pump, 0.0f);
                st[i].res.stopMl = d;
                st[i].stopMs = now;
                st[i].phase = 3;
            }

            // Phase 3 -> Phase 4 (Settle elapsed -> Valve CLOSED, Line Done!)
            if (st[i].phase == 3 && (now - st[i].stopMs >= SETTLE_MS)) {
                setValve(st[i].sv, false);
                st[i].phase = 4;
                st[i].done = true;
                running--;
                st[i].res.deliveredMl = d;
                st[i].res.coastMl = d - st[i].res.stopMl;
                st[i].res.pulses = (uint32_t)(cnt[st[i].ft] - st[i].base);
                st[i].res.secs = (now - t0) / 1000.0f;
                printf("[THRESHOLD REACHED] %s: target %.1f mL -> delivered %.1f mL (Pump OFF, SV%d CLOSED)\n",
                       LINE_NAME[l], st[i].targetMl, d, st[i].sv + 1);
            }
        }

        // Live telemetry stream to Serial Studio
        if (now - lastPrint >= 200) {
            float dt = (now - lastPrint) / 1000.0f;
            lastPrint = now;
            float totDelivered = 0.0f;
            float maxDuty = 0.0f;
            for (int i = 0; i < activeCount; i++) {
                totDelivered += st[i].deliveredMl;
                if (!st[i].done && pumpPct[st[i].pump] > maxDuty) {
                    maxDuty = pumpPct[st[i].pump];
                }
            }
            static float lastParallelDelivered = 0.0f;
            float flowLpm = (dt > 0.0f) ? (totDelivered - lastParallelDelivered) / dt * 0.06f : 0.0f;
            lastParallelDelivered = totDelivered;

            // Target and delivered volume representation:
            // When dosing 2 lines (e.g. Red & Blue 300 mL): gauge shows 300 mL target and per-line delivered volume reaching 300 mL
            // When dosing full 3-line batch: gauge shows 3000 mL batch total
            float targetDisp = (activeCount == 2) ? st[0].targetMl : BATCH_ML;
            float deliveredDisp = (activeCount == 2) ? fmaxf(st[0].deliveredMl, st[1].deliveredMl) : totDelivered;

            printf("/*%.2f,-2,1,%.1f,%.1f,%.0f,%.2f,%lu,%lu,%lu,%lu*/\n",
                   now / 1000.0f, targetDisp, deliveredDisp, maxDuty, flowLpm,
                   (unsigned long)cnt[0], (unsigned long)cnt[1], (unsigned long)cnt[2], (unsigned long)cnt[3]);
        }

        // Live console status line once per second
        if (now - lastStatus >= 1000) {
            lastStatus = now;
            printf("[PARALLEL]");
            for (int i = 0; i < activeCount; i++) {
                printf("  %s: %.1f/%.1f mL (%s)",
                       LINE_NAME[st[i].line], st[i].deliveredMl, st[i].targetMl,
                       st[i].done ? "OFF" : (st[i].phase == 1 ? "FAST" : (st[i].phase == 2 ? "SLOW" : "SETTLE")));
            }
            printf("\n");
        }
    }

    for (int i = 0; i < activeCount; i++) {
        results[i] = st[i].res;
    }
    return true;
}

static bool runBatch(bool twoStage = false) {
    if (!beginBusy()) return false;
    printf("\n==============================================================\n");
    printf("  [BATCH] Starting automated 3,000 mL Mustard Batch\n");
    printf("  Targets: Yellow = 2,400 mL (80%%), Blue = 300 mL (10%%), Red = 300 mL (10%%)\n");
    printf("  Mode: %s\n", twoStage ? "Two-Stage (Yellow first, then Red & Blue together)" : "All 3 Lines Parallel");
    printf("  Flush Pump: DISABLED (Motor Driver 2 M2 offline)\n");
    printf("==============================================================\n");

    bool ok = true;
    if (twoStage) {
        printf("\n--- STAGE 1: Dosing Yellow (2,400.0 mL) ---\n");
        DoseResult resY;
        ok = doseLine(L_YELLOW, BATCH_ML * PCT[L_YELLOW] / 100.0f, true, resY);

        if (ok) {
            printf("\n--- STAGE 2: Dosing Red & Blue Simultaneously at Equal PWM (300.0 mL each) ---\n");
            int lines[2] = {L_RED, L_BLUE};
            float targets[2] = {BATCH_ML * PCT[L_RED] / 100.0f, BATCH_ML * PCT[L_BLUE] / 100.0f};
            DoseResult resRB[2];
            ok = doseParallel(lines, targets, 2, resRB);

            if (ok) {
                printf("\n================ [BATCH COMPLETE SUMMARY] ================\n");
                printf("  %-7s target %7.1f mL  delivered %7.1f mL  (%+.2f%%)\n", "YELLOW", resY.targetMl, resY.deliveredMl,
                       (resY.deliveredMl - resY.targetMl) / resY.targetMl * 100.0f);
                printf("  %-7s target %7.1f mL  delivered %7.1f mL  (%+.2f%%)\n", "RED", resRB[0].targetMl, resRB[0].deliveredMl,
                       (resRB[0].deliveredMl - resRB[0].targetMl) / resRB[0].targetMl * 100.0f);
                printf("  %-7s target %7.1f mL  delivered %7.1f mL  (%+.2f%%)\n", "BLUE", resRB[1].targetMl, resRB[1].deliveredMl,
                       (resRB[1].deliveredMl - resRB[1].targetMl) / resRB[1].targetMl * 100.0f);
                float tot = resY.deliveredMl + resRB[0].deliveredMl + resRB[1].deliveredMl;
                printf("  TOTAL   target %7.1f mL  delivered %7.1f mL  (%+.2f%%)\n", BATCH_ML, tot, (tot - BATCH_ML) / BATCH_ML * 100.0f);
                printf("==========================================================\n");
                printf("[SYSTEM] All pumps off, all solenoids closed. Process stopped automatically.\n\n");
            }
        }
    } else {
        int lines[3] = {L_RED, L_BLUE, L_YELLOW};
        float targets[3] = {
            BATCH_ML * PCT[L_RED] / 100.0f,
            BATCH_ML * PCT[L_BLUE] / 100.0f,
            BATCH_ML * PCT[L_YELLOW] / 100.0f
        };
        DoseResult res[3];
        ok = doseParallel(lines, targets, 3, res);

        if (ok) {
            printf("\n================ [BATCH COMPLETE SUMMARY] ================\n");
            float tot = 0.0f;
            for (int i = 0; i < 3; i++) {
                int l = lines[i];
                printf("  %-7s target %7.1f mL  delivered %7.1f mL  (%+.2f%%)\n",
                       LINE_NAME[l], res[i].targetMl, res[i].deliveredMl,
                       (res[i].deliveredMl - res[i].targetMl) / res[i].targetMl * 100.0f);
                tot += res[i].deliveredMl;
            }
            printf("  TOTAL   target %7.1f mL  delivered %7.1f mL  (%+.2f%%)\n", BATCH_ML, tot, (tot - BATCH_ML) / BATCH_ML * 100.0f);
            printf("==========================================================\n");
            printf("[SYSTEM] All pumps off, all solenoids closed. Process stopped automatically.\n\n");
        }
    }
    endBusy(ok);
    return ok;
}

// ---------------------------------------------------------------------------
// Calibration Commands
// ---------------------------------------------------------------------------
static void cmdRun(int line, float duty, float sec) {
    if (line == L_FLUSH || !LINE_ENABLED[line]) {
        printf("[ERROR] Line %s is disabled (flush tank is not used)\n", LINE_NAME[line]);
        return;
    }
    if (!beginBusy()) return;
    const int ft = LINE_FT[line], pump = LINE_PUMP[line], sv = LINE_SV[line];
    allValvesOff();
    setValve(sv, true);
    bool ok = waitMs(SV_OPEN_LEAD_MS);
    const uint32_t base = cnt[ft];
    const uint32_t t0 = millis();
    if (ok) ok = rampPump(pump, 0.0f, duty, START_RAMP_MS);
    if (ok) {
        uint32_t runStart = millis();
        uint32_t durationMs = (uint32_t)(sec * 1000.0f);
        uint32_t lastTelem = millis();
        float lastRunMl = 0.0f;
        while (millis() - runStart < durationMs) {
            if (!waitMs(5)) { ok = false; break; }
            uint32_t now = millis();
            if (now - lastTelem >= 200) {
                float dt = (now - lastTelem) / 1000.0f;
                lastTelem = now;
                float curMl = (float)(uint32_t)(cnt[ft] - base) / cfg[line].kPpm;
                float flowLpm = (dt > 0.0f) ? (curMl - lastRunMl) / dt * 0.06f : 0.0f;
                lastRunMl = curMl;
                printf("/*%.2f,%d,1,%.1f,%.1f,%.0f,%.2f,%lu,%lu,%lu,%lu*/\n",
                       now / 1000.0f, line, sec * 10.0f, curMl, duty, flowLpm,
                       (unsigned long)cnt[0], (unsigned long)cnt[1], (unsigned long)cnt[2], (unsigned long)cnt[3]);
            }
        }
    }
    setPumpPct(pump, 0.0f);
    if (ok) ok = waitMs(SETTLE_MS);
    setValve(sv, false);
    if (ok) {
        const uint32_t p = cnt[ft] - base;
        const float s = (millis() - t0) / 1000.0f;
        lastRunLine = line;
        lastRunPulses = p;
        printf("[RUN] %s duty %.0f%%: pulses %lu in %.1f s = %.1f Hz, %.1f mL at K=%.3f\n",
               LINE_NAME[line], duty, (unsigned long)p, s, p / s, p / cfg[line].kPpm, cfg[line].kPpm);
        printf("[RUN] collect the liquid in measuring jar, weigh it, then: meas %s <actual_mL>\n", LINE_NAME[line]);
    }
    endBusy(ok);
}

static void cmdDeadband(int line) {
    if (line == L_FLUSH || !LINE_ENABLED[line]) {
        printf("[ERROR] Line %s is disabled (flush tank is not used)\n", LINE_NAME[line]);
        return;
    }
    if (!beginBusy()) return;
    const int ft = LINE_FT[line], pump = LINE_PUMP[line], sv = LINE_SV[line];
    allValvesOff();
    setValve(sv, true);
    bool ok = waitMs(SV_OPEN_LEAD_MS);
    int startDuty = -1, stopDuty = -1;
    for (int d = 5; ok && d <= 100; d++) {
        setPumpPct(pump, (float)d);
        const uint32_t p0 = cnt[ft];
        ok = waitMs(500);
        if (ok && (uint32_t)(cnt[ft] - p0) >= 3) { startDuty = d; break; }
    }
    if (ok && startDuty > 0) {
        for (int d = startDuty; ok && d >= 1; d--) {
            setPumpPct(pump, (float)d);
            const uint32_t p0 = cnt[ft];
            ok = waitMs(500);
            if (ok && (uint32_t)(cnt[ft] - p0) < 2) { stopDuty = d; break; }
        }
        printf("[DEADBAND] %s: starts at %d%%, stops below %d%%. Keep slow duty well above %d%%.\n",
               LINE_NAME[line], startDuty, stopDuty, startDuty);
    } else if (ok) {
        printf("[DEADBAND] no flow detected up to 100%% - check priming, valve, sensor\n");
    }
    endBusy(ok);
}

static void cmdSweep(int line) {
    if (line == L_FLUSH || !LINE_ENABLED[line]) {
        printf("[ERROR] Line %s is disabled (flush tank is not used)\n", LINE_NAME[line]);
        return;
    }
    if (!beginBusy()) return;
    const int ft = LINE_FT[line], pump = LINE_PUMP[line], sv = LINE_SV[line];
    allValvesOff();
    setValve(sv, true);
    bool ok = waitMs(SV_OPEN_LEAD_MS);
    printf("[SWEEP] %s: duty%%, pulses/s, mL/s, L/min\n", LINE_NAME[line]);
    for (int duty = 30; ok && duty <= 100; duty += 10) {
        setPumpPct(pump, (float)duty);
        ok = waitMs(1500);
        const uint32_t p0 = cnt[ft];
        const uint32_t t0 = millis();
        ok = waitMs(2000);
        const float s = (millis() - t0) / 1000.0f;
        const float f = (cnt[ft] - p0) / s;
        const float mlps = f / cfg[line].kPpm;
        printf("[SWEEP] %d, %.1f, %.2f, %.3f\n", duty, f, mlps, mlps * 0.06f);
    }
    endBusy(ok);
}

static void cmdCoast(int line, float duty) {
    if (line == L_FLUSH || !LINE_ENABLED[line]) {
        printf("[ERROR] Line %s is disabled (flush tank is not used)\n", LINE_NAME[line]);
        return;
    }
    if (!beginBusy()) return;
    const int ft = LINE_FT[line], pump = LINE_PUMP[line], sv = LINE_SV[line];
    allValvesOff();
    setValve(sv, true);
    bool ok = waitMs(SV_OPEN_LEAD_MS);
    if (ok) ok = rampPump(pump, 0.0f, duty, START_RAMP_MS);
    if (ok) ok = waitMs(4000);
    const uint32_t pa = cnt[ft];
    setPumpPct(pump, 0.0f);
    const uint32_t tStop = millis();
    if (ok) ok = waitMs(SETTLE_MS);
    const uint32_t pb = cnt[ft];
    setValve(sv, false);
    if (ok) {
        const uint32_t coastP = pb - pa;
        const float coastMl = (float)coastP / cfg[line].kPpm;
        printf("[COAST] %s at %.0f%%: %lu pulses coasted in %.1f s = %.2f mL -> recommended lead is approx %.1f mL\n",
               LINE_NAME[line], duty, (unsigned long)coastP, (millis() - tStop) / 1000.0f, coastMl, coastMl);
    }
    endBusy(ok);
}

// ---------------------------------------------------------------------------
// NVS Storage
// ---------------------------------------------------------------------------
static void saveCfg() {
    nvs_handle_t h;
    if (nvs_open("lam_cfg", NVS_READWRITE, &h) == ESP_OK) {
        for (int i = 0; i < NUM_LINES; i++) {
            char k[16];
            snprintf(k, sizeof(k), "k%d", i);  nvs_set_blob(h, k, &cfg[i].kPpm, sizeof(float));
            snprintf(k, sizeof(k), "df%d", i); nvs_set_blob(h, k, &cfg[i].dutyFast, sizeof(float));
            snprintf(k, sizeof(k), "ds%d", i); nvs_set_blob(h, k, &cfg[i].dutySlow, sizeof(float));
            snprintf(k, sizeof(k), "ld%d", i); nvs_set_blob(h, k, &cfg[i].leadMl, sizeof(float));
            snprintf(k, sizeof(k), "sf%d", i); nvs_set_blob(h, k, &cfg[i].slowFrac, sizeof(float));
        }
        nvs_commit(h);
        nvs_close(h);
        printf("[CONFIG] saved to NVS flash\n");
    }
}

static void loadCfg() {
    for (int i = 0; i < NUM_LINES; i++) {
        cfg[i].kPpm     = DEFAULT_LINE_K[i];
        cfg[i].dutyFast = DEFAULT_LINE_FAST[i];
        cfg[i].dutySlow = DEFAULT_LINE_SLOW[i];
        cfg[i].leadMl   = DEFAULT_LINE_LEAD[i];
        cfg[i].slowFrac = DEFAULT_LINE_SLOWFRAC[i];
    }
    nvs_handle_t h;
    if (nvs_open("lam_cfg", NVS_READONLY, &h) == ESP_OK) {
        size_t sz = sizeof(float);
        for (int i = 0; i < NUM_LINES; i++) {
            char k[16];
            snprintf(k, sizeof(k), "k%d", i);  nvs_get_blob(h, k, &cfg[i].kPpm, &sz);
            snprintf(k, sizeof(k), "df%d", i); nvs_get_blob(h, k, &cfg[i].dutyFast, &sz);
            snprintf(k, sizeof(k), "ds%d", i); nvs_get_blob(h, k, &cfg[i].dutySlow, &sz);
            snprintf(k, sizeof(k), "ld%d", i); nvs_get_blob(h, k, &cfg[i].leadMl, &sz);
            snprintf(k, sizeof(k), "sf%d", i); nvs_get_blob(h, k, &cfg[i].slowFrac, &sz);
        }
        nvs_close(h);
    }
    bool needSave = false;
    for (int i = 0; i < NUM_LINES; i++) {
        if (cfg[i].dutyFast < 90.0f) { cfg[i].dutyFast = DEFAULT_LINE_FAST[i]; needSave = true; }
        if (cfg[i].dutySlow < 50.0f) { cfg[i].dutySlow = DEFAULT_LINE_SLOW[i]; needSave = true; }
        if (cfg[i].kPpm < 0.5f || cfg[i].kPpm > 3.0f) { cfg[i].kPpm = DEFAULT_LINE_K[i]; needSave = true; }
    }
    if (cfg[L_YELLOW].kPpm < 1.8f) { cfg[L_YELLOW].kPpm = DEFAULT_LINE_K[L_YELLOW]; needSave = true; }
    if (needSave) saveCfg();
}

// ---------------------------------------------------------------------------
// UART Link Parser from ESP32 #2
// ---------------------------------------------------------------------------
static void parseFrame(const char* s) {
    if (s[0] != '$') return;
    const char* star = strchr(s, '*');
    if (!star) return;
    uint8_t x = 0;
    for (const char* p = s + 1; p < star; p++) x ^= (uint8_t)*p;
    unsigned int rx = 0;
    if (sscanf(star + 1, "%2x", &rx) != 1 || rx != x) { badFrames++; return; }

    unsigned long seq, ms, c1, c2, c3, c4, c5, fsw;
    if (sscanf(s + 1, "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu", &seq, &ms, &c1, &c2, &c3, &c4, &c5, &fsw) != 8) {
        badFrames++;
        return;
    }
    cnt[0] = (uint32_t)c1; cnt[1] = (uint32_t)c2; cnt[2] = (uint32_t)c3;
    cnt[3] = (uint32_t)c4; cnt[4] = (uint32_t)c5;
    gFsw = (uint32_t)fsw;
    lastFrameMs = millis();
    linkSeen = true;
    goodFrames++;
}

static void uartRxTask(void *pv) {
    char buf[128];
    int n = 0;
    uint8_t ch;
    while (1) {
        int len = uart_read_bytes(LINK_UART_NUM, &ch, 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            if (ch == '\n') {
                buf[n] = 0;
                parseFrame(buf);
                n = 0;
            } else if (ch != '\r' && n < sizeof(buf) - 1) {
                buf[n++] = (char)ch;
            } else if (n >= sizeof(buf) - 1) {
                n = 0;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Start Switch Monitor Task
// ---------------------------------------------------------------------------
static void switchTask(void *pv) {
    if (PIN_SW < 0) {
        // Physical switch is disabled; task exits immediately
        vTaskDelete(NULL);
        return;
    }
    int lastState = 1;
    uint32_t lastChange = 0;
    while (1) {
        int cur = gpio_get_level((gpio_num_t)PIN_SW);
        if (cur != lastState && (millis() - lastChange > 50)) {
            lastChange = millis();
            lastState = cur;
            if (cur == 0) { // Pressed (Active LOW)
                if (gBusy) {
                    printf("\n[SWITCH] STOP button pressed!\n");
                    gAbort = true;
                } else {
                    printf("\n[SWITCH] START button pressed -> Launching Batch!\n");
                    gStartReq = true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------------------------------------------------------------------
// CLI Command Processor
// ---------------------------------------------------------------------------
static int parseLineArg(const char* s) {
    if (!s) return -1;
    if (s[0] == 'r' || s[0] == 'R') return L_RED;
    if (s[0] == 'b' || s[0] == 'B') return L_BLUE;
    if (s[0] == 'y' || s[0] == 'Y') return L_YELLOW;
    if (s[0] == 'f' || s[0] == 'F') return L_FLUSH;
    int i = atoi(s);
    if (i >= 0 && i < NUM_LINES) return i;
    return -1;
}

static void printHelp() {
    printf("\n---- LAM Paint Mixing Controller Commands ----\n");
    printf(" start                     start automated Mustard batch (All 3 lines parallel)\n");
    printf(" start 2step               start automated Mustard batch (Yellow first, then Red & Blue)\n");
    printf(" dose rb <mL>              dose Red and Blue concurrently at equal PWM (e.g. dose rb 300)\n");
    printf(" dose <line> <mL>          dose specified line to mixing tank (e.g. dose y 2400)\n");
    printf(" x / stop                  emergency stop everything immediately\n");
    printf(" status                    show link, pulse counts, and line configurations\n");
    printf(" run <line> <duty> <sec>   constant-duty run, reports pulses for calibration\n");
    printf(" meas <line> <actual_mL>   trim K-factor from weighed volume\n");
    printf(" deadband <line>           detect pump starting and stopping duty\n");
    printf(" sweep <line>              measure duty vs flow rate table\n");
    printf(" coast <line> <duty>       measure coast pulses and recommended lead\n");
    printf(" k <line> <pulses_per_mL>  set pulses per mL calibration factor\n");
    printf(" lead <line> <mL>          set early stop lead in mL\n");
    printf(" duty <line> <fast> <slow> set fast and slow PWM duties (%%)\n");
    printf(" slowfrac <line> <frac>    fraction of dose to run slow (0.05..0.40)\n");
    printf(" valve <1..6> <0|1>        manually toggle a solenoid valve\n");
    printf(" pump <1..5> <duty%%>      manually test pump duty\n");
    printf(" default                   restore TEAM_DISTRO empirical calibration\n");
    printf(" save                      save all settings to NVS flash\n");
    printf(" help                      print this menu\n");
    printf("----------------------------------------------\n");
}

static void printStatus() {
    printf("\n[STATUS] Link: %s (frames good=%lu bad=%lu, age=%lu ms)\n",
           (linkSeen && millis() - lastFrameMs < LINK_TIMEOUT_MS) ? "ONLINE" : "OFFLINE",
           (unsigned long)goodFrames, (unsigned long)badFrames,
           linkSeen ? (unsigned long)(millis() - lastFrameMs) : 0UL);
    printf("  Flow Pulses: FS1(Red)=%lu  FS2(Yellow)=%lu  FS3(Blue)=%lu  FS4(Flush)=%lu  FS5=%lu\n",
           (unsigned long)cnt[0], (unsigned long)cnt[1], (unsigned long)cnt[2],
           (unsigned long)cnt[3], (unsigned long)cnt[4]);
    printf("  P&ID Lines Config:\n");
    for (int i = 0; i < NUM_LINES; i++) {
        printf("    [%-6s] %s | K=%.4f p/mL | Fast=%.0f%% | Slow=%.0f%% | Lead=%.1f mL | SlowFrac=%.2f | Pump: PWM_%d | SV: SV%d | FT: FS%d (%lu cnt)\n",
               LINE_NAME[i], LINE_ENABLED[i] ? "ACTIVE  " : "DISABLED", cfg[i].kPpm, cfg[i].dutyFast, cfg[i].dutySlow, cfg[i].leadMl, cfg[i].slowFrac,
               LINE_PUMP[i] + 1, LINE_SV[i] + 1, LINE_FT[i] + 1, (unsigned long)cnt[LINE_FT[i]]);
    }
}

static void handleCommand(char* cmd) {
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (!*cmd) return;

    char *saveptr;
    char *tokens[8];
    int n = 0;
    char *tok = strtok_r(cmd, " \r\n", &saveptr);
    while (tok && n < 8) {
        tokens[n++] = tok;
        tok = strtok_r(NULL, " \r\n", &saveptr);
    }
    if (n == 0) return;

    const char* c = tokens[0];

    if (strcmp(c, "help") == 0) { printHelp(); return; }
    if (strcmp(c, "status") == 0) { printStatus(); return; }
    if (strcmp(c, "x") == 0 || strcmp(c, "stop") == 0) {
        gAbort = true;
        allOff();
        printf("[CMD] ALL OFF\n");
        return;
    }
    if (strcmp(c, "telem") == 0 && n >= 2) {
        gStreamTelemetry = (atoi(tokens[1]) > 0);
        printf("[TELEM] streaming: %s\n", gStreamTelemetry ? "ON" : "OFF");
        return;
    }
    if (strcmp(c, "default") == 0 || strcmp(c, "resetcal") == 0) {
        for (int i = 0; i < NUM_LINES; i++) {
            cfg[i].kPpm     = DEFAULT_LINE_K[i];
            cfg[i].dutyFast = DEFAULT_LINE_FAST[i];
            cfg[i].dutySlow = DEFAULT_LINE_SLOW[i];
            cfg[i].leadMl   = DEFAULT_LINE_LEAD[i];
            cfg[i].slowFrac = DEFAULT_LINE_SLOWFRAC[i];
        }
        saveCfg();
        printf("[CAL] Reset to TEAM_DISTRO empirical calibration and saved to NVS!\n");
        printStatus();
        return;
    }
    if (strcmp(c, "save") == 0) { saveCfg(); return; }

    if (strcmp(c, "batch") == 0 || strcmp(c, "start") == 0) {
        bool twoStage = (n >= 2 && (strcmp(tokens[1], "2step") == 0 || strcmp(tokens[1], "seq") == 0));
        runBatch(twoStage);
        return;
    }

    if (strcmp(c, "valve") == 0 && n >= 3) {
        int v = atoi(tokens[1]) - 1;
        if (v == LINE_SV[L_FLUSH]) {
            printf("[VALVE] SV%d is Flush Valve -> DISABLED (no flush tank in use)\n", v + 1);
            return;
        }
        int state = atoi(tokens[2]);
        setValve(v, state > 0);
        printf("[VALVE] SV%d = %s\n", v + 1, state > 0 ? "OPEN" : "CLOSED");
        return;
    }

    if (strcmp(c, "pump") == 0 && n >= 3) {
        int p = atoi(tokens[1]) - 1;
        if (p == LINE_PUMP[L_FLUSH]) {
            printf("[PUMP] PWM_%d is Flush Pump -> DISABLED (Motor Driver 2 M2 offline)\n", p + 1);
            return;
        }
        float d = atof(tokens[2]);
        setPumpPct(p, d);
        printf("[PUMP] PWM_%d = %.1f%%\n", p + 1, d);
        return;
    }

    if (strcmp(c, "dose") == 0 && n >= 3) {
        if (strcmp(tokens[1], "rb") == 0 || strcmp(tokens[1], "br") == 0) {
            float ml = atof(tokens[2]);
            if (ml <= 0) { printf("Usage: dose rb <mL>\n"); return; }
            if (!beginBusy()) return;
            int lines[2] = {L_RED, L_BLUE};
            float targets[2] = {ml, ml};
            DoseResult res[2];
            bool ok = doseParallel(lines, targets, 2, res);
            endBusy(ok);
            return;
        }
        int line = parseLineArg(tokens[1]);
        if (line < 0) {
            printf("Unknown line: %s (use r, b, y, rb or 0..2)\n", tokens[1]);
            return;
        }
        float ml = atof(tokens[2]);
        if (ml <= 0) { printf("Usage: dose <line> <mL>\n"); return; }
        if (!beginBusy()) return;
        DoseResult r;
        bool ok = doseLine(line, ml, true, r);
        endBusy(ok);
        if (ok) printf("[DOSE] Weigh the liquid, then type: meas %s <actual_mL>\n", LINE_NAME[line]);
        return;
    }

    int line = parseLineArg(tokens[1]);
    if (line < 0 && n > 1) {
        printf("Unknown line: %s (use r, b, y, f or 0..3)\n", tokens[1]);
        return;
    }

    if (strcmp(c, "run") == 0 && n >= 4) {
        cmdRun(line, atof(tokens[2]), atof(tokens[3]));
        return;
    }

    if (strcmp(c, "deadband") == 0) { cmdDeadband(line); return; }
    if (strcmp(c, "sweep") == 0)    { cmdSweep(line); return; }
    if (strcmp(c, "coast") == 0 && n >= 3) { cmdCoast(line, atof(tokens[2])); return; }

    if (strcmp(c, "meas") == 0 && n >= 3) {
        float actual = atof(tokens[2]);
        if (actual <= 0 || lastRunLine != line || lastRunPulses == 0) {
            printf("[CAL] Needs a fresh 'dose' or 'run' on that line first, and a volume > 0\n");
            return;
        }
        float kTrial = (float)lastRunPulses / actual;
        calN[line] += lastRunPulses;
        calV[line] += actual;
        calCount[line]++;
        float old = cfg[line].kPpm;
        cfg[line].kPpm = (float)(calN[line] / calV[line]);
        lastRunPulses = 0;
        printf("[CAL] %s trial %d: %.3f p/mL | pooled K=%.4f (was %.4f) | type 'save' to persist\n",
               LINE_NAME[line], calCount[line], kTrial, cfg[line].kPpm, old);
        return;
    }

    if (strcmp(c, "k") == 0 && n >= 3)         { cfg[line].kPpm = atof(tokens[2]); }
    else if (strcmp(c, "lead") == 0 && n >= 3) { cfg[line].leadMl = atof(tokens[2]); }
    else if (strcmp(c, "slowfrac") == 0 && n >= 3) { cfg[line].slowFrac = atof(tokens[2]); }
    else if (strcmp(c, "duty") == 0 && n >= 4) { cfg[line].dutyFast = atof(tokens[2]); cfg[line].dutySlow = atof(tokens[3]); }
    else { printf("Unknown command, type 'help'\n"); return; }

    printf("[CFG] %s: K=%.4f fast=%.0f slow=%.0f lead=%.1f (type 'save' to keep)\n",
           LINE_NAME[line], cfg[line].kPpm, cfg[line].dutyFast, cfg[line].dutySlow, cfg[line].leadMl);
}

static void consoleTask(void *pv) {
    char lineBuf[128];
    int pos = 0;
    uint32_t lastStreamMs = 0;

    while (1) {
        if (gStartReq && !gBusy) {
            gStartReq = false;
            char bCmd[] = "batch";
            handleCommand(bCmd);
        }

        // Stream idle/manual telemetry for Serial Studio (air-blow test & monitoring)
        if (gStreamTelemetry && !gBusy) {
            uint32_t now = millis();
            if (now - lastStreamMs >= 200) {
                lastStreamMs = now;
                float maxActiveDuty = 0.0f;
                int activeLine = -1;
                for (int i = 0; i < NUM_LINES; i++) {
                    int p = LINE_PUMP[i];
                    if (pumpPct[p] > maxActiveDuty) {
                        maxActiveDuty = pumpPct[p];
                        activeLine = i;
                    }
                }
                printf("/*%.2f,%d,0,0.0,0.0,%.0f,0.00,%lu,%lu,%lu,%lu*/\n",
                       now / 1000.0f, activeLine, maxActiveDuty,
                       (unsigned long)cnt[0], (unsigned long)cnt[1], (unsigned long)cnt[2], (unsigned long)cnt[3]);
            }
        }

        uint8_t ch;
        int len = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            if (ch == 'x' || ch == 'X' || ch == 27 || ch == 3) {
                gAbort = true;
                allOff();
                pos = 0;
                printf("\n[EMERGENCY STOP] '%c' received -> All pumps & valves OFF!\n", ch);
            } else if (ch == '\r' || ch == '\n') {
                if (pos > 0) {
                    lineBuf[pos] = 0;
                    handleCommand(lineBuf);
                    pos = 0;
                }
            } else if (ch == '\b' || ch == 127) {
                if (pos > 0) pos--;
            } else if (pos < sizeof(lineBuf) - 1) {
                lineBuf[pos++] = (char)ch;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Hardware Initialization
// ---------------------------------------------------------------------------
static void initHardware() {
    // 1. Valves GPIO outputs
    uint64_t out_mask = 0;
    for (int i = 0; i < 6; i++) {
        if (PIN_SV[i] >= 0) out_mask |= (1ULL << PIN_SV[i]);
    }
    gpio_config_t out_conf = {};
    out_conf.pin_bit_mask = out_mask;
    out_conf.mode = GPIO_MODE_OUTPUT;
    out_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    out_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&out_conf);
    allValvesOff();

    // 2. LEDC PWM for Pumps
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = LEDC_TIMER_8_BIT;
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.freq_hz = PWM_HZ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    for (int i = 0; i < 5; i++) {
        if (PIN_PWM[i] >= 0) {
            ledc_channel_config_t ch_conf = {};
            ch_conf.gpio_num = PIN_PWM[i];
            ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
            ch_conf.channel = (ledc_channel_t)i;
            ch_conf.intr_type = LEDC_INTR_DISABLE;
            ch_conf.timer_sel = LEDC_TIMER_0;
            ch_conf.duty = 0;
            ch_conf.hpoint = 0;
            ledc_channel_config(&ch_conf);
        }
    }

    // 3. Inputs: Start switch & Float switches
    uint64_t in_mask = 0;
    if (PIN_SW >= 0) in_mask |= (1ULL << PIN_SW);
    for (int i = 0; i < 4; i++) {
        if (PIN_FSW[i] >= 0) in_mask |= (1ULL << PIN_FSW[i]);
    }
    gpio_config_t in_conf = {};
    in_conf.pin_bit_mask = in_mask;
    in_conf.mode = GPIO_MODE_INPUT;
    in_conf.pull_up_en = GPIO_PULLUP_DISABLE;   // External pull-up on PCB
    in_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&in_conf);

    // 4. UART2 Link from ESP32 #2
    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = LINK_BAUD_RATE;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;
    uart_param_config(LINK_UART_NUM, &uart_cfg);
    uart_set_pin(LINK_UART_NUM, LINK_TX_PIN, LINK_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LINK_UART_NUM, 1024, 0, 0, NULL, 0);

    // 5. UART0 Console for CLI
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
}

extern "C" void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    initHardware();
    loadCfg();

    printf("\n==============================================================\n");
    printf("  LAM RESEARCH CHALLENGE 2026 - STAGE 3 - DAY 1: MUSTARD MIX\n");
    printf("  ESP32 #1 Controller Node (ESP-IDF v5.x). Type 'help'.\n");
    printf("==============================================================\n");
    printf("[READY] Press Start Switch (SW) or type 'batch' to begin.\n\n");

    xTaskCreate(uartRxTask, "uart_rx", 4096, NULL, 10, NULL);
    xTaskCreate(switchTask, "switch_mon", 2048, NULL, 5, NULL);
    xTaskCreate(consoleTask, "console_cli", 4096, NULL, 5, NULL);
}

