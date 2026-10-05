#include <android/log.h>
#include <cstring>
#include <cmath>
#include <atomic>
#include <mutex>
#include <cstdint>

#define LOG_TAG "aimlock_head"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define EXPORT extern "C" __attribute__((visibility("default")))

// ===== KALMAN 2D =====
struct Kalman2D {
    double x, y, vx, vy;
    double p00, p11, p22, p33;
    double q, r;
    bool init;

    Kalman2D() : x(0), y(0), vx(0), vy(0),
                 p00(100), p11(100), p22(100), p33(100),
                 q(0.02), r(6.0), init(false) {}

    void reset(double mx, double my) {
        x = mx; y = my; vx = 0; vy = 0; init = true;
        p00 = p22 = 50.0;
        p11 = p33 = 50.0;
    }

    void predict() {
        if (!init) return;
        x += vx; y += vy;
        p00 += p11 + q;
        p11 += q;
        p22 += p33 + q;
        p33 += q;
    }

    void update(double mx, double my) {
        if (!init) { reset(mx, my); return; }
        double kx = p00 / (p00 + r);
        double ky = p22 / (p22 + r);
        double ex = mx - x, ey = my - y;
        x += kx * ex; y += ky * ey;
        vx += kx * ex * 0.5;
        vy += ky * ey * 0.5;
        p00 *= (1 - kx);
        p22 *= (1 - ky);
        p11 *= 0.95;
        p33 *= 0.95;
    }
};

// ===== TRẠNG THÁI =====
static std::atomic<bool> g_running{false};
static std::mutex g_lock;

static int g_screenW = 1080, g_screenH = 2340;
static int g_fovX = 260, g_fovY = 260;
static int g_smooth = 5;
static double g_pidKp = 0.85, g_pidKi = 0.02, g_pidKd = 0.12;

static int g_rMin = 80,  g_rMax = 255;
static int g_gMin = 30,  g_gMax = 210;
static int g_bMin = 15,  g_bMax = 180;

static int g_lockX = -1, g_lockY = -1, g_lockW = 0, g_lockH = 0;
static int g_lostFrames = 0;
static const int MAX_LOST = 20;
static const double IOU_THRESHOLD = 0.25;

static Kalman2D g_kalman;
static double g_integralX = 0, g_integralY = 0;
static double g_prevErrX = 0, g_prevErrY = 0;

// ===== TIỆN ÍCH =====
static inline bool is_skin(int r, int g, int b) {
    if (r < g_rMin || r > g_rMax) return false;
    if (g < g_gMin || g > g_gMax) return false;
    if (b < g_bMin || b > g_bMax) return false;
    if (r <= g || r <= b) return false;
    int mx = g > b ? g : b;
    if (r - mx < 12) return false;
    int mn = g < b ? g : b;
    if (mx - mn < 15 && r > 200) return false;
    return true;
}

static inline double iou(int ax, int ay, int aw, int ah,
                         int bx, int by, int bw, int bh) {
    int x1 = ax > bx ? ax : bx;
    int y1 = ay > by ? ay : by;
    int x2 = (ax+aw) < (bx+bw) ? (ax+aw) : (bx+bw);
    int y2 = (ay+ah) < (by+bh) ? (ay+ah) : (by+bh);
    if (x2 <= x1 || y2 <= y1) return 0.0;
    double inter = (double)(x2-x1) * (y2-y1);
    double uni = (double)aw*ah + (double)bw*bh - inter;
    return uni > 0 ? inter / uni : 0.0;
}

static bool track_locked(const uint8_t* rgba, int stride,
                         int& outX, int& outY, int& outW, int& outH) {
    if (g_lockW <= 0 || g_lockH <= 0) return false;
    int pad = g_lockW / 2 > 30 ? g_lockW / 2 : 30;
    int x0 = g_lockX - pad; if (x0 < 0) x0 = 0;
    int y0 = g_lockY - pad; if (y0 < 0) y0 = 0;
    int x1 = g_lockX + g_lockW + pad; if (x1 > g_screenW) x1 = g_screenW;
    int y1 = g_lockY + g_lockH + pad; if (y1 > g_screenH) y1 = g_screenH;

    long sumX = 0, sumY = 0, count = 0;
    int minX = x1, minY = y1, maxX = x0, maxY = y0;
    for (int y = y0; y < y1; y += 2) {
        const uint8_t* row = rgba + y * stride;
        for (int x = x0; x < x1; x += 2) {
            const uint8_t* p = row + x * 4;
            if (is_skin(p[0], p[1], p[2])) {
                sumX += x; sumY += y; count++;
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
        }
    }
    if (count < 25) return false;
    outX = minX; outY = minY;
    outW = maxX - minX; outH = maxY - minY;
    return true;
}

static bool acquire_target(const uint8_t* rgba, int stride,
                           int& outX, int& outY, int& outW, int& outH) {
    int cx = g_screenW / 2, cy = g_screenH / 2;
    long bestScore = 0;
    int bx = -1, by = -1, bw = 0, bh = 0;
    const int CELL = 32;
    for (int gy = cy - g_fovY; gy < cy + g_fovY; gy += CELL) {
        for (int gx = cx - g_fovX; gx < cx + g_fovX; gx += CELL) {
            if (gx < 0 || gy < 0 || gx + CELL >= g_screenW || gy + CELL >= g_screenH)
                continue;
            long cnt = 0;
            int minX = gx + CELL, minY = gy + CELL, maxX = gx, maxY = gy;
            for (int y = gy; y < gy + CELL; y += 2) {
                const uint8_t* row = rgba + y * stride;
                for (int x = gx; x < gx + CELL; x += 2) {
                    const uint8_t* p = row + x * 4;
                    if (is_skin(p[0], p[1], p[2])) {
                        cnt++;
                        if (x < minX) minX = x;
                        if (x > maxX) maxX = x;
                        if (y < minY) minY = y;
                        if (y > maxY) maxY = y;
                    }
                }
            }
            if (cnt < 20) continue;
            int w = maxX - minX, h = maxY - minY;
            if (w <= 0 || h <= 0) continue;
            double ratio = (double)w / h;
            if (ratio < 0.5 || ratio > 1.8) continue;
            long dx = (minX + maxX) / 2 - cx;
            long dy = (minY + maxY) / 2 - cy;
            long score = cnt * 1500 - (dx*dx + dy*dy) / 40;
            if (score > bestScore) {
                bestScore = score;
                bx = minX; by = minY; bw = w; bh = h;
            }
        }
    }
    if (bx < 0) return false;
    outX = bx; outY = by; outW = bw; outH = bh;
    return true;
}

// ===== API EXPORT C-STYLE =====
EXPORT void aimlock_start() {
    g_running.store(true);
    LOGI("aimlock started");
}

EXPORT void aimlock_stop() {
    g_running.store(false);
    std::lock_guard<std::mutex> lk(g_lock);
    g_lockX = g_lockY = -1; g_lockW = g_lockH = 0;
    g_kalman.init = false;
    LOGI("aimlock stopped");
}

EXPORT void aimlock_clear() {
    std::lock_guard<std::mutex> lk(g_lock);
    g_lockX = g_lockY = -1; g_lockW = g_lockH = 0; g_lostFrames = 0;
    g_kalman.init = false;
    g_integralX = g_integralY = 0;
    g_prevErrX = g_prevErrY = 0;
}

EXPORT int aimlock_is_locked() {
    std::lock_guard<std::mutex> lk(g_lock);
    return (g_lockW > 0 && g_lockH > 0) ? 1 : 0;
}

EXPORT void aimlock_config(int fovX, int fovY, int smooth) {
    std::lock_guard<std::mutex> lk(g_lock);
    g_fovX = fovX; g_fovY = fovY;
    g_smooth = smooth > 0 ? smooth : 1;
}

EXPORT void aimlock_skin(int rMin, int rMax, int gMin, int gMax,
                          int bMin, int bMax) {
    std::lock_guard<std::mutex> lk(g_lock);
    g_rMin = rMin; g_rMax = rMax;
    g_gMin = gMin; g_gMax = gMax;
    g_bMin = bMin; g_bMax = bMax;
}

EXPORT void aimlock_pid(float kp, float ki, float kd) {
    std::lock_guard<std::mutex> lk(g_lock);
    g_pidKp = kp; g_pidKi = ki; g_pidKd = kd;
}

// out[0]=aimX, out[1]=aimY, out[2]=lockX, out[3]=lockY, out[4]=(lockW<<16)|(lockH&0xFFFF)
// Trả về 1 nếu có mục tiêu, 0 nếu không
EXPORT int aimlock_process(const uint8_t* rgba, int width, int height, int* out) {
    if (!g_running.load()) return 0;
    if (!rgba || !out || width <= 0 || height <= 0) return 0;

    g_screenW = width; g_screenH = height;
    int stride = width * 4;

    int aimX = -1, aimY = -1;
    std::lock_guard<std::mutex> lk(g_lock);

    bool locked = false;
    int nx = 0, ny = 0, nw = 0, nh = 0;

    if (g_lockW > 0) {
        int lx, ly, lw, lh;
        if (track_locked(rgba, stride, lx, ly, lw, lh) &&
            iou(lx, ly, lw, lh, g_lockX, g_lockY, g_lockW, g_lockH) > IOU_THRESHOLD) {
            g_lockX = lx; g_lockY = ly; g_lockW = lw; g_lockH = lh;
            g_lostFrames = 0;
            locked = true;
            nx = lx; ny = ly; nw = lw; nh = lh;
        } else {
            g_lostFrames++;
            if (g_lostFrames < MAX_LOST) {
                g_kalman.predict();
                nx = (int)(g_kalman.x - g_lockW / 2);
                ny = (int)(g_kalman.y - g_lockH / 2);
                nw = g_lockW; nh = g_lockH;
                locked = true;
            } else {
                g_lockX = g_lockY = -1; g_lockW = g_lockH = 0;
                g_lostFrames = 0; g_kalman.init = false;
            }
        }
    }

    if (!locked && g_lockW <= 0) {
        int tx, ty, tw, th;
        if (acquire_target(rgba, stride, tx, ty, tw, th)) {
            g_lockX = tx; g_lockY = ty; g_lockW = tw; g_lockH = th;
            g_lostFrames = 0;
            g_kalman.reset(tx + tw / 2.0, ty + th / 2.0);
            g_integralX = g_integralY = 0;
            g_prevErrX = g_prevErrY = 0;
            nx = tx; ny = ty; nw = tw; nh = th;
            locked = true;
        }
    }

    if (!locked) return 0;

    double cx = nx + nw / 2.0;
    double cy = ny + nh / 2.0;
    g_kalman.predict();
    g_kalman.update(cx, cy);

    double screenCX = g_screenW / 2.0;
    double screenCY = g_screenH / 2.0;
    double errX = g_kalman.x - screenCX;
    double errY = g_kalman.y - screenCY;

    g_integralX += errX * 0.016;
    g_integralY += errY * 0.016;
    if (g_integralX > 400) g_integralX = 400;
    if (g_integralX < -400) g_integralX = -400;
    if (g_integralY > 400) g_integralY = 400;
    if (g_integralY < -400) g_integralY = -400;

    double derX = (errX - g_prevErrX) / 0.016;
    double derY = (errY - g_prevErrY) / 0.016;
    g_prevErrX = errX; g_prevErrY = errY;

    double outX = g_pidKp * errX + g_pidKi * g_integralX + g_pidKd * derX;
    double outY = g_pidKp * errY + g_pidKi * g_integralY + g_pidKd * derY;

    aimX = (int)(screenCX + outX / g_smooth);
    aimY = (int)(screenCY + outY / g_smooth);

    out[0] = aimX;
    out[1] = aimY;
    out[2] = g_lockX;
    out[3] = g_lockY;
    out[4] = (g_lockW << 16) | (g_lockH & 0xFFFF);
    return 1;
}
