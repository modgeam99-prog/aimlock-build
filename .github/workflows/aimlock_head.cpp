#include <jni.h>
#include <android/log.h>
#include <cstring>
#include <cmath>
#include <atomic>
#include <mutex>

#define LOG_TAG "aimlock_head"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

static std::atomic<bool> g_running{false};
static std::mutex g_lock;
static int g_fovX = 240, g_fovY = 240, g_smooth = 4, g_tolerance = 40;
static int g_skinRmin = 95,  g_skinRmax = 255;
static int g_skinGmin = 40,  g_skinGmax = 200;
static int g_skinBmin = 20,  g_skinBmax = 170;
static int g_lockX = -1, g_lockY = -1, g_lockW = 0, g_lockH = 0;
static int g_lostFrames = 0;
static const int MAX_LOST = 15;
static int g_screenW = 1080, g_screenH = 2340;

static inline bool is_skin(int r, int g, int b) {
    return r >= g_skinRmin && r <= g_skinRmax &&
           g >= g_skinGmin && g <= g_skinGmax &&
           b >= g_skinBmin && b <= g_skinBmax &&
           r > g && r > b && (r - (g < b ? g : b)) > 15;
}

static bool track_locked(const uint8_t* rgba, int stride, int& outX, int& outY) {
    if (g_lockW <= 0 || g_lockH <= 0) return false;
    long sumX = 0, sumY = 0, count = 0;
    int x0 = g_lockX - 20 < 0 ? 0 : g_lockX - 20;
    int y0 = g_lockY - 20 < 0 ? 0 : g_lockY - 20;
    int x1 = g_lockX + g_lockW + 20; if (x1 > g_screenW) x1 = g_screenW;
    int y1 = g_lockY + g_lockH + 20; if (y1 > g_screenH) y1 = g_screenH;
    for (int y = y0; y < y1; y += 2) {
        const uint8_t* row = rgba + y * stride;
        for (int x = x0; x < x1; x += 2) {
            const uint8_t* p = row + x * 4;
            if (is_skin(p[0], p[1], p[2])) { sumX += x; sumY += y; count++; }
        }
    }
    if (count < 30) return false;
    outX = (int)(sumX / count);
    outY = (int)(sumY / count);
    return true;
}

static bool acquire_target(const uint8_t* rgba, int stride, int& outX, int& outY,
                           int& outW, int& outH) {
    int cx = g_screenW / 2, cy = g_screenH / 2;
    long bestScore = 0; int bx = -1, by = -1, bw = 0, bh = 0;
    const int CELL = 40;
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
            long dx = gx + CELL/2 - cx;
            long dy = gy + CELL/2 - cy;
            long score = cnt * 1000 - (dx*dx + dy*dy) / 100;
            if (cnt > 20 && score > bestScore) {
                bestScore = score;
                bx = minX; by = minY;
                bw = maxX - minX; bh = maxY - minY;
            }
        }
    }
    if (bx < 0) return false;
    outX = bx + bw/2; outY = by + bh/2;
    outW = bw; outH = bh;
    return true;
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_aimlock_Native_processFrame(JNIEnv* env, jclass,
                                     jbyteArray frameData, jint width, jint height) {
    if (!g_running.load()) return nullptr;
    g_screenW = width; g_screenH = height;
    jbyte* data = env->GetByteArrayElements(frameData, nullptr);
    if (!data) return nullptr;
    const uint8_t* rgba = (const uint8_t*)data;
    int stride = width * 4;
    int outX = -1, outY = -1;
    std::lock_guard<std::mutex> lk(g_lock);
    if (g_lockW > 0) {
        int lx, ly;
        if (track_locked(rgba, stride, lx, ly)) {
            int dx = lx - (g_lockX + g_lockW/2);
            int dy = ly - (g_lockY + g_lockH/2);
            g_lockX += dx; g_lockY += dy;
            g_lostFrames = 0;
            outX = lx; outY = ly;
        } else {
            g_lostFrames++;
            if (g_lostFrames > MAX_LOST) {
                g_lockX = g_lockY = -1;
                g_lockW = g_lockH = 0;
                g_lostFrames = 0;
            }
        }
    }
    if (g_lockW <= 0) {
        int tx, ty, tw, th;
        if (acquire_target(rgba, stride, tx, ty, tw, th)) {
            g_lockX = tx - tw/2; g_lockY = ty - th/2;
            g_lockW = tw; g_lockH = th;
            g_lostFrames = 0;
            outX = tx; outY = ty;
        }
    }
    env->ReleaseByteArrayElements(frameData, data, JNI_ABORT);
    if (outX < 0) return nullptr;
    jintArray result = env->NewIntArray(4);
    jint vals[4] = { outX, outY, g_lockW, g_lockH };
    env->SetIntArrayRegion(result, 0, 4, vals);
    return result;
}

extern "C" JNIEXPORT void JNICALL
Java_com_aimlock_Native_setConfig(JNIEnv*, jclass, jint f, jint f2, jint s, jint t) {
    std::lock_guard<std::mutex> lk(g_lock);
    g_fovX = f; g_fovY = f2; g_smooth = s; g_tolerance = t;
}

extern "C" JNIEXPORT void JNICALL
Java_com_aimlock_Native_setSkinRange(JNIEnv*, jclass,
                                     jint rMin, jint rMax,
                                     jint gMin, jint gMax,
                                     jint bMin, jint bMax) {
    std::lock_guard<std::mutex> lk(g_lock);
    g_skinRmin = rMin; g_skinRmax = rMax;
    g_skinGmin = gMin; g_skinGmax = gMax;
    g_skinBmin = bMin; g_skinBmax = bMax;
}

extern "C" JNIEXPORT void JNICALL
Java_com_aimlock_Native_start(JNIEnv*, jclass) { g_running.store(true); }

extern "C" JNIEXPORT void JNICALL
Java_com_aimlock_Native_stop(JNIEnv*, jclass) {
    g_running.store(false);
    std::lock_guard<std::mutex> lk(g_lock);
    g_lockX = g_lockY = -1; g_lockW = g_lockH = 0;
}

extern "C" JNIEXPORT void JNICALL
Java_com_aimlock_Native_clearTarget(JNIEnv*, jclass) {
    std::lock_guard<std::mutex> lk(g_lock);
    g_lockX = g_lockY = -1; g_lockW = g_lockH = 0; g_lostFrames = 0;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_aimlock_Native_isLocked(JNIEnv*, jclass) {
    std::lock_guard<std::mutex> lk(g_lock);
    return (g_lockW > 0 && g_lockH > 0) ? JNI_TRUE : JNI_FALSE;
}
