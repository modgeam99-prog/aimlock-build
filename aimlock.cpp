#include <jni.h>
#include <android/log.h>
#include <pthread.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>

#define LOG_TAG "AIMLOCK"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

struct Vec3 { float x, y, z; };
static bool g_running = false;
static pthread_t g_thread;
static void* g_game_base = nullptr;

static bool write_memory(void* addr, const void* data, size_t size) {
    if (!addr) return false;
    uintptr_t page = reinterpret_cast<uintptr_t>(addr) & ~0xFFF;
    mprotect(reinterpret_cast<void*>(page), 0x1000, PROT_READ | PROT_WRITE | PROT_EXEC);
    memcpy(addr, data, size);
    return true;
}
static bool read_memory(void* addr, void* out, size_t size) {
    if (!addr || !out) return false;
    memcpy(out, addr, size);
    return true;
}
static Vec3 calc_angle(const Vec3& local, const Vec3& target) {
    Vec3 delta = { target.x - local.x, target.y - local.y, target.z - local.z };
    float hyp = sqrtf(delta.x * delta.x + delta.y * delta.y);
    Vec3 angle;
    angle.x = atan2f(-delta.z, hyp) * 180.0f / M_PI;
    angle.y = atan2f(delta.y, delta.x) * 180.0f / M_PI;
    angle.z = 0.0f;
    return angle;
}
static float normalize_angle(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
static void* aimlock_thread(void*) {
    LOGI("Aimlock thread started");
    const uintptr_t OFFSET_LOCAL_PLAYER = 0x0;
    const uintptr_t OFFSET_ENTITY_LIST  = 0x0;
    const uintptr_t OFFSET_VIEW_ANGLE   = 0x0;
    const uintptr_t OFFSET_HEAD_BONE    = 0x0;
    while (g_running) {
        if (!g_game_base) { usleep(100000); continue; }
        Vec3 local_pos;
        read_memory((void*)((uintptr_t)g_game_base + OFFSET_LOCAL_PLAYER), &local_pos, sizeof(Vec3));
        Vec3 target_pos;
        read_memory((void*)((uintptr_t)g_game_base + OFFSET_ENTITY_LIST + OFFSET_HEAD_BONE), &target_pos, sizeof(Vec3));
        Vec3 aim = calc_angle(local_pos, target_pos);
        aim.x = normalize_angle(aim.x);
        aim.y = normalize_angle(aim.y);
        write_memory((void*)((uintptr_t)g_game_base + OFFSET_VIEW_ANGLE), &aim, sizeof(Vec3));
        usleep(1000);
    }
    return nullptr;
}
__attribute__((constructor))
void init_aimlock() {
    LOGI("Aimlock library loaded");
    g_running = true;
    pthread_create(&g_thread, nullptr, aimlock_thread, nullptr);
}
__attribute__((destructor))
void deinit_aimlock() {
    g_running = false;
    pthread_join(g_thread, nullptr);
    LOGI("Aimlock library unloaded");
}