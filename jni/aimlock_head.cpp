#include <jni.h>
#include <android/log.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <pthread.h>
#include <cmath>
#include <cstring>
#include <cstdio>

#define LOG_TAG "AimLockHead"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

struct AimConfig {
    int lockStrength = 100;
    int aimSpeed     = 100;
    int headshotBias = 100;
    int fov          = 180;
    int maxRange     = 500;
    float deadZone   = 0.0f;
    int touchHz      = 240;
    int sticky       = 1;
    int prediction   = 0;
    int headOffsetY  = -60;
};

static AimConfig g_cfg;
static int g_screenW = 0, g_screenH = 0;
static float g_centerX = 0, g_centerY = 0;
static float g_targetX = 0, g_targetY = 0;
static bool g_hasTarget = false, g_running = false;
static pthread_t g_thread;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_inputFd = -1, g_uinputFd = -1;

static int openInputDevice() {
    for (int i = 0; i < 32; i++) {
        char path[64];
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDWR);
        if (fd >= 0) { LOGI("Mo: %s", path); return fd; }
    }
    return -1;
}

static int createUinputDevice() {
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) return -1;
    ioctl(fd, UI_SET_EVBIT, EV_ABS);
    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    ioctl(fd, UI_SET_EVBIT, EV_SYN);
    ioctl(fd, UI_SET_ABSBIT, ABS_MT_POSITION_X);
    ioctl(fd, UI_SET_ABSBIT, ABS_MT_POSITION_Y);
    ioctl(fd, UI_SET_ABSBIT, ABS_MT_SLOT);
    ioctl(fd, UI_SET_ABSBIT, ABS_MT_TRACKING_ID);
    ioctl(fd, UI_SET_KEYBIT, BTN_TOUCH);
    struct uinput_user_dev uidev;
    memset(&uidev, 0, sizeof(uidev));
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "aimlock_touch");
    uidev.id.bustype = BUS_VIRTUAL;
    uidev.absmin[ABS_MT_POSITION_X] = 0;
    uidev.absmax[ABS_MT_POSITION_X] = 4096;
    uidev.absmin[ABS_MT_POSITION_Y] = 0;
    uidev.absmax[ABS_MT_POSITION_Y] = 4096;
    write(fd, &uidev, sizeof(uidev));
    ioctl(fd, UI_DEV_CREATE);
    LOGI("Da tao uinput");
    return fd;
}

static void writeEvent(int fd, int type, int code, int value) {
    struct input_event ev;
    memset(&ev, 0, sizeof(ev));
    gettimeofday(&ev.time, nullptr);
    ev.type = type; ev.code = code; ev.value = value;
    write(fd, &ev, sizeof(ev));
}

static void sendTouchDown(int fd, int x, int y, int id) {
    writeEvent(fd, EV_ABS, ABS_MT_SLOT, 0);
    writeEvent(fd, EV_ABS, ABS_MT_TRACKING_ID, id);
    writeEvent(fd, EV_ABS, ABS_MT_POSITION_X, x);
    writeEvent(fd, EV_ABS, ABS_MT_POSITION_Y, y);
    writeEvent(fd, EV_KEY, BTN_TOUCH, 1);
    writeEvent(fd, EV_SYN, SYN_REPORT, 0);
}

static void sendTouchMove(int fd, int x, int y) {
    writeEvent(fd, EV_ABS, ABS_MT_POSITION_X, x);
    writeEvent(fd, EV_ABS, ABS_MT_POSITION_Y, y);
    writeEvent(fd, EV_SYN, SYN_REPORT, 0);
}

static void sendTouchUp(int fd) {
    writeEvent(fd, EV_ABS, ABS_MT_TRACKING_ID, -1);
    writeEvent(fd, EV_KEY, BTN_TOUCH, 0);
    writeEvent(fd, EV_SYN, SYN_REPORT, 0);
}

struct AimResult { int moveX, moveY; bool valid; };

static AimResult computeHeadLock(float tx, float ty) {
    AimResult r = {0, 0, false};
    float dx = tx - g_centerX;
    float dy = ty - g_centerY;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < g_cfg.deadZone) return r;
    if (dist > g_cfg.maxRange) return r;
    float nx = dx / dist;
    float ny = dy / dist;
    float strength = g_cfg.lockStrength / 100.0f;
    float speed    = g_cfg.aimSpeed / 100.0f;
    float headBias = (g_cfg.headshotBias / 100.0f) * g_cfg.headOffsetY;
    float pf = g_cfg.prediction ? 1.15f : 1.0f;
    float mx = nx * strength * speed * dist * 0.1f * pf;
    float my = (ny * strength * speed * dist * 0.1f * pf) + headBias;
    if (mx > 200) mx = 200; if (mx < -200) mx = -200;
    if (my > 200) my = 200; if (my < -200) my = -200;
    r.moveX = (int)(g_centerX + mx);
    r.moveY = (int)(g_centerY + my);
    r.valid = true;
    return r;
}

static void* aimLoop(void*) {
    int delayUs = 1000000 / g_cfg.touchHz;
    int trackingId = 1;
    bool touching = false;
    while (g_running) {
        pthread_mutex_lock(&g_mutex);
        bool has = g_hasTarget;
        float tx = g_targetX, ty = g_targetY;
        pthread_mutex_unlock(&g_mutex);
        int fd = (g_inputFd >= 0) ? g_inputFd : g_uinputFd;
        if (has) {
            AimResult r = computeHeadLock(tx, ty);
            if (r.valid && fd >= 0) {
                if (!touching) {
                    sendTouchDown(fd, (int)g_centerX, (int)g_centerY, trackingId++);
                    touching = true;
                }
                sendTouchMove(fd, r.moveX, r.moveY);
            }
        } else if (touching && fd >= 0) {
            sendTouchUp(fd);
            touching = false;
        }
        usleep(delayUs);
    }
    if (touching) {
        int fd = (g_inputFd >= 0) ? g_inputFd : g_uinputFd;
        if (fd >= 0) sendTouchUp(fd);
    }
    return nullptr;
}

extern "C" {

JNIEXPORT void JNICALL
Java_com_aimlock_noroot_NativeBridge_start(JNIEnv*, jclass, jint w, jint h) {
    if (g_running) return;
    g_screenW = w; g_screenH = h;
    g_centerX = w / 2.0f; g_centerY = h / 2.0f;
    g_inputFd = openInputDevice();
    if (g_inputFd < 0) g_uinputFd = createUinputDevice();
    if (g_inputFd < 0 && g_uinputFd < 0) { LOGE("Khong mo duoc input"); return; }
    g_running = true;
    pthread_create(&g_thread, nullptr, aimLoop, nullptr);
    LOGI("Started %dx%d", w, h);
}

JNIEXPORT void JNICALL
Java_com_aimlock_noroot_NativeBridge_stop(JNIEnv*, jclass) {
    if (!g_running) return;
    g_running = false;
    pthread_join(g_thread, nullptr);
    if (g_inputFd >= 0) { close(g_inputFd); g_inputFd = -1; }
    if (g_uinputFd >= 0) { ioctl(g_uinputFd, UI_DEV_DESTROY); close(g_uinputFd); g_uinputFd = -1; }
}

JNIEXPORT void JNICALL
Java_com_aimlock_noroot_NativeBridge_setTarget(JNIEnv*, jclass, jfloat x, jfloat y) {
    pthread_mutex_lock(&g_mutex);
    g_targetX = x; g_targetY = y; g_hasTarget = true;
    pthread_mutex_unlock(&g_mutex);
}

JNIEXPORT void JNICALL
Java_com_aimlock_noroot_NativeBridge_clearTarget(JNIEnv*, jclass) {
    pthread_mutex_lock(&g_mutex);
    g_hasTarget = false;
    pthread_mutex_unlock(&g_mutex);
}

JNIEXPORT void JNICALL
Java_com_aimlock_noroot_NativeBridge_setConfig(JNIEnv*, jclass,
    jint ls, jint asp, jint hb, jint fov, jint mr, jint hz,
    jint st, jint pr, jint hoy)
{
    g_cfg.lockStrength = ls; g_cfg.aimSpeed = asp; g_cfg.headshotBias = hb;
    g_cfg.fov = fov; g_cfg.maxRange = mr; g_cfg.touchHz = hz;
    g_cfg.sticky = st; g_cfg.prediction = pr; g_cfg.headOffsetY = hoy;
}

JNIEXPORT jboolean JNICALL
Java_com_aimlock_noroot_NativeBridge_isRunning(JNIEnv*, jclass) {
    return g_running ? JNI_TRUE : JNI_FALSE;
}

}
