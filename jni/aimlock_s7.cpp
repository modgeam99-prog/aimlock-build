#include <jni.h>
#include <android/log.h>
#include <dlfcn.h>
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
#include <cstdlib>
#include <sys/system_properties.h>

#define LOG_TAG "AimLockHead"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// ===== CẤU HÌNH GHIM ĐẦU CHẶT =====
static int   g_lockStrength  = 100;
static int   g_aimSpeed      = 100;
static int   g_headshotBias  = 100;
static int   g_touchHz       = 240;
static int   g_headOffsetY   = -80;    // kéo cao hơn về phía đầu
static float g_maxRange      = 1200.0f;
static float g_deadZone      = 0.0f;

// Ghim chặt đầu
static float g_headMultiplier   = 1.5f;   // kéo mạnh hơn khi gần đầu
static float g_stickyStrength   = 1.0f;   // giữ chặt mục tiêu
static float g_lockCurve        = 0.15f;  // đường cong kéo (phi tuyến tính)
static int   g_snapThreshold    = 25;     // khoảng cách để "dính chặt"
static int   g_snapStrength     = 200;    // lực snap khi trong ngưỡng

static int   g_screenW = 0, g_screenH = 0;
static float g_centerX = 0, g_centerY = 0;
static float g_targetX = 0, g_targetY = 0;
static bool  g_hasTarget = false, g_running = false;
static pthread_t g_thread;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_inputFd = -1, g_uinputFd = -1;

static void readDeviceProps() {
    char model[PROP_VALUE_MAX] = {0};
    __system_property_get("ro.product.model", model);
    LOGI("Device: %s", model);
}

static int openInputDevice() {
    for (int i = 0; i < 32; i++) {
        char p[64];
        snprintf(p, sizeof(p), "/dev/input/event%d", i);
        int fd = open(p, O_RDWR);
        if (fd >= 0) {
            unsigned long evbit[4] = {0};
            if (ioctl(fd, EVIOCGBIT(0, sizeof(evbit)), evbit) >= 0) {
                if (evbit[0] & (1 << EV_ABS)) {
                    char name[256] = {0};
                    ioctl(fd, EVIOCGNAME(sizeof(name)), name);
                    LOGI("Touch: %s (%s)", name, p);
                    return fd;
                }
            }
            close(fd);
        }
    }
    return -1;
}

static int createUinput() {
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        LOGE("Khong mo duoc /dev/uinput");
        return -1;
    }
    ioctl(fd, UI_SET_EVBIT, EV_ABS);
    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    ioctl(fd, UI_SET_EVBIT, EV_SYN);
    ioctl(fd, UI_SET_ABSBIT, ABS_MT_SLOT);
    ioctl(fd, UI_SET_ABSBIT, ABS_MT_TOUCH_MAJOR);
    ioctl(fd, UI_SET_ABSBIT, ABS_MT_POSITION_X);
    ioctl(fd, UI_SET_ABSBIT, ABS_MT_POSITION_Y);
    ioctl(fd, UI_SET_ABSBIT, ABS_MT_TRACKING_ID);
    ioctl(fd, UI_SET_PROPBIT, INPUT_PROP_DIRECT);
    ioctl(fd, UI_SET_KEYBIT, BTN_TOUCH);

    struct uinput_user_dev uidev;
    memset(&uidev, 0, sizeof(uidev));
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "sec_touchscreen");
    uidev.id.bustype = BUS_I2C;
    uidev.id.vendor  = 0x04e8;
    uidev.id.product = 0x0001;
    uidev.id.version = 1;
    uidev.absmin[ABS_MT_POSITION_X] = 0;
    uidev.absmax[ABS_MT_POSITION_X] = 1440;
    uidev.absmin[ABS_MT_POSITION_Y] = 0;
    uidev.absmax[ABS_MT_POSITION_Y] = 2560;
    uidev.absmin[ABS_MT_TOUCH_MAJOR] = 0;
    uidev.absmax[ABS_MT_TOUCH_MAJOR] = 255;
    write(fd, &uidev, sizeof(uidev));
    ioctl(fd, UI_DEV_CREATE);
    LOGI("Da tao uinput");
    return fd;
}

static void writeEv(int fd, int type, int code, int val) {
    struct input_event ev;
    memset(&ev, 0, sizeof(ev));
    gettimeofday(&ev.time, nullptr);
    ev.type = type; ev.code = code; ev.value = val;
    write(fd, &ev, sizeof(ev));
}

static void touchDown(int fd, int x, int y, int id) {
    writeEv(fd, EV_ABS, ABS_MT_SLOT, 0);
    writeEv(fd, EV_ABS, ABS_MT_TRACKING_ID, id);
    writeEv(fd, EV_ABS, ABS_MT_POSITION_X, x);
    writeEv(fd, EV_ABS, ABS_MT_POSITION_Y, y);
    writeEv(fd, EV_ABS, ABS_MT_TOUCH_MAJOR, 100);
    writeEv(fd, EV_KEY, BTN_TOUCH, 1);
    writeEv(fd, EV_SYN, SYN_REPORT, 0);
}

static void touchMove(int fd, int x, int y) {
    writeEv(fd, EV_ABS, ABS_MT_POSITION_X, x);
    writeEv(fd, EV_ABS, ABS_MT_POSITION_Y, y);
    writeEv(fd, EV_SYN, SYN_REPORT, 0);
}

static void touchUp(int fd) {
    writeEv(fd, EV_ABS, ABS_MT_TRACKING_ID, -1);
    writeEv(fd, EV_KEY, BTN_TOUCH, 0);
    writeEv(fd, EV_SYN, SYN_REPORT, 0);
}

// ===== TÍNH TOÁN GHIM ĐẦU CHẶT =====
static void computeHeadLock(float tx, float ty, int* ox, int* oy, bool* ok) {
    *ok = false;
    float dx = tx - g_centerX;
    float dy = ty - g_centerY;
    float dist = sqrtf(dx*dx + dy*dy);

    if (dist > g_maxRange) return;
    if (dist < g_deadZone) {
        *ox = (int)g_centerX;
        *oy = (int)g_centerY;
        *ok = true;
        return;
    }

    // Vector đơn vị
    float nx = dx / dist;
    float ny = dy / dist;

    // Hệ số cơ bản
    float st = g_lockStrength / 100.0f;
    float sp = g_aimSpeed / 100.0f;
    float hb = (g_headshotBias / 100.0f) * g_headOffsetY;

    // Đường cong phi tuyến tính - kéo mạnh khi gần, chậm khi xa
    float curveFactor = 1.0f + (g_lockCurve * (1.0f - dist / g_maxRange));
    if (dist < 100.0f) {
        curveFactor *= g_headMultiplier;
    }

    float mx = nx * st * sp * dist * 0.1f * curveFactor;
    float my = (ny * st * sp * dist * 0.1f * curveFactor) + hb;

    // SNAP - DÍNH CHẶT khi trong ngưỡng
    if (dist < g_snapThreshold) {
        // Kéo gần như tức thời về target
        float snapFactor = (1.0f - (dist / (float)g_snapThreshold));
        mx = nx * snapFactor * g_snapStrength * g_stickyStrength;
        my = (ny * snapFactor * g_snapStrength * g_stickyStrength) + hb;
    }

    // Clamp
    if (mx > 400) mx = 400; if (mx < -400) mx = -400;
    if (my > 400) my = 400; if (my < -400) my = -400;

    *ox = (int)(g_centerX + mx);
    *oy = (int)(g_centerY + my);
    *ok = true;
}

static void* aimLoop(void*) {
    int delayUs = 1000000 / g_touchHz;
    int tid = 1;
    bool touching = false;

    while (g_running) {
        pthread_mutex_lock(&g_mutex);
        bool has = g_hasTarget;
        float tx = g_targetX, ty = g_targetY;
        pthread_mutex_unlock(&g_mutex);

        int fd = (g_inputFd >= 0) ? g_inputFd : g_uinputFd;

        if (has) {
            int ox, oy; bool ok;
            computeHeadLock(tx, ty, &ox, &oy, &ok);
            if (ok && fd >= 0) {
                if (!touching) {
                    touchDown(fd, (int)g_centerX, (int)g_centerY, tid++);
                    touching = true;
                }
                touchMove(fd, ox, oy);
            }
        } else if (touching && fd >= 0) {
            touchUp(fd);
            touching = false;
        }
        usleep(delayUs);
    }

    if (touching) {
        int fd = (g_inputFd >= 0) ? g_inputFd : g_uinputFd;
        if (fd >= 0) touchUp(fd);
    }
    return nullptr;
}

__attribute__((constructor))
static void onLoad() {
    LOGI("=== AimLock HeadLock loaded ===");
    readDeviceProps();
    sleep(5);

    FILE* fp = popen("wm size", "r");
    if (fp) {
        char buf[128];
        if (fgets(buf, sizeof(buf), fp)) {
            int w, h;
            if (sscanf(buf, "Physical size: %dx%d", &w, &h) == 2) {
                g_screenW = w; g_screenH = h;
                g_centerX = w / 2.0f;
                g_centerY = h / 2.0f;
                LOGI("Screen: %dx%d", w, h);
            }
        }
        pclose(fp);
    }

    g_inputFd = openInputDevice();
    if (g_inputFd < 0) g_uinputFd = createUinput();
    if (g_inputFd < 0 && g_uinputFd < 0) {
        LOGE("Khong mo duoc input");
        return;
    }

    g_running = true;
    pthread_create(&g_thread, nullptr, aimLoop, nullptr);
    LOGI("AimLock HeadLock da bat");

    // Demo target - có thể thay bằng setTarget() từ vision module
    pthread_mutex_lock(&g_mutex);
    g_targetX = g_centerX;
    g_targetY = g_centerY - 300;
    g_hasTarget = true;
    pthread_mutex_unlock(&g_mutex);
}

__attribute__((destructor))
static void onUnload() {
    LOGI("Unloading");
    g_running = false;
    if (g_inputFd >= 0) close(g_inputFd);
    if (g_uinputFd >= 0) {
        ioctl(g_uinputFd, UI_DEV_DESTROY);
        close(g_uinputFd);
    }
}

extern "C" {
JNIEXPORT void JNICALL Java_com_aimlock_Native_setTarget(JNIEnv*, jclass, jfloat x, jfloat y) {
    pthread_mutex_lock(&g_mutex);
    g_targetX = x; g_targetY = y; g_hasTarget = true;
    pthread_mutex_unlock(&g_mutex);
}
JNIEXPORT void JNICALL Java_com_aimlock_Native_clearTarget(JNIEnv*, jclass) {
    pthread_mutex_lock(&g_mutex);
    g_hasTarget = false;
    pthread_mutex_unlock(&g_mutex);
}
JNIEXPORT void JNICALL Java_com_aimlock_Native_setConfig(JNIEnv*, jclass,
    jint ls, jint asp, jint hb, jint hz, jint hoy) {
    g_lockStrength = ls;
    g_aimSpeed     = asp;
    g_headshotBias = hb;
    g_touchHz      = hz;
    g_headOffsetY  = hoy;
}
JNIEXPORT void JNICALL Java_com_aimlock_Native_setHeadLockConfig(JNIEnv*, jclass,
    jfloat headMult, jfloat sticky, jfloat curve, jint snapThresh, jint snapStr) {
    g_headMultiplier = headMult;
    g_stickyStrength = sticky;
    g_lockCurve      = curve;
    g_snapThreshold  = snapThresh;
    g_snapStrength   = snapStr;
}
JNIEXPORT jboolean JNICALL Java_com_aimlock_Native_isRunning(JNIEnv*, jclass) {
    return g_running ? JNI_TRUE : JNI_FALSE;
}
}
