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

#define LOG_TAG "AimLockS7"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// ===== CẤU HÌNH =====
static int   g_lockStrength   = 100;
static int   g_aimSpeed       = 140;      // tăng tốc kéo tâm nhẹ
static int   g_headshotBias   = 100;
static int   g_touchHz        = 240;
static int   g_headOffsetY    = -400;
static float g_maxRange       = 1500.0f;
static float g_deadZone       = 12.0f;    // vùng chết - fix rung tâm

// Ghim đầu
static float g_headMultiplier = 2.0f;
static float g_stickyStrength = 1.2f;
static float g_lockCurve      = 0.25f;
static int   g_snapThreshold  = 40;
static int   g_snapStrength   = 300;
static float g_distanceBias   = 0.15f;

// Kéo tâm nhẹ - fix rung
static float g_dragSmooth     = 0.75f;    // giảm để nhẹ tay hơn
static float g_touchPressure  = 0.6f;     // lực chạm nhẹ
static float g_damping        = 0.85f;    // giảm chấn - chống rung
static float g_overshootLimit = 0.3f;     // giới hạn vượt - fix lố đầu
static int   g_stabilizeCount = 3;        // số frame ổn định trước khi snap
static float g_stabilizeRadius= 5.0f;     // bán kính ổn định (px)

// Đạn thẳng
static int   g_noRecoil       = 1;
static int   g_noSpread       = 1;
static float g_recoilMult     = 0.0f;
static float g_spreadMult     = 0.0f;

static int   g_screenW = 0, g_screenH = 0;
static float g_centerX = 0, g_centerY = 0;
static float g_targetX = 0, g_targetY = 0;
static bool  g_hasTarget = false, g_running = false;
static pthread_t g_thread;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_inputFd = -1, g_uinputFd = -1;

// State chống rung + chống lố
static float g_smoothX = 0, g_smoothY = 0;
static float g_prevX = 0, g_prevY = 0;
static int   g_stableCounter = 0;
static float g_lastSentX = 0, g_lastSentY = 0;

static void readDeviceProps() {
    char model[PROP_VALUE_MAX] = {0};
    char androidVer[PROP_VALUE_MAX] = {0};
    __system_property_get("ro.product.model", model);
    __system_property_get("ro.build.version.release", androidVer);
    LOGI("Device: %s | Android: %s", model, androidVer);
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
    writeEv(fd, EV_ABS, ABS_MT_TOUCH_MAJOR, (int)(100 * g_touchPressure));
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

// ===== TÍNH TOÁN GHIM ĐẦU NHẸ TAY =====
static void computeHeadLock(float tx, float ty, int* ox, int* oy, bool* ok) {
    *ok = false;
    float dx = tx - g_centerX;
    float dy = ty - g_centerY;
    float dist = sqrtf(dx*dx + dy*dy);

    // VÙNG CHẾT - fix rung tâm khi tâm đã gần mục tiêu
    if (dist < g_deadZone) {
        *ox = (int)(g_centerX + g_lastSentX);
        *oy = (int)(g_centerY + g_lastSentY);
        *ok = true;
        return;
    }

    if (dist > g_maxRange) return;

    float nx = dx / dist;
    float ny = dy / dist;
    float st = g_lockStrength / 100.0f;
    float sp = g_aimSpeed / 100.0f;

    // Offset động theo khoảng cách
    float dynamicOffset = g_headOffsetY - (dist * g_distanceBias);
    float hb = (g_headshotBias / 100.0f) * dynamicOffset;

    // Curve factor
    float curveFactor = 1.0f + (g_lockCurve * (1.0f - dist / g_maxRange));
    if (dist < 150.0f) curveFactor *= g_headMultiplier;

    // Kéo tâm thô
    float rawMx = nx * st * sp * dist * 0.1f * curveFactor;
    float rawMy = (ny * st * sp * dist * 0.1f * curveFactor) + hb;

    // LÀM MƯỢT - nhẹ tay
    g_smoothX = g_smoothX * g_dragSmooth + rawMx * (1.0f - g_dragSmooth);
    g_smoothY = g_smoothY * g_dragSmooth + rawMy * (1.0f - g_dragSmooth);

    // GIẢM CHẤN - chống rung
    g_smoothX *= g_damping;
    g_smoothY *= g_damping;

    float mx = g_smoothX;
    float my = g_smoothY;

    // GIỚI HẠN VƯỢT - fix lố đầu
    float dxMove = mx - g_prevX;
    float dyMove = my - g_prevY;
    float moveDist = sqrtf(dxMove*dxMove + dyMove*dyMove);
    float targetDist = dist * g_overshootLimit;

    if (moveDist > targetDist && targetDist > 0) {
        float scale = targetDist / moveDist;
        mx = g_prevX + dxMove * scale;
        my = g_prevY + dyMove * scale;
    }

    // SNAP với ổn định - chỉ snap khi đã đứng yên
    if (dist < g_snapThreshold) {
        float dxStable = fabsf(mx - g_prevX);
        float dyStable = fabsf(my - g_prevY);

        if (dxStable < g_stabilizeRadius && dyStable < g_stabilizeRadius) {
            g_stableCounter++;
            if (g_stableCounter >= g_stabilizeCount) {
                // Đã ổn định - snap về đúng đầu
                float snapFactor = (1.0f - (dist / (float)g_snapThreshold));
                mx = nx * snapFactor * g_snapStrength * g_stickyStrength;
                my = (ny * snapFactor * g_snapStrength * g_stickyStrength) + hb;
                g_stableCounter = 0;
            }
        } else {
            g_stableCounter = 0;
        }
    } else {
        g_stableCounter = 0;
    }

    // Clamp
    if (mx > 500) mx = 500; if (mx < -500) mx = -500;
    if (my > 500) my = 500; if (my < -500) my = -500;

    // Lưu state
    g_prevX = mx;
    g_prevY = my;
    g_lastSentX = mx;
    g_lastSentY = my;

    *ox = (int)(g_centerX + mx);
    *oy = (int)(g_centerY + my);
    *ok = true;
}

static void applyBulletStraight(float* recoil, float* spread) {
    if (g_noRecoil) *recoil *= g_recoilMult;
    if (g_noSpread) *spread *= g_spreadMult;
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

        float recoil = 0.0f, spread = 0.0f;
        applyBulletStraight(&recoil, &spread);

        if (has) {
            int ox, oy; bool ok;
            computeHeadLock(tx, ty, &ox, &oy, &ok);
            if (ok && fd >= 0) {
                if (!touching) {
                    touchDown(fd, (int)g_centerX, (int)g_centerY, tid++);
                    touching = true;
                }
                ox += (int)recoil;
                oy += (int)spread;
                touchMove(fd, ox, oy);
            }
        } else if (touching && fd >= 0) {
            touchUp(fd);
            touching = false;
            g_smoothX = 0; g_smoothY = 0;
            g_prevX = 0; g_prevY = 0;
            g_lastSentX = 0; g_lastSentY = 0;
            g_stableCounter = 0;
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
    LOGI("=== AimLock Smooth + NoShake + NoOvershoot loaded ===");
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
    LOGI("AimLock da bat");

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
    g_lockStrength = ls; g_aimSpeed = asp; g_headshotBias = hb;
    g_touchHz = hz; g_headOffsetY = hoy;
}
JNIEXPORT void JNICALL Java_com_aimlock_Native_setHeadLockConfig(JNIEnv*, jclass,
    jfloat headMult, jfloat sticky, jfloat curve, jint snapThresh,
    jint snapStr, jfloat distBias) {
    g_headMultiplier = headMult; g_stickyStrength = sticky;
    g_lockCurve = curve; g_snapThreshold = snapThresh;
    g_snapStrength = snapStr; g_distanceBias = distBias;
}
JNIEXPORT void JNICALL Java_com_aimlock_Native_setDragSmooth(JNIEnv*, jclass,
    jfloat smooth, jfloat pressure) {
    g_dragSmooth = smooth; g_touchPressure = pressure;
}
JNIEXPORT void JNICALL Java_com_aimlock_Native_setStabilizeConfig(JNIEnv*, jclass,
    jfloat damping, jfloat overshoot, jint stabCount, jfloat stabRadius) {
    g_damping = damping;
    g_overshootLimit = overshoot;
    g_stabilizeCount = stabCount;
    g_stabilizeRadius = stabRadius;
}
JNIEXPORT void JNICALL Java_com_aimlock_Native_setBulletStraight(JNIEnv*, jclass,
    jint noRecoil, jint noSpread, jfloat recoilMult, jfloat spreadMult) {
    g_noRecoil = noRecoil; g_noSpread = noSpread;
    g_recoilMult = recoilMult; g_spreadMult = spreadMult;
}
JNIEXPORT jboolean JNICALL Java_com_aimlock_Native_isRunning(JNIEnv*, jclass) {
    return g_running ? JNI_TRUE : JNI_FALSE;
}
}
