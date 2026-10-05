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

static int   g_lockStrength    = 100;
static int   g_aimSpeed        = 220;
static int   g_headshotBias    = 100;
static int   g_touchHz         = 240;
static int   g_headOffsetY     = -480;
static float g_maxRange        = 2200.0f;
static float g_deadZone        = 4.0f;

static float g_headMultiplier  = 3.5f;
static float g_stickyStrength  = 2.5f;
static float g_lockCurve       = 0.40f;
static int   g_snapThreshold   = 100;
static int   g_snapStrength    = 700;
static float g_distanceBias    = 0.28f;

static float g_predictionFactor= 0.50f;
static int   g_predictionFrames= 5;

static float g_liaSmoothing    = 0.58f;
static float g_liaDamping      = 0.93f;
static float g_liaMaxSpeed     = 1300.0f;
static float g_liaAccel        = 2.3f;
static float g_liaDecel        = 0.82f;

static float g_overshootLimit  = 0.11f;
static int   g_stabilizeCount  = 1;
static float g_stabilizeRadius = 2.0f;

static int   g_autoDragHead    = 1;
static float g_autoDragY       = -38.0f;
static float g_autoDragMax     = -190.0f;

static int   g_screenW = 0, g_screenH = 0;
static float g_centerX = 0, g_centerY = 0;
static float g_targetX = 0, g_targetY = 0;
static bool  g_hasTarget = false, g_running = false;
static pthread_t g_thread;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_inputFd = -1, g_uinputFd = -1;

static float g_smoothX = 0, g_smoothY = 0;
static float g_prevX = 0, g_prevY = 0;
static float g_velocityX = 0, g_velocityY = 0;
static float g_lastSentX = 0, g_lastSentY = 0;
static int   g_stableCounter = 0;
static float g_autoDragAccum = 0.0f;

static float g_targetHistoryX[8] = {0};
static float g_targetHistoryY[8] = {0};
static int   g_historyIndex = 0;

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
    if (fd < 0) return -1;
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
    writeEv(fd, EV_ABS, ABS_MT_TOUCH_MAJOR, 80);
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

static void predictTarget(float* predX, float* predY) {
    g_targetHistoryX[g_historyIndex] = g_targetX;
    g_targetHistoryY[g_historyIndex] = g_targetY;
    g_historyIndex = (g_historyIndex + 1) % 8;

    float sumDX = 0, sumDY = 0;
    int count = 0;
    for (int i = 0; i < g_predictionFrames && i < 7; i++) {
        int cur = (g_historyIndex - 1 - i + 8) % 8;
        int prev = (g_historyIndex - 2 - i + 8) % 8;
        sumDX += g_targetHistoryX[cur] - g_targetHistoryX[prev];
        sumDY += g_targetHistoryY[cur] - g_targetHistoryY[prev];
        count++;
    }

    if (count > 0) {
        float vx = sumDX / count;
        float vy = sumDY / count;
        *predX = g_targetX + vx * g_predictionFactor * g_predictionFrames;
        *predY = g_targetY + vy * g_predictionFactor * g_predictionFrames;
    } else {
        *predX = g_targetX;
        *predY = g_targetY;
    }
}

static void computeHeadLock(float tx, float ty, int* ox, int* oy, bool* ok) {
    *ok = false;

    float predX, predY;
    predictTarget(&predX, &predY);

    float dx = predX - g_centerX;
    float dy = predY - g_centerY;
    float dist = sqrtf(dx*dx + dy*dy);

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

    float dynamicOffset = g_headOffsetY - (dist * g_distanceBias);
    float hb = (g_headshotBias / 100.0f) * dynamicOffset;

    float curveFactor = 1.0f + (g_lockCurve * (1.0f - dist / g_maxRange));
    if (dist < 280.0f) curveFactor *= g_headMultiplier;

    float targetVelX = nx * sp * dist * 0.1f * curveFactor;
    float targetVelY = (ny * sp * dist * 0.1f * curveFactor) + hb;

    g_velocityX += (targetVelX - g_velocityX) * g_liaAccel;
    g_velocityY += (targetVelY - g_velocityY) * g_liaAccel;

    if (g_velocityX > g_liaMaxSpeed) g_velocityX = g_liaMaxSpeed;
    if (g_velocityX < -g_liaMaxSpeed) g_velocityX = -g_liaMaxSpeed;
    if (g_velocityY > g_liaMaxSpeed) g_velocityY = g_liaMaxSpeed;
    if (g_velocityY < -g_liaMaxSpeed) g_velocityY = -g_liaMaxSpeed;

    g_velocityX *= g_liaDecel;
    g_velocityY *= g_liaDecel;

    g_smoothX = g_smoothX * g_liaSmoothing + g_velocityX * (1.0f - g_liaSmoothing);
    g_smoothY = g_smoothY * g_liaSmoothing + g_velocityY * (1.0f - g_liaSmoothing);

    g_smoothX *= g_liaDamping;
    g_smoothY *= g_liaDamping;

    float mx = g_smoothX;
    float my = g_smoothY;

    if (g_autoDragHead) {
        if (g_autoDragAccum > g_autoDragMax) {
            g_autoDragAccum += g_autoDragY;
            my += g_autoDragY;
        }
    }

    float dxMove = mx - g_prevX;
    float dyMove = my - g_prevY;
    float moveDist = sqrtf(dxMove*dxMove + dyMove*dyMove);
    float maxMove = dist * g_overshootLimit;

    if (moveDist > maxMove && maxMove > 0) {
        float scale = maxMove / moveDist;
        mx = g_prevX + dxMove * scale;
        my = g_prevY + dyMove * scale;
    }

    if (dist < g_snapThreshold) {
        float dxStable = fabsf(mx - g_prevX);
        float dyStable = fabsf(my - g_prevY);

        if (dxStable < g_stabilizeRadius && dyStable < g_stabilizeRadius) {
            g_stableCounter++;
            if (g_stableCounter >= g_stabilizeCount) {
                float snapFactor = (1.0f - (dist / (float)g_snapThreshold));
                float snapX = nx * snapFactor * g_snapStrength * g_stickyStrength;
                float snapY = (ny * snapFactor * g_snapStrength * g_stickyStrength) + hb;

                mx = snapX;
                my = snapY;

                g_velocityX = 0;
                g_velocityY = 0;

                g_stableCounter = 0;
            }
        } else {
            g_stableCounter = 0;
        }
    } else {
        g_stableCounter = 0;
    }

    if (mx > 850) mx = 850; if (mx < -850) mx = -850;
    if (my > 850) my = 850; if (my < -850) my = -850;

    g_prevX = mx;
    g_prevY = my;
    g_lastSentX = mx;
    g_lastSentY = my;

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
            g_smoothX = 0; g_smoothY = 0;
            g_prevX = 0; g_prevY = 0;
            g_velocityX = 0; g_velocityY = 0;
            g_lastSentX = 0; g_lastSentY = 0;
            g_stableCounter = 0;
            g_autoDragAccum = 0;
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
    LOGI("=== AimLock S7 Sticky Head loaded ===");
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
    LOGI("AimLock S7 da bat");

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
JNIEXPORT jboolean JNICALL Java_com_aimlock_Native_isRunning(JNIEnv*, jclass) {
    return g_running ? JNI_TRUE : JNI_FALSE;
}
}
