#include <jni.h>
#include <android/log.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <pthread.h>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>

#define LOG_TAG "AimPatch"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

static bool g_patchEnabled = false;
static bool g_headshot = true;
static bool g_silentAim = false;
static bool g_noRecoil = false;
static bool g_noSpread = false;
static bool g_infiniteAmmo = false;
static bool g_fastReload = false;
static bool g_noFallDamage = false;
static bool g_fastFire = false;
static float g_fireRate = 1.0f;

struct IL2CPP {
    void* base = nullptr;
    size_t size = 0;

    void* (*domain_get)();
    void* (*domain_assembly_open)(void*, const char*);
    void* (*assembly_get_image)(void*);
    void* (*class_from_name)(void*, const char*, const char*);
    void* (*class_get_method_from_name)(void*, const char*, int);
    void* (*thread_attach)(void*);
    void* (*object_new)(void*);
    void* (*runtime_invoke)(void*, void*, void**, void**);
    void* (*string_new)(const char*);
    void* (*field_get_offset)(void*, void*);
    void* (*field_get_value)(void*, void*);
    void* (*field_set_value)(void*, void*, void*);
} g_api = {0};

static void* findLibBase(const char* name, size_t* outSize) {
    FILE* fp = fopen("/proc/self/maps", "r");
    if (!fp) return nullptr;

    char line[512];
    void* base = nullptr;
    size_t totalSize = 0;

    while (fgets(line, sizeof(line), fp)) {
        if (strstr(line, name)) {
            unsigned long start, end;
            if (sscanf(line, "%lx-%lx", &start, &end) == 2) {
                if (!base) base = (void*)start;
                totalSize = end - start;
            }
        }
    }
    fclose(fp);

    if (base && outSize) *outSize = totalSize;
    return base;
}

static bool loadIL2CPP() {
    g_api.base = findLibBase("libil2cpp.so", &g_api.size);
    if (!g_api.base) {
        LOGE("Khong tim thay libil2cpp.so");
        return false;
    }
    LOGI("libil2cpp.so base: %p size: 0x%zx", g_api.base, g_api.size);

    void* h = dlopen("libil2cpp.so", RTLD_NOW | RTLD_GLOBAL);
    if (!h) {
        LOGE("dlopen that bai: %s", dlerror());
        return false;
    }

    g_api.domain_get = (void*(*)())dlsym(h, "il2cpp_domain_get");
    g_api.domain_assembly_open = (void*(*)(void*, const char*))dlsym(h, "il2cpp_domain_assembly_open");
    g_api.assembly_get_image = (void*(*)(void*))dlsym(h, "il2cpp_assembly_get_image");
    g_api.class_from_name = (void*(*)(void*, const char*, const char*))dlsym(h, "il2cpp_class_from_name");
    g_api.class_get_method_from_name = (void*(*)(void*, const char*, int))dlsym(h, "il2cpp_class_get_method_from_name");
    g_api.thread_attach = (void*(*)(void*))dlsym(h, "il2cpp_thread_attach");
    g_api.object_new = (void*(*)(void*))dlsym(h, "il2cpp_object_new");
    g_api.runtime_invoke = (void*(*)(void*, void*, void**, void**))dlsym(h, "il2cpp_runtime_invoke");
    g_api.string_new = (void*(*)(const char*))dlsym(h, "il2cpp_string_new");
    g_api.field_get_offset = (void*(*)(void*, void*))dlsym(h, "il2cpp_field_get_offset");
    g_api.field_get_value = (void*(*)(void*, void*))dlsym(h, "il2cpp_field_get_value");
    g_api.field_set_value = (void*(*)(void*, void*, void*))dlsym(h, "il2cpp_field_set_value");

    int count = 0;
    if (g_api.domain_get) count++;
    if (g_api.domain_assembly_open) count++;
    if (g_api.assembly_get_image) count++;
    if (g_api.class_from_name) count++;
    if (g_api.class_get_method_from_name) count++;
    if (g_api.thread_attach) count++;
    if (g_api.object_new) count++;
    if (g_api.runtime_invoke) count++;
    if (g_api.string_new) count++;
    if (g_api.field_get_offset) count++;
    if (g_api.field_get_value) count++;
    if (g_api.field_set_value) count++;

    LOGI("Loaded %d/12 IL2CPP functions", count);
    return count >= 5;
}

static void* getClass(const char* ns, const char* name) {
    if (!g_api.domain_get) return nullptr;
    void* domain = g_api.domain_get();
    if (!domain) return nullptr;
    void* assembly = g_api.domain_assembly_open(domain, "Assembly-CSharp.dll");
    if (!assembly) return nullptr;
    void* image = g_api.assembly_get_image(assembly);
    if (!image) return nullptr;
    return g_api.class_from_name(image, ns, name);
}

static void* getMethod(void* klass, const char* name, int argc) {
    if (!klass || !g_api.class_get_method_from_name) return nullptr;
    return g_api.class_get_method_from_name(klass, name, argc);
}

static bool patchMethodPointer(void* methodInfo, void* newFunc, void** origFunc) {
    if (!methodInfo || !newFunc) return false;

    uintptr_t pageStart = (uintptr_t)methodInfo & ~0xFFF;
    if (mprotect((void*)pageStart, 0x1000, PROT_READ | PROT_WRITE | PROT_EXEC) != 0) {
        LOGE("mprotect failed: %s", strerror(errno));
        return false;
    }

    void** ptr = (void**)methodInfo;
    if (origFunc) *origFunc = ptr[0];
    ptr[0] = newFunc;

    LOGI("Patched %p: %p -> %p", methodInfo, *origFunc, newFunc);
    return true;
}

static void* (*g_orig_GetHeadCollider)(void*, void*) = nullptr;
static void* g_lastHead = nullptr;

static void* hook_GetHeadCollider(void* __this, void* methodInfo) {
    void* result = g_orig_GetHeadCollider ? g_orig_GetHeadCollider(__this, methodInfo) : nullptr;
    if (result) g_lastHead = result;
    return result;
}

static void (*g_orig_SetLocked)(void*, void*, void*) = nullptr;

static void hook_SetLocked(void* __this, void* collider, void* methodInfo) {
    if (g_headshot && g_lastHead && g_patchEnabled) {
        if (g_orig_SetLocked) g_orig_SetLocked(__this, g_lastHead, methodInfo);
        return;
    }
    if (g_orig_SetLocked) g_orig_SetLocked(__this, collider, methodInfo);
}

static int (*g_orig_GetAmmo)(void*, void*) = nullptr;

static int hook_GetAmmo(void* __this, void* methodInfo) {
    if (g_infiniteAmmo && g_patchEnabled) {
        return 999;
    }
    return g_orig_GetAmmo ? g_orig_GetAmmo(__this, methodInfo) : 0;
}

static void (*g_orig_SetAmmo)(void*, int, void*) = nullptr;

static void hook_SetAmmo(void* __this, int ammo, void* methodInfo) {
    if (g_infiniteAmmo && g_patchEnabled) {
        if (g_orig_SetAmmo) g_orig_SetAmmo(__this, 999, methodInfo);
        return;
    }
    if (g_orig_SetAmmo) g_orig_SetAmmo(__this, ammo, methodInfo);
}

static float (*g_orig_GetRecoil)(void*, void*) = nullptr;

static float hook_GetRecoil(void* __this, void* methodInfo) {
    if (g_noRecoil && g_patchEnabled) return 0.0f;
    return g_orig_GetRecoil ? g_orig_GetRecoil(__this, methodInfo) : 0.0f;
}

static float (*g_orig_GetSpread)(void*, void*) = nullptr;

static float hook_GetSpread(void* __this, void* methodInfo) {
    if (g_noSpread && g_patchEnabled) return 0.0f;
    return g_orig_GetSpread ? g_orig_GetSpread(__this, methodInfo) : 0.0f;
}

static float (*g_orig_GetFireRate)(void*, void*) = nullptr;

static float hook_GetFireRate(void* __this, void* methodInfo) {
    float rate = g_orig_GetFireRate ? g_orig_GetFireRate(__this, methodInfo) : 1.0f;
    if (g_fastFire && g_patchEnabled) return rate * g_fireRate;
    return rate;
}

static float (*g_orig_GetReloadTime)(void*, void*) = nullptr;

static float hook_GetReloadTime(void* __this, void* methodInfo) {
    float t = g_orig_GetReloadTime ? g_orig_GetReloadTime(__this, methodInfo) : 1.0f;
    if (g_fastReload && g_patchEnabled) return t * 0.1f;
    return t;
}

static float (*g_orig_GetFallDamage)(void*, float, void*) = nullptr;

static float hook_GetFallDamage(void* __this, float speed, void* methodInfo) {
    if (g_noFallDamage && g_patchEnabled) return 0.0f;
    return g_orig_GetFallDamage ? g_orig_GetFallDamage(__this, speed, methodInfo) : 0.0f;
}

static bool (*g_orig_IsFiring)(void*, void*) = nullptr;
static bool g_isFiring = false;

static bool hook_IsFiring(void* __this, void* methodInfo) {
    bool r = g_orig_IsFiring ? g_orig_IsFiring(__this, methodInfo) : false;
    g_isFiring = r;
    return r;
}

static float (*g_orig_GetCurHP)(void*, void*) = nullptr;

static float hook_GetCurHP(void* __this, void* methodInfo) {
    return g_orig_GetCurHP ? g_orig_GetCurHP(__this, methodInfo) : 0.0f;
}

static float (*g_orig_GetMaxHP)(void*, void*) = nullptr;

static float hook_GetMaxHP(void* __this, void* methodInfo) {
    return g_orig_GetMaxHP ? g_orig_GetMaxHP(__this, methodInfo) : 100.0f;
}

static void* hookThread(void*) {
    LOGI("Patch thread bat dau");
    sleep(10);

    if (!loadIL2CPP()) {
        LOGE("Khong load duoc IL2CPP");
        return nullptr;
    }

    if (g_api.thread_attach && g_api.domain_get) {
        g_api.thread_attach(g_api.domain_get());
    }

    void* playerClass = getClass("COW.GamePlay", "Player");
    if (!playerClass) {
        LOGE("Khong tim thay Player");
        return nullptr;
    }
    LOGI("Tim thay Player: %p", playerClass);

    void* mHead = getMethod(playerClass, "get_HeadCollider", 0);
    if (mHead) patchMethodPointer(mHead, (void*)hook_GetHeadCollider, (void**)&g_orig_GetHeadCollider);

    void* mSetLock = getMethod(playerClass, "set_LockedAimingCollider", 1);
    if (mSetLock) patchMethodPointer(mSetLock, (void*)hook_SetLocked, (void**)&g_orig_SetLocked);

    void* mIsFiring = getMethod(playerClass, "IsFiring", 0);
    if (mIsFiring) patchMethodPointer(mIsFiring, (void*)hook_IsFiring, (void**)&g_orig_IsFiring);

    void* mCurHP = getMethod(playerClass, "get_CurHP", 0);
    if (mCurHP) patchMethodPointer(mCurHP, (void*)hook_GetCurHP, (void**)&g_orig_GetCurHP);

    void* mMaxHP = getMethod(playerClass, "get_MaxHP", 0);
    if (mMaxHP) patchMethodPointer(mMaxHP, (void*)hook_GetMaxHP, (void**)&g_orig_GetMaxHP);

    void* weaponClass = getClass("COW.GamePlay", "Weapon");
    if (weaponClass) {
        void* mAmmo = getMethod(weaponClass, "get_CurAmmo", 0);
        if (mAmmo) patchMethodPointer(mAmmo, (void*)hook_GetAmmo, (void**)&g_orig_GetAmmo);

        void* mSetAmmo = getMethod(weaponClass, "set_CurAmmo", 1);
        if (mSetAmmo) patchMethodPointer(mSetAmmo, (void*)hook_SetAmmo, (void**)&g_orig_SetAmmo);

        void* mRecoil = getMethod(weaponClass, "get_Recoil", 0);
        if (mRecoil) patchMethodPointer(mRecoil, (void*)hook_GetRecoil, (void**)&g_orig_GetRecoil);

        void* mSpread = getMethod(weaponClass, "get_Spread", 0);
        if (mSpread) patchMethodPointer(mSpread, (void*)hook_GetSpread, (void**)&g_orig_GetSpread);

        void* mFireRate = getMethod(weaponClass, "get_FireRate", 0);
        if (mFireRate) patchMethodPointer(mFireRate, (void*)hook_GetFireRate, (void**)&g_orig_GetFireRate);

        void* mReload = getMethod(weaponClass, "get_ReloadTime", 0);
        if (mReload) patchMethodPointer(mReload, (void*)hook_GetReloadTime, (void**)&g_orig_GetReloadTime);
    }

    LOGI("Patch hoan tat");
    return nullptr;
}

__attribute__((constructor))
static void onLoad() {
    LOGI("=== Aim Patch loaded ===");
    g_patchEnabled = true;
    g_headshot = true;

    pthread_t t;
    pthread_create(&t, nullptr, hookThread, nullptr);
}

__attribute__((destructor))
static void onUnload() {
    LOGI("Unloading");
    g_patchEnabled = false;
}

extern "C" {

JNIEXPORT void JNICALL
Java_com_aimpatch_Native_enable(JNIEnv*, jclass, jboolean enable) {
    g_patchEnabled = enable;
}

JNIEXPORT void JNICALL
Java_com_aimpatch_Native_setHeadshot(JNIEnv*, jclass, jboolean e) { g_headshot = e; }

JNIEXPORT void JNICALL
Java_com_aimpatch_Native_setSilentAim(JNIEnv*, jclass, jboolean e) { g_silentAim = e; }

JNIEXPORT void JNICALL
Java_com_aimpatch_Native_setNoRecoil(JNIEnv*, jclass, jboolean e) { g_noRecoil = e; }

JNIEXPORT void JNICALL
Java_com_aimpatch_Native_setNoSpread(JNIEnv*, jclass, jboolean e) { g_noSpread = e; }

JNIEXPORT void JNICALL
Java_com_aimpatch_Native_setInfiniteAmmo(JNIEnv*, jclass, jboolean e) { g_infiniteAmmo = e; }

JNIEXPORT void JNICALL
Java_com_aimpatch_Native_setFastReload(JNIEnv*, jclass, jboolean e) { g_fastReload = e; }

JNIEXPORT void JNICALL
Java_com_aimpatch_Native_setNoFallDamage(JNIEnv*, jclass, jboolean e) { g_noFallDamage = e; }

JNIEXPORT void JNICALL
Java_com_aimpatch_Native_setFastFire(JNIEnv*, jclass, jboolean e, jfloat rate) {
    g_fastFire = e;
    g_fireRate = rate;
}

JNIEXPORT jboolean JNICALL
Java_com_aimpatch_Native_isFiring(JNIEnv*, jclass) {
    return g_isFiring ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jlong JNICALL
Java_com_aimpatch_Native_getIL2CPPBase(JNIEnv*, jclass) {
    return (jlong)g_api.base;
}

JNIEXPORT jboolean JNICALL
Java_com_aimpatch_Native_isRunning(JNIEnv*, jclass) {
    return g_api.base != nullptr ? JNI_TRUE : JNI_FALSE;
}

}
