name: Build Aimlock APK

on:
  push:
    branches: [ main ]
  workflow_dispatch:

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-java@v4
        with:
          distribution: temurin
          java-version: 17

      - uses: android-actions/setup-android@v3

      - name: Sinh project
        run: |
          set -e

          mkdir -p app/src/main/java/com/aimlock
          mkdir -p app/src/main/cpp
          mkdir -p app/src/main/res/xml
          mkdir -p app/src/main/res/values
          mkdir -p gradle/wrapper

          cat > settings.gradle <<'EOF'
          pluginManagement {
              repositories { google(); mavenCentral(); gradlePluginPortal() }
          }
          dependencyResolutionManagement {
              repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
              repositories { google(); mavenCentral() }
          }
          rootProject.name = "AimlockHead"
          include ':app'
          EOF

          cat > build.gradle <<'EOF'
          plugins {
              id 'com.android.application' version '8.2.2' apply false
          }
          EOF

          cat > gradle.properties <<'EOF'
          org.gradle.jvmargs=-Xmx2048m
          android.useAndroidX=true
          android.nonTransitiveRClass=true
          EOF

          cat > gradle/wrapper/gradle-wrapper.properties <<'EOF'
          distributionBase=GRADLE_USER_HOME
          distributionPath=wrapper/dists
          distributionUrl=https\://services.gradle.org/distributions/gradle-8.2-bin.zip
          zipStoreBase=GRADLE_USER_HOME
          zipStorePath=wrapper/dists
          EOF

          wget -q https://raw.githubusercontent.com/gradle/gradle/v8.2.0/gradlew
          chmod +x gradlew
          wget -q https://raw.githubusercontent.com/gradle/gradle/v8.2.0/gradle/wrapper/gradle-wrapper.jar -O gradle/wrapper/gradle-wrapper.jar

          cat > app/build.gradle <<'EOF'
          plugins { id 'com.android.application' }

          android {
              namespace 'com.aimlock'
              compileSdk 34
              ndkVersion "25.2.9519653"

              defaultConfig {
                  applicationId "com.aimlock"
                  minSdk 24
                  targetSdk 34
                  versionCode 1
                  versionName "1.0"
                  externalNativeBuild { cmake { cppFlags "-std=c++17" } }
                  ndk { abiFilters 'arm64-v8a' }
              }

              externalNativeBuild {
                  cmake {
                      path "src/main/cpp/CMakeLists.txt"
                      version "3.22.1"
                  }
              }

              buildTypes { release { minifyEnabled false } }
          }
          EOF

          cat > app/src/main/AndroidManifest.xml <<'EOF'
          <?xml version="1.0" encoding="utf-8"?>
          <manifest xmlns:android="http://schemas.android.com/apk/res/android"
              package="com.aimlock">

              <uses-permission android:name="android.permission.SYSTEM_ALERT_WINDOW"/>
              <uses-permission android:name="android.permission.FOREGROUND_SERVICE"/>
              <uses-permission android:name="android.permission.FOREGROUND_SERVICE_MEDIA_PROJECTION"/>

              <application android:label="AimlockHead"
                  android:theme="@android:style/Theme.Material.Light">

                  <activity android:name=".MainActivity" android:exported="true">
                      <intent-filter>
                          <action android:name="android.intent.action.MAIN"/>
                          <category android:name="android.intent.category.LAUNCHER"/>
                      </intent-filter>
                  </activity>

                  <service android:name=".AimService" android:exported="true"
                      android:permission="android.permission.BIND_ACCESSIBILITY_SERVICE">
                      <intent-filter>
                          <action android:name="android.accessibilityservice.AccessibilityService"/>
                      </intent-filter>
                      <meta-data android:name="android.accessibilityservice"
                          android:resource="@xml/accessibility_service_config"/>
                  </service>
              </application>
          </manifest>
          EOF

          cat > app/src/main/res/xml/accessibility_service_config.xml <<'EOF'
          <?xml version="1.0" encoding="utf-8"?>
          <accessibility-service xmlns:android="http://schemas.android.com/apk/res/android"
              android:accessibilityEventTypes="typeAllMask"
              android:accessibilityFeedbackType="feedbackGeneric"
              android:accessibilityFlags="flagDefault"
              android:canPerformGestures="true"
              android:canRetrieveWindowContent="true"
              android:notificationTimeout="50"/>
          EOF

          cat > app/src/main/res/values/strings.xml <<'EOF'
          <?xml version="1.0" encoding="utf-8"?>
          <resources>
              <string name="app_name">AimlockHead</string>
          </resources>
          EOF

          cat > app/src/main/cpp/CMakeLists.txt <<'EOF'
          cmake_minimum_required(VERSION 3.22)
          project(aimlock_head)

          add_library(aimlock_head SHARED aimlock_head.cpp)
          target_link_libraries(aimlock_head log)
          target_compile_options(aimlock_head PRIVATE -O2 -std=c++17)
          EOF

          cat > app/src/main/cpp/aimlock_head.cpp <<'EOF'
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
          EOF

          cat > app/src/main/java/com/aimlock/Native.java <<'EOF'
          package com.aimlock;

          public class Native {
              static { System.loadLibrary("aimlock_head"); }
              public static native int[] processFrame(byte[] rgba, int w, int h);
              public static native void setConfig(int fovX, int fovY, int smooth, int tol);
              public static native void setSkinRange(int rMin, int rMax,
                                                     int gMin, int gMax,
                                                     int bMin, int bMax);
              public static native void start();
              public static native void stop();
              public static native void clearTarget();
              public static native boolean isLocked();
          }
          EOF

          cat > app/src/main/java/com/aimlock/AimService.java <<'EOF'
          package com.aimlock;

          import android.accessibilityservice.AccessibilityService;
          import android.accessibilityservice.GestureDescription;
          import android.graphics.Path;
          import android.graphics.PixelFormat;
          import android.hardware.display.DisplayManager;
          import android.hardware.display.VirtualDisplay;
          import android.media.Image;
          import android.media.ImageReader;
          import android.media.projection.MediaProjection;
          import android.os.Handler;
          import android.os.Looper;
          import android.util.DisplayMetrics;
          import android.view.accessibility.AccessibilityEvent;
          import java.nio.ByteBuffer;

          public class AimService extends AccessibilityService {
              public static AimService instance;
              private MediaProjection projection;
              private VirtualDisplay display;
              private ImageReader reader;
              private int W, H, D;
              private final Handler h = new Handler(Looper.getMainLooper());
              private volatile boolean running = false;

              @Override public void onServiceConnected() {
                  instance = this;
                  DisplayMetrics dm = getResources().getDisplayMetrics();
                  W = dm.widthPixels; H = dm.heightPixels; D = dm.densityDpi;
                  Native.setConfig(240, 240, 4, 40);
                  Native.setSkinRange(95, 255, 40, 200, 20, 170);
              }

              public void startCapture(MediaProjection mp) {
                  projection = mp;
                  reader = ImageReader.newInstance(W, H, PixelFormat.RGBA_8888, 3);
                  display = projection.createVirtualDisplay("aimlock",
                          W, H, D,
                          DisplayManager.VIRTUAL_DISPLAY_FLAG_AUTO_MIRROR,
                          reader.getSurface(), null, h);
                  running = true;
                  Native.start();
                  h.post(this::loop);
              }

              private void loop() {
                  if (!running) return;
                  Image img = reader.acquireLatestImage();
                  if (img != null) {
                      ByteBuffer buf = img.getPlanes()[0].getBuffer();
                      byte[] data = new byte[buf.remaining()];
                      buf.get(data);
                      int[] target = Native.processFrame(data, W, H);
                      if (target != null && target[0] >= 0) tap(target[0], target[1]);
                      img.close();
                  }
                  h.postDelayed(this::loop, 8);
              }

              private void tap(int x, int y) {
                  Path p = new Path(); p.moveTo(x, y);
                  GestureDescription.StrokeDescription s =
                          new GestureDescription.StrokeDescription(p, 0, 18);
                  GestureDescription.Builder b = new GestureDescription.Builder();
                  b.addStroke(s);
                  dispatchGesture(b.build(), null, null);
              }

              @Override public void onAccessibilityEvent(AccessibilityEvent e) {}
              @Override public void onInterrupt() { running = false; Native.stop(); }
          }
          EOF

          cat > app/src/main/java/com/aimlock/MainActivity.java <<'EOF'
          package com.aimlock;

          import android.app.Activity;
          import android.content.Intent;
          import android.media.projection.MediaProjectionManager;
          import android.os.Bundle;
          import android.provider.Settings;
          import android.widget.Button;
          import android.widget.LinearLayout;

          public class MainActivity extends Activity {
              private static final int REQ = 1001;
              private MediaProjectionManager mpm;

              @Override protected void onCreate(Bundle b) {
                  super.onCreate(b);
                  LinearLayout ll = new LinearLayout(this);
                  ll.setOrientation(LinearLayout.VERTICAL);

                  Button a = new Button(this);
                  a.setText("1. Bat Accessibility");
                  a.setOnClickListener(v -> startActivity(
                          new Intent(Settings.ACTION_ACCESSIBILITY_SETTINGS)));
                  ll.addView(a);

                  Button m = new Button(this);
                  m.setText("2. Bat MediaProjection");
                  mpm = (MediaProjectionManager) getSystemService(MEDIA_PROJECTION_SERVICE);
                  m.setOnClickListener(v -> startActivityForResult(
                          mpm.createScreenCaptureIntent(), REQ));
                  ll.addView(m);

                  setContentView(ll);
              }

              @Override protected void onActivityResult(int req, int res, Intent data) {
                  if (req == REQ && res == RESULT_OK && AimService.instance != null) {
                      AimService.instance.startCapture(mpm.getMediaProjection(res, data));
                  }
              }
          }
          EOF

      - name: Build APK
        run: ./gradlew assembleDebug --no-daemon --stacktrace

      - uses: actions/upload-artifact@v4
        with:
          name: aimlock-apk
          path: app/build/outputs/apk/debug/*.apk
