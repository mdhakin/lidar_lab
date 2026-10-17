#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "gnc/target_selector.h"
#include "imgui.h"
#include "lidar.h"
#include "scan_filter.h"
static constexpr float PI = 3.14159265358979323846f;

static void glfw_error_callback(int error, const char* description) {
  std::fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

static void DrawLidarView(const LidarScan& scan, const NavigationTarget& target, float& displayRangeM) {
  ImGui::Begin("C1 Lidar");

  ImGui::SliderFloat("Display Range", &displayRangeM, 1.0f, 10.0f, "%.1f m");

  ImGui::Text("Points: %zu", scan.points.size());
  if (target.valid) {
    ImGui::Text("GNC Target: %.1f deg | %.2f m | Confidence: %.2f", target.headingDeg, target.distanceM,
                target.confidence);
  } else {
    ImGui::TextDisabled("GNC Target: No valid candidate");
  }

  const ImVec2 available = ImGui::GetContentRegionAvail();

  const float size = std::min(available.x, available.y);

  const ImVec2 topLeft = ImGui::GetCursorScreenPos();

  const ImVec2 center(topLeft.x + size * 0.5f, topLeft.y + size * 0.5f);

  ImDrawList* drawList = ImGui::GetWindowDrawList();

  const float pixelsPerMeter = (size * 0.5f) / displayRangeM;

  // Background boundary.
  drawList->AddRect(topLeft, ImVec2(topLeft.x + size, topLeft.y + size), IM_COL32(100, 100, 100, 255));

  // Range rings.
  for (int meter = 1; meter <= static_cast<int>(displayRangeM); ++meter) {
    drawList->AddCircle(center, meter * pixelsPerMeter, IM_COL32(70, 70, 70, 255), 96);
  }

  // Crosshairs.
  drawList->AddLine(ImVec2(center.x, topLeft.y), ImVec2(center.x, topLeft.y + size), IM_COL32(70, 70, 70, 255));

  drawList->AddLine(ImVec2(topLeft.x, center.y), ImVec2(topLeft.x + size, center.y), IM_COL32(70, 70, 70, 255));

  // Sensor center.
  drawList->AddCircleFilled(center, 5.0f, IM_COL32(255, 255, 0, 255));

  for (const LidarPoint& point : scan.points) {
    if (point.distanceM > displayRangeM) {
      continue;
    }

    const float radians = point.angleDeg * PI / 180.0f;

    const float x = point.distanceM * std::cos(radians);

    const float y = point.distanceM * std::sin(radians);

    const ImVec2 screenPoint(center.x + y * pixelsPerMeter, center.y - x * pixelsPerMeter);

    drawList->AddCircleFilled(screenPoint, 2.0f, IM_COL32(0, 255, 0, 255));
  }

  // Aries-GNC candidate target.
  if (target.valid) {
    const float radians = target.headingDeg * PI / 180.0f;

    const float x = target.distanceM * std::cos(radians);
    const float y = target.distanceM * std::sin(radians);

    // Use the same C1-to-screen transformation as the LiDAR points.
    const ImVec2 targetScreen(center.x + y * pixelsPerMeter, center.y - x * pixelsPerMeter);

    const ImU32 targetColor = IM_COL32(255, 210, 40, 255);

    // Direction line.
    drawList->AddLine(center, targetScreen, targetColor, 3.0f);

    // Target marker.
    drawList->AddCircleFilled(targetScreen, 6.0f, targetColor);

    drawList->AddCircle(targetScreen, 11.0f, targetColor, 24, 2.0f);
  }

  // Reserve the canvas area in ImGui.
  ImGui::Dummy(ImVec2(size, size));

  ImGui::End();
}

int main() {
  constexpr const char* LIDAR_PORT = "/dev/ttyUSB0";
  constexpr int LIDAR_BAUD = 460800;

  Lidar lidar;

  std::printf("Connecting to C1 on %s...\n", LIDAR_PORT);

  if (!lidar.connect(LIDAR_PORT, LIDAR_BAUD)) {
    std::fprintf(stderr, "Failed to connect to lidar.\n");
    return 1;
  }

  if (!lidar.start()) {
    std::fprintf(stderr, "Failed to start lidar acquisition.\n");
    return 1;
  }

  glfwSetErrorCallback(glfw_error_callback);

  if (!glfwInit()) {
    return 1;
  }

  const char* glslVersion = "#version 130";

  GLFWwindow* window = glfwCreateWindow(900, 900, "Lidar Lab", nullptr, nullptr);

  if (window == nullptr) {
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glslVersion);

  float displayRangeM = 5.0f;
  ScanFilter scanFilter;
  uint64_t lastProcessedSequence = 0;

  LidarScan rawScan;
  LidarScan stableScan;

  TargetSelector targetSelector;
  NavigationTarget currentTarget;
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    LidarScan incomingScan;

    if (lidar.getLatestScan(incomingScan) && incomingScan.sequence != lastProcessedSequence) {
      lastProcessedSequence = incomingScan.sequence;

      rawScan = std::move(incomingScan);

      scanFilter.addScan(rawScan);
      stableScan = scanFilter.getStableScan();
      currentTarget = targetSelector.selectTarget(stableScan);
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    DrawLidarView(stableScan, currentTarget, displayRangeM);

    ImGui::Render();

    int displayWidth;
    int displayHeight;

    glfwGetFramebufferSize(window, &displayWidth, &displayHeight);

    glViewport(0, 0, displayWidth, displayHeight);

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  lidar.stop();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}