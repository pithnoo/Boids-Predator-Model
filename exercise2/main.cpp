#include <glad/glad.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <numbers>
#include <stdexcept>
#include <typeinfo>

#include <cstdio>
#include <cstdlib>

#include <vector>

#include "defaults.hpp"

#include "../support/checkpoint.hpp"
#include "../support/debug_output.hpp"
#include "../support/error.hpp"
#include "../support/program.hpp"

#include "../vmlib/mat22.hpp"
#include "../vmlib/vec2.hpp"

#include "boid.hpp"
#include "predator.hpp"

namespace {
constexpr char const *kWindowTitle = "Boids Program";

struct State_ {
  ShaderProgram *prog;
  bool isPaused = false;

  BoidSystem *bs;
  Predator *p;
};

void updateGui(State_ &, bool &, float &, float &, float &, float &, float &, float &,
               float &, float &, float);

void glfw_callback_error_(int, char const *);

void glfw_callback_key_(GLFWwindow *, int, int, int, int);

struct GLFWCleanupHelper {
  ~GLFWCleanupHelper();
};
struct GLFWWindowDeleter {
  ~GLFWWindowDeleter();
  GLFWwindow *window;
};
} // namespace

int main() try {

  // Initialize GLFW
  if (GLFW_TRUE != glfwInit()) {
    char const *msg = nullptr;
    int ecode = glfwGetError(&msg);
    throw Error("glfwInit() failed with '%s' (%d)", msg, ecode);
  }

  // Ensure that we call glfwTerminate() at the end of the program.
  GLFWCleanupHelper cleanupHelper;

  // Configure GLFW and create window
  glfwSetErrorCallback(&glfw_callback_error_);

  glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

#if !defined(__APPLE__)
  // Most platforms will support OpenGL 4.3
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#else  // defined(__APPLE__)
  // Apple has at most OpenGL 4.1, so don't ask for something newer.
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#endif // ~ __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#if !defined(NDEBUG)
  // When building in debug mode, request an OpenGL debug context. This
  // enables additional debugging features. However, this can carry extra
  // overheads. We therefore do not do this for release builds.
  glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
#endif // ~ !NDEBUG

  GLFWwindow *window =
      glfwCreateWindow(1280, 720, kWindowTitle, nullptr, nullptr);

  if (!window) {
    char const *msg = nullptr;
    int ecode = glfwGetError(&msg);
    throw Error("glfwCreateWindow() failed with '%s' (%d)", msg, ecode);
  }

  GLFWWindowDeleter windowDeleter{window};

  // Set up event handling
  State_ state;

  glfwSetWindowUserPointer(window, &state);

  glfwSetKeyCallback(window, &glfw_callback_key_);

  // Set up drawing stuff
  glfwMakeContextCurrent(window);

  glfwSwapInterval(1); // V-Sync is on.

  // Initialize GLAD
  // This will load the OpenGL API. We mustn't make any OpenGL calls before
  // this!
  if (!gladLoadGLLoader((GLADloadproc)&glfwGetProcAddress))
    throw Error("gladLoadGLLoader() failed - cannot load GL API!");

    // print out specs
  std::printf("RENDERER %s\n", glGetString(GL_RENDERER));
  std::printf("VENDOR %s\n", glGetString(GL_VENDOR));
  std::printf("VERSION %s\n", glGetString(GL_VERSION));
  std::printf("SHADING_LANGUAGE_VERSION %s\n",
			  glGetString(GL_SHADING_LANGUAGE_VERSION));

    // Debug output
#if !defined(NDEBUG)
  setup_gl_debug_output();
#endif // ~ !NDEBUG

  // Global GL state
  OGL_CHECKPOINT_ALWAYS();

  // for conversion of fragment shader colours to srgb
  glEnable(GL_FRAMEBUFFER_SRGB);

  // cull based on clockwise / anti-clockwise
  glEnable(GL_CULL_FACE);

  // pixels will be cleared to this by default
  glClearColor(0.25f, 0.25f, 0.25f, 1.f);

  // imgui setup
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.FontGlobalScale = 1.5f;

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init();

  OGL_CHECKPOINT_ALWAYS();

  // Get actual framebuffer size.
  // This can be different from the window size, as standard window
  // decorations (title bar, borders, ...) may be included in the window size
  // but not be part of the drawable surface area.
  int iwidth, iheight;
  glfwGetFramebufferSize(window, &iwidth, &iheight);

  glViewport(0, 0, iwidth, iheight);

  // Load shader program
  ShaderProgram prog({{GL_VERTEX_SHADER, "assets/ex2/passthrough.vert"},
                      {GL_FRAGMENT_SHADER, "assets/ex2/modulate.frag"}});

  state.prog = &prog;

  BoidSystem bs(300);
  Predator p(state.prog);

  state.bs = &bs;
  state.p = &p;

  // boid default values
  float boidSpeed = 0.25f;
  float boidVision = (5.f * M_PI) / 6.f;
  float diveSpeed = boidSpeed;
  float seperationFactor = 1.9f;
  float alignmentFactor = 0.7f;
  float cohesionFactor = 0.4f;
  float boundaryForce = 0.25f;
  float steeringFactor = 0.1f;
  float predatorFactor = 0.25f;
  bool predatorActive = false;

  // Animation state
  auto last = Clock::now();
  float timeElapsed = 0.f;
  float displayFps = 0.f;

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    // Let GLFW process events
    glfwPollEvents();

    // Check if window was resized.
    {
      int nwidth, nheight;
      glfwGetFramebufferSize(window, &nwidth, &nheight);

      if (0 == nwidth || 0 == nheight) {
        // Window minimized? Pause until it is unminimized.
        // This is a bit of a hack.
        do {
          glfwWaitEvents();
          glfwGetFramebufferSize(window, &nwidth, &nheight);
        } while (0 == nwidth || 0 == nheight);
      }

      glViewport(0, 0, nwidth, nheight);
    }

    // start draw new frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Update state
    auto const now = Clock::now();
    float dt = std::chrono::duration_cast<Secondsf>(now - last).count();
    timeElapsed += dt;
    last = now;

    if (timeElapsed >= 1.f) {
      displayFps = 1.f / dt;
      timeElapsed = 0.f;
    }

    updateGui(state, predatorActive, boidVision, boidSpeed, seperationFactor, alignmentFactor,
              cohesionFactor, boundaryForce, steeringFactor, predatorFactor, displayFps);

    // Draw scene
    OGL_CHECKPOINT_DEBUG();

    // TODO: draw frame
    glClear(GL_COLOR_BUFFER_BIT);

    bs.update(state.prog, p.position, dt, boidSpeed, boidVision, predatorFactor,
              seperationFactor, alignmentFactor, cohesionFactor, boundaryForce,
              steeringFactor, predatorActive, state.isPaused);

    if(predatorActive){
      p.update(bs, dt, boidSpeed * 1.05f, diveSpeed, boundaryForce * 4.f,
             state.isPaused);
    }
    else{
      state.p->resetPosition();
    }

    // render gui window
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    OGL_CHECKPOINT_DEBUG();

    // Display results
    glfwSwapBuffers(window);
  }

  // Cleanup.
  state.prog = nullptr;

  return 0;
} catch (std::exception const &eErr) {
  std::fprintf(stderr, "Top-level Exception (%s):\n", typeid(eErr).name());
  std::fprintf(stderr, "%s\n", eErr.what());
  std::fprintf(stderr, "Bye.\n");
  return 1;
}

namespace {
void glfw_callback_error_(int aErrNum, char const *aErrDesc) {
  std::fprintf(stderr, "GLFW error: %s (%d)\n", aErrDesc, aErrNum);
}

  void updateGui(State_ &state, bool &predatorActive, float &boidVision, float &boidSpeed,
               float &seperationFactor, float &alignmentFactor,
               float &cohesionFactor, float &boundaryForce,
               float &steeringFactor, float &predatorFactor, float fps) {

  ImGui::Begin("Boid Settings");
  ImGui::Text("%.1f FPS", fps);

  // add pause button
  if (ImGui::Button("Pause")) {
    state.isPaused = !state.isPaused;
  }
  ImGui::SameLine();
  // add reset button
  if (ImGui::Button("Reset")) {
    state.bs->resetPositions();
    state.p->resetPosition();
  }

  ImGui::Checkbox("Predator Active", &predatorActive);

  ImGui::Text("Boid Properties");
  // implement boid vision angle
  ImGui::SliderFloat("Boid Speed", &boidSpeed, 0.0f, 3.f);
  ImGui::SliderFloat("Boid Vision Angle", &boidVision, 0.0f, 2 * M_PI);
  ImGui::Text("Boid Rules");
  ImGui::SliderFloat("Seperation Factor", &seperationFactor, 0.0f, 10.0f);
  ImGui::SliderFloat("Alignment Factor", &alignmentFactor, 0.0f, 3.0f);
  ImGui::SliderFloat("Cohesion Factor", &cohesionFactor, 0.0f, 3.0f);
  ImGui::SliderFloat("Steering Factor", &steeringFactor, 0.0f, 3.f);
  ImGui::Text("Misc");
  ImGui::SliderFloat("Boundary Factor", &boundaryForce, 0.0f, 3.f);
  ImGui::SliderFloat("Predator Factor", &predatorFactor, 0.0f, 3.f);
  ImGui::End();
}

void glfw_callback_key_(GLFWwindow *aWindow, int aKey, int, int aAction, int) {
  if (GLFW_KEY_ESCAPE == aKey && GLFW_PRESS == aAction) {
    /*
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    */
    glfwSetWindowShouldClose(aWindow, GLFW_TRUE);
    return;
  }

  if (auto *state = static_cast<State_ *>(glfwGetWindowUserPointer(aWindow))) {
    // R-key reloads shaders.
    if (GLFW_KEY_R == aKey && GLFW_PRESS == aAction) {
      if (state->prog) {
        try {
          state->prog->reload();
          state->bs->resetPositions();
          state->p->resetPosition();
          std::fprintf(stderr, "Shaders reloaded and recompiled.\n");

        } catch (std::exception const &eErr) {
          std::fprintf(stderr, "Error when reloading shader:\n");
          std::fprintf(stderr, "%s\n", eErr.what());
          std::fprintf(stderr, "Keeping old shader.\n");
        }
      }
    }

    if (GLFW_KEY_P == aKey && GLFW_PRESS == aAction) {
      state->isPaused = !state->isPaused;
    }
  }
}
} // namespace

namespace {
GLFWCleanupHelper::~GLFWCleanupHelper() { glfwTerminate(); }

GLFWWindowDeleter::~GLFWWindowDeleter() {
  if (window)
    glfwDestroyWindow(window);
}
} // namespace
