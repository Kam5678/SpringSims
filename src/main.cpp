#include "givio.h"
#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include "panel.h"
#include "picking_controls.h"
#include "turntable_controls.h"

#include "models.h"
#include "renderable_models.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;
using namespace simulation;

//
// program entry point
//
int main(void) {
  //
  // initialize OpenGL and window
  //
  namespace givio = giv::io; // perhaps better than giv::io
  givio::GLFWContext glContext;
  glContext.glMajorVesion(4)
      .glMinorVesion(0)
      .glForwardComaptability(true)
      .glCoreProfile()
      .glAntiAliasingSamples(4)
      .matchPrimaryMonitorVideoMode();

  std::cout << givio::glfwVersionString() << '\n';

  //
  // setup window (OpenGL context)
  //
  auto window =
      glContext.makeImGuiWindow(givio::Properties()
                                    .size(givio::dimensions{1000, 1000})
                                    .title("Models, models, models... oh my!")
                                    .glslVersionString("#version 330 core"));

  auto view = View(TurnTable(), Perspective());
  // Preset Bindings
  TurnTableControls controls(window, view.camera);

  //
  // setup simulation
  //

  //  Custom Bind keys
  window.keyboardCommands() |
      givio::Key(GLFW_KEY_V, [&](auto) { view.camera.reset(); }) |
      givio::Key(GLFW_KEY_P, [&](auto event) {
        if (event.action == GLFW_PRESS) {
          panel::showPanel = !panel::showPanel;
        }
      });

  // instanced monkey
  auto point_geometry = Sphere(Radius(0.25f));
  auto point_style =
      Phong(Colour(1.f, 1.f, 0.f), LightPosition(100.f, 100.f, 100.f));
  auto point_renders = createInstancedRenderable(point_geometry, point_style);

  // this assigns the new model
  auto defaultModel = std::make_unique<SmallAnglePendulumModel>();
  auto modelRenderable = makeModelRenderable(*defaultModel, view);
  std::unique_ptr<Model> model = std::move(defaultModel);

  //
  // main loop
  //
  mainloop(std::move(window), [&](float) {
    //
    // updates from panel
    //
    if (panel::resetView) {
      view.camera.reset();
    }

    if (panel::loadPendulumModel) {
      auto newModel = std::make_unique<SmallAnglePendulumModel>();
      modelRenderable = makeModelRenderable(*newModel, view);
      model = std::move(newModel);
      panel::playModel = false;
    }
    if (panel::loadDoublePendulumModel) {
      auto newModel = std::make_unique<Chain>();
      modelRenderable = makeModelRenderable(*newModel, view);
      model = std::move(newModel);
      panel::playModel = false;
    }
    if (panel::loadParticleModel) {
      auto newModel = std::make_unique<JellyCube>();
      modelRenderable = makeModelRenderable(*newModel, view);
      model = std::move(newModel);
      panel::playModel = false;
    }
    if (panel::loadCloth) {
        auto newModel = std::make_unique<Cloth>();
        modelRenderable = makeModelRenderable(*newModel, view);
        model = std::move(newModel);
        panel::playModel = false;
    }


    if (panel::resetModel) {
      model->reset();
    }

    //
    // simulation (Currently doing a fixed time step)
    //
    if (panel::playModel || panel::stepModel) {
      model->step(panel::dt);
    }

    //
    // render
    //
    auto color = panel::clear_color;
    glClearColor(color.x, color.y, color.z, color.z);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    view.projection.updateAspectRatio(window.width(), window.height());

    render(modelRenderable, view);

  });

  return EXIT_SUCCESS;
}
