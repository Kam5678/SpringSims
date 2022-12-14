#pragma once

#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include "models.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

namespace simulation {

//
// Pendulum model
//
template <typename View>
void render(SmallAnglePendulumModel const &model, View const &view) {


  auto mass_geometry = Sphere(Radius(1.f));
  auto mass_style = Phong(Colour(1.f, 0.f, 1.f), //
                          LightPosition(100.f, 100.f, 100.f));
  static auto mass_renderable =
      createInstancedRenderable(mass_geometry, mass_style);

  //auto point = pendulumPosition(model.theta, model.armLength);
  
  //std::cout << model.middle.b1.x.y << "\n";
  auto point = model.moving.x;

  auto M = translate(mat4f{1.f}, point);
  addInstance(mass_renderable, M);

  auto N = translate(mat4f{ 1.f }, vec3f(0.f,0.f,0.f));
  addInstance(mass_renderable, N);


  static auto arm_renderable = createRenderable(
      Line(Point1(0.f, 0.f, 0.f), Point2(0.f, -1.f, 0.f)), // geometry
      LineStyle(Colour(1.f, 0.f, 1.f))                     // style
  );

  auto arm_geometry = Line(Point1(0.f, 0.f, 0.f), Point2(point));

  updateRenderable(arm_geometry,                     // new position
                   LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
                   arm_renderable);

  draw(arm_renderable, view);
  draw(mass_renderable, view);
}

//
// Chain
//
template <typename View>
void render(Chain const &model, View const &view) {

  auto mass_geometry = Sphere(Radius(1.f));
  auto mass_style = Phong(Colour(1.f, 0.f, 1.f), //
                          LightPosition(100.f, 100.f, 100.f));
  static auto mass_renderable =
      createInstancedRenderable(mass_geometry, mass_style);

  /*
  auto m0 = model.mass0Position();
  auto m1 = model.mass1Position();

  auto M = translate(mat4f{1.f}, m0);
  addInstance(mass_renderable, M);
  M = translate(mat4f{1.f}, m1);
  addInstance(mass_renderable, M);
  */
  for (int i = 0; i < model.particleList.size(); i++) {
      auto M = translate(mat4f{ 1.f }, model.particleList[i].x);
      addInstance(mass_renderable, M);
  }
  

  static auto arm_renderable =
      createRenderable(PolyLine<PrimitiveType::LINE_STRIP>(), // geometry
                       LineStyle(Colour(1.f, 1.f, 0.f))       // style
      );

  //auto arm_geometry = PolyLine<PrimitiveType::LINE_STRIP>(Point(0.f, 0.f, 0.f),
   //                                                       Point(m0), Point(m1));
  
 auto arm_geometry = PolyLine<PrimitiveType::LINE_STRIP>();
 for (int i = 0; i < model.particleList.size(); i++) {
     arm_geometry.push_back(Point(model.particleList[i].x));
 }
  updateRenderable(arm_geometry,                     // new position
                   LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
                   arm_renderable);

  draw(arm_renderable, view);
  draw(mass_renderable, view);
}

//
// Particle Model
//
template <typename View>
void render(JellyCube const &model, View const &view) {
  // static: only initiallized once, the first time this function is called.
  // This isn't the most elegant method to do this but it works. Just put your
  // draw calls in here.

  static auto point_renders = createInstancedRenderable(
      Sphere(Radius(1.f)), // geometry
      Phong(Colour(1.f, 0.f, 0.f),
            LightPosition(100.f, 100.f, 100.f)) // style
  );

  // load up renderable
  auto const &particles = model.particleList;

  for (auto const &particle : particles) {
    auto M = translate(mat4f{1.f}, particle.x);
    addInstance(point_renders, M);
  }

  static auto arm_renderable = createRenderable(
      Line(Point1(0.f, 0.f, 0.f), Point2(0.f, -1.f, 0.f)), // geometry
      LineStyle(Colour(1.f, 0.f, 1.f))                     // style
  );

  auto l = MultiLine();
  for (int i = 0; i < model.springList.size(); i++) {
      //auto arm_geometry = Line(Point1(model.springList[i].a1->x), Point2(model.springList[i].b1->x));

      //updateRenderable(arm_geometry,          // new position
      //    LineStyle(Colour(0.f, 1.f, 0.f)), // new shading
     //     arm_renderable);
      
     // draw(arm_renderable, view);
      l.push_back(Line(Point1(model.springList[i].a1->x), Point2(model.springList[i].b1->x)));
  }
  auto line_renderable = createRenderable(
      l,
      LineStyle(Colour(1.f, 1.f, 0.f))
  );
  
  

  auto ts = TriangleSoup();
  ts.push_back(
      Triangle(
          Point1(-1000.f, -20.f, -1000.f),
          Point2(1000.f, -20.f, -1000.f),
          Point3(0.f, -20.f, 1000.f)
      )
      
  );
  
  
  static auto plane_renderable = createRenderable(
      ts,
      Phong(Colour(1.f, 0.f, 1.f),
          LightPosition(100.f, 100.f, 100.f))
  );
  
  

  draw(plane_renderable, view);
  draw(line_renderable, view);


  // draw the renderable
  draw(point_renders, view);

  
}

template <typename View>
void render(Cloth const& model, View const& view) {
    // static: only initiallized once, the first time this function is called.
    // This isn't the most elegant method to do this but it works. Just put your
    // draw calls in here.

    static auto point_renders = createInstancedRenderable(
        Sphere(Radius(0.5)), // geometry
        Phong(Colour(1.f, 0.f, 0.f),
            LightPosition(100.f, 100.f, 100.f)) // style
    );

    // load up renderable
    auto const& particles = model.particleList;

    for (auto const& particle : particles) {
        auto M = translate(mat4f{ 1.f }, particle.x);
        addInstance(point_renders, M);
    }

    auto l = MultiLine();
    for (int i = 0; i < model.springList.size(); i++) {
        //auto arm_geometry = Line(Point1(model.springList[i].a1->x), Point2(model.springList[i].b1->x));

        //updateRenderable(arm_geometry,          // new position
        //    LineStyle(Colour(0.f, 1.f, 0.f)), // new shading
       //     arm_renderable);

       // draw(arm_renderable, view);
        l.push_back(Line(Point1(model.springList[i].a1->x), Point2(model.springList[i].b1->x)));
    }
    auto line_renderable = createRenderable(
        l,
        LineStyle(Colour(1.f, 1.f, 0.f))
    );





   draw(line_renderable, view);
    // draw the renderable
   draw(point_renders, view);


}

//
// Helper class/functions
//
template <typename View> struct RenderableModel {

  template <typename Model>
  RenderableModel(Model const &model) : m_self(new model_t<Model>(model)) {}

  friend void render(RenderableModel const &renderable, View const &view) {
    renderable.m_self->renderSelf(view);
  }

  struct concept_t {
    virtual ~concept_t() = default;
    virtual void renderSelf(View const &view) const = 0;
  };

  template <typename Model> struct model_t : public concept_t {
    model_t(Model const &model) : data(model) {}
    void renderSelf(View const &view) const override { render(data, view); }
    Model const &data;
  };

  std::shared_ptr<concept_t const> m_self;
};

template <typename Model, typename View>
RenderableModel<View> makeModelRenderable(Model const &model,
                                          View const &view) {
  return RenderableModel<View>(model);
}

} // namespace simulation
