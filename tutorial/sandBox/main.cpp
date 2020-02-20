
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include <igl/opengl/glfw/Viewer.h>
#include "igl/AABB.h"

using namespace std;

static void createSnake(igl::opengl::glfw::Viewer &viewer) {
	Eigen::Matrix4f parents = Eigen::Matrix4f().Identity();
	for (int i = 0; i < viewer.snake_size; i++) {
		//Line snake cylinders up
		viewer.data_list[i].MyTranslate(Eigen::Vector3f(0, 1.6, 0));
		viewer.data_list[i].SetCenterOfRotation(Eigen::Vector3f(0, -0.8, 0));

		//Cosmetic:
		viewer.data_list[i].show_overlay_depth = false;
		viewer.data_list[i].show_lines = false;

		//AABB tree init - for snake head only
		if (i == viewer.snake_size - 1) {
			viewer.data_list[i].tree.init(viewer.data_list[i].V, viewer.data_list[i].F);
			igl::AABB<Eigen::MatrixXd, 3> tree = viewer.data_list[i].tree;
			Eigen::AlignedBox<double, 3> box = tree.m_box;
			//viewer.data_list[i].drawBox(box, 0);
		}
	}
	for (int i = viewer.snake_size; i < viewer.data_list.size(); i++) {
		//Cosmetic:
		viewer.data_list[i].show_overlay_depth = false;
		viewer.data_list[i].show_lines = false;

		//AABB tree init
		viewer.data_list[i].tree.init(viewer.data_list[i].V, viewer.data_list[i].F);
		igl::AABB<Eigen::MatrixXd, 3> tree = viewer.data_list[i].tree;
		Eigen::AlignedBox<double, 3> box = tree.m_box;
		//viewer.data_list[i].drawBox(box, 0);
	}
}

int main(int argc, char *argv[])
{
  float w = 1400.0;
  float h = 900.0;
  unsigned int left_view, right_view;
  Display *disp = new Display(w, h, "Snake Game");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;
  
  viewer.load_configuration();
  createSnake(viewer);

  //-----Move balls to locations - TODO: create moving patterns and shit-----
  viewer.data_list[10].MyTranslate(Eigen::Vector3f(-5, 20, 0));
  viewer.data_list[11].MyTranslate(Eigen::Vector3f(5, 20, 0));
  viewer.data_list[12].MyTranslate(Eigen::Vector3f(6, 6, -3));
  viewer.data_list[13].MyTranslate(Eigen::Vector3f(3, 12, -3));
  viewer.data_list[14].MyTranslate(Eigen::Vector3f(-6, 5, -3));
  //----------------------------------------------------------------------

  Init(*disp);
  renderer.init(&viewer);
  
  left_view = renderer.core_list[0].id;
  renderer.core().viewport = Eigen::Vector4f(0, 0, w / 2, h);
  right_view = renderer.append_core(Eigen::Vector4f(w / 2, 0, w / 2, h));
  renderer.core(left_view).camera_translation = Eigen::Vector3f(0, -14.5, -30);
  renderer.core(right_view).camera_translation = Eigen::Vector3f(0, -14.5, -30);
  for (int i = 0; i < viewer.data_list.size(); i++) {
	  renderer.core(right_view).set(viewer.data_list[i].show_faces, true);
  }

  renderer.selected_core_index = 0;

  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
