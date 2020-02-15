
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "igl/AABB.h"

using namespace std;

static void createSnake(igl::opengl::glfw::Viewer &viewer) {
	Eigen::Matrix4f parents = Eigen::Matrix4f().Identity();
	for (int i = 0; i < viewer.snake_size; i++) {
		//Line snake cylinders up
		viewer.data_list[i].MyTranslate(Eigen::Vector3f(0, 1.6, 0));
		viewer.data_list[i].SetCenterOfRotation(Eigen::Vector3f(0, -0.8, 0));

		//Cosmentic:
		viewer.data_list[i].show_overlay_depth = false;
		viewer.data_list[i].show_lines = false;

		//AABB tree init - for snake head only
		if (i == viewer.snake_size - 1) {
			viewer.data_list[i].tree.init(viewer.data_list[i].V, viewer.data_list[i].F);
			igl::AABB<Eigen::MatrixXd, 3> tree = viewer.data_list[i].tree;
			Eigen::AlignedBox<double, 3> box = tree.m_box;
			viewer.data_list[i].drawBox(box, 0);
		}
	}
	for (int i = viewer.snake_size; i < viewer.data_list.size(); i++) {
		//Cosmentic:
		viewer.data_list[i].show_overlay_depth = false;
		viewer.data_list[i].show_lines = false;

		//AABB tree init
		viewer.data_list[i].tree.init(viewer.data_list[i].V, viewer.data_list[i].F);
		igl::AABB<Eigen::MatrixXd, 3> tree = viewer.data_list[i].tree;
		Eigen::AlignedBox<double, 3> box = tree.m_box;
		viewer.data_list[i].drawBox(box, 0);
	}
}

int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 1000, "Snake Game");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;
  
  viewer.load_configuration();
  //Some scaling and location adjustments
  viewer.MyTranslate(Eigen::Vector3f(0, -1.5, 0));
  viewer.MyScale(Eigen::Vector3f(0.15, 0.15, 0.15));
  createSnake(viewer);

  //Move balls to locations - TODO: create moving patterns and shit
  viewer.data_list[10].MyTranslate(Eigen::Vector3f(4, 5, 0));
  viewer.data_list[11].MyTranslate(Eigen::Vector3f(-4, 5, 0));
  viewer.data_list[12].MyTranslate(Eigen::Vector3f(5, 12, 2));
  viewer.data_list[13].MyTranslate(Eigen::Vector3f(4, 10, 0));
  viewer.data_list[14].MyTranslate(Eigen::Vector3f(-4, 12, 0));

  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
