#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>
#include <iostream>
using namespace std;
Renderer::Renderer() : selected_core_index(0),
next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	highdpi = 1;

	xold = 0;
	yold = 0;

}

IGL_INLINE void Renderer::draw( GLFWwindow* window)
{
	using namespace std;
	using namespace Eigen;

	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);
	
	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window,width, height);
		highdpi = highdpi_tmp;
	}

	for (auto& core : core_list)
	{
		core.clear_framebuffers();
	}

	for (auto& core : core_list)
	{
		for (auto& mesh : scn->data_list)
		{
			if (mesh.is_visible & core.id)
			{
				if (core.id == 2) {
					Eigen::Matrix4f headTransMat = scn->MakeTrans() * scn->ParentsTrans(scn->snake_size - 1) * scn->data(scn->snake_size - 1).MakeTrans();
					core.camera_translation = (headTransMat * Eigen::Vector4f(0, 0.8, 0.8, -1)).block(0, 0, 3, 1);
					core.camera_eye = (headTransMat.block(0, 0, 3, 3) * Eigen::Vector3f(0, -1, 0)).block(0, 0, 3, 1);
					core.camera_up = (headTransMat.block(0, 0, 3, 3) * Eigen::Vector3f(0, 0, -1)).block(0, 0, 3, 1);
				}

				Matrix4f parents = Matrix4f().Identity();	
				parents = scn->ParentsTrans(mesh.id);
				Matrix4f temp = scn->MakeTrans() * parents;
				core.draw(temp, mesh);
			}
		}
	}

}

void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
	core().init(); 

	core().align_camera_center(scn->data().V, scn->data().F);
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}

void Renderer::MouseProcessing(int button)
{
	if (button == 1)
	{
		if (scn->selected_data_index == -1) {
			scn->MyTranslate(Eigen::Vector3f(-xrel / 180.0f, 0, 0));
			scn->MyTranslate(Eigen::Vector3f(0, yrel / 180.0f, 0));
		}
		else if (scn->selected_data_index >= 0 && (scn->selected_data_index < scn->snake_size)) {
			scn->data(0).MyTranslate(Eigen::Vector3f(-xrel / 180.0f, 0, 0));
			scn->data(0).MyTranslate(Eigen::Vector3f(0, yrel / 180.0f, 0));
		}
		else {
			scn->data().MyTranslate(Eigen::Vector3f(-xrel / 180.0f, 0, 0));
			scn->data().MyTranslate(Eigen::Vector3f(0, yrel / 180.0f, 0));
		}
	}
	else
	{
		if (scn->selected_data_index == -1) {
			scn->MyRotate(Eigen::Vector3f(0, -1, 0), xrel / 180.0f, true);
			scn->MyRotate(Eigen::Vector3f(-1, 0, 0), yrel / 180.0f, false);
		}
		else {
			scn->data().MyRotate(Eigen::Vector3f(0, -1, 0), xrel / 180.0f, true);
			scn->data().MyRotate(Eigen::Vector3f(-1, 0, 0), yrel / 180.0f, false);
		}
	}
}

using namespace std;

//Returns true if box 1 and box 2 collide
bool Renderer::checkCollisionHelper(Eigen::AlignedBox<double, 3> &box1, Eigen::AlignedBox<double, 3> &box2, Eigen::Matrix3d &A, Eigen::Matrix3d &B, Eigen::Matrix3d &C, int i1, int i2) {
	double R, R0, R1;

	//cout << " Checking box collision for " << i1 << " " << i2 << endl;
	double a0 = box1.sizes()[0] / 2;
	double b0 = box2.sizes()[0] / 2;
	double a1 = box1.sizes()[1] / 2;
	double b1 = box2.sizes()[1] / 2;
	double a2 = box1.sizes()[2] / 2;
	double b2 = box2.sizes()[2] / 2;

	Eigen::RowVector3d A0 = A * Eigen::Vector3d(1, 0, 0);
	Eigen::RowVector3d A1 = A * Eigen::Vector3d(0, 1, 0);
	Eigen::RowVector3d A2 = A * Eigen::Vector3d(0, 0, 1);

	Eigen::RowVector3d B0 = B * Eigen::Vector3d(1, 0, 0);
	Eigen::RowVector3d B1 = B * Eigen::Vector3d(0, 1, 0);
	Eigen::RowVector3d B2 = B * Eigen::Vector3d(0, 0, 1);

	Eigen::Vector4f temp = Eigen::Vector4f(box1.center()[0], box1.center()[1], box1.center()[2], 1);
	temp = scn->ParentsTrans(i1) * scn->data_list[i1].MakeTrans() * temp;
	Eigen::Vector3d C0 = Eigen::Vector3d(temp[0], temp[1], temp[2]);
	temp = Eigen::Vector4f(box2.center()[0], box2.center()[1], box2.center()[2], 1);
	temp = scn->ParentsTrans(i2) * scn->data_list[i2].MakeTrans() * temp;
	Eigen::Vector3d C1 = Eigen::Vector3d(temp[0], temp[1], temp[2]);
	Eigen::Vector3d D = C1 - C0;

	// -------- Table Row 1 ---------
	R0 = a0;
	R1 = b0 * abs(C(0, 0)) + b1 * abs(C(0, 1)) + b2 * abs(C(0, 2));
	R = abs(A0.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 2 ---------
	R0 = a1;
	R1 = b0 * abs(C(1, 0)) + b1 * abs(C(1, 1)) + b2 * abs(C(1, 2));
	R = abs(A1.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 3 ---------
	R0 = a2;
	R1 = b0 * abs(C(2, 0)) + b1 * abs(C(2, 1)) + b2 * abs(C(2, 2));
	R = abs(A2.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 4 ---------
	R0 = a0 * abs(C(0, 0)) + a1 * abs(C(1, 0)) + a2 * abs(C(2, 0));
	R1 = b0;
	R = abs(B0.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 5 ---------
	R0 = a0 * abs(C(0, 1)) + a1 * abs(C(1, 1)) + a2 * abs(C(2, 1));
	R1 = b1;
	R = abs(B1.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 6 ---------
	R0 = a0 * abs(C(0, 2)) + a1 * abs(C(1, 2)) + a2 * abs(C(2, 2));
	R1 = b2;
	R = abs(B2.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 7 ---------
	R0 = a1 * abs(C(2, 0)) + a2 * abs(C(1, 0));
	R1 = b1 * abs(C(0, 2)) + b2 * abs(C(0, 1));
	R = abs(C(1, 0) * A2.dot(D) - C(2, 0) * A1.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 8 ---------
	R0 = a1 * abs(C(2, 1)) + a2 * abs(C(1, 1));
	R1 = b0 * abs(C(0, 2)) + b2 * abs(C(0, 0));
	R = abs(C(1, 1) * A2.dot(D) - C(2, 1) * A1.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 9 ---------
	R0 = a1 * abs(C(2, 2)) + a2 * abs(C(1, 2));
	R1 = b0 * abs(C(0, 1)) + b1 * abs(C(0, 0));
	R = abs(C(1, 2) * A2.dot(D) - C(2, 2) * A1.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 10 ---------
	R0 = a0 * abs(C(2, 0)) + a2 * abs(C(0, 0));
	R1 = b1 * abs(C(1, 2)) + b2 * abs(C(1, 1));
	R = abs(C(2, 0) * A0.dot(D) - C(0, 0) * A2.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 11 ---------
	R0 = a0 * abs(C(2, 1)) + a2 * abs(C(0, 1));
	R1 = b0 * abs(C(1, 2)) + b2 * abs(C(1, 0));
	R = abs(C(2, 1) * A0.dot(D) - C(0, 1) * A2.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 12 ---------
	R0 = a0 * abs(C(2, 2)) + a2 * abs(C(0, 2));
	R1 = b0 * abs(C(1, 1)) + b1 * abs(C(1, 0));
	R = abs(C(2, 2) * A0.dot(D) - C(0, 2) * A2.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 13 ---------
	R0 = a0 * abs(C(1, 0)) + a1 * abs(C(0, 0));
	R1 = b1 * abs(C(2, 2)) + b2 * abs(C(2, 1));
	R = abs(C(0, 0) * A1.dot(D) - C(1, 0) * A0.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 14 ---------
	R0 = a0 * abs(C(1, 1)) + a1 * abs(C(0, 1));
	R1 = b0 * abs(C(2, 2)) + b2 * abs(C(2, 0));
	R = abs(C(0, 1) * A1.dot(D) - C(1, 1) * A0.dot(D));
	if (R > R0 + R1)
		return false;
	// -------- Table Row 15 ---------
	R0 = a0 * abs(C(1, 2)) + a1 * abs(C(0, 2));
	R1 = b0 * abs(C(2, 1)) + b1 * abs(C(2, 0));
	R = abs(C(0, 2) * A1.dot(D) - C(1, 2) * A0.dot(D));
	if (R > R0 + R1)
		return false;
	return true;
}

//Recursion call for checking collision, retruns true if node1 and node2 collide (checking untill leafs recursivly)
//If they collide, populate leaf field in each data items
bool Renderer::checkCollisionRec(igl::AABB<Eigen::MatrixXd, 3> *node1, igl::AABB<Eigen::MatrixXd, 3> *node2, Eigen::Matrix3d &A, Eigen::Matrix3d &B, Eigen::Matrix3d &C, int i1, int i2) {
	if (checkCollisionHelper(node1->m_box, node2->m_box, A, B, C, i1, i2))
	{
		//No children, this is a leaf! populate field
		if (node1->is_leaf() && node2->is_leaf())
		{
			/*scn->data_list[i1].clear();
			scn->data_list[i2].clear();
			scn->data_list[i1].drawBox(node1->m_box, 1);
			scn->data_list[i2].drawBox(node2->m_box, 1);*/
			return true;
		}
		else {
			//Children pointers
			igl::AABB<Eigen::MatrixXd, 3> *left1 = node1->is_leaf() ? node1 : node1->m_left;
			igl::AABB<Eigen::MatrixXd, 3> *right1 = node1->is_leaf() ? node1 : node1->m_right;
			igl::AABB<Eigen::MatrixXd, 3> *left2 = node2->is_leaf() ? node2 : node2->m_left;;
			igl::AABB<Eigen::MatrixXd, 3> *right2 = node2->is_leaf() ? node2 : node2->m_right;

			if (checkCollisionRec(left1, left2, A, B, C, i1, i2))
				return true;
			else if (checkCollisionRec(left1, right2, A, B, C, i1, i2))
				return true;
			else if (checkCollisionRec(right1, left2, A, B, C, i1, i2))
				return true;
			else if (checkCollisionRec(right1, right2, A, B, C, i1, i2))
				return true;
			else return false;
		}
	}
	return false;
}

//Main collision checking function, inits leaf for each box to NULL
//Call recursion func and let it do all the work
//Leafs should be populated at the end, in case of collision
void Renderer::checkCollision() {
	int head_index = scn->snake_size - 1;
	int ball_index = scn->selected_ball;
	igl::AABB<Eigen::MatrixXd, 3> *node1 = &scn->data_list[head_index].tree;
	igl::AABB<Eigen::MatrixXd, 3> *node2 = &scn->data_list[ball_index].tree;
	Eigen::Matrix3d *A = &((Eigen::Matrix3d)scn->data_list[head_index].Tout.rotation().matrix().cast<double>());
	Eigen::Matrix3d *B = &((Eigen::Matrix3d)scn->data_list[ball_index].Tout.rotation().matrix().cast<double>());
	Eigen::Matrix3d *C = &((Eigen::Matrix3d)((*A).transpose() * (*B)));

	if (checkCollisionRec(node1, node2, *A, *B, *C, head_index, ball_index)) {
		//cout << "Collision detected! " << endl;
		//cout << "Scored! | Score: " << ++scn->score << endl;
		scn->IKon = false;
		scn->fixAxis();
	}
}

Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}

bool Renderer::Picking(double newx, double newy)
{
		int fid;
		//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
		Eigen::Vector3f bc;
		double x = newx;
		double y = core().viewport(3) - newy;
		Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
		igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
		view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
			* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTrans() * scn->ParentsTrans(scn->selected_data_index) * scn->data().MakeTrans();
		if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
 			core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc))
		{
			return true;
		}
		return false;
	
}

IGL_INLINE void Renderer::resize(GLFWwindow* window,int w, int h)
	{
		if (window) {
			glfwSetWindowSize(window, w / highdpi, h / highdpi);
		}
		post_resize(window,w, h);
	}

	IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
	{
		if (core_list.size() == 1)
		{
			core().viewport = Eigen::Vector4f(0, 0, w, h);
		}
		else
		{
			// It is up to the user to define the behavior of the post_resize() function
			// when there are multiple viewports (through the `callback_post_resize` callback)
		}
		//for (unsigned int i = 0; i < plugins.size(); ++i)
		//{
		//	plugins[i]->post_resize(w, h);
		//}
		if (callback_post_resize)
		{
			callback_post_resize(window, w, h);
		}
	}

	IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE bool Renderer::erase_core(const size_t index)
	{
		assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
		//assert(data_list.size() >= 1);
		if (core_list.size() == 1)
		{
			// Cannot remove last viewport
			return false;
		}
		core_list[index].shut(); // does nothing
		core_list.erase(core_list.begin() + index);
		if (selected_core_index >= index && selected_core_index > 0)
		{
			selected_core_index--;
		}
		return true;
	}

	IGL_INLINE size_t Renderer::core_index(const int id) const {
		for (size_t i = 0; i < core_list.size(); ++i)
		{
			if (core_list[i].id == id)
				return i;
		}
		return 0;
	}

	IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
	{
		core_list.push_back(core()); // copies the previous active core and only changes the viewport
		core_list.back().viewport = viewport;
		core_list.back().id = next_core_id;
		next_core_id <<= 1;
		if (!append_empty)
		{
			for (auto& data : scn->data_list)
			{
				data.set_visible(true, core_list.back().id);
				//data.copy_options(core(), core_list.back());
			}
		}
		selected_core_index = core_list.size() - 1;
		return core_list.back().id;
	}

	//IGL_INLINE void Viewer::select_hovered_core()
	//{
	//	int width_window, height_window = 800;
	//   glfwGetFramebufferSize(window, &width_window, &height_window);
	//	for (int i = 0; i < core_list.size(); i++)
	//	{
	//		Eigen::Vector4f viewport = core_list[i].viewport;

	//		if ((current_mouse_x > viewport[0]) &&
	//			(current_mouse_x < viewport[0] + viewport[2]) &&
	//			((height_window - current_mouse_y) > viewport[1]) &&
	//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
	//		{
	//			selected_core_index = i;
	//			break;
	//		}
	//	}
	//}