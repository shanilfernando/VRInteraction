#include "libfreenect2_grabber.h"
#include <signal.h>
#include <pcl/point_representation.h>
#include "DepthBasics.h"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <iostream>
#include <list>
#include "Button.h"
#include "Gesture.h"
#include "Dashboard.h"
#include "Workstation.h"
#include <fstream>
#include "build\UDPserver.h"
#include <GL\glew.h>
#include <GLFW/glfw3.h>

bool Quit = false;
bool windows = false;
int clickCallback(Button button)
{
	std::cout << button.getId() << std::endl;

	return 1;
}


GLFWwindow* initWindow(const int resX, const int resY)
{
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return NULL;
	}
	glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing

	// Open a window and create its OpenGL context
	GLFWwindow* window = glfwCreateWindow(resX, resY, "TEST", NULL, NULL);

	if (window == NULL)
	{
		fprintf(stderr, "Failed to open GLFW window.\n");
		glfwTerminate();
		return NULL;
	}

	glfwMakeContextCurrent(window);

	// Get info of GPU and supported OpenGL version
	printf("Renderer: %s\n", glGetString(GL_RENDERER));
	printf("OpenGL version supported %s\n", glGetString(GL_VERSION));

	glEnable(GL_DEPTH_TEST); // Depth Testing
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	return window;
}

void drawCube(float Rx,float Ry,float Rz)
{
	GLfloat vertices[] =
	{
		-1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1,
		1, -1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1,
		-1, -1, -1, -1, -1, 1, 1, -1, 1, 1, -1, -1,
		-1, 1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1,
		-1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1,
		-1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1
	};

	/*GLfloat colors[] =
	{
		0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0,
		1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0,
		0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0,
		0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0,
		0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0,
		0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1
	};*/

	//attempt to rotate cube
	glRotatef(Rx, 1, 0, 0);
	glRotatef(Ry, 0, 1, 0);
	glRotatef(Rz, 0, 0, 1);

	/* We have a color array and a vertex array */
	glEnableClientState(GL_VERTEX_ARRAY);
	//glEnableClientState(GL_COLOR_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, vertices);
	//glColorPointer(3, GL_FLOAT, 0, colors);

	/* Send data : 24 vertices */
	glDrawArrays(GL_QUADS, 0, 24);

	/* Cleanup states */
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void display(GLFWwindow* window,float Rx, float Ry,float Rz)
{
	if (NULL != window)
	{
		glfwMakeContextCurrent(window);
		// Scale to window size
		GLint windowWidth, windowHeight;
		glfwGetWindowSize(window, &windowWidth, &windowHeight);
		glViewport(0, 0, windowWidth, windowHeight);

		// Draw stuff
		glClearColor(0.0, 0.0, 0.0, 0.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION_MATRIX);
		glLoadIdentity();
		gluPerspective(60, (double)windowWidth / (double)windowHeight, 0.1, 100);

		glMatrixMode(GL_MODELVIEW_MATRIX);
		glTranslatef(0, 0, -5);

		drawCube(Rx,Ry,Rz);

		// Update Screen
		glfwSwapBuffers(window);

	}
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,void* viewer_void)
{
	if (event.getKeySym() == "q")
	{
		Quit = true;

	}
	else if (event.getKeySym() == "a" && event.keyDown()){
		windows = !windows;
	}
}


int main(int argc, char *argv[])
{
	
	GLFWwindow* window = initWindow(640, 480);
	display(window, 0.0f,0.0f, 0.0f);
	
	CDepthBasics* frames=new CDepthBasics();

	libfreenect2_grabber grabber("../calibration/rgb_calibration.yaml", "../calibration/depth_calibration.yaml", "../calibration/pose_calibration.yaml");


	Eigen::Matrix4d depth2world_ = grabber.getDepth2World();

	Registration* registration = new Registration("../calibration/camera_param.yaml");
	registration->cudaInit(depth2world_);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(temp1);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud"));
	viewer->setCameraPosition(0, 0, -1, 0, -1, 0);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(temp1, rgb, "Cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
	Button button1("button1", &clickCallback);
	Button button2("button2", &clickCallback);
	Button button3("button3", &clickCallback);
	Dashboard dashboard1;
	dashboard1.addButton(button1);
	dashboard1.addButton(button2);
	Dashboard dashboard2;
	dashboard2.addButton(button3);
	INPUT Input = { 0 };
	Input.type = INPUT_MOUSE;
	Input.mi.dwFlags = MOUSEEVENTF_LEFTDOWN | MOUSEEVENTF_LEFTUP;
	int screenWidth = GetSystemMetrics(SM_CXVIRTUALSCREEN);
	int screenHeight = GetSystemMetrics(SM_CYVIRTUALSCREEN);
	while (!Quit)
	{
		HRESULT hr = frames->Update();
		if (SUCCEEDED(hr))
		{
			if (!windows)
			{
				if (frames->engagedUsers.size()>0)
				{
				
						int extrapix = 0;
						for (auto& x : frames->engagedUsers){
							if (x.second.workstation->getNumDashboards() == 0){
								x.second.workstation->addDashboard(dashboard1);
								x.second.workstation->addDashboard(dashboard2);
							}
							if (x.second.interactiveState == InteractiveState::InteractiveState_Swipe)
							{
								x.second.toggleDashboard();
							}
							else if (x.second.interactiveState == InteractiveState::InteractiveState_Rotate){
								display(window, 0.0f, x.second.GetRotationAroundYAxis() * 180 / PI, -x.second.GetRotationAroundZAxis() * 180 / PI);
							}
							x.second.update();
							if (x.second.interactiveState == InteractiveState::InteractiveState_Click)
							{
								x.second.isRightClicked();
							}
							extrapix += x.second.workstation->getCurrentDashboard()->getNumberPixel() + 1;
						}
						registration->apply9(frames->getDepthBuffer(), frames->getColorBuffer(), temp1, frames->getBodyIndexBuffer(), &extrapix);
						for (auto& x : frames->engagedUsers){
							x.second.vizDashboard(extrapix, temp1);
							extrapix += x.second.workstation->getCurrentDashboard()->getNumberPixel() + 1;
						}
				
				}
				else
				{
					registration->apply5(frames->getDepthBuffer(), frames->getColorBuffer(), temp1);
				}
				
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(temp1);
				viewer->updatePointCloud<pcl::PointXYZRGB>(temp1, rgb1, "Cloud");
			}
			else{
				if (frames->engagedUsers.size()>0)
				{
					User* x = &frames->engagedUsers.begin()->second;
					SetCursorPos((int)(x->pointer->X*(float)screenWidth / 0.3), (int)(x->pointer->Y*(float)screenHeight / 0.2));
					if (x->isMouseClicked()){
						SendInput(1, &Input, sizeof(INPUT));
					}
				}
				
			}
			viewer->spinOnce();
		}
		
		frames->release();
		
	}
	viewer->close();	
	glfwDestroyWindow(window);
	glfwTerminate();
	frames->release();
	registration->freeMem();
	frames->~CDepthBasics();
	return 0;
}
