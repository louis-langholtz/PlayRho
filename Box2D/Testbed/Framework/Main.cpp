/*
* Original work Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "imgui.h"
#include "RenderGL3.h"
#include "DebugDraw.hpp"
#include "Test.hpp"
#include <sstream>
#include <iostream>
#include <iomanip>

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <glew/glew.h>
#endif

#include <glfw/glfw3.h>
#include <stdio.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

using namespace box2d;

//
struct UIState
{
	bool showMenu;
	int scroll;
	int scrollarea1;
	bool mouseOverMenu;
	bool chooseTest;
};

//
namespace
{
	auto CountTests()
	{
		auto count = int32{0};
		while (g_testEntries[count].createFcn)
		{
			++count;
		}
		return count;
	}
	
	Camera g_camera;
	
	GLFWwindow* mainWindow = nullptr;
	UIState ui;

	int32 testIndex = 0;
	int32 testSelection = 0;
	const auto testCount = CountTests();
	const TestEntry* entry;
	Test* test;
	Settings settings;
	bool rightMouseDown;
	Vec2 lastp;
}

//
static void sCreateUI()
{
	ui.showMenu = true;
	ui.scroll = 0;
	ui.scrollarea1 = 0;
	ui.chooseTest = false;
	ui.mouseOverMenu = false;

	// Init UI
    const char* fontPath = "../Data/DroidSans.ttf";
    
	if (!RenderGLInit(fontPath))
	{
		fprintf(stderr, "Could not init GUI renderer.\n");
		assert(false);
		return;
	}
}

//
static void sResizeWindow(GLFWwindow*, int width, int height)
{
	g_camera.m_width = width;
	g_camera.m_height = height;
}

static Test::Key GlfwKeyToTestKey(int key)
{
	switch (key)
	{
		case GLFW_KEY_COMMA: return Test::Key_Comma;
		case GLFW_KEY_MINUS: return Test::Key_Minus;
		case GLFW_KEY_PERIOD: return Test::Key_Period;
		case GLFW_KEY_EQUAL: return Test::Key_Equal;
		case GLFW_KEY_0: return Test::Key_0;
		case GLFW_KEY_1: return Test::Key_1;
		case GLFW_KEY_2: return Test::Key_2;
		case GLFW_KEY_3: return Test::Key_3;
		case GLFW_KEY_4: return Test::Key_4;
		case GLFW_KEY_5: return Test::Key_5;
		case GLFW_KEY_6: return Test::Key_6;
		case GLFW_KEY_7: return Test::Key_7;
		case GLFW_KEY_8: return Test::Key_8;
		case GLFW_KEY_9: return Test::Key_9;
		case GLFW_KEY_A: return Test::Key_A;
		case GLFW_KEY_B: return Test::Key_B;
		case GLFW_KEY_C: return Test::Key_C;
		case GLFW_KEY_D: return Test::Key_D;
		case GLFW_KEY_E: return Test::Key_E;
		case GLFW_KEY_F: return Test::Key_F;
		case GLFW_KEY_G: return Test::Key_G;
		case GLFW_KEY_H: return Test::Key_H;
		case GLFW_KEY_I: return Test::Key_I;
		case GLFW_KEY_J: return Test::Key_J;
		case GLFW_KEY_K: return Test::Key_K;
		case GLFW_KEY_L: return Test::Key_L;
		case GLFW_KEY_M: return Test::Key_M;
		case GLFW_KEY_N: return Test::Key_N;
		case GLFW_KEY_O: return Test::Key_O;
		case GLFW_KEY_P: return Test::Key_P;
		case GLFW_KEY_Q: return Test::Key_Q;
		case GLFW_KEY_R: return Test::Key_R;
		case GLFW_KEY_S: return Test::Key_S;
		case GLFW_KEY_T: return Test::Key_T;
		case GLFW_KEY_U: return Test::Key_U;
		case GLFW_KEY_V: return Test::Key_V;
		case GLFW_KEY_W: return Test::Key_W;
		case GLFW_KEY_X: return Test::Key_X;
		case GLFW_KEY_Y: return Test::Key_Y;
		case GLFW_KEY_Z: return Test::Key_Z;
		case GLFW_KEY_KP_SUBTRACT: return Test::Key_Subtract;
		case GLFW_KEY_KP_ADD: return Test::Key_Add;
	}
	return Test::Key_Unknown;
}

//
static void sKeyCallback(GLFWwindow*, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS)
	{
		switch (key)
		{
		case GLFW_KEY_ESCAPE:
			// Quit
			glfwSetWindowShouldClose(mainWindow, GL_TRUE);
			break;

		case GLFW_KEY_LEFT:
			// Pan left
			if (mods == GLFW_MOD_CONTROL)
			{
				test->ShiftOrigin(Vec2(2.0f, 0.0f));
			}
			else
			{
				g_camera.m_center.x -= 0.5f;
			}
			break;

		case GLFW_KEY_RIGHT:
			// Pan right
			if (mods == GLFW_MOD_CONTROL)
			{
				test->ShiftOrigin(Vec2(-2.0f, 0.0f));
			}
			else
			{
				g_camera.m_center.x += 0.5f;
			}
			break;

		case GLFW_KEY_DOWN:
			// Pan down
			if (mods == GLFW_MOD_CONTROL)
			{
				test->ShiftOrigin(Vec2(0.0f, 2.0f));
			}
			else
			{
				g_camera.m_center.y -= 0.5f;
			}
			break;

		case GLFW_KEY_UP:
			// Pan up
			if (mods == GLFW_MOD_CONTROL)
			{
				test->ShiftOrigin(Vec2(0.0f, -2.0f));
			}
			else
			{
				g_camera.m_center.y += 0.5f;
			}
			break;

		case GLFW_KEY_HOME:
			// Reset view
			g_camera.m_zoom = 1.0f;
			g_camera.m_center = Coord2D{0.0f, 20.0f};
			break;

		case GLFW_KEY_Z:
			// Zoom out
			g_camera.m_zoom = Min(1.1f * g_camera.m_zoom, 20.0f);
			break;

		case GLFW_KEY_X:
			// Zoom in
			g_camera.m_zoom = Max(0.9f * g_camera.m_zoom, 0.02f);
			break;

		case GLFW_KEY_R:
			// Reset test
			delete test;
			test = entry->createFcn();
			break;

		case GLFW_KEY_SPACE:
			// Launch a bomb.
			if (test)
			{
				test->LaunchBomb();
			}
			break;

		case GLFW_KEY_P:
			// Pause
			settings.pause = !settings.pause;
			break;

		case GLFW_KEY_LEFT_BRACKET:
			// Switch to previous test
			--testSelection;
			if (testSelection < 0)
			{
				testSelection = testCount - 1;
			}
			break;

		case GLFW_KEY_RIGHT_BRACKET:
			// Switch to next test
			++testSelection;
			if (testSelection == testCount)
			{
				testSelection = 0;
			}
			break;

		case GLFW_KEY_TAB:
			ui.showMenu = !ui.showMenu;

		default:
			if (test)
			{
				test->Keyboard(GlfwKeyToTestKey(key));
			}
		}
	}
	else if (action == GLFW_RELEASE)
	{
		test->KeyboardUp(GlfwKeyToTestKey(key));
	}
	// else GLFW_REPEAT
}

//
static void sMouseButton(GLFWwindow*, int32 button, int32 action, int32 mods)
{
	double xd, yd;
	glfwGetCursorPos(mainWindow, &xd, &yd);
	const auto ps = Coord2D{(float)xd, (float)yd};

	// Use the mouse to move things around.
	if (button == GLFW_MOUSE_BUTTON_1)
	{
        //ps = Vec2(0, 0);
		const auto pw = ConvertScreenToWorld(g_camera, ps);
		if (action == GLFW_PRESS)
		{
			if (mods == GLFW_MOD_SHIFT)
			{
				test->ShiftMouseDown(pw);
			}
			else
			{
				test->MouseDown(pw);
			}
		}
		
		if (action == GLFW_RELEASE)
		{
			test->MouseUp(pw);
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_2)
	{
		if (action == GLFW_PRESS)
		{	
			lastp = ConvertScreenToWorld(g_camera, ps);
			rightMouseDown = true;
		}

		if (action == GLFW_RELEASE)
		{
			rightMouseDown = false;
		}
	}
}

//
static void sMouseMotion(GLFWwindow*, double xd, double yd)
{
	const auto ps = Coord2D{static_cast<float>(xd), static_cast<float>(yd)};
	const auto pw = ConvertScreenToWorld(g_camera, ps);

	test->MouseMove(pw);
	
	if (rightMouseDown)
	{
		const auto movement = pw - lastp;
		g_camera.m_center.x -= static_cast<float>(movement.x);
		g_camera.m_center.y -= static_cast<float>(movement.y);
		lastp = ConvertScreenToWorld(g_camera, ps);
	}
}

//
static void sScrollCallback(GLFWwindow*, double, double dy)
{
	if (ui.mouseOverMenu)
	{
		ui.scroll = -int(dy);
	}
	else
	{
		if (dy > 0)
		{
			g_camera.m_zoom /= 1.1f;
		}
		else
		{
			g_camera.m_zoom *= 1.1f;
		}
	}
}

//
static void sRestart()
{
	delete test;
	entry = g_testEntries + testIndex;
	test = entry->createFcn();
}

//
static void sSimulate(Drawer& drawer)
{
	glEnable(GL_DEPTH_TEST);
	
	settings.dt = (settings.hz > 0)? 1.0f / settings.hz : 0.0f;
	if (settings.pause)
	{
		if (settings.singleStep)
		{
			settings.singleStep = false;
		}
		else
		{
			settings.dt = 0.0f;
		}
	}
	
	test->Step(settings, drawer);

	test->DrawTitle(drawer, entry->name);
	glDisable(GL_DEPTH_TEST);

	if (testSelection != testIndex)
	{
		testIndex = testSelection;
		delete test;
		entry = g_testEntries + testIndex;
		test = entry->createFcn();
		g_camera.m_zoom = 1.0f;
		g_camera.m_center = Coord2D{0.0f, 20.0f};
	}
}

//
static void sInterface()
{
	int menuWidth = 200;
	ui.mouseOverMenu = false;
	if (ui.showMenu)
	{
		bool over = imguiBeginScrollArea("Testbed Controls", g_camera.m_width - menuWidth - 10, 10, menuWidth, g_camera.m_height - 20, &ui.scrollarea1);
		if (over) ui.mouseOverMenu = true;

		imguiSeparatorLine();

		imguiLabel("Test:");
		if (imguiButton(entry->name, true))
		{
			ui.chooseTest = !ui.chooseTest;
		}

		imguiSeparatorLine();

		imguiSlider("Reg Vel Iters", &settings.regVelocityIterations, 0, 100, 1, true);
		imguiSlider("Reg Pos Iters", &settings.regPositionIterations, 0, 100, 1, true);
		imguiSlider("TOI Vel Iters", &settings.toiVelocityIterations, 0, 100, 1, true);
		imguiSlider("TOI Pos Iters", &settings.toiPositionIterations, 0, 100, 1, true);
		imguiSlider("Hertz", &settings.hz, 5.0f, 120.0f, 5.0f, true);
		imguiSlider("Linear Slop", &settings.linearSlop,
					static_cast<float>(DefaultLinearSlop / 10),
					static_cast<float>(DefaultLinearSlop),
					static_cast<float>(DefaultLinearSlop / 100),
					true);
		imguiSlider("Angular Slop", &settings.angularSlop, (Pi * 2 / 1800), (Pi * 2 / 18), 0.001f, true);
		imguiSlider("Max Translation", &settings.maxTranslation, 0.0f, 8.0f, 0.05f, true);
		imguiSlider("Max Rotation", &settings.maxRotation, 0.0f, 360.0f, 1.0f, true);
		imguiSlider("Max Lin Correct", &settings.maxLinearCorrection, 0.0f, 1.0f, 0.01f, true);
		imguiSlider("Max Ang Correct", &settings.maxAngularCorrection, 0.0f, 90.0f, 1.0f, true);
		imguiSlider("Reg Resol % Rate", &settings.regPosResRate, 0, 100, 1, true);
		imguiSlider("TOI Resol % Rate", &settings.toiPosResRate, 0, 100, 1, true);
		
		if (imguiCheck("Sleep", settings.enableSleep, true))
			settings.enableSleep = !settings.enableSleep;
		if (imguiCheck("Warm Starting", settings.enableWarmStarting, true))
			settings.enableWarmStarting = !settings.enableWarmStarting;
		if (imguiCheck("Time of Impact", settings.enableContinuous, true))
			settings.enableContinuous = !settings.enableContinuous;
		if (imguiCheck("Sub-Stepping", settings.enableSubStepping, true))
			settings.enableSubStepping = !settings.enableSubStepping;

		imguiSeparatorLine();

		if (imguiCheck("Shapes", settings.drawShapes, true))
			settings.drawShapes = !settings.drawShapes;
		if (imguiCheck("Joints", settings.drawJoints, true))
			settings.drawJoints = !settings.drawJoints;
		if (imguiCheck("Skins", settings.drawSkins, true))
			settings.drawSkins = !settings.drawSkins;
		if (imguiCheck("AABBs", settings.drawAABBs, true))
			settings.drawAABBs = !settings.drawAABBs;
		if (imguiCheck("Contact Points", settings.drawContactPoints, true))
			settings.drawContactPoints = !settings.drawContactPoints;
		if (imguiCheck("Contact Normals", settings.drawContactNormals, true))
			settings.drawContactNormals = !settings.drawContactNormals;
		if (imguiCheck("Contact Impulses", settings.drawContactImpulse, true))
			settings.drawContactImpulse = !settings.drawContactImpulse;
		if (imguiCheck("Friction Impulses", settings.drawFrictionImpulse, true))
			settings.drawFrictionImpulse = !settings.drawFrictionImpulse;
		if (imguiCheck("Center of Masses", settings.drawCOMs, true))
			settings.drawCOMs = !settings.drawCOMs;
		if (imguiCheck("Statistics", settings.drawStats, true))
			settings.drawStats = !settings.drawStats;
		if (imguiCheck("Pause", settings.pause, true))
			settings.pause = !settings.pause;

		if (imguiButton("Single Step", true))
			settings.singleStep = !settings.singleStep;
		if (imguiButton("Restart", true))
			sRestart();
		if (imguiButton("Quit", true))
			glfwSetWindowShouldClose(mainWindow, GL_TRUE);

		imguiEndScrollArea();
	}

	int testMenuWidth = 200;
	if (ui.chooseTest)
	{
		static int testScroll = 0;
		bool over = imguiBeginScrollArea("Choose Sample", g_camera.m_width - menuWidth - testMenuWidth - 20, 10, testMenuWidth, g_camera.m_height - 20, &testScroll);
		if (over) ui.mouseOverMenu = true;

		for (int i = 0; i < testCount; ++i)
		{
			if (imguiItem(g_testEntries[i].name, true))
			{
				delete test;
				entry = g_testEntries + i;
				test = entry->createFcn();
				ui.chooseTest = false;
			}
		}

		imguiEndScrollArea();
	}

	imguiEndFrame();

}

//
int main(int argc, char** argv)
{
#if defined(_WIN32)
	// Enable memory-leak reports
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

	g_camera.m_width = 1200; // 1152;
	g_camera.m_height = 890; // 864;
    
	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	char title[64];
	sprintf(title, "Box2D Testbed Version %d.%d.%d", BuiltVersion.major, BuiltVersion.minor, BuiltVersion.revision);

#if defined(__APPLE__)
	// Not sure why, but these settings cause glewInit below to crash.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif

    mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, title, nullptr, nullptr);
	if (mainWindow == nullptr)
	{
		fprintf(stderr, "Failed to open GLFW mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(mainWindow);
	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

	glfwSetScrollCallback(mainWindow, sScrollCallback);
	glfwSetWindowSizeCallback(mainWindow, sResizeWindow);
	glfwSetKeyCallback(mainWindow, sKeyCallback);
	glfwSetMouseButtonCallback(mainWindow, sMouseButton);
	glfwSetCursorPosCallback(mainWindow, sMouseMotion);
	glfwSetScrollCallback(mainWindow, sScrollCallback);

#if defined(__APPLE__) == FALSE
	//glewExperimental = GL_TRUE;
    GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(EXIT_FAILURE);
	}
#endif

	
	sCreateUI();
	
	testIndex = Clamp(testIndex, 0, testCount - 1);
	testSelection = testIndex;
	
	entry = g_testEntries + testIndex;
	test = entry->createFcn();
	
	// Control the frame rate. One draw per monitor refresh.
	glfwSwapInterval(1);
	
	auto time1 = glfwGetTime();
	auto frameTime = 0.0;
	auto fps = 0.0;
	
	glClearColor(0.3f, 0.3f, 0.3f, 1.f);
	
	{
		DebugDraw drawer(g_camera);
		while (!glfwWindowShouldClose(mainWindow))
		{
			glfwGetWindowSize(mainWindow, &g_camera.m_width, &g_camera.m_height);
			glViewport(0, 0, g_camera.m_width, g_camera.m_height);

			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			unsigned char mousebutton = 0;
			int mscroll = ui.scroll;
			ui.scroll = 0;

			double xd, yd;
			glfwGetCursorPos(mainWindow, &xd, &yd);
			const auto mousex = int(xd);
			const auto mousey = g_camera.m_height - int(yd);

			const auto leftButton = glfwGetMouseButton(mainWindow, GLFW_MOUSE_BUTTON_LEFT);
			if (leftButton == GLFW_PRESS)
				mousebutton |= IMGUI_MBUT_LEFT;

			imguiBeginFrame(mousex, mousey, mousebutton, mscroll);

			sSimulate(drawer);
			sInterface();
			
			// Measure speed
			const auto time2 = glfwGetTime();
			const auto timeElapsed = time2 - time1;
			const auto alpha = 0.9;
			frameTime = alpha * frameTime + (1.0 - alpha) * timeElapsed;
			fps = 0.99 * fps + (1.0 - 0.99) / timeElapsed;
			time1 = time2;

			{
				std::stringstream stream;
				const auto viewport = ConvertScreenToWorld(g_camera);
				stream << "Zoom=" << g_camera.m_zoom;
				stream << " Center=";
				stream << "{" << g_camera.m_center.x << "," << g_camera.m_center.y << "}";
				stream << " Viewport=";
				stream << "{";
				stream << viewport.GetLowerBound().x << "..." << viewport.GetUpperBound().x;
				stream << ", ";
				stream << viewport.GetLowerBound().y << "..." << viewport.GetUpperBound().y;
				stream << "}";
				stream << std::setprecision(1);
				stream << std::fixed;
				stream << " Refresh=" << (1000.0 * frameTime) << "ms";
				stream << std::setprecision(0);
				stream << " FPS=" << fps;
				AddGfxCmdText(5, 5, TEXT_ALIGN_LEFT, stream.str().c_str(), static_cast<unsigned int>(WHITE));
			}
			
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDisable(GL_DEPTH_TEST);
			RenderGLFlush(g_camera.m_width, g_camera.m_height);

			glfwSwapBuffers(mainWindow);

			glfwPollEvents();
		}
	}

	RenderGLDestroy();
	glfwTerminate();

	return 0;
}
