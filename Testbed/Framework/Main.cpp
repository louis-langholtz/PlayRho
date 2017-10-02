/*
* Original work Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Common/Version.hpp>

#include "imgui.h"
#include "RenderGL3.h"
#include "DebugDraw.hpp"
#include "Test.hpp"
#include "TestEntry.hpp"

// Uncomment the following define if you'd prefer to use an external file for font data.
//#define DONT_EMBED_FONT_DATA
#ifndef DONT_EMBED_FONT_DATA
#include "DroidSansTtfData.h"
#endif

#include <sstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <string>

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <GL/glew.h>
#endif

#include <GLFW/glfw3.h>
#include <cstdio>

#if defined(_WIN32) || defined(_WIN64)
#include <direct.h>
#else
#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#endif

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

using namespace playrho;

//
struct UIState
{
    bool showMenu;
    int scroll;
    int scrollarea1;
    bool mouseOverMenu;
    bool chooseTest;
};

class Selection
{
public:
    /// @brief Size type.
    using size_type = std::size_t;
    
    Selection(size_type size, size_type selection = 0):
	    m_size(size),
    	m_selection(selection < size? selection: 0)
    {
        assert(size > 0);
        assert(selection < size);
    }
    
    size_type Get() const
    {
        return m_selection;
    }
    
    void Set(size_type selection)
    {
        assert(selection < m_size);
        if (selection < m_size)
        {
            m_selection = selection;
        }
    }
    
    void Increment()
    {
        const auto next = m_selection + 1;
        m_selection = (next < m_size)? next: 0;
    }
    
    void Decrement()
    {
        m_selection = (m_selection > 0)? m_selection - 1: m_size - 1;
    }
    
private:
    size_type m_selection = 0;
    size_type m_size = 0;
};

class TestSuite
{
public:
    /// @brief Size type.
    using size_type = std::size_t;
    
    TestSuite(Span<const TestEntry> testEntries, size_type index = 0):
    	m_testEntries(testEntries),
    	m_testIndex(index < testEntries.size()? index: 0)
    {
        assert(testEntries.size() > 0);
        m_test = testEntries[m_testIndex].createFcn();
    }
    
    size_type GetTestCount() const
    {
        return m_testEntries.size();
    }
    
    Test* GetTest() const
    {
        return m_test.get();
    }
    
    size_type GetIndex() const
    {
        return m_testIndex;
    }
    
    const char* GetName(std::size_t index) const
    {
        return m_testEntries[index].name;
    }
    
    const char* GetName() const
    {
        return m_testEntries[m_testIndex].name;
    }
    
    void SetIndex(size_type index)
    {
        assert(index < GetTestCount());
        
        m_testIndex = index;
        RestartTest();
    }
    
    void RestartTest()
    {
        m_test = m_testEntries[m_testIndex].createFcn();
    }
    
private:
    Span<const TestEntry> m_testEntries;
    size_type m_testIndex;
    std::unique_ptr<Test> m_test;
};
        
//
namespace
{
    TestSuite *g_testSuite = nullptr;
    Selection *g_selection = nullptr;

    Camera camera;
    
    UIState ui;

    Settings settings;
    auto rightMouseDown = false;
    auto leftMouseDown = false;
    Length2D lastp;
    
    Coord2D mouseScreen = Coord2D{0.0, 0.0};
    Length2D mouseWorld = Length2D{};
    
    const auto menuY = 10;
    const auto menuWidth = 200;
    auto menuX = 0;
    auto menuHeight = 0;
}

#ifdef DONT_EMBED_FONT_DATA
static auto GetCwd()
{
    // In C++17 this implementation should be replaced with fs::current_path()
    auto retval = std::string();
#if defined(_WIN32) || defined(_WIN64)
    const auto buffer = _getcwd(NULL, 0);
    if (buffer)
    {
        retval += std::string(buffer);
        std::free(buffer);
    }
#else
    char buffer[1024];
    if (getcwd(buffer, sizeof(buffer)))
    {
        retval += buffer;
    }
#endif
    return retval;
}
#endif

static void CreateUI()
{
    ui.showMenu = true;
    ui.scroll = 0;
    ui.scrollarea1 = 0;
    ui.chooseTest = false;
    ui.mouseOverMenu = false;

    // Init UI
#ifdef DONT_EMBED_FONT_DATA
    const char* fontPaths[] = {
        // Path if Testbed running from MSVS or Xcode Build folder.
        "../../Testbed/Data/DroidSans.ttf",
        
        // This is the original path...
        "../Data/DroidSans.ttf",

        // Path if Testbed app running from Testbed folder
        "Data/DroidSans.ttf",
        
        // Possibly a relative path for windows...
        "../../../../Data/DroidSans.ttf",
        
        // Try the current working directory...
        "./DroidSans.ttf",
    };

    const auto cwd = GetCwd();
    if (cwd.empty())
    {
        std::perror("GetCwd");
    }

    auto fontLoaded = false;
    for (auto&& fontPath: fontPaths)
    {
        fprintf(stderr, "Attempting to load font from \"%s/%s\", ", cwd.c_str(), fontPath);
        const auto data = RenderGLGetFileData(fontPath);
	    if (data)
    	{
            fontLoaded = RenderGLInitFont(data);
            std::free(data);
            
            if (fontLoaded)
            {
                fprintf(stderr, "succeeded.\n");
                break;
            }
    	}
        fprintf(stderr, " failed.\n");
    }
    if (!fontLoaded)
    {
        fprintf(stderr, "Unable to find the font data file. GUI text support disabled.\n",
                "http://www.kottke.org/plus/type/silkscreen/");
    }
#else
    if (RenderGLInitFont(DroidSans_ttf))
    {
        printf("Using embedded DroidSans TTF data.\n");
    }
    else
    {
        fprintf(stderr, "Unable to use embedded font. GUI text support disabled.\n");
    }
#endif

    if (!RenderGLInit())
    {
        fprintf(stderr, "Could not init GUI renderer.\n");
        assert(false);
        return;
    }
}

static void ResizeWindow(GLFWwindow*, int width, int height)
{
    camera.m_width = width;
    camera.m_height = height;
    
    menuX = camera.m_width - menuWidth - 10;
    menuHeight = camera.m_height - 20;
}

static Test::Key GlfwKeyToTestKey(int key)
{
    switch (key)
    {
        case GLFW_KEY_SPACE: return Test::Key_Space;
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
        case GLFW_KEY_BACKSPACE: return Test::Key_Backspace;
        case GLFW_KEY_KP_SUBTRACT: return Test::Key_Subtract;
        case GLFW_KEY_KP_ADD: return Test::Key_Add;
    }
    return Test::Key_Unknown;
}

static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    NOT_USED(scancode);

    if (action == GLFW_PRESS)
    {
        switch (key)
        {
        case GLFW_KEY_ESCAPE:
            // Quit
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;

        case GLFW_KEY_LEFT:
            // Pan left
            if (mods == GLFW_MOD_CONTROL)
            {
                g_testSuite->GetTest()->ShiftOrigin(Length2D(Real(2) * Meter, Real(0) * Meter));
            }
            else
            {
                camera.m_center.x -= 0.5f;
            }
            break;

        case GLFW_KEY_RIGHT:
            // Pan right
            if (mods == GLFW_MOD_CONTROL)
            {
                g_testSuite->GetTest()->ShiftOrigin(Length2D(-Real(2) * Meter, Real(0) * Meter));
            }
            else
            {
                camera.m_center.x += 0.5f;
            }
            break;

        case GLFW_KEY_DOWN:
            // Pan down
            if (mods == GLFW_MOD_CONTROL)
            {
                g_testSuite->GetTest()->ShiftOrigin(Length2D(Real(0) * Meter, Real(2) * Meter));
            }
            else
            {
                camera.m_center.y -= 0.5f;
            }
            break;

        case GLFW_KEY_UP:
            // Pan up
            if (mods == GLFW_MOD_CONTROL)
            {
                g_testSuite->GetTest()->ShiftOrigin(Length2D(Real(0) * Meter, -Real(2) * Meter));
            }
            else
            {
                camera.m_center.y += 0.5f;
            }
            break;

        case GLFW_KEY_HOME:
            // Reset view
            camera.m_zoom = 1.0f;
            camera.m_center = Coord2D{0.0f, 20.0f};
            break;

        case GLFW_KEY_Z:
            // Zoom out
            camera.m_zoom = std::min(1.1f * camera.m_zoom, 20.0f);
            break;

        case GLFW_KEY_X:
            // Zoom in
            camera.m_zoom = std::max(0.9f * camera.m_zoom, 0.02f);
            break;

        case GLFW_KEY_R:
            // Reset test
            g_testSuite->RestartTest();
            break;

        case GLFW_KEY_SPACE:
            // Launch a bomb.
            if (g_testSuite->GetTest())
            {
                g_testSuite->GetTest()->LaunchBomb();
            }
            break;

        case GLFW_KEY_P:
            // Pause
            settings.pause = !settings.pause;
            break;

        case GLFW_KEY_LEFT_BRACKET:
            // Switch to previous test
            g_selection->Decrement();
            break;

        case GLFW_KEY_RIGHT_BRACKET:
            // Switch to next test
            g_selection->Increment();
            break;

        case GLFW_KEY_TAB:
            ui.showMenu = !ui.showMenu;

        default:
            if (g_testSuite->GetTest())
            {
                g_testSuite->GetTest()->KeyboardDown(GlfwKeyToTestKey(key));
            }
        }
    }
    else if (action == GLFW_RELEASE)
    {
        g_testSuite->GetTest()->KeyboardUp(GlfwKeyToTestKey(key));
    }
    // else GLFW_REPEAT
}

static void MouseButton(GLFWwindow*, const int button, const int action, const int mods)
{
    const auto forMenu = (mouseScreen.x >= menuX);

    switch (button)
    {
        case GLFW_MOUSE_BUTTON_LEFT:
        {
            switch (action)
            {
                case GLFW_PRESS:
                    leftMouseDown = true;
                    if (!forMenu)
                    {
                        if (mods == GLFW_MOD_SHIFT)
                        {
                            g_testSuite->GetTest()->ShiftMouseDown(mouseWorld);
                        }
                        else
                        {
                            g_testSuite->GetTest()->MouseDown(mouseWorld);
                        }
                    }
                    break;
                case GLFW_RELEASE:
                    leftMouseDown = false;
                    if (!forMenu)
                    {
                        g_testSuite->GetTest()->MouseUp(mouseWorld);
                    }
                    break;
                default:
                    break;
            }
            break;
        }
        case GLFW_MOUSE_BUTTON_RIGHT:
        {
            switch (action)
            {
                case GLFW_PRESS:
                    lastp = mouseWorld;
                    rightMouseDown = true;
                    break;
                case GLFW_RELEASE:
                    rightMouseDown = false;
                    break;
                default:
                    break;
            }
        }
        default:
            break;
    }
}

static void MouseMotion(GLFWwindow*, double xd, double yd)
{
    // Expects that xd and yd are the new mouse position coordinates,
    // in screen coordinates, relative to the upper-left corner of the
    // client area of the window.
    
    mouseScreen = Coord2D{static_cast<float>(xd), static_cast<float>(yd)};
    mouseWorld = ConvertScreenToWorld(camera, mouseScreen);

    g_testSuite->GetTest()->MouseMove(mouseWorld);
    
    if (rightMouseDown)
    {
        const auto movement = mouseWorld - lastp;
        camera.m_center.x -= static_cast<float>(Real{GetX(movement) / Meter});
        camera.m_center.y -= static_cast<float>(Real{GetY(movement) / Meter});
        lastp = ConvertScreenToWorld(camera, mouseScreen);
    }
}

static void ScrollCallback(GLFWwindow*, double, double dy)
{
    if (ui.mouseOverMenu)
    {
        ui.scroll = -int(dy);
    }
    else
    {
        if (dy > 0)
        {
            camera.m_zoom /= 1.1f;
        }
        else
        {
            camera.m_zoom *= 1.1f;
        }
    }
}

static void Simulate(Drawer& drawer)
{
    glEnable(GL_DEPTH_TEST);
    
    settings.dt = (settings.hz != 0)? 1 / settings.hz : 0;
    if (settings.pause)
    {
        if (!settings.singleStep)
        {
            settings.dt = 0.0f;
        }
    }
    
    g_testSuite->GetTest()->DrawTitle(drawer, g_testSuite->GetName());
    g_testSuite->GetTest()->Step(settings, drawer);

    glDisable(GL_DEPTH_TEST);

    if (settings.pause)
    {
        if (settings.singleStep)
        {
            settings.singleStep = false;
        }
    }
    
    if (g_testSuite->GetIndex() != g_selection->Get())
    {
        g_testSuite->SetIndex(g_selection->Get());
        camera.m_zoom = 1.0f;
        camera.m_center = Coord2D{0.0f, 20.0f};
    }
}

static bool UserInterface(int mousex, int mousey, unsigned char mousebutton, int mscroll)
{
    auto shouldQuit = false;

    imguiBeginFrame(mousex, mousey, mousebutton, mscroll);

    ui.mouseOverMenu = false;
    if (ui.showMenu)
    {
        const auto over = imguiBeginScrollArea("Testbed Controls",
                                               menuX, menuY, menuWidth, menuHeight,
                                               &ui.scrollarea1);
        if (over) ui.mouseOverMenu = true;

        imguiLabel("Test:");
        if (imguiButton(g_testSuite->GetName(), true))
        {
            ui.chooseTest = !ui.chooseTest;
        }

        imguiSeparatorLine();

        const auto defaultLinearSlop = StripUnit(DefaultLinearSlop);
        imguiSlider("Reg Vel Iters", &settings.regVelocityIterations, 0, 100, 1, true);
        imguiSlider("Reg Pos Iters", &settings.regPositionIterations, 0, 100, 1, true);
        imguiSlider("TOI Vel Iters", &settings.toiVelocityIterations, 0, 100, 1, true);
        imguiSlider("TOI Pos Iters", &settings.toiPositionIterations, 0, 100, 1, true);
        imguiSlider("Max Sub Steps", &settings.maxSubSteps, 0, 100, 1, true);
        imguiSlider("Hertz", &settings.hz, -120.0f, 120.0f, 5.0f, true);
        imguiSlider("Linear Slop", &settings.linearSlop,
                    static_cast<float>(defaultLinearSlop / 10),
                    static_cast<float>(defaultLinearSlop),
                    static_cast<float>(defaultLinearSlop / 100),
                    true);
        imguiSlider("Angular Slop", &settings.angularSlop,
                    static_cast<float>(Pi * 2 / 1800.0),
                    static_cast<float>(Pi * 2 / 18.0), 0.001f,
                    true);
        imguiSlider("Reg Min Sep", &settings.regMinSeparation,
                    -5 * static_cast<float>(defaultLinearSlop),
                    -0 * static_cast<float>(defaultLinearSlop),
                    static_cast<float>(defaultLinearSlop) / 20,
                    true);
        imguiSlider("TOI Min Sep", &settings.toiMinSeparation,
                    -5 * static_cast<float>(defaultLinearSlop),
                    -0 * static_cast<float>(defaultLinearSlop),
                    static_cast<float>(defaultLinearSlop) / 20,
                    true);
        imguiSlider("Max Translation", &settings.maxTranslation, 0.0f, 12.0f, 0.05f, true);
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
        if (imguiCheck("Labels", settings.drawLabels, true))
            settings.drawLabels = !settings.drawLabels;
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
            g_testSuite->RestartTest();
        if (imguiButton("Quit", true))
            shouldQuit = true;

        imguiEndScrollArea();
    }

    const auto testMenuWidth = 200;
    if (ui.chooseTest)
    {
        static int testScroll = 0;
        const auto over = imguiBeginScrollArea("Choose Sample",
                                               camera.m_width - menuWidth - testMenuWidth - 20, 10,
                                               testMenuWidth, camera.m_height - 20,
                                               &testScroll);
        if (over) ui.mouseOverMenu = true;

        const auto testCount = g_testSuite->GetTestCount();
        for (auto i = decltype(testCount){0}; i < testCount; ++i)
        {
            if (imguiItem(g_testSuite->GetName(i), true))
            {
                g_selection->Set(i);
                g_testSuite->SetIndex(i);
                ui.chooseTest = false;
            }
        }

        imguiEndScrollArea();
    }

    imguiEndFrame();
    
    return !shouldQuit;
}

static void GlfwErrorCallback(int code, const char* str)
{
    fprintf(stderr, "GLFW error (%d): %s\n", code, str);
}

static void ShowFrameInfo(double frameTime, double fps)
{
    std::stringstream stream;
    const auto viewport = ConvertScreenToWorld(camera);
    stream << "Zoom=" << camera.m_zoom;
    stream << " Center=";
    stream << "{" << camera.m_center.x << "," << camera.m_center.y << "}";
    stream << " Viewport=" << viewport;
    stream << std::setprecision(1);
    stream << std::fixed;
    stream << " Refresh=" << (1000.0 * frameTime) << "ms";
    stream << std::setprecision(0);
    stream << " FPS=" << fps;
    AddGfxCmdText(5, 5, TEXT_ALIGN_LEFT, stream.str().c_str(), static_cast<unsigned int>(WHITE));
}

int main()
{
    TestSuite testSuite(GetTestEntries());
    Selection selection(testSuite.GetTestCount());
    g_testSuite = &testSuite;
    g_selection = &selection;

#if defined(_WIN32)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

    camera.m_width = 1280; // 1152;
    camera.m_height = 980; // 864;
    menuX = camera.m_width - menuWidth - 10;
    menuHeight = camera.m_height - 20;

    if (glfwSetErrorCallback(GlfwErrorCallback))
    {
        fprintf(stderr, "Warning: overriding previously installed GLFW error callback function.\n");
    }

    if (glfwInit() == 0)
    {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    const auto buildVersion = GetVersion();
    const auto buildDetails = GetBuildDetails();
    
    char title[64];
    sprintf(title, "PlayRho Testbed Version %d.%d.%d",
            buildVersion.major, buildVersion.minor, buildVersion.revision);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    const auto mainWindow = glfwCreateWindow(camera.m_width, camera.m_height, title,
                                             nullptr, nullptr);
    if (mainWindow == nullptr)
    {
        fprintf(stderr, "Failed to open GLFW main window.\n");
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(mainWindow);
    printf("PlayRho %d.%d.%d (%s), OpenGL %s, GLSL %s\n",
           buildVersion.major, buildVersion.minor, buildVersion.revision, buildDetails.c_str(),
           glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

    glfwSetScrollCallback(mainWindow, ScrollCallback);
    glfwSetWindowSizeCallback(mainWindow, ResizeWindow);
    glfwSetKeyCallback(mainWindow, KeyCallback);
    glfwSetMouseButtonCallback(mainWindow, MouseButton);
    glfwSetCursorPosCallback(mainWindow, MouseMotion);
    glfwSetScrollCallback(mainWindow, ScrollCallback);

#if !defined(__APPLE__)
    //glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
        exit(EXIT_FAILURE);
    }
#endif
    
    CreateUI();
    
    // Control the frame rate. One draw per monitor refresh.
    glfwSwapInterval(1);
    
    auto time1 = glfwGetTime();
    auto frameTime = 0.0;
    auto fps = 0.0;
    
    glClearColor(0.3f, 0.3f, 0.3f, 1.f);
    {
        DebugDraw drawer(camera);
        while (!glfwWindowShouldClose(mainWindow))
        {
            glViewport(0, 0, camera.m_width, camera.m_height);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            const auto mscroll = ui.scroll;
            ui.scroll = 0;
            const auto mousex = int(mouseScreen.x);
            const auto mousey = camera.m_height - int(mouseScreen.y);
            unsigned char mousebutton = (leftMouseDown)? IMGUI_MBUT_LEFT: 0;
            if (!UserInterface(mousex, mousey, mousebutton, mscroll))
	            glfwSetWindowShouldClose(mainWindow, GL_TRUE);

            Simulate(drawer);
            
            // Measure speed
            const auto time2 = glfwGetTime();
            const auto timeElapsed = time2 - time1;
            const auto alpha = 0.9;
            frameTime = alpha * frameTime + (1.0 - alpha) * timeElapsed;
            fps = 0.99 * fps + (1.0 - 0.99) / timeElapsed;
            time1 = time2;
            ShowFrameInfo(frameTime, fps);
            
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDisable(GL_DEPTH_TEST);
            RenderGLFlush(camera.m_width, camera.m_height);

            glfwSwapBuffers(mainWindow);
            glfwPollEvents();
        }
    }

    RenderGLDestroy();
    glfwTerminate();

    return 0;
}
