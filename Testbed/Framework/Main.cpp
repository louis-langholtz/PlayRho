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

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <GL/glew.h>
#endif

#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_glfw_gl3.h"
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
#include <cctype>

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
    bool showMenu = true;
    bool showAboutTest = true;
};

class Selection
{
public:
    
    Selection(int size, int selection = 0):
	    m_size(size),
    	m_selection(selection < size? selection: 0)
    {
        assert(size > 0);
        assert(selection < size);
    }
    
    int Get() const
    {
        return m_selection;
    }
    
    void Set(int selection)
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
    int m_selection = 0;
    int m_size = 0;
};

class TestSuite
{
public:
    TestSuite(Span<const TestEntry> testEntries, int index = 0):
    	m_testEntries(testEntries),
    	m_testIndex(index < static_cast<int>(testEntries.size())? index: 0)
    {
        assert(testEntries.size() > 0);
        m_test = testEntries[static_cast<unsigned>(m_testIndex)].createFcn();
    }
    
    int GetTestCount() const
    {
        return static_cast<int>(m_testEntries.size());
    }
    
    Test* GetTest() const
    {
        return m_test.get();
    }
    
    int GetIndex() const
    {
        return m_testIndex;
    }
    
    const char* GetName(std::size_t index) const
    {
        return m_testEntries[index].name;
    }
    
    const char* GetName() const
    {
        return m_testEntries[static_cast<unsigned>(m_testIndex)].name;
    }
    
    void SetIndex(int index)
    {
        assert(index < GetTestCount());
        
        m_testIndex = index;
        RestartTest();
    }
    
    void RestartTest()
    {
        m_test = m_testEntries[static_cast<unsigned>(m_testIndex)].createFcn();
    }
    
private:
    Span<const TestEntry> m_testEntries;
    std::unique_ptr<Test> m_test;
public:
    int m_testIndex;
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

static void CreateUI(GLFWwindow* window)
{
    // Init UI
#ifdef DONT_EMBED_FONT_DATA
    const char* fontPaths[] = {
        // Path if Testbed app running from Testbed folder
        "Data/DroidSans.ttf",
        
        // This is the original path...
        "../Data/DroidSans.ttf",

        // Path if Testbed running from MSVS or Xcode Build folder.
        "../../Testbed/Data/DroidSans.ttf",
        
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
        auto data_size = 0;
        void* data = ImFileLoadToMemory(fontPath, "rb", &data_size, 0);
        if (data)
        {
            const auto font = ImGui::GetIO().Fonts->AddFontFromMemoryTTF(data, data_size, 14.f);
            if (font)
            {
                fontLoaded = true;
                fprintf(stderr, "succeeded.\n");
                break;
            }
        }
        fprintf(stderr, " failed.\n");
    }
    if (!fontLoaded)
    {
        fprintf(stderr, "Unable to load external font data. No text may appear.\n");
    }
#else
    auto fontConf = ImFontConfig{};
    fontConf.FontDataOwnedByAtlas = false;
    if (ImGui::GetIO().Fonts->AddFontFromMemoryTTF(DroidSans_ttf,
                                                   static_cast<int>(DroidSans_ttf_len),
                                                   14.0f, &fontConf))
    {
        printf("Using embedded DroidSans TTF data.\n");
    }
    else
    {
        fprintf(stderr, "Unable to use embedded font. GUI text support disabled.\n");
    }
#endif
    
    if (!ImGui_ImplGlfwGL3_Init(window, false))
    {
        fprintf(stderr, "Could not init GUI renderer.\n");
        assert(false);
        return;
    }
    
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = style.GrabRounding = style.ScrollbarRounding = 2.0f;
    style.FramePadding = ImVec2(4, 2);
    style.DisplayWindowPadding = ImVec2(0, 0);
    style.DisplaySafeAreaPadding = ImVec2(0, 0);

    //ImGuiIO& io = ImGui::GetIO();
    //io.FontGlobalScale = 0.95f;
}

static void ResizeWindow(GLFWwindow*, int width, int height)
{
    camera.m_width = width;
    camera.m_height = height;
    
    menuX = camera.m_width - menuWidth - 10;
    menuHeight = camera.m_height - 20;
}

static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    ImGui_ImplGlfwGL3_KeyCallback(window, key, scancode, action, mods);
    const auto keys_for_ui = ImGui::GetIO().WantCaptureKeyboard;
    if (keys_for_ui)
        return;

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
                g_testSuite->GetTest()->KeyboardHandler(key, action, mods);
            }
        }
    }
    else if (action == GLFW_RELEASE)
    {
        g_testSuite->GetTest()->KeyboardHandler(key, action, mods);
    }
    // else GLFW_REPEAT
}

static void MouseButton(GLFWwindow* window, const int button, const int action, const int mods)
{
    ImGui_ImplGlfwGL3_MouseButtonCallback(window, button, action, mods);

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

static void ScrollCallback(GLFWwindow* window, double dx, double dy)
{
    ImGui_ImplGlfwGL3_ScrollCallback(window, dx, dy);
    const auto mouse_for_ui = ImGui::GetIO().WantCaptureMouse;

    if (!mouse_for_ui)
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

static bool TestEntriesGetName(void*, int idx, const char** out_name)
{
    *out_name = g_testSuite->GetName(static_cast<unsigned>(idx));
    return true;
}

static const char* GetKeyActionName(int action)
{
    switch (action)
    {
        case GLFW_PRESS: return "Press";
        case GLFW_RELEASE: return "Release";
        case GLFW_REPEAT: return "Press+Hold";
        default: return "Unknown action";
    }
}

static const char* GetKeyShortName(int key)
{
    switch (key)
    {
        case GLFW_KEY_SPACE: return "SPACE";
        case GLFW_KEY_BACKSPACE: return "BS";
        case GLFW_KEY_TAB: return "TAB";
        case GLFW_KEY_DELETE: return "DEL";
        case GLFW_KEY_ESCAPE: return "ESC";
        case GLFW_KEY_KP_ADD: return "KP+";
        case GLFW_KEY_KP_SUBTRACT: return "KP-";
        default: break;
    }
    return "Unknown";
}

static const char* GetKeyLongName(int key)
{
    switch (key)
    {
        case GLFW_KEY_BACKSPACE: return "Backspace";
        case GLFW_KEY_DELETE: return "Delete";
        case GLFW_KEY_ESCAPE: return "Escape";
        case GLFW_KEY_KP_ADD: return "KeyPad+";
        case GLFW_KEY_KP_SUBTRACT: return "KeyPad-";
        default: break;
    }
    return nullptr;
}

static bool UserInterface()
{
    auto shouldQuit = false;
    const auto test = g_testSuite->GetTest();

    if (ui.showAboutTest)
    {
        // Note: Use ImGuiCond_Appearing to set the position on first appearance of Test
        //   About info and allow later relocation by user. This is preferred over using
        //   another condition like ImGuiCond_Once, since user could move this window out
        //   of viewport and otherwise having no visual way to recover it.
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Appearing);
        
        // Note: without ImGuiWindowFlags_AlwaysAutoResize, ImGui adds a handle icon
        //   which allows manual resizing but stops automatic resizing.
        ImGui::Begin("About This Test", &ui.showAboutTest,
                     ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize);
        
        const auto name = g_testSuite->GetName();
        ImGui::LabelText("Test Name", "%s", name);

        if (!test->GetSeeAlso().empty())
        {
            const auto length = test->GetSeeAlso().size();
            char buffer[512];
            std::strncpy(buffer, test->GetSeeAlso().c_str(), length);
            buffer[length] = '\0';
            ImGui::InputText("See Also", buffer, 512,
                             ImGuiInputTextFlags_ReadOnly|ImGuiInputTextFlags_AutoSelectAll);
        }

        if (!test->GetDescription().empty())
        {
            if (ImGui::CollapsingHeader("Description", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::TextWrapped("%s", test->GetDescription().c_str());
            }
        }
        
        const auto handledKeys = test->GetHandledKeys();
        if (!handledKeys.empty())
        {
            if (ImGui::CollapsingHeader("Key Controls", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Columns(3, nullptr, false);
                ImGui::SetColumnWidth(0, 50);
                ImGui::SetColumnWidth(1, 50);
                //ImGui::SetColumnWidth(2, 200);
                for (auto& handledKey: handledKeys)
                {
                    const auto keyID = handledKey.first.key;
                    
                    ImGui::TextUnformatted(GetKeyActionName(handledKey.first.action));
                    ImGui::NextColumn();

                    if (std::isgraph(keyID))
                    {
                        ImGui::Text("%c", keyID);
                    }
                    else
                    {
                        ImGui::Text("%s", GetKeyShortName(handledKey.first.key));
                        if (ImGui::IsItemHovered() && GetKeyLongName(handledKey.first.key))
                        {
                            ImGui::SetTooltip("%s", GetKeyLongName(handledKey.first.key));
                        }
                    }
                    ImGui::NextColumn();
                    //ImGui::SameLine();
                    const auto info = test->GetKeyHandlerInfo(handledKey.second);
                    ImGui::TextWrapped("%s", info.c_str());
                    ImGui::NextColumn();
                }
                ImGui::Columns(1);
            }
        }
        
        if (!test->GetStatus().empty())
        {
            if (ImGui::CollapsingHeader("Status Info", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::TextWrapped("%s", test->GetStatus().c_str());
            }
        }
        
        if (!test->GetCredits().empty())
        {
            if (ImGui::CollapsingHeader("Credits"))
            {
                ImGui::TextWrapped("%s", test->GetCredits().c_str());
            }
        }
        
        ImGui::End();
    }
    
    if (ui.showMenu)
    {
        const auto neededSettings = test->GetNeededSettings();
        const auto testSettings = test->GetSettings();

        ImGui::SetNextWindowPos(ImVec2(camera.m_width - menuWidth - 10, 10));
        ImGui::SetNextWindowSize(ImVec2(menuWidth, camera.m_height - 20));
        ImGui::Begin("Testbed Controls", &ui.showMenu,
                     ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse);
        ImGui::PushAllowKeyboardFocus(false); // Disable TAB
        
        ImGui::Text("Test:");
        ImGui::SameLine();
        auto current_item = g_selection->Get();
        if (ImGui::Combo("##Test", &current_item, TestEntriesGetName, nullptr,
                         g_testSuite->GetTestCount(), g_testSuite->GetTestCount()))
        {
            g_selection->Set(current_item);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        const auto defaultLinearSlop = static_cast<float>(StripUnit(DefaultLinearSlop));

        ImGui::PushItemWidth(100);

        if (ImGui::CollapsingHeader("Basic Step Options", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::SliderFloat("Frequency", &settings.hz, -120.0f, 120.0f, "%.0f hz");
            ImGui::SliderInt("Vel. Iter.", &settings.regVelocityIterations, 0, 100);
            ImGui::SliderInt("Pos. Iter.", &settings.regPositionIterations, 0, 100);
        }

        if (ImGui::CollapsingHeader("Advanced Step Options"))
        {
            ImGui::SliderFloat("Frequency", &settings.hz, -120.0f, 120.0f, "%.0f hz");
            ImGui::SliderFloat("Max Translation", &settings.maxTranslation, 0.0f, 12.0f);
            ImGui::SliderFloat("Max Rotation", &settings.maxRotation, 0.0f, 360.0f);
            ImGui::SliderFloat("Linear Slop", &settings.linearSlop,
                               defaultLinearSlop / 10, defaultLinearSlop);
            ImGui::SliderFloat("Angular Slop", &settings.angularSlop,
                               static_cast<float>(Pi * 2 / 1800.0),
                               static_cast<float>(Pi * 2 / 18.0));
            ImGui::SliderFloat("Max Lin Correct", &settings.maxLinearCorrection, 0.0f, 1.0f);
            ImGui::SliderFloat("Max Ang Correct", &settings.maxAngularCorrection, 0.0f, 90.0f);

            if (ImGui::CollapsingHeader("Reg Phase Processing", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::SliderInt("Vel Iters", &settings.regVelocityIterations, 0, 100);
                ImGui::SliderInt("Pos Iters", &settings.regPositionIterations, 0, 100);
                ImGui::SliderFloat("Min Sep", &settings.regMinSeparation,
                                   -5 * defaultLinearSlop, -0 * defaultLinearSlop);
                ImGui::SliderInt("Resol Rate", &settings.regPosResRate, 0, 100, "%.0f %%");
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("This is the %% of overlap that will"
                                      " be resolved per position iteration.");
                }
                ImGui::Checkbox("Allow Sleeping", &settings.enableSleep);
                ImGui::Checkbox("Warm Starting", &settings.enableWarmStarting);
            }
            if (ImGui::CollapsingHeader("TOI Phase Processing", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Checkbox("Perform Continuous", &settings.enableContinuous);
                ImGui::SliderInt("Vel Iters", &settings.toiVelocityIterations, 0, 100);
                ImGui::SliderInt("Pos Iters", &settings.toiPositionIterations, 0, 100);
                ImGui::SliderFloat("Min Sep", &settings.toiMinSeparation,
                                   -5 * defaultLinearSlop, -0 * defaultLinearSlop);
                ImGui::SliderInt("Resol Rate", &settings.toiPosResRate, 0, 100, "%.0f %%");
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("This is the %% of overlap that will"
                                      " be resolved per position iteration.");
                }
                ImGui::SliderInt("Max Sub Steps", &settings.maxSubSteps, 0, 100);
                ImGui::Checkbox("Sub-Step", &settings.enableSubStepping);
            }
        }

        ImGui::PopItemWidth();

        if (ImGui::CollapsingHeader("Output Options", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Checkbox("Shapes", &settings.drawShapes);
            ImGui::Checkbox("Joints", &settings.drawJoints);
            if (neededSettings & (0x1u << Test::NeedDrawSkinsField))
            {
                auto value = testSettings.drawSkins;
                ImGui::Checkbox("Skins (required)", &value);
            }
            else
            {
                ImGui::Checkbox("Skins", &settings.drawSkins);
            }
            ImGui::Checkbox("AABBs", &settings.drawAABBs);
            if (neededSettings & (0x1u << Test::NeedDrawLabelsField))
            {
                auto value = testSettings.drawLabels;
                ImGui::Checkbox("Labels (required)", &value);
            }
            else
            {
                ImGui::Checkbox("Labels", &settings.drawLabels);
            }
            ImGui::Checkbox("Contact Points", &settings.drawContactPoints);
            ImGui::Checkbox("Contact Normals", &settings.drawContactNormals);
            ImGui::Checkbox("Contact Impulses", &settings.drawContactImpulse);
            ImGui::Checkbox("Friction Impulses", &settings.drawFrictionImpulse);
            ImGui::Checkbox("Center of Masses", &settings.drawCOMs);
            ImGui::Checkbox("Statistics", &settings.drawStats);
            ImGui::Checkbox("About Test", &ui.showAboutTest);
        }
        
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Checkbox("Pause", &settings.pause);

        ImVec2 button_sz = ImVec2(-1, 0);
        if (ImGui::Button("Single Step", button_sz))
            settings.singleStep = !settings.singleStep;
        if (ImGui::Button("Restart", button_sz))
            g_testSuite->RestartTest();
        if (ImGui::Button("Quit", button_sz))
            shouldQuit = true;

        ImGui::PopAllowKeyboardFocus();
        ImGui::End();
    }
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
    
    ImGui::Begin("Overlay", nullptr, ImVec2(0,0), 0.0f,
                 ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoInputs|
                 ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoScrollbar);
    ImGui::SetCursorPos(ImVec2(5, camera.m_height - 20));
    ImGui::TextUnformatted(stream.str().c_str());
    ImGui::End();
}

int main()
{
#if defined(_WIN32)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

    TestSuite testSuite(GetTestEntries());
    Selection selection(testSuite.GetTestCount());
    g_testSuite = &testSuite;
    g_selection = &selection;

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

    glfwSwapInterval(1); // Control the frame rate. One draw per monitor refresh.
    glfwInit();

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
    
    CreateUI(mainWindow);
    
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

            ImGui_ImplGlfwGL3_NewFrame();

            if (!UserInterface())
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
            //RenderGLFlush(camera.m_width, camera.m_height);

            ImGui::Render();

            glfwSwapBuffers(mainWindow);
            glfwPollEvents();
        }
    }

    //RenderGLDestroy();
    ImGui_ImplGlfwGL3_Shutdown();
    glfwTerminate();

    return 0;
}
