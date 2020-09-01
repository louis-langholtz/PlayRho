/*
 * Original work Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include <PlayRho/Common/Version.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <GL/glew.h>
#endif

#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"
#include "DebugDraw.hpp"
#include "Test.hpp"
#include "TestEntry.hpp"
#include "UiState.hpp"

// Uncomment the following define if you'd prefer to use an external file for font data.
//#define DONT_EMBED_FONT_DATA
#ifndef DONT_EMBED_FONT_DATA
#include "DroidSansTtfData.h"
#endif

#include <algorithm>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <cstring>
#include <cctype>
#include <map>
#include <set>

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

using namespace testbed;
using namespace playrho;
using namespace playrho::d2;

class TestSuite;
class Selection;

using BodiesRange = SizedRange<World::Bodies::iterator>;
using JointsRange = SizedRange<World::Joints::iterator>;
using BodyJointsRange = SizedRange<Body::Joints::iterator>;
using ContactsRange = SizedRange<World::Contacts::const_iterator>;
using FixturesRange = SizedRange<Body::Fixtures::iterator>;
using FixtureSet = Test::FixtureSet;
using BodySet = Test::BodySet;

static void EntityUI(Contact& contact);
static void EntityUI(Joint& e);
static void CollectionUI(const ContactsRange& contacts);
static void CollectionUI(const JointsRange& joints);
static void CollectionUI(const BodyJointsRange& joints);

namespace
{
    TestSuite *g_testSuite = nullptr;
    Selection *g_selection = nullptr;
        
    UiState ui;
    
    Test::NeededSettings neededSettings = 0u;
    Settings testSettings;
    Settings settings; ///< User settings.
    auto rightMouseDown = false;
    auto leftMouseDown = false;
    Length2 lastp;
    
    Coord2D mouseScreen = Coord2D{0.0, 0.0};
    Length2 mouseWorld = Length2{};
    
    const auto menuWidth = 200;
    const auto tooltipWrapWidth = 400.0f;
    auto menuX = 0;
    auto menuHeight = 0;
    auto refreshRate = 0;
}

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
    
    int Get() const noexcept
    {
        return m_selection;
    }
    
    void Set(int selection) noexcept
    {
        assert(selection < m_size);
        if (selection < m_size)
        {
            m_selection = selection;
        }
    }
    
    void Increment() noexcept
    {
        const auto next = m_selection + 1;
        m_selection = (next < m_size)? next: 0;
    }
    
    void Decrement() noexcept
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
    	m_testIndex(index < static_cast<int>(size(testEntries))? index: 0)
    {
        assert(!empty(testEntries));
        m_test = testEntries[static_cast<unsigned>(m_testIndex)].createFcn();
    }
    
    int GetTestCount() const
    {
        return static_cast<int>(size(m_testEntries));
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
        if (neededSettings & (0x1u << Test::NeedCameraZoom))
        {
            g_camera.m_zoom = testSettings.cameraZoom;
        }
        else
        {
            g_camera.m_zoom = 1.0f;
        }
        g_camera.m_center = Coord2D{0.0f, 20.0f};
    }
    
    void RestartTest()
    {
        m_test = m_testEntries[static_cast<unsigned>(m_testIndex)].createFcn();
        neededSettings = m_test->GetNeededSettings();
        testSettings = m_test->GetSettings();
    }
    
private:
    Span<const TestEntry> m_testEntries;
    std::unique_ptr<Test> m_test;
public:
    int m_testIndex;
};

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
    if (empty(cwd))
    {
        std::perror("GetCwd");
    }

    auto fontLoaded = false;
    for (auto&& fontPath: fontPaths)
    {
        std::fprintf(stderr, "Attempting to load font from \"%s/%s\", ", cwd.c_str(), fontPath);
        auto data_size = 0;
        void* data = ImFileLoadToMemory(fontPath, "rb", &data_size, 0);
        if (data)
        {
            const auto font = ImGui::GetIO().Fonts->AddFontFromMemoryTTF(data, data_size, 14.f);
            if (font)
            {
                fontLoaded = true;
                std::fprintf(stderr, "succeeded.\n");
                break;
            }
        }
        std::fprintf(stderr, " failed.\n");
    }
    if (!fontLoaded)
    {
        std::fprintf(stderr, "Unable to load external font data. No text may appear.\n");
    }
#else
    auto fontConf = ImFontConfig{};
    fontConf.FontDataOwnedByAtlas = false;
    if (ImGui::GetIO().Fonts->AddFontFromMemoryTTF(DroidSans_ttf,
                                                   static_cast<int>(DroidSans_ttf_len),
                                                   12.0f, &fontConf))
    {
        std::printf("Using embedded DroidSans TTF data.\n");
    }
    else
    {
        std::fprintf(stderr, "Unable to use embedded font. GUI text support disabled.\n");
    }
#endif
    
    if (!ImGui_ImplGlfwGL3_Init(window, false))
    {
        std::fprintf(stderr, "Could not init GUI renderer.\n");
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

static const char* ToString(BodyType type)
{
    switch (type)
    {
        case BodyType::Static: return "Static";
        case BodyType::Kinematic: return "Kinematic";
        case BodyType::Dynamic: return "Dynamic";
    }
    return "Unknown"; // should not be reached
}

static BodyType ToBodyType(int val)
{
    switch (val)
    {
        case 0: return BodyType::Static;
        case 1: return BodyType::Kinematic;
        case 2: return BodyType::Dynamic;
    }
    return BodyType::Static; // should not be reached
}

static void ResizeWindow(GLFWwindow*, int width, int height)
{
    g_camera.m_width = width;
    g_camera.m_height = height;
    
    menuX = g_camera.m_width - menuWidth - 10;
    menuHeight = g_camera.m_height - 20;
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
                g_testSuite->GetTest()->ShiftOrigin(Length2(2_m, 0_m));
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
                g_testSuite->GetTest()->ShiftOrigin(Length2(-2_m, 0_m));
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
                g_testSuite->GetTest()->ShiftOrigin(Length2(0_m, 2_m));
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
                g_testSuite->GetTest()->ShiftOrigin(Length2(0_m, -2_m));
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
            g_camera.m_zoom = std::min(1.1f * g_camera.m_zoom, 20.0f);
            break;

        case GLFW_KEY_X:
            // Zoom in
            g_camera.m_zoom = std::max(0.9f * g_camera.m_zoom, 0.02f);
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
    const auto mouseForUI = ImGui::GetIO().WantCaptureMouse;
    //const auto forMenu = (mouseScreen.x >= menuX);

    switch (button)
    {
        case GLFW_MOUSE_BUTTON_LEFT:
        {
            switch (action)
            {
                case GLFW_PRESS:
                    leftMouseDown = true;
                    if (!mouseForUI)
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
                    if (!mouseForUI)
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
    mouseWorld = ConvertScreenToWorld(mouseScreen);

    g_testSuite->GetTest()->MouseMove(mouseWorld);
    
    if (rightMouseDown)
    {
        const auto movement = mouseWorld - lastp;
        g_camera.m_center.x -= static_cast<float>(Real{GetX(movement) / Meter});
        g_camera.m_center.y -= static_cast<float>(Real{GetY(movement) / Meter});
        lastp = ConvertScreenToWorld(mouseScreen);
    }
}

static void ScrollCallback(GLFWwindow* window, double dx, double dy)
{
    ImGui_ImplGlfwGL3_ScrollCallback(window, dx, dy);
    const auto mouseForUI = ImGui::GetIO().WantCaptureMouse;
    if (!mouseForUI)
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

static void Simulate(Drawer& drawer)
{
    glEnable(GL_DEPTH_TEST);
    
    {
        auto mergedSettings = settings;
        if (neededSettings & (0x1u << Test::NeedDrawSkinsField))
        {
            mergedSettings.drawSkins = testSettings.drawSkins;
        }
        if (neededSettings & (0x1u << Test::NeedDrawLabelsField))
        {
            mergedSettings.drawLabels = testSettings.drawLabels;
        }
        if (neededSettings & (0x1u << Test::NeedLinearSlopField))
        {
            mergedSettings.linearSlop = testSettings.linearSlop;
        }
        if (neededSettings & (0x1u << Test::NeedMaxTranslation))
        {
            mergedSettings.maxTranslation = testSettings.maxTranslation;
        }
        if (neededSettings & (0x1u << Test::NeedDeltaTime))
        {
            mergedSettings.dt = testSettings.dt;
        }
        if (settings.pause)
        {
            if (!settings.singleStep)
            {
                mergedSettings.dt = 0.0f;
            }
        }
        g_testSuite->GetTest()->Step(mergedSettings, drawer, ui);
    }

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

static void AboutTestUI()
{
    const auto test = g_testSuite->GetTest();
    const auto name = g_testSuite->GetName();

    ImGui::LabelText("Test Name", "%s", name);
    
    if (!empty(test->GetSeeAlso()))
    {
        const auto length = size(test->GetSeeAlso());
        char buffer[512];
        std::strncpy(buffer, test->GetSeeAlso().c_str(), std::min(length, sizeof(buffer)));
        buffer[length] = '\0';
        ImGui::InputText("See Also", buffer, sizeof(buffer),
                         ImGuiInputTextFlags_ReadOnly|ImGuiInputTextFlags_AutoSelectAll);
    }
    
    if (!empty(test->GetDescription()))
    {
        if (ImGui::CollapsingHeader("Description", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextWrapped("%s", test->GetDescription().c_str());
        }
    }
    
    const auto handledKeys = test->GetHandledKeys();
    if (!empty(handledKeys))
    {
        if (ImGui::CollapsingHeader("Key Controls", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Columns(3, "KeyColumns", false);
            ImGui::SetColumnWidth(0, 50);
            ImGui::SetColumnWidth(1, 50);
            //ImGui::SetColumnWidth(2, 200);
            for (auto& handledKey: handledKeys)
            {
                const auto keyActionMods = std::get<0>(handledKey);
                const auto keyID = keyActionMods.key;
                const auto mods = keyActionMods.mods;
                
                ImGui::TextUnformatted(GetKeyActionName(keyActionMods.action));
                ImGui::NextColumn();
                
                if (std::isgraph(keyID))
                {
                    const auto shift = mods & GLFW_MOD_SHIFT;
                    const auto ctrl = mods & GLFW_MOD_CONTROL;
                    ImGui::Text("%s%s%c", (ctrl? "ctrl-": ""), (shift? "shift-": ""), keyID);
                }
                else
                {
                    ImGui::Text("%s", GetKeyShortName(keyID));
                    if (ImGui::IsItemHovered() && GetKeyLongName(keyID))
                    {
                        ImGui::SetTooltip("%s", GetKeyLongName(keyID));
                    }
                }
                ImGui::NextColumn();
                //ImGui::SameLine();
                const auto info = test->GetKeyHandlerInfo(std::get<1>(handledKey));
                ImGui::TextWrapped("%s", info.c_str());
                ImGui::NextColumn();
            }
            ImGui::Columns(1);
        }
    }
    
    if (!empty(test->GetStatus()))
    {
        if (ImGui::CollapsingHeader("Status Info", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextWrapped("%s", test->GetStatus().c_str());
        }
    }
    
    if (!empty(test->GetCredits()))
    {
        if (ImGui::CollapsingHeader("Credits"))
        {
            ImGui::TextWrapped("%s", test->GetCredits().c_str());
        }
    }
}

static std::pair<float,int> ToScientific(float val)
{
    std::ostringstream os;
    os << std::scientific << val;
    const auto str = os.str();
    const auto ePos = str.find('e');
    if (ePos != std::string::npos)
    {
        const auto floatPart = str.substr(0, ePos);
        const auto intPart = str.substr(ePos + 1);
        return std::make_pair(std::stof(floatPart), std::stoi(intPart));
    }
    return std::make_pair(0.0f, 0);
}

static void BasicStepOptionsUI()
{
    if (neededSettings & (0x1u << Test::NeedDeltaTime))
    {
        auto frequency = 1.0f / testSettings.dt;
        const auto max = 1.0f / testSettings.minDt;
        const auto min = 1.0f / testSettings.maxDt;
        ImGui::SliderFloat("Frequency", &frequency, min, max, "%.2e Hz");
        frequency = std::clamp(frequency, min, max);
        testSettings.dt = 1.0f / frequency;
    }
    else
    {
        auto frequency = static_cast<int>(std::nearbyint(1.0f / settings.dt));
        ImGui::SliderInt("Frequency", &frequency, 5, 120, "%.0f Hz");
        settings.dt = 1.0f / frequency;
    }
    const auto dt = (neededSettings & (0x1u << Test::NeedDeltaTime))? testSettings.dt: settings.dt;
    if (ImGui::IsItemHovered())
    {
        std::ostringstream os;
        os << "Simulating " << dt << " seconds every step.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    
    ImGui::SliderInt("Vel. Iter.", &settings.regVelocityIterations, 0, 100);
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Maximum number of velocity iterations per step.");
    }
    
    ImGui::SliderInt("Pos. Iter.", &settings.regPositionIterations, 0, 100);
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Maximum number of position iterations per step.");
    }
}

static void AdvancedStepOptionsUI()
{
    const auto defaultLinearSlop = static_cast<float>(Real{DefaultLinearSlop / Meter});

    if (neededSettings & (0x1u << Test::NeedDeltaTime))
    {
        ImGui::SliderFloat("Sim Time", &testSettings.dt,
                           testSettings.minDt, testSettings.maxDt, "%.2e s");
    }
    else
    {
        ImGui::SliderFloat("Sim Time", &settings.dt,
                           settings.minDt, settings.maxDt, "%.2e s");
    }
    const auto dt = (neededSettings & (0x1u << Test::NeedDeltaTime))? testSettings.dt: settings.dt;
    if (ImGui::IsItemHovered())
    {
        std::ostringstream os;
        os << "Simulating " << dt << " seconds every step.";
        os << " This is inversely tied to the frequency.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    
    if (neededSettings & (0x1u << Test::NeedMaxTranslation))
    {
        ImGui::LabelText("Max Translation", "%.2e m", testSettings.maxTranslation);
    }
    else
    {
        ImGui::SliderFloat("Max Translation", &settings.maxTranslation, 0.0f, 12.0f, "%.1f m");
    }
    if (ImGui::IsItemHovered())
    {
        const auto maxTranslation = (neededSettings & (0x1u << Test::NeedMaxTranslation))?
            testSettings.maxTranslation: settings.maxTranslation;
        const auto maxLinearVelocity = maxTranslation / dt;
        std::ostringstream os;
        os << "Max translation is the maximum distance of travel allowed per step." \
            " At its current setting and the current simulation time," \
            " this establishes a max linear velocity of ";
        os << maxLinearVelocity << " m/s.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    
    ImGui::SliderFloat("Max Rotation", &settings.maxRotation, 0.0f, 180.0f, "%.1f °");
    if (ImGui::IsItemHovered())
    {
        std::ostringstream os;
        const auto maxRotationalVelocity = settings.maxRotation / dt;
        os << "Max. rotation in degrees allowed per step." \
            " At its current setting and the current simulation time," \
            " this establishes a max rotational velocity of ";
        os << maxRotationalVelocity << " °/s.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    
    const auto neededLinearSlop = !!(neededSettings & (0x1u << Test::NeedLinearSlopField));
    if (neededLinearSlop)
    {
        ImGui::LabelText("Linear Slop", "%.2e m", testSettings.linearSlop);
    }
    else
    {
        ImGui::SliderFloat("Linear Slop", &settings.linearSlop,
                           defaultLinearSlop / 5, defaultLinearSlop, "%.2e m");
    }
    const auto linearSlop = neededLinearSlop? testSettings.linearSlop: settings.linearSlop;
    const auto targetDepth = 3 * linearSlop;
    if (ImGui::IsItemHovered())
    {
        std::ostringstream os;
        os << "A general basis of \"slop\" to allow for in various length-related calculations.";
        os << " Usually this should be below the visual threshold of scaling used in visualizing the simulation.";
        os << " Results in a TOI-phase target depth of ";
        os << std::scientific << std::setprecision(2) << targetDepth << " m.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    
    ImGui::SliderFloat("Angular Slop", &settings.angularSlop, 1.0f, 20.0f, "%.1f °");
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("A general basis of \"slop\" to allow for in various angle-related calculations.",
                           tooltipWrapWidth);
    }
    
    ImGui::SliderFloat("Max Lin Correct", &settings.maxLinearCorrection, 0.0f, 1.0f, "%.2f m");
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("Maximum linear correction. Should be greater than the linear slop value.",
                           tooltipWrapWidth);
    }
    
    ImGui::SliderFloat("Max Ang Correct", &settings.maxAngularCorrection, 0.0f, 90.0f, "%.1f °");
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Maximum angular correction.");
    }
    
    ImGui::SliderFloat("AABB Exten.", &settings.aabbExtension, 0.0f, defaultLinearSlop * 1000, "%.1e m");
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Linear amount to additively extend all AABBs by.");
    }
    
    if (ImGui::CollapsingHeader("Reg-Phase Processing", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::SliderInt("Vel Iters", &settings.regVelocityIterations, 0, 100);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Maximum number of regular-phase velocity iterations per step.");
        }

        ImGui::SliderInt("Pos Iters", &settings.regPositionIterations, 0, 100);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Maximum number of regular-phase position iterations per step.");
        }
        
        ImGui::SliderFloat("Min Sep", &settings.regMinSeparation,
                           -5 * defaultLinearSlop, -0 * defaultLinearSlop);
        ImGui::SliderInt("Resol Rate", &settings.regPosResRate, 0, 100, "%.0f %%");
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("This is the %% of overlap that will"
                               " be resolved per position iteration.", tooltipWrapWidth);
        }
        ImGui::Checkbox("Allow Sleeping", &settings.enableSleep);
        ImGui::InputFloat("Still To Sleep", &settings.minStillTimeToSleep);
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("The min. time in seconds (in simulated time) that a body"
                               " must be still for before it will be put to sleep.",
                               tooltipWrapWidth);
        }
        ImGui::Checkbox("Warm Starting", &settings.enableWarmStarting);
    }
    if (ImGui::CollapsingHeader("TOI-Phase Processing", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Perform Continuous", &settings.enableContinuous);

        ImGui::SliderInt("Vel Iters", &settings.toiVelocityIterations, 0, 100);
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Maximum number of TOI-phase velocity iterations per step.",
                               tooltipWrapWidth);
        }

        ImGui::SliderInt("Pos Iters", &settings.toiPositionIterations, 0, 100);
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Maximum number of TOI-phase position iterations per step.",
                               tooltipWrapWidth);
        }
        
        settings.tolerance = std::min(settings.tolerance, targetDepth);
        ImGui::SliderFloat("Tolerance", &settings.tolerance, 0.0f, targetDepth, "%.2e m");
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("+/- Tolerance from target depth.", tooltipWrapWidth);
        }

        ImGui::SliderFloat("Min Sep", &settings.toiMinSeparation,
                           -5 * defaultLinearSlop, -0 * defaultLinearSlop);
        ImGui::SliderInt("Resol Rate", &settings.toiPosResRate, 0, 100, "%.0f %%");
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("This is the %% of overlap that will"
                               " be resolved per position iteration.", tooltipWrapWidth);
        }
        ImGui::SliderInt("Max Sub Steps", &settings.maxSubSteps, 0, 200);
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Max # of of sub steps that should be tried in resolving"
                               " collisions at a particular time of impact.", tooltipWrapWidth);
        }
        ImGui::SliderInt("Max Root Iters", &settings.maxToiRootIters, 0, 200);
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Max # of iterations root finder should try before giving up.",
                               tooltipWrapWidth);
        }
        ImGui::Checkbox("Sub-Step", &settings.enableSubStepping);
    }
}

static void OutputOptionsUI()
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
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("Whether or not to show the shape \"skins\" - skins are buffer zones"
                           " around shapes used in collision processing.",
                           tooltipWrapWidth);
    }
    ImGui::Checkbox("AABBs", &settings.drawAABBs);
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("Whether or not to show the Axis Aligned Bounding Boxes (AABB).",
                           tooltipWrapWidth);
    }
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
}

static bool MenuUI()
{
    auto shouldQuit = false;
    const auto button_sz = ImVec2(-1, 0);

    ImGui::PushAllowKeyboardFocus(false); // Disable TAB
    
    ImGui::Text("Test:");
    ImGui::SameLine();
    auto current_item = g_selection->Get();
    if (ImGui::Combo("##Test", &current_item, TestEntriesGetName, nullptr,
                     g_testSuite->GetTestCount(), g_testSuite->GetTestCount()))
    {
        g_selection->Set(current_item);
    }

    ImGui::Columns(2, "TestButtons", false);
    {
        if (ImGui::Button("Previous", button_sz))
        {
            g_selection->Decrement();
        }
        if (ImGui::IsItemHovered())
        {
            // See also support for GLFW_KEY_LEFT_BRACKET
            ImGui::ShowTooltip("Switches to previous test. This can also be invoked by pressing the left bracket key (i.e. '[').",
                               tooltipWrapWidth);
        }
    }
    ImGui::NextColumn();
    {
        if (ImGui::Button("Next", button_sz))
        {
            g_selection->Increment();
        }
        if (ImGui::IsItemHovered())
        {
            // See also support for GLFW_KEY_RIGHT_BRACKET
            ImGui::ShowTooltip("Switches to next test. This can also be invoked by pressing the right bracket key (i.e. ']').",
                               tooltipWrapWidth);
        }
    }
    ImGui::Columns(1);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    
    ImGui::PushItemWidth(100);
    
    if (ImGui::CollapsingHeader("Basic Step Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::IsItemHovered())
        {
            std::ostringstream os;
            os << "These are basic per-\"step\" options. ";
            os << "One step of the simulation is performed for every display refresh.";
            ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
        }
        BasicStepOptionsUI();
    }
    
    if (ImGui::CollapsingHeader("Advanced Step Options"))
    {
        AdvancedStepOptionsUI();
    }
    
    ImGui::PopItemWidth();
    
    if (ImGui::CollapsingHeader("Output Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        OutputOptionsUI();
    }
    
    if (ImGui::CollapsingHeader("Windows", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("About Test", &ui.showAboutTest);
        ImGui::Checkbox("Step Statistics", &ui.showStats);
        ImGui::Checkbox("Entity Editor", &ui.showEntities);
    }
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Checkbox("Pause", &settings.pause);
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("\"Pauses\" the simulation by overriding the simulation time per step"
                          " with a value of zero until un-paused. This can also be toggled by"
                          " pressing the 'P' key.", tooltipWrapWidth);
    }
    
    if (ImGui::Button("Single Step", button_sz))
    {
        settings.singleStep = !settings.singleStep;
    }
    if (ImGui::Button("Restart", button_sz))
    {
        g_testSuite->RestartTest();
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("Restarts the current test. This can also be invoked by pressing the 'R' key.",
                           tooltipWrapWidth);
    }
    if (ImGui::Button("Quit", button_sz))
    {
        shouldQuit = true;
    }
    
    ImGui::PopAllowKeyboardFocus();
    
    return shouldQuit;
}

static void EntityUI(Body& b)
{
    ImGui::ItemWidthContext itemWidthCtx(100);
    {
        const auto location = b.GetLocation();
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(location) / Meter});
        vals[1] = static_cast<float>(Real{GetY(location) / Meter});
        if (ImGui::InputFloat2("Lin. Pos.", vals, -1, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            SetLocation(b, Length2{vals[0] * 1_m, vals[1] * 1_m});
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Linear position in meters.", tooltipWrapWidth);
        }
        const auto angle = b.GetAngle();
        auto val = static_cast<float>(Real{angle / Degree});
        if (ImGui::InputFloat("Ang. Pos.", &val, 0, 0, -1, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            SetAngle(b, val * Degree);
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Angular position in degrees.", tooltipWrapWidth);
        }
    }
    {
        const auto velocity = b.GetVelocity();
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(velocity.linear) / MeterPerSecond});
        vals[1] = static_cast<float>(Real{GetY(velocity.linear) / MeterPerSecond});
        if (ImGui::InputFloat2("Lin. Vel.", vals, -1, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            SetLinearVelocity(b, LinearVelocity2{vals[0] * 1_mps, vals[1] * 1_mps});
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Linear velocity in meters/second.", tooltipWrapWidth);
        }
        auto val = static_cast<float>(Real{velocity.angular / DegreePerSecond});
        if (ImGui::InputFloat("Ang. Vel.", &val, 0, 0, -1, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            SetAngularVelocity(b, val * DegreePerSecond);
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Angular velocity in degrees/second.", tooltipWrapWidth);
        }
    }
    {
        const auto acceleration = GetAcceleration(b);
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(acceleration.linear) / MeterPerSquareSecond});
        vals[1] = static_cast<float>(Real{GetY(acceleration.linear) / MeterPerSquareSecond});
        if (ImGui::InputFloat2("Lin. Acc.", vals, -1, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            SetLinearAcceleration(b, LinearAcceleration2{vals[0] * 1_mps2, vals[1] * 1_mps2});
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Linear acceleration in meters/second².", tooltipWrapWidth);
        }
        auto val = static_cast<float>(Real{acceleration.angular / DegreePerSquareSecond});
        if (ImGui::InputFloat("Ang. Acc.", &val, 0, 0, -1, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            SetAngularAcceleration(b, val * DegreePerSquareSecond);
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::ShowTooltip("Angular acceleration in degrees/second².", tooltipWrapWidth);
        }
    }
    {
        auto v = b.IsImpenetrable();
        if (ImGui::Checkbox("Bullet", &v))
        {
            b.SetBullet(v);
        }
    }
    ImGui::SameLine();
    {
        auto v = !b.IsFixedRotation();
        if (ImGui::Checkbox("Rotatable", &v))
        {
            b.SetFixedRotation(!v);
        }
    }

    {
        auto v = b.IsSleepingAllowed();
        if (ImGui::Checkbox("Sleepable", &v))
        {
            b.SetSleepingAllowed(v);
        }
    }
    ImGui::SameLine();
    {
        auto v = b.IsAwake();
        if (ImGui::Checkbox("Awake", &v))
        {
            if (v)
            {
                b.SetAwake();
            }
            else
            {
                b.UnsetAwake();
            }
        }
    }

    {
        ImGui::GroupContext grpCtx;
        auto v = static_cast<int>(b.GetType());
        ImGui::RadioButton("Static", &v, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Kinem.", &v, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Dynam.", &v, 2);
        b.SetType(ToBodyType(v));
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("Body type selection: either Static, Kinematic, or Dynamic.",
                           tooltipWrapWidth);
    }
    
    {
        auto v = b.IsEnabled();
        if (ImGui::Checkbox("Enabled", &v))
        {
            b.SetEnabled(v);
        }
    }
    
    ImGui::LabelText("Mass", "%.2e kg", static_cast<double>(Real{GetMass(b) / Kilogram}));
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("Mass of the body.", tooltipWrapWidth);
    }
    
    ImGui::LabelText("Rot. Inertia", "%.2e kg·m²",
                     static_cast<double>(Real{GetRotInertia(b) / (1_kg * 1_m2 / Square(1_rad))}));
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("Rotational inertia of the body. This may be the calculated value"
                           " or a set value.", tooltipWrapWidth);
    }
}

static void EntityUI(const Shape& shape)
{
    ImGui::ItemWidthContext itemWidthCtx(60);

    const auto density = GetDensity(shape);
    //const auto vertexRadius = GetVertexRadius(shape);
    const auto friction = GetFriction(shape);
    const auto restitution = GetRestitution(shape);
    const auto childCount = GetChildCount(shape);

    ImGui::LabelText("Density (kg/m²)", "%.2e",
                     static_cast<double>(Real{density * SquareMeter / Kilogram}));
    ImGui::LabelText("Friction", "%f", static_cast<double>(friction));
    ImGui::LabelText("Restitution", "%f", static_cast<double>(restitution));
    ImGui::LabelText("Child Count", "%u", childCount);
    //ImGui::LabelText("Vertex Radius (m)", "%.2e", static_cast<double>(Real{vertexRadius / Meter}));
}

static void EntityUI(Fixture& fixture)
{
    //const auto body = fixture.GetBody();
    
    ImGui::Spacing();

    {
        auto v = fixture.IsSensor();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
        if (ImGui::Checkbox("Sensor", &v))
        {
            fixture.SetSensor(v);
        }
        ImGui::PopStyleVar();
    }

    ImGui::Spacing();
    ImGui::Spacing();

    {
        using CheckboxFlagType = unsigned int;
        const auto oldFilterData = fixture.GetFilterData();
        auto cateBits = CheckboxFlagType{oldFilterData.categoryBits};
        auto maskBits = CheckboxFlagType{oldFilterData.maskBits};
        
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0,0));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(-2.5f,-2.5f));
        auto cateChanged = false;
        for (auto bit = 15u; bit < 16u; --bit)
        {
            ImGui::IdContext idCtx(static_cast<int>(bit));
            auto flags = (0x1u << bit);
            cateChanged |= ImGui::CheckboxFlags("##catebits", &cateBits, flags);
            if (bit > 0) ImGui::SameLine();
        }
        ImGui::PopStyleVar();
        ImGui::PopStyleVar();
        ImGui::SameLine(0, 4);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 4);
        ImGui::Text("Category");

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0,0));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(-2.5f,-2.5f));
        auto maskChanged = false;
        for (auto bit = 15u; bit < 16u; --bit)
        {
            ImGui::IdContext idCtx(static_cast<int>(bit));
            auto flags = (0x1u << bit);
            maskChanged |= ImGui::CheckboxFlags("##maskbits", &maskBits, flags);
            if (bit > 0) ImGui::SameLine();
        }
        ImGui::PopStyleVar();
        ImGui::PopStyleVar();
        ImGui::SameLine(0, 4);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 4);
        ImGui::Text("Mask");

        auto groupIndex = int{oldFilterData.groupIndex};
        {
            ImGui::ItemWidthContext itemWidthCtx(80);
            //ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0,0));
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
            ImGui::InputInt("Group Index", &groupIndex);
            ImGui::PopStyleVar();
            //ImGui::PopStyleVar();
        }

        const auto newFilterData = Filter{
            static_cast<Filter::bits_type>(cateBits),
            static_cast<Filter::bits_type>(maskBits),
            static_cast<Filter::index_type>(groupIndex)
        };
        if (newFilterData != oldFilterData)
        {
            fixture.SetFilterData(newFilterData);
        }
    }
    
    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::Button("Refilter"))
    {
        fixture.Refilter();
    }
    
    {
        const auto& shape = fixture.GetShape();
        if (ImGui::TreeNodeEx(&shape, 0, "Shape/Part"))
        {
            EntityUI(shape);
            ImGui::TreePop();
        }
    }
#if 0
    {
        const auto proxies = fixture.GetProxies();
        if (ImGui::TreeNodeEx("Proxies", 0, "Proxies (%u)", size(proxies)))
        {
            CollectionUI(proxies);
            ImGui::TreePop();
        }
    }
#endif
}

static void EntityUI(const Manifold& m)
{
    std::ostringstream stream;
    stream << "lp=" << m.GetLocalPoint();
    switch (m.GetType())
    {
        case Manifold::e_circles:
            stream << " circles";
            break;
        case Manifold::e_faceA:
        {
            const auto count = m.GetPointCount();
            stream << " faceA=" << int{count};
            for (auto i = decltype(count){0}; i < count; ++i)
            {
                const auto mp = m.GetPoint(i);
                stream << " p[" << int{i} << "]={";
                stream << mp.contactFeature;
                stream << ",";
                stream << mp.localPoint;
                stream << ",";
                stream << mp.normalImpulse;
                stream << ",";
                stream << mp.tangentImpulse;
                stream << "}";
            }
            break;
        }
        case Manifold::e_faceB:
        {
            const auto count = m.GetPointCount();
            stream << " faceB=" << int{count};
            for (auto i = decltype(count){0}; i < count; ++i)
            {
                const auto mp = m.GetPoint(i);
                stream << " p[" << int{i} << "]={";
                stream << mp.contactFeature;
                stream << ",";
                stream << mp.localPoint;
                stream << ",";
                stream << mp.normalImpulse;
                stream << ",";
                stream << mp.tangentImpulse;
                stream << "}";
            }
            break;
        }
        default: break;
    }
    
    ImGui::TextUnformatted("Manifold:");
    ImGui::SameLine();
    ImGui::TextWrappedUnformatted(stream.str());
}

static void CollectionUI(const FixturesRange& fixtures, const FixtureSet& selectedFixtures)
{
    auto fnum = 0;
    for (auto& f: fixtures)
    {
        const auto flags = IsWithin(selectedFixtures, f)? ImGuiTreeNodeFlags_DefaultOpen: 0;
        if (ImGui::TreeNodeEx(f, flags, "Fixture %d", fnum))
        {
            EntityUI(*f);
            ImGui::TreePop();
        }
        ++fnum;
    }
}

static void EntityUI(Body& b, const FixtureSet& selectedFixtures)
{
    EntityUI(b);
    {
        const auto fixtures = b.GetFixtures();
        if (ImGui::TreeNodeEx("Fixtures", 0, "Fixtures (%lu)", size(fixtures)))
        {
            CollectionUI(fixtures, selectedFixtures);
            ImGui::TreePop();
        }
    }
    {
        const auto joints = b.GetJoints();
        if (ImGui::TreeNodeEx("Joints", 0,
                              "Joints (%lu)", size(joints)))
        {
            CollectionUI(joints);
            ImGui::TreePop();
        }
    }
    {
        const auto contacts = b.GetContacts();
        if (ImGui::TreeNodeEx("Contacts", 0,
                              "Contacts (%lu)", size(contacts)))
        {
            CollectionUI(contacts);
            ImGui::TreePop();
        }
    }
}

static void EntityUI(RevoluteJoint& j)
{
    ImGui::LabelText("Ref. Angle (°)", "%.1e",
                     static_cast<double>(Real{j.GetReferenceAngle() / Degree}));
    ImGui::LabelText("Limit State", "%s", ToString(j.GetLimitState()));
    ImGui::LabelText("Motor Impulse (N·m·s)", "%.1e",
                     static_cast<double>(Real{j.GetMotorImpulse() / NewtonMeterSecond}));
    {
        auto v = j.IsLimitEnabled();
        if (ImGui::Checkbox("Enable Limit", &v))
        {
            j.EnableLimit(v);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetLowerLimit() / Degree});
        if (ImGui::InputFloat("Lower Limit (°)", &v, 0, 0, 2))
        {
            j.SetLimits(v * Degree, j.GetUpperLimit());
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetUpperLimit() / Degree});
        if (ImGui::InputFloat("Upper Limit (°)", &v, 0, 0, 2))
        {
            j.SetLimits(j.GetLowerLimit(), v * Degree);
        }
    }
    {
        auto v = j.IsMotorEnabled();
        if (ImGui::Checkbox("Enable Motor", &v))
        {
            j.EnableMotor(v);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetMotorSpeed() / DegreePerSecond});
        if (ImGui::InputFloat("Motor Speed (°/sec)", &v, 0, 0, 2))
        {
            j.SetMotorSpeed(v * DegreePerSecond);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetMaxMotorTorque() / NewtonMeter});
        if (ImGui::InputFloat("Max Mot. Torq. (N·m)", &v))
        {
            j.SetMaxMotorTorque(v * NewtonMeter);
        }
    }
    {
        const auto b = j.GetBodyA();
        if (ImGui::TreeNodeEx(b, 0, "Body A: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

static void EntityUI(PrismaticJoint& j)
{
    ImGui::LabelText("Limit State", "%s", ToString(j.GetLimitState()));
    ImGui::LabelText("Motor Impulse (N·s)", "%.1e",
                     static_cast<double>(Real{j.GetMotorImpulse() / NewtonSecond}));
    ImGui::LabelText("Ref. Angle (°)", "%.1e",
                     static_cast<double>(Real{j.GetReferenceAngle() / Degree}));
    {
        auto v = j.IsLimitEnabled();
        if (ImGui::Checkbox("Enable Limit", &v))
        {
            j.EnableLimit(v);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetLowerLimit() / Meter});
        if (ImGui::InputFloat("Lower Limit (m)", &v, 0, 0, 2))
        {
            j.SetLimits(v * Meter, j.GetUpperLimit());
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetUpperLimit() / Meter});
        if (ImGui::InputFloat("Upper Limit (m)", &v, 0, 0, 2))
        {
            j.SetLimits(j.GetLowerLimit(), v * Meter);
        }
    }
    {
        auto v = j.IsMotorEnabled();
        if (ImGui::Checkbox("Enable Motor", &v))
        {
            j.EnableMotor(v);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetMotorSpeed() / DegreePerSecond});
        if (ImGui::InputFloat("Motor Speed (°/sec)", &v))
        {
            j.SetMotorSpeed(v * DegreePerSecond);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetMaxMotorForce() / Newton});
        if (ImGui::InputFloat("Max. Motor Force (N)", &v))
        {
            j.SetMaxMotorForce(v * Newton);
        }
    }
    {
        const auto b = j.GetBodyA();
        if (ImGui::TreeNodeEx(b, 0, "Body A: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

static void EntityUI(DistanceJoint& j)
{
    // All settings implemented here...
    {
        auto v = static_cast<float>(Real{j.GetLength() / Meter});
        if (ImGui::InputFloat("Length (m)", &v))
        {
            j.SetLength(v * Meter);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetFrequency() / Hertz});
        if (ImGui::InputFloat("Frequency (Hz)", &v))
        {
            j.SetFrequency(v * Hertz);
        }
    }
    {
        auto v = static_cast<float>(j.GetDampingRatio());
        if (ImGui::InputFloat("Damping Ratio", &v))
        {
            j.SetDampingRatio(static_cast<Real>(v));
        }
    }
    {
        const auto b = j.GetBodyA();
        if (ImGui::TreeNodeEx(b, 0, "Body A: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

static void EntityUI(PulleyJoint& j)
{
    ImGui::LabelText("Length A (m)", "%f", static_cast<double>(Real{j.GetLengthA()/Meter}));
    ImGui::LabelText("Length B (m)", "%f", static_cast<double>(Real{j.GetLengthB()/Meter}));
    ImGui::LabelText("Ratio", "%f", static_cast<double>(j.GetRatio()));
}

static void EntityUI(TargetJoint& j)
{
    {
        const auto target = j.GetTarget();
        auto x = static_cast<float>(Real{GetX(target) / Meter});
        auto y = static_cast<float>(Real{GetY(target) / Meter});
        if (ImGui::InputFloat("Target X (m)", &x))
        {
            j.SetTarget(Length2{x * Meter, y * Meter});
        }
        if (ImGui::InputFloat("Target Y (m)", &y))
        {
            j.SetTarget(Length2{x * Meter, y * Meter});
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetMaxForce() / Newton});
        if (ImGui::InputFloat("Max Force (N)", &v))
        {
            j.SetMaxForce(v * Newton);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetFrequency() / Hertz});
        if (ImGui::InputFloat("Frequency (Hz)", &v))
        {
            j.SetFrequency(v * Hertz);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetDampingRatio()});
        if (ImGui::InputFloat("Damping Ratio", &v))
        {
            j.SetDampingRatio(static_cast<Real>(v));
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

static void EntityUI(GearJoint& j)
{
    ImGui::LabelText("Constant", "%.2e", static_cast<double>(j.GetConstant()));
    {
        auto v = static_cast<float>(j.GetRatio());
        if (ImGui::InputFloat("Ratio", &v))
        {
            j.SetRatio(static_cast<Real>(v));
        }
    }
    {
        const auto j1 = j.GetJoint1();
        if (ImGui::TreeNodeEx(j1, 0, "Joint 1 (%s)", ToString(GetType(*j1))))
        {
            EntityUI(*j1);
            ImGui::TreePop();
        }
    }
    {
        const auto j2 = j.GetJoint2();
        if (ImGui::TreeNodeEx(j2, 0, "Joint 2 (%s)", ToString(GetType(*j2))))
        {
            EntityUI(*j2);
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyA();
        if (ImGui::TreeNodeEx(b, 0, "Body A: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

static void EntityUI(WheelJoint& j)
{
    {
        auto v = j.IsMotorEnabled();
        if (ImGui::Checkbox("Enable Motor", &v))
        {
            j.EnableMotor(v);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetMotorSpeed() / DegreePerSecond});
        if (ImGui::InputFloat("Motor Speed (°/sec)", &v))
        {
            j.SetMotorSpeed(v * DegreePerSecond);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetMaxMotorTorque() / NewtonMeter});
        if (ImGui::InputFloat("Max Mot. Torq. (N·m)", &v))
        {
            j.SetMaxMotorTorque(v * NewtonMeter);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetSpringFrequency() / Hertz});
        if (ImGui::InputFloat("Spring Freq. (Hz)", &v))
        {
            j.SetSpringFrequency(v * Hertz);
        }
    }
    {
        auto v = static_cast<float>(j.GetSpringDampingRatio());
        if (ImGui::InputFloat("Spring Damp. Ratio", &v))
        {
            j.SetSpringDampingRatio(static_cast<Real>(v));
        }
    }
    {
        const auto b = j.GetBodyA();
        if (ImGui::TreeNodeEx(b, 0, "Body A: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

static void EntityUI(WeldJoint& j)
{
    ImGui::LabelText("Ref. Angle (°)", "%.1e",
                     static_cast<double>(Real{j.GetReferenceAngle() / Degree}));
    {
        auto v = static_cast<float>(Real{j.GetFrequency() / Hertz});
        if (ImGui::InputFloat("Frequency (Hz)", &v))
        {
            j.SetFrequency(v * Hertz);
        }
    }
    {
        auto v = static_cast<float>(j.GetDampingRatio());
        if (ImGui::InputFloat("Damping Ratio", &v))
        {
            j.SetDampingRatio(static_cast<Real>(v));
        }
    }
    {
        const auto b = j.GetBodyA();
        if (ImGui::TreeNodeEx(b, 0, "Body A: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

static void EntityUI(FrictionJoint& j)
{
    {
        auto v = static_cast<float>(Real{j.GetMaxForce() / Newton});
        if (ImGui::InputFloat("Max Force (N)", &v))
        {
            j.SetMaxForce(v * Newton);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetMaxTorque() / NewtonMeter});
        if (ImGui::InputFloat("Max Torq. (N·m)", &v))
        {
            j.SetMaxTorque(v * NewtonMeter);
        }
    }
    {
        const auto b = j.GetBodyA();
        if (ImGui::TreeNodeEx(b, 0, "Body A: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

static void EntityUI(RopeJoint& j)
{
    ImGui::LabelText("Limit State", "%s", ToString(j.GetLimitState()));
    {
        auto v = static_cast<float>(Real{j.GetMaxLength() / Meter});
        if (ImGui::InputFloat("Max. Length (m)", &v))
        {
            j.SetMaxLength(v * Meter);
        }
    }
    {
        const auto b = j.GetBodyA();
        if (ImGui::TreeNodeEx(b, 0, "Body A: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

static void EntityUI(MotorJoint& j)
{
    {
        const auto linearError = j.GetLinearError();
        ImGui::LabelText("Lin. Error X (m)", "%.2e",
                         static_cast<double>(Real{GetX(linearError) / Meter}));
        ImGui::LabelText("Lin. Error Y (m)", "%.2e",
                         static_cast<double>(Real{GetY(linearError) / Meter}));
    }

    ImGui::LabelText("Ang. Error (°)", "%.2e",
                     static_cast<double>(Real{j.GetAngularError() / Degree}));

    {
        const auto linOff = j.GetLinearOffset();
        auto x = static_cast<float>(Real{GetX(linOff) / Meter});
        auto y = static_cast<float>(Real{GetY(linOff) / Meter});
        if (ImGui::InputFloat("Lin. Offset X (m)", &x))
        {
            j.SetLinearOffset(Length2{x * Meter, y * Meter});
        }
        if (ImGui::InputFloat("Lin. Offset Y (m)", &y))
        {
            j.SetLinearOffset(Length2{x * Meter, y * Meter});
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetAngularOffset() / Degree});
        if (ImGui::InputFloat("Ang. Offset (°)", &v, 0, 0, 2))
        {
            j.SetAngularOffset(v * Degree);
        }
    }

    {
        auto v = static_cast<float>(Real{j.GetMaxForce() / Newton});
        if (ImGui::InputFloat("Max Force (N)", &v))
        {
            j.SetMaxForce(v * Newton);
        }
    }
    {
        auto v = static_cast<float>(Real{j.GetMaxTorque() / NewtonMeter});
        if (ImGui::InputFloat("Max Torq. (N·m)", &v))
        {
            j.SetMaxTorque(v * NewtonMeter);
        }
    }

    {
        auto v = static_cast<float>(j.GetCorrectionFactor());
        if (ImGui::InputFloat("Correction Factor", &v))
        {
            j.SetCorrectionFactor(static_cast<Real>(v));
        }
    }

    {
        const auto b = j.GetBodyA();
        if (ImGui::TreeNodeEx(b, 0, "Body A: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
    {
        const auto b = j.GetBodyB();
        if (ImGui::TreeNodeEx(b, 0, "Body B: %s", ToString(b->GetType())))
        {
            EntityUI(*b, FixtureSet{});
            ImGui::TreePop();
        }
    }
}

class JointVisitorUI: public JointVisitor
{
public:
    void Visit(const RevoluteJoint&) override {}
    void Visit(RevoluteJoint& j) override { EntityUI(j); }
    
    void Visit(const PrismaticJoint&) override {}
    void Visit(PrismaticJoint& j) override { EntityUI(j); }

    void Visit(const DistanceJoint&) override {}
    void Visit(DistanceJoint& j) override { EntityUI(j); }

    void Visit(const PulleyJoint&) override {}
    void Visit(PulleyJoint& j) override { EntityUI(j); }

    void Visit(const TargetJoint&) override {}
    void Visit(TargetJoint& j) override { EntityUI(j); }

    void Visit(const GearJoint&) override {}
    void Visit(GearJoint& j) override { EntityUI(j); }

    void Visit(const WheelJoint&) override {}
    void Visit(WheelJoint& j) override { EntityUI(j); }

    void Visit(const WeldJoint&) override {}
    void Visit(WeldJoint& j) override { EntityUI(j); }

    void Visit(const FrictionJoint&) override {}
    void Visit(FrictionJoint& j) override { EntityUI(j); }

    void Visit(const RopeJoint&) override {}
    void Visit(RopeJoint& j) override { EntityUI(j); }

    void Visit(const MotorJoint&) override {}
    void Visit(MotorJoint& j) override { EntityUI(j); }
};

static void EntityUI(Joint& e)
{
    ImGui::IdContext idCtx(&e);
    ImGui::ItemWidthContext itemWidthCtx(50);

    ImGui::LabelText("Collide Connected", "%s", e.GetCollideConnected()? "true": "false");
    {
        const auto linReact = e.GetLinearReaction();
        ImGui::LabelText("Lin. Reaction X (N·s)", "%.2e",
                         static_cast<double>(Real{GetX(linReact) / NewtonSecond}));
        ImGui::LabelText("Lin. Reaction Y (N·s)", "%.2e",
                         static_cast<double>(Real{GetY(linReact) / NewtonSecond}));
    }
    ImGui::LabelText("Ang. Reaction (N·m·s)", "%.2e",
                     static_cast<double>(Real{e.GetAngularReaction() / NewtonMeterSecond}));
    auto visitor = JointVisitorUI{};
    e.Accept(visitor);
}

static void EntityUI(Contact& c)
{
    ImGui::ItemWidthContext itemWidthCtx(50);
    {
        auto v = c.IsEnabled();
        if (ImGui::Checkbox("Enabled", &v))
        {
            if (v)
            {
                c.SetEnabled();
            }
            else
            {
                c.UnsetEnabled();
            }
        }
    }
    {
        auto val = static_cast<float>(c.GetRestitution());
        if (ImGui::InputFloat("Restitution", &val, 0, 0, -1, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            c.SetRestitution(val);
        }
    }
    {
        auto val = static_cast<float>(c.GetFriction());
        if (ImGui::InputFloat("Friction", &val, 0, 0, -1, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            c.SetFriction(val);
        }
    }
    {
        auto val = static_cast<float>(Real{c.GetTangentSpeed() / MeterPerSecond});
        if (ImGui::InputFloat("Belt Speed", &val, 0, 0, -1, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            c.SetTangentSpeed(val * MeterPerSecond);
        }
    }
    if (c.HasValidToi())
    {
        ImGui::LabelText("TOI", "%f", static_cast<double>(c.GetToi()));
    }
    ImGui::LabelText("TOI Count", "%d", c.GetToiCount());

    if (c.IsTouching())
    {
        EntityUI(c.GetManifold());
    }
    
    {
        const auto f = c.GetFixtureA();
        if (ImGui::TreeNodeEx(f, 0, "Fixture A"))
        {
            EntityUI(*f);
            ImGui::TreePop();
        }
    }
    {
        const auto f = c.GetFixtureB();
        if (ImGui::TreeNodeEx(f, 0, "Fixture B"))
        {
            EntityUI(*f);
            ImGui::TreePop();
        }
    }
}

static void CollectionUI(const BodiesRange& bodies,
                     const BodySet& selectedBodies,
                     const FixtureSet& selectedFixtures)
{
    auto i = 0;
    for (auto e: bodies)
    {
        const auto typeName = ToString(e->GetType());
        const auto flags = IsWithin(selectedBodies, e)? ImGuiTreeNodeFlags_DefaultOpen: 0;
        if (ImGui::TreeNodeEx(e, flags, "Body %d: %s", i, typeName))
        {
            EntityUI(*e, selectedFixtures);
            ImGui::TreePop();
        }
        ++i;
    }
}

static void CollectionUI(const JointsRange& joints)
{
    auto i = 0;
    for (auto& e: joints)
    {
        const auto flags = 0;
        if (ImGui::TreeNodeEx(e, flags, "Joint %d (%s)", i, ToString(GetType(*e))))
        {
            EntityUI(*e);
            ImGui::TreePop();
        }
        ++i;
    }
}

static void CollectionUI(const BodyJointsRange& joints)
{
    auto i = 0;
    for (auto& e: joints)
    {
        const auto j = std::get<1>(e);
        const auto flags = 0;
        if (ImGui::TreeNodeEx(j, flags, "Joint %d (%s)", i, ToString(GetType(*j))))
        {
            EntityUI(*j);
            ImGui::TreePop();
        }
        ++i;
    }
}

static void CollectionUI(const ContactsRange& contacts)
{
    auto i = 0;
    for (auto& ct: contacts)
    {
        const auto e = std::get<1>(ct);
        const auto flags = 0;
        if (ImGui::TreeNodeEx(e, flags, "Contact %d%s",
                              i, ((e->IsTouching())? " (touching)": "")))
        {
            EntityUI(*e);
            ImGui::TreePop();
        }
        ++i;
    }
}

static void ModelEntitiesUI()
{
    const auto test = g_testSuite->GetTest();
    const auto selectedFixtures = test->GetSelectedFixtures();
    const auto selectedBodies = test->GetSelectedBodies();
    const auto selBodies = !empty(selectedFixtures);
    const auto selJoints = false;
    const auto selContacts = false;

    ImGui::PushStyleVar(ImGuiStyleVar_IndentSpacing, ImGui::GetFontSize()*1);
    {
        const auto bodies = test->m_world.GetBodies();
        if (ImGui::TreeNodeEx("Bodies", selBodies? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Bodies (%lu)", size(bodies)))
        {
            CollectionUI(bodies, selectedBodies, selectedFixtures);
            ImGui::TreePop();
        }
    }
    {
        const auto joints = test->m_world.GetJoints();
        if (ImGui::TreeNodeEx("Joints", selJoints? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Joints (%lu)", size(joints)))
        {
            CollectionUI(joints);
            ImGui::TreePop();
        }
    }
    {
        const auto contacts = test->m_world.GetContacts();
        if (ImGui::TreeNodeEx("Contacts", selContacts? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Contacts (%lu)", size(contacts)))
        {
            CollectionUI(contacts);
            ImGui::TreePop();
        }
    }
    ImGui::PopStyleVar();
}

static bool UserInterface()
{
    auto shouldQuit = false;
    
    if (ui.showAboutTest)
    {
        // Note: Use ImGuiCond_Appearing to set the position on first appearance of Test
        //   About info and allow later relocation by user. This is preferred over using
        //   another condition like ImGuiCond_Once, since user could move this window out
        //   of viewport and otherwise having no visual way to recover it.
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Appearing);
        ImGui::SetNextWindowSize(ImVec2(261, 136), ImGuiCond_Once);
        
        // Note: without ImGuiWindowFlags_AlwaysAutoResize, ImGui adds a handle icon
        //   which allows manual resizing but stops automatic resizing.
        ImGui::WindowContext window("About This Test", &ui.showAboutTest,
                                    ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize);
        AboutTestUI();
    }

    if (ui.showMenu)
    {
        ImGui::SetNextWindowPos(ImVec2(float(g_camera.m_width - menuWidth - 10), 10));
        ImGui::SetNextWindowSize(ImVec2(float(menuWidth), float(g_camera.m_height - 20)));
        ImGui::WindowContext window("Testbed Controls", &ui.showMenu,
                                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse);
        shouldQuit = MenuUI();
    }
    
    if (ui.showEntities)
    {
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(240, 700), ImGuiCond_FirstUseEver);
        ImGui::WindowContext window("Entity Editor", &ui.showEntities,
                                    ImGuiWindowFlags_HorizontalScrollbar|ImGuiWindowFlags_NoCollapse);
        ModelEntitiesUI();
    }

    return !shouldQuit;
}

static void GlfwErrorCallback(int code, const char* str)
{
    std::fprintf(stderr, "GLFW error (%d): %s\n", code, str);
}

static void ShowFrameInfo(double frameTime, double fps)
{
    std::stringstream stream;
    const auto viewport = ConvertScreenToWorld();
    stream << "Zoom=" << g_camera.m_zoom;
    stream << " Center=";
    stream << "{" << g_camera.m_center.x << "," << g_camera.m_center.y << "}";
    stream << " Viewport=" << viewport;
    stream << std::setprecision(1);
    stream << std::fixed;
    stream << " Refresh=" << (1000.0 * frameTime) << "ms";
    stream << std::setprecision(0);
    stream << " FPS=" << fps;
    
    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(float(g_camera.m_width), float(g_camera.m_height)));
    ImGui::WindowContext wc("Frame Info", nullptr, ImVec2(0,0), 0.0f,
                 ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoInputs|
                 ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoScrollbar);
    ImGui::SetCursorPos(ImVec2(5, float(g_camera.m_height - 20)));
    ImGui::TextUnformatted(stream.str());
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

    g_camera.m_width = 1280; // 1152;
    g_camera.m_height = 980; // 864;
    menuX = g_camera.m_width - menuWidth - 10;
    menuHeight = g_camera.m_height - 20;

    if (glfwSetErrorCallback(GlfwErrorCallback))
    {
        std::fprintf(stderr, "Warning: overriding previously installed GLFW error callback function.\n");
    }

    if (glfwInit() == 0)
    {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    const auto buildVersion = GetVersion();
    const auto buildDetails = GetBuildDetails();
    
    char title[64];
    std::sprintf(title, "PlayRho Testbed Version %d.%d.%d",
                 buildVersion.major, buildVersion.minor, buildVersion.revision);

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    
    const auto mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, title,
                                             nullptr, nullptr);
    if (!mainWindow)
    {
        std::fprintf(stderr, "Failed to open GLFW main window.\n");
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(mainWindow);
    std::printf("PlayRho %d.%d.%d (%s), OpenGL %s, GLSL %s\n",
                buildVersion.major, buildVersion.minor, buildVersion.revision, buildDetails.c_str(),
                glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

    glfwSwapInterval(1); // Control the frame rate. One draw per monitor refresh.

    const auto monitor = glfwGetPrimaryMonitor();
    const auto vidmode = monitor? glfwGetVideoMode(monitor): static_cast<const GLFWvidmode*>(nullptr);
    refreshRate = vidmode? vidmode->refreshRate: decltype(vidmode->refreshRate){0};
    std::printf("Primary monitor refresh rate: %d Hz\n", refreshRate);

    glfwSetScrollCallback(mainWindow, ScrollCallback);
    glfwSetWindowSizeCallback(mainWindow, ResizeWindow);
    glfwSetKeyCallback(mainWindow, KeyCallback);
    glfwSetMouseButtonCallback(mainWindow, MouseButton);
    glfwSetCursorPosCallback(mainWindow, MouseMotion);
    glfwSetScrollCallback(mainWindow, ScrollCallback);
    glfwSetCharCallback(mainWindow, ImGui_ImplGlfwGL3_CharCallback);

#if !defined(__APPLE__)
    //glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        std::fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
        exit(EXIT_FAILURE);
    }
#endif
    
    CreateUI(mainWindow);
    
    auto time1 = glfwGetTime();
    auto frameTime = 0.0;
    auto fps = 0.0;
    
    glClearColor(0.3f, 0.3f, 0.3f, 1.f);
    {
        DebugDraw drawer(g_camera);
        while (!glfwWindowShouldClose(mainWindow))
        {
            glViewport(0, 0, g_camera.m_width, g_camera.m_height);
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

            ImGui::Render();

            glfwSwapBuffers(mainWindow);
            glfwPollEvents();
        }
    }

    ImGui_ImplGlfwGL3_Shutdown();
    glfwTerminate();

    return 0;
}
