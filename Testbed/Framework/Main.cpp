/*
 * Original work Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "ExtensionsForImgui.hpp"

// From imgui example code:
//  "About Desktop OpenGL function loaders:
//   Modern desktop OpenGL doesn't have a standard portable header file to load OpenGL function
//   pointers. Helper libraries are often used for this purpose! Here we are supporting a few
//   common ones (gl3w, glew, glad). You may use another loader/header of your choice (glext,
//   glLoadGen, etc.), or chose to manually implement your own."
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>            // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>            // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>          // Initialize with gladLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD2)
#include <glad/gl.h>            // Initialize with gladLoadGL(...) or gladLoaderLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING2)
#define GLFW_INCLUDE_NONE       // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/Binding.h>  // Initialize with glbinding::Binding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING3)
#define GLFW_INCLUDE_NONE       // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/glbinding.h>// Initialize with glbinding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

#if defined(__APPLE__)
//#define GL_SILENCE_DEPRECATION
//#include <OpenGL/gl3.h>
#include <GL/gl3w.h>
#else
#include <GL/glew.h>
#endif

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

using namespace testbed;
using namespace playrho;
using namespace playrho::d2;

class TestSuite;
class Selection;

using FixtureSet = Test::FixtureSet;
using BodySet = Test::BodySet;

static void EntityUI(World& world, ContactID contact);
static void EntityUI(World& world, JointID e);
static void CollectionUI(World& world, const World::Contacts& contacts, bool interactive = true);
static void CollectionUI(World& world, const World::Joints& joints);
static void CollectionUI(World& world, const World::BodyJoints& joints);

namespace
{
    TestSuite *g_testSuite = nullptr;
    Selection *g_selection = nullptr;
    UiState *ui = nullptr;

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

    auto shapeTransformationMatrix = GetIdentity<Mat22>();
    constexpr char shapeTransformButtonName[] = "Transform";
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
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    if (!ImGui_ImplGlfw_InitForOpenGL(window, false))
    {
        std::fprintf(stderr, "Could not init GUI renderer.\n");
        assert(false);
        return;
    }
    ImGui_ImplOpenGL3_Init();

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
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = style.GrabRounding = style.ScrollbarRounding = 2.0f;
    style.FramePadding = ImVec2(4, 2);
    style.DisplayWindowPadding = ImVec2(0, 0);
    style.DisplaySafeAreaPadding = ImVec2(0, 0);

    //ImGuiIO& io = ImGui::GetIO();
    //io.FontGlobalScale = 0.95f;
}

static const char* ToString(BodyType type) noexcept
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
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
    const auto keys_for_ui = ImGui::GetIO().WantCaptureKeyboard;
    if (keys_for_ui)
        return;

    if (action == GLFW_PRESS) {
        switch (key) {
        case GLFW_KEY_ESCAPE:
            // Quit
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;

        case GLFW_KEY_LEFT:
            // Pan left
            if (mods == GLFW_MOD_CONTROL) {
                g_testSuite->GetTest()->ShiftOrigin(Length2(2_m, 0_m));
            }
            else {
                g_camera.m_center.x -= 0.5f;
            }
            break;

        case GLFW_KEY_RIGHT:
            // Pan right
            if (mods == GLFW_MOD_CONTROL) {
                g_testSuite->GetTest()->ShiftOrigin(Length2(-2_m, 0_m));
            }
            else {
                g_camera.m_center.x += 0.5f;
            }
            break;

        case GLFW_KEY_DOWN:
            // Pan down
            if (mods == GLFW_MOD_CONTROL) {
                g_testSuite->GetTest()->ShiftOrigin(Length2(0_m, 2_m));
            }
            else {
                g_camera.m_center.y -= 0.5f;
            }
            break;

        case GLFW_KEY_UP:
            // Pan up
            if (mods == GLFW_MOD_CONTROL) {
                g_testSuite->GetTest()->ShiftOrigin(Length2(0_m, -2_m));
            }
            else {
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
            if (g_testSuite->GetTest()) {
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
            ui->showMenu = !ui->showMenu;
            break;

        default:
            if (g_testSuite->GetTest()) {
                g_testSuite->GetTest()->KeyboardHandler(key, action, mods);
            }
            break;
        }
    }
    else if (action == GLFW_RELEASE) {
        g_testSuite->GetTest()->KeyboardHandler(key, action, mods);
    }
    // else GLFW_REPEAT
}

static void MouseButton(GLFWwindow* window, const int button, const int action, const int mods)
{
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
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
    ImGui_ImplGlfw_ScrollCallback(window, dx, dy);
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
        g_testSuite->GetTest()->Step(mergedSettings, drawer, *ui);
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
            ImGui::ColumnsContext cc(3, "KeyColumns", false);
            ImGui::SetColumnWidth(0, 50);
            ImGui::SetColumnWidth(1, 50);
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

#if 0
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
#endif

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
        ImGui::IdContext idContext{"Reg-Phase Processing"};

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
        ImGui::IdContext idContext{"TOI-Phase Processing"};

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
        ImGui::Checkbox("About Test", &ui->showAboutTest);
        ImGui::Checkbox("Step Statistics", &ui->showStats);
        ImGui::Checkbox("Entity Editor", &ui->showEntities);
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
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip("Quits the application. This can also be invoked by pressing the 'ESC' key.",
                           tooltipWrapWidth);
    }

    ImGui::PopAllowKeyboardFocus();
    
    return shouldQuit;
}

static void EntityUI(World& world, BodyID b)
{
    ImGui::IdContext idCtx(to_underlying(b));
    {
        const auto location = GetLocation(world, b);
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(location) / Meter});
        vals[1] = static_cast<float>(Real{GetY(location) / Meter});
        if (ImGui::InputFloat2("Lin. Pos.", vals, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetLocation(world, b, Length2{vals[0] * 1_m, vals[1] * 1_m});
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Linear position in meters.", tooltipWrapWidth);
        }
        const auto angle = GetAngle(world, b);
        auto val = static_cast<float>(Real{angle / Degree});
        if (ImGui::InputFloat("Ang. Pos.", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetAngle(world, b, val * Degree);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Angular position in degrees.", tooltipWrapWidth);
        }
    }
    {
        const auto velocity = GetVelocity(world, b);
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(velocity.linear) / MeterPerSecond});
        vals[1] = static_cast<float>(Real{GetY(velocity.linear) / MeterPerSecond});
        if (ImGui::InputFloat2("Lin. Vel.", vals, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetVelocity(world, b, LinearVelocity2{vals[0] * 1_mps, vals[1] * 1_mps});
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Linear velocity in meters/second.", tooltipWrapWidth);
        }
        auto val = static_cast<float>(Real{velocity.angular / DegreePerSecond});
        if (ImGui::InputFloat("Ang. Vel.", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetVelocity(world, b, val * DegreePerSecond);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Angular velocity in degrees/second.", tooltipWrapWidth);
        }
    }
    {
        const auto acceleration = GetAcceleration(world, b);
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(acceleration.linear) / MeterPerSquareSecond});
        vals[1] = static_cast<float>(Real{GetY(acceleration.linear) / MeterPerSquareSecond});
        if (ImGui::InputFloat2("Lin. Acc.", vals, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetAcceleration(world, b, LinearAcceleration2{vals[0] * 1_mps2, vals[1] * 1_mps2});
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Linear acceleration in meters/second².", tooltipWrapWidth);
        }
        auto val = static_cast<float>(Real{acceleration.angular / DegreePerSquareSecond});
        if (ImGui::InputFloat("Ang. Acc.", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetAcceleration(world, b, val * DegreePerSquareSecond);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Angular acceleration in degrees/second².", tooltipWrapWidth);
        }
    }
    {
        auto v = IsImpenetrable(world, b);
        if (ImGui::Checkbox("Bullet", &v)) {
            if (v)
                SetImpenetrable(world, b);
            else
                UnsetImpenetrable(world, b);
        }
    }
    ImGui::SameLine();
    {
        auto v = !IsFixedRotation(world, b);
        if (ImGui::Checkbox("Rotatable", &v)) {
            SetFixedRotation(world, b, !v);
        }
    }
    {
        auto v = IsSleepingAllowed(world, b);
        if (ImGui::Checkbox("Sleepable", &v)) {
            SetSleepingAllowed(world, b, v);
        }
    }
    ImGui::SameLine();
    {
        auto v = IsAwake(world, b);
        if (ImGui::Checkbox("Awake", &v)) {
            if (v) {
                SetAwake(world, b);
            }
            else {
                UnsetAwake(world, b);
            }
        }
    }

    {
        ImGui::GroupContext grpCtx;
        auto v = static_cast<int>(GetType(world, b));
        ImGui::RadioButton("Static", &v, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Kinem.", &v, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Dynam.", &v, 2);
        SetType(world, b, ToBodyType(v));
    }
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Body type selection: either Static, Kinematic, or Dynamic.",
                           tooltipWrapWidth);
    }
    
    {
        auto v = IsEnabled(world, b);
        if (ImGui::Checkbox("Enabled", &v)) {
            SetEnabled(world, b, v);
        }
    }
    
    ImGui::LabelText("Mass", "%.2e kg", static_cast<double>(Real{GetMass(world, b) / Kilogram}));
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Mass of the body.", tooltipWrapWidth);
    }
    
    ImGui::LabelText("Rot. Inertia", "%.2e kg·m²",
                     static_cast<double>(Real{GetRotInertia(world, b) / (1_kg * 1_m2 / Square(1_rad))}));
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Rotational inertia of the body. This may be the calculated value"
                           " or a set value.", tooltipWrapWidth);
    }
}

static void EntityUI(const DistanceProxy& proxy)
{
    ImGui::IdContext idCtx("DistanceProxyCtx");
    ImGui::ItemWidthContext itemWidthCtx(60);
    ImGui::LabelText("Vertex radius (m)", "%.2e",
                     static_cast<double>(Real{GetVertexRadius(proxy) / 1_m}));
    const auto numVertices = proxy.GetVertexCount();
    if (ImGui::TreeNodeEx("Vertices", 0, "Vertices (%u)", numVertices)) {
        for (auto i = static_cast<VertexCounter>(0); i < numVertices; ++i) {
            ImGui::ColumnsContext cc(3, "VertexColumns", false);
            const auto vertex = proxy.GetVertex(i);
            ImGui::SetColumnWidths(140, {15, 68, 68});
            ImGui::Text("%u", i);
            ImGui::NextColumn();
            ImGui::Text(((GetX(vertex) >= 0_m)? "+%fm": "%fm"),
                        static_cast<float>(Real{GetX(vertex)/1_m}));
            ImGui::NextColumn();
            ImGui::Text(((GetY(vertex) >= 0_m)? "+%fm": "%fm"),
                        static_cast<float>(Real{GetY(vertex)/1_m}));
            ImGui::NextColumn();
        }
        ImGui::TreePop();
    }
}

static void ChildrenUI(Shape &shape)
{
    ImGui::IdContext idCtx("ShapeChildrenCtx");
    const auto n = GetChildCount(shape);
    for (auto i = ChildCounter(0); i < n; ++i) {
        if (ImGui::TreeNodeEx(reinterpret_cast<const void*>(i), 0, "Child %u", i)) {
            EntityUI(GetChild(shape, i));
            ImGui::TreePop();
        }
    }
}

static void EntityUI(Shape &shape)
{
    {
        ImGui::ItemWidthContext itemWidthCtx(60);
        //const auto vertexRadius = GetVertexRadius(shape);
        {
            auto val = static_cast<float>(Real{GetDensity(shape) * SquareMeter / Kilogram});
            if (ImGui::InputFloat("Density (kg/m²)", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                try {
                    SetDensity(shape, val * Kilogram / SquareMeter);
                } catch (const std::invalid_argument& ex) {
                    ui->message = ex.what();
                }
            }
        }
        ImGui::LabelText("Mass (kg)", "%.2e",
                         static_cast<double>(Real{GetMassData(shape).mass / 1_kg}));
        {
            auto val = static_cast<float>(GetFriction(shape));
            if (ImGui::InputFloat("Friction", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                try {
                    SetFriction(shape, val);
                } catch (const std::invalid_argument& ex) {
                    ui->message = ex.what();
                }
            }
            if (ImGui::IsItemHovered()) {
                ImGui::ShowTooltip("Friction for the shape. Value must be non-negative!",
                                   tooltipWrapWidth);
            }
        }
        {
            auto val = static_cast<float>(GetRestitution(shape));
            if (ImGui::InputFloat("Restitution", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                try {
                    SetRestitution(shape, val);
                } catch (const std::invalid_argument& ex) {
                    ui->message = ex.what();
                }
            }
            if (ImGui::IsItemHovered()) {
                ImGui::ShowTooltip("Restitution/bounciness for the shape. Value must be finite!",
                                   tooltipWrapWidth);
            }
        }
    }

    ImGui::Spacing();

    {
        auto v = IsSensor(shape);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
        if (ImGui::Checkbox("Sensor", &v)) {
            try {
                SetSensor(shape, v);
            } catch (const std::invalid_argument& ex) {
                ui->message = ex.what();
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Whether or not this acts as a sensor."
                               " Sensors detect collisions but don't participate"
                               " in their resolution - i.e. bodies will pass right through"
                               " bodies having just sensor shapes.",
                               tooltipWrapWidth);
        }
        ImGui::PopStyleVar();
    }

    ImGui::Spacing();

    {
        using CheckboxFlagType = unsigned int;
        const auto oldFilterData = GetFilter(shape);
        auto cateBits = CheckboxFlagType{oldFilterData.categoryBits};
        auto maskBits = CheckboxFlagType{oldFilterData.maskBits};

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0,0));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(-2.5f,-2.5f));
        auto cateChanged = false;
        for (auto bit = 15u; bit < 16u; --bit) {
            ImGui::IdContext subIdCtx(static_cast<int>(bit));
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
        for (auto bit = 15u; bit < 16u; --bit) {
            ImGui::IdContext subIdCtx(static_cast<int>(bit));
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
        if (newFilterData != oldFilterData) {
            try {
                SetFilter(shape, newFilterData);
            } catch (const std::invalid_argument& ex) {
                ui->message = ex.what();
            }
        }
    }

    ImGui::Spacing();

    if (ImGui::TreeNodeEx("ShapeChildren", 0, "Children (%u)", GetChildCount(shape))) {
        ChildrenUI(shape);
        ImGui::TreePop();
    }

    if (ImGui::Button(shapeTransformButtonName, ImVec2(-1, 0))) {
        Transform(shape, shapeTransformationMatrix);
    }
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

static void EntityUI(World& world, ShapeID shapeId)
{
    auto shape = GetShape(world, shapeId);
    if (shape.has_value()) {
        if (ImGui::TreeNodeEx(reinterpret_cast<const void*>(to_underlying(shapeId)), 0,
                              "Shape %u (%s)", to_underlying(shapeId),
                              Test::ToName(GetType(shape)))) {
            ImGui::IdContext shapeIdCtx(to_underlying(shapeId));
            EntityUI(shape);
            if (GetShape(world, shapeId) != shape) {
                SetShape(world, shapeId, shape);
            }
            if (ImGui::Button("Destroy", ImVec2(-1, 0))) {
                Destroy(world, shapeId);
            }
            ImGui::TreePop();
        }
    }
    else {
        ImGui::Text("Shape %u (empty)", to_underlying(shapeId));
    }
}

static void ShapesUI(World& world)
{
    ImGui::IdContext idCtx("WorldShapes");
    const auto y = ImGui::GetCursorPosY();
    {
        const auto columnWidth = 40.0f;
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2,2));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
        ImGui::Columns(2, "TransformationMatrix", false);
        ImGui::SetColumnWidth(0, columnWidth);
        ImGui::SetColumnWidth(1, columnWidth);
        {
            ImGui::ItemWidthContext itemWidthCtx(columnWidth);
            ImGui::InputFloat("##00", &shapeTransformationMatrix[0][0]);
        }
        ImGui::NextColumn();
        {
            ImGui::ItemWidthContext itemWidthCtx(columnWidth);
            ImGui::InputFloat("##01", &shapeTransformationMatrix[0][1]);
        }
        ImGui::NextColumn();
        {
            ImGui::ItemWidthContext itemWidthCtx(columnWidth);
            ImGui::InputFloat("##10", &shapeTransformationMatrix[1][0]);
        }
        ImGui::NextColumn();
        {
            ImGui::ItemWidthContext itemWidthCtx(columnWidth);
            ImGui::InputFloat("##11", &shapeTransformationMatrix[1][1]);
        }
        ImGui::Columns(1);
        ImGui::PopStyleVar();
        ImGui::PopStyleVar();
    }
    ImGui::SameLine();
    ImGui::SetCursorPosY(y + 3);
    ImGui::TextUnformatted("Transformation\nMatrix");
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Matrix that's applied on pressing a shape's transform button.",
                           tooltipWrapWidth);
    }
    //ImGui::NewLine();
    ImGui::Spacing();
    ImGui::Spacing();
    const auto numShapes = world.GetShapeRange();
    for (auto i = static_cast<ShapeCounter>(0); i < numShapes; ++i) {
        EntityUI(world, ShapeID(i));
    }
}

static void AttachShapeUI(World& world, BodyID bodyId, const std::vector<ShapeID>& shapeIds)
{
    const auto shapeRange = GetShapeRange(world);
    auto available = std::vector<decltype(GetShapeRange(world))>{};
    for (auto i = static_cast<decltype(GetShapeRange(world))>(0); i < shapeRange; ++i) {
        const auto shapeId = ShapeID(i);
        const auto last = end(shapeIds);
        if (find(begin(shapeIds), last, shapeId) == last) {
            available.push_back(i);
        }
    }
    if (empty(available)) return;

    static auto itemCurrentIdx = 0u;
    static auto flags = ImGuiComboFlags(0);
    const auto comboLabel = std::to_string(available[itemCurrentIdx]);
    if (ImGui::BeginCombo("##BodyShapeSelectionCombo", comboLabel.c_str(), flags)) {
        for (const auto& i: available) {
            const auto pos = static_cast<decltype(itemCurrentIdx)>(&i - available.data());
            const bool isSelected = (itemCurrentIdx == pos);
            const auto label = std::to_string(i);
            if (ImGui::Selectable(label.c_str(), isSelected)) {
                itemCurrentIdx = pos;
            }
            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (isSelected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Button("Attach", ImVec2(-1, 0))) {
        Attach(world, bodyId, ShapeID(available[itemCurrentIdx]));
    }
}

static void CollectionUI(World& world, const std::vector<ShapeID>& shapeIds, BodyID bodyId,
                         const FixtureSet& selectedFixtures)
{
    ImGui::IdContext idCtx("BodyShapesCtx");
    AttachShapeUI(world, bodyId, shapeIds);
    for (const auto& shapeId: shapeIds) {
        ImGui::Text("Shape %u (%s %s)", to_underlying(shapeId),
                    Test::ToName(GetType(world, shapeId)),
                    (IsWithin(selectedFixtures, std::make_pair(bodyId, shapeId))?
                     "selected": "not-selected"));
    }
}

static void EntityUI(World& world, BodyID bodyId, const FixtureSet& selectedFixtures)
{
    ImGui::ItemWidthContext itemWidthCtx(100);
    EntityUI(world, bodyId);
    {
        const auto shapes = GetShapes(world, bodyId);
        if (ImGui::TreeNodeEx("BodyShapes", 0, "Shapes (%lu)", size(shapes))) {
            CollectionUI(world, shapes, bodyId, selectedFixtures);
            ImGui::TreePop();
        }
    }
    {
        const auto joints = GetJoints(world, bodyId);
        if (ImGui::TreeNodeEx("BodyJoints", 0, "Joints (%lu)", size(joints))) {
            CollectionUI(world, joints);
            ImGui::TreePop();
        }
    }
    {
        const auto contacts = GetContacts(world, bodyId);
        if (ImGui::TreeNodeEx("BodyContacts", 0, "Contacts (%lu)", size(contacts))) {
            CollectionUI(world, contacts, false);
            ImGui::TreePop();
        }
    }
    if (ImGui::Button("Destroy", ImVec2(-1, 0))) {
        Destroy(world, bodyId);
    }
}

static void EntityUI(RevoluteJointConf& conf, BodyCounter bodyRange)
{
    ImGui::IdContext idCtx("RevoluteJointConf");

    ImGui::LabelText("Ref. Angle (°)", "%.1e",
                     static_cast<double>(Real{GetReferenceAngle(conf) / Degree}));
    ImGui::LabelText("Limit State", "%s", ToString(GetLimitState(conf)));
    ImGui::LabelText("Motor Impulse (N·m·s)", "%.1e",
                     static_cast<double>(Real{GetAngularMotorImpulse(conf) / NewtonMeterSecond}));
    {
        auto v = IsLimitEnabled(conf);
        if (ImGui::Checkbox("Enable Limit", &v))
        {
            EnableLimit(conf, v);
        }
    }
    {
        auto v = static_cast<float>(Real{GetAngularLowerLimit(conf) / Degree});
        if (ImGui::InputFloat("Lower Limit (°)", &v, 0, 0, "%.2f"))
        {
            SetAngularLimits(conf, v * Degree, GetAngularUpperLimit(conf));
        }
    }
    {
        auto v = static_cast<float>(Real{GetAngularUpperLimit(conf) / Degree});
        if (ImGui::InputFloat("Upper Limit (°)", &v, 0, 0, "%.2f"))
        {
            SetAngularLimits(conf, GetAngularLowerLimit(conf), v * Degree);
        }
    }
    {
        auto v = IsMotorEnabled(conf);
        if (ImGui::Checkbox("Enable Motor", &v))
        {
            EnableMotor(conf, v);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMotorSpeed(conf) / DegreePerSecond});
        if (ImGui::InputFloat("Motor Speed (°/sec)", &v, 0, 0, "%.2f"))
        {
            SetMotorSpeed(conf, v * DegreePerSecond);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMaxMotorTorque(conf) / NewtonMeter});
        if (ImGui::InputFloat("Max Mot. Torq. (N·m)", &v))
        {
            SetMaxMotorTorque(conf, v * NewtonMeter);
        }
    }
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(PrismaticJointConf& conf, BodyCounter bodyRange)
{
    ImGui::LabelText("Limit State", "%s", ToString(GetLimitState(conf)));
    ImGui::LabelText("Motor Impulse (N·s)", "%.1e",
                     static_cast<double>(Real{GetLinearMotorImpulse(conf) / NewtonSecond}));
    ImGui::LabelText("Ref. Angle (°)", "%.1e",
                     static_cast<double>(Real{GetReferenceAngle(conf) / Degree}));
    {
        auto v = IsLimitEnabled(conf);
        if (ImGui::Checkbox("Enable Limit", &v))
        {
            EnableLimit(conf, v);
        }
    }
    {
        auto v = static_cast<float>(Real{GetLinearLowerLimit(conf) / Meter});
        if (ImGui::InputFloat("Lower Limit (m)", &v, 0, 0, "%.2f"))
        {
            SetLinearLimits(conf, v * Meter, GetLinearUpperLimit(conf));
        }
    }
    {
        auto v = static_cast<float>(Real{GetLinearUpperLimit(conf) / Meter});
        if (ImGui::InputFloat("Upper Limit (m)", &v, 0, 0, "%.2f"))
        {
            SetLinearLimits(conf, GetLinearLowerLimit(conf), v * Meter);
        }
    }
    {
        auto v = IsMotorEnabled(conf);
        if (ImGui::Checkbox("Enable Motor", &v))
        {
            EnableMotor(conf, v);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMotorSpeed(conf) / DegreePerSecond});
        if (ImGui::InputFloat("Motor Speed (°/sec)", &v))
        {
            SetMotorSpeed(conf, v * DegreePerSecond);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMaxMotorForce(conf) / Newton});
        if (ImGui::InputFloat("Max. Motor Force (N)", &v))
        {
            SetMaxMotorForce(conf, v * Newton);
        }
    }
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(DistanceJointConf& conf, BodyCounter bodyRange)
{
    // All settings implemented here...
    {
        auto v = static_cast<float>(Real{GetLength(conf) / Meter});
        if (ImGui::InputFloat("Length (m)", &v))
        {
            SetLength(conf, v * Meter);
        }
    }
    {
        auto v = static_cast<float>(Real{GetFrequency(conf) / Hertz});
        if (ImGui::InputFloat("Frequency (Hz)", &v))
        {
            SetFrequency(conf, v * Hertz);
        }
    }
    {
        auto v = static_cast<float>(GetDampingRatio(conf));
        if (ImGui::InputFloat("Damping Ratio", &v))
        {
            SetDampingRatio(conf, static_cast<Real>(v));
        }
    }
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(PulleyJointConf& conf, BodyCounter bodyRange)
{
    ImGui::LabelText("Length A (m)", "%f", static_cast<double>(Real{GetLengthA(conf)/Meter}));
    ImGui::LabelText("Length B (m)", "%f", static_cast<double>(Real{GetLengthB(conf)/Meter}));
    {
        auto v = static_cast<float>(GetRatio(conf));
        if (ImGui::InputFloat("Ratio", &v)) {
            SetRatio(conf, static_cast<Real>(v));
        }
    }
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(TargetJointConf& conf, BodyCounter bodyRange)
{
    {
        const auto target = GetTarget(conf);
        auto x = static_cast<float>(Real{GetX(target) / Meter});
        auto y = static_cast<float>(Real{GetY(target) / Meter});
        if (ImGui::InputFloat("Target X (m)", &x))
        {
            SetTarget(conf, Length2{x * Meter, y * Meter});
        }
        if (ImGui::InputFloat("Target Y (m)", &y))
        {
            SetTarget(conf, Length2{x * Meter, y * Meter});
        }
    }
    {
        auto v = static_cast<float>(Real{GetMaxForce(conf) / Newton});
        if (ImGui::InputFloat("Max Force (N)", &v))
        {
            SetMaxForce(conf, v * Newton);
        }
    }
    {
        auto v = static_cast<float>(Real{GetFrequency(conf) / Hertz});
        if (ImGui::InputFloat("Frequency (Hz)", &v))
        {
            SetFrequency(conf, v * Hertz);
        }
    }
    {
        auto v = static_cast<float>(Real{GetDampingRatio(conf)});
        if (ImGui::InputFloat("Damping Ratio", &v))
        {
            SetDampingRatio(conf, static_cast<Real>(v));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(GearJointConf& conf, BodyCounter bodyRange)
{
    ImGui::LabelText("Constant", "%.2e", static_cast<double>(GetConstant(conf)));
    {
        auto v = static_cast<float>(GetRatio(conf));
        if (ImGui::InputFloat("Ratio", &v))
        {
            SetRatio(conf, static_cast<Real>(v));
        }
    }
    const auto type1 = GetType1(conf);
    const auto type2 = GetType2(conf);
    ImGui::LabelText("Type 1", "%s", (type1 != GetTypeID<void>())? Test::ToName(type1): "unset");
    ImGui::LabelText("Type 2", "%s", (type2 != GetTypeID<void>())? Test::ToName(type2): "unset");
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(WheelJointConf& conf, BodyCounter bodyRange)
{
    {
        auto v = IsMotorEnabled(conf);
        if (ImGui::Checkbox("Enable Motor", &v))
        {
            EnableMotor(conf, v);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMotorSpeed(conf) / DegreePerSecond});
        if (ImGui::InputFloat("Motor Speed (°/sec)", &v))
        {
            SetMotorSpeed(conf, v * DegreePerSecond);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMaxMotorTorque(conf) / NewtonMeter});
        if (ImGui::InputFloat("Max Mot. Torq. (N·m)", &v))
        {
            SetMaxMotorTorque(conf, v * NewtonMeter);
        }
    }
    {
        auto v = static_cast<float>(Real{GetFrequency(conf) / Hertz});
        if (ImGui::InputFloat("Spring Freq. (Hz)", &v))
        {
            SetFrequency(conf, v * Hertz);
        }
    }
    {
        auto v = static_cast<float>(GetDampingRatio(conf));
        if (ImGui::InputFloat("Spring Damp. Ratio", &v))
        {
            SetDampingRatio(conf, static_cast<Real>(v));
        }
    }
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(WeldJointConf& conf, BodyCounter bodyRange)
{
    ImGui::LabelText("Ref. Angle (°)", "%.1e",
                     static_cast<double>(Real{GetReferenceAngle(conf) / Degree}));
    {
        auto v = static_cast<float>(Real{GetFrequency(conf) / Hertz});
        if (ImGui::InputFloat("Frequency (Hz)", &v))
        {
            SetFrequency(conf, v * Hertz);
        }
    }
    {
        auto v = static_cast<float>(GetDampingRatio(conf));
        if (ImGui::InputFloat("Damping Ratio", &v))
        {
            SetDampingRatio(conf, static_cast<Real>(v));
        }
    }
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(FrictionJointConf& conf, BodyCounter bodyRange)
{
    {
        auto v = static_cast<float>(Real{GetMaxForce(conf) / Newton});
        if (ImGui::InputFloat("Max Force (N)", &v))
        {
            SetMaxForce(conf, v * Newton);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMaxTorque(conf) / NewtonMeter});
        if (ImGui::InputFloat("Max Torq. (N·m)", &v))
        {
            SetMaxTorque(conf, v * NewtonMeter);
        }
    }
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(RopeJointConf& conf, BodyCounter bodyRange)
{
    ImGui::LabelText("Limit State", "%s", ToString(GetLimitState(conf)));
    {
        auto v = static_cast<float>(Real{GetMaxLength(conf) / Meter});
        if (ImGui::InputFloat("Max. Length (m)", &v))
        {
            SetMaxLength(conf, v * Meter);
        }
    }
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static void EntityUI(MotorJointConf& conf, BodyCounter bodyRange)
{
    {
        const auto linearError = GetLinearError(conf);
        ImGui::LabelText("Lin. Error X (m)", "%.2e",
                         static_cast<double>(Real{GetX(linearError) / Meter}));
        ImGui::LabelText("Lin. Error Y (m)", "%.2e",
                         static_cast<double>(Real{GetY(linearError) / Meter}));
    }
    ImGui::LabelText("Ang. Error (°)", "%.2e",
                     static_cast<double>(Real{GetAngularError(conf) / Degree}));
    {
        const auto linOff = GetLinearOffset(conf);
        auto x = static_cast<float>(Real{GetX(linOff) / Meter});
        auto y = static_cast<float>(Real{GetY(linOff) / Meter});
        if (ImGui::InputFloat("Lin. Offset X (m)", &x))
        {
            SetLinearOffset(conf, Length2{x * Meter, y * Meter});
        }
        if (ImGui::InputFloat("Lin. Offset Y (m)", &y))
        {
            SetLinearOffset(conf, Length2{x * Meter, y * Meter});
        }
    }
    {
        auto v = static_cast<float>(Real{GetAngularOffset(conf) / Degree});
        if (ImGui::InputFloat("Ang. Offset (°)", &v, 0, 0, "%.2f"))
        {
            SetAngularOffset(conf, v * Degree);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMaxForce(conf) / Newton});
        if (ImGui::InputFloat("Max Force (N)", &v))
        {
            SetMaxForce(conf, v * Newton);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMaxTorque(conf) / NewtonMeter});
        if (ImGui::InputFloat("Max Torq. (N·m)", &v))
        {
            SetMaxTorque(conf, v * NewtonMeter);
        }
    }
    {
        auto v = static_cast<float>(GetCorrectionFactor(conf));
        if (ImGui::InputFloat("Correction Factor", &v))
        {
            SetCorrectionFactor(conf, static_cast<Real>(v));
        }
    }
    {
        auto bodyA = static_cast<int>(to_underlying(GetBodyA(conf)));
        ImGui::SliderInt("ID of Body A", &bodyA, 0, int(bodyRange) - 1);
        if (bodyA >= 0 && bodyA < static_cast<int>(bodyRange)) {
            conf.bodyA = BodyID(static_cast<BodyID::underlying_type>(bodyA));
        }
    }
    {
        auto bodyB = static_cast<int>(to_underlying(GetBodyB(conf)));
        ImGui::SliderInt("ID of Body B", &bodyB, 0, int(bodyRange) - 1);
        if (bodyB >= 0 && bodyB < static_cast<int>(bodyRange)) {
            conf.bodyB = BodyID(static_cast<BodyID::underlying_type>(bodyB));
        }
    }
}

static bool ChangeType(Joint& joint, TypeID newType)
{
    const auto oldType = GetType(joint);
    if (oldType == newType) {
        return true;
    }
    if (newType == GetTypeID<DistanceJointConf>()) {
        auto conf = DistanceJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<FrictionJointConf>()) {
        auto conf = FrictionJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<GearJointConf>()) {
        auto conf = GearJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<MotorJointConf>()) {
        auto conf = MotorJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<PrismaticJointConf>()) {
        auto conf = PrismaticJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<PulleyJointConf>()) {
        auto conf = PulleyJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<RevoluteJointConf>()) {
        auto conf = RevoluteJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<RopeJointConf>()) {
        auto conf = RopeJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<TargetJointConf>()) {
        auto conf = TargetJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<WheelJointConf>()) {
        auto conf = WheelJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    if (newType == GetTypeID<WeldJointConf>()) {
        auto conf = WeldJointConf{};
        conf.bodyA = GetBodyA(joint);
        conf.bodyB = GetBodyB(joint);
        joint = conf;
        return true;
    }
    return false;
}

static void ChangeTypeUI(Joint& joint)
{
    const auto type = GetType(joint);
    auto itemCurrentIdx = static_cast<std::ptrdiff_t>(size(Test::jointTypeToNameMap));
    const auto found = Test::jointTypeToNameMap.find(type);
    if (found != end(Test::jointTypeToNameMap)) {
        itemCurrentIdx = std::distance(begin(Test::jointTypeToNameMap), found);
    }
    const auto typeName = Test::ToName(type);
    static auto flags = ImGuiComboFlags(0);
    if (ImGui::BeginCombo("##JointTypeSelectionCombo", typeName, flags)) {
        const auto first = begin(Test::jointTypeToNameMap);
        const auto last = end(Test::jointTypeToNameMap);
        auto pos = static_cast<std::ptrdiff_t>(0);
        for (auto iter = first; iter != last; ++iter) {
            const bool isSelected = (itemCurrentIdx == pos);
            const auto label = iter->second;
            if (ImGui::Selectable(label, isSelected)) {
                itemCurrentIdx = pos;
                ChangeType(joint, iter->first);
            }
            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (isSelected)
                ImGui::SetItemDefaultFocus();
            ++pos;
        }
        ImGui::EndCombo();
    }
}

static void EntityUI(Joint& joint, BodyCounter bodyRange)
{
    ChangeTypeUI(joint);

    ImGui::ItemWidthContext itemWidthCtx(100);
    ImGui::LabelText("Collide Connected", "%s", GetCollideConnected(joint)? "true": "false");
    {
        const auto linReact = GetLinearReaction(joint);
        ImGui::LabelText("Lin. Reaction X (N·s)", "%.2e",
                         static_cast<double>(Real{GetX(linReact) / NewtonSecond}));
        ImGui::LabelText("Lin. Reaction Y (N·s)", "%.2e",
                         static_cast<double>(Real{GetY(linReact) / NewtonSecond}));
    }
    ImGui::LabelText("Ang. Reaction (N·m·s)", "%.2e",
                     static_cast<double>(Real{GetAngularReaction(joint) / NewtonMeterSecond}));

    const auto type = GetType(joint);
    if (type == GetTypeID<DistanceJointConf>()) {
        auto conf = TypeCast<DistanceJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    else if (type == GetTypeID<FrictionJointConf>()) {
        auto conf = TypeCast<FrictionJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    else if (type == GetTypeID<GearJointConf>()) {
        auto conf = TypeCast<GearJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    else if (type == GetTypeID<MotorJointConf>()) {
        auto conf = TypeCast<MotorJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    else if (type == GetTypeID<PrismaticJointConf>()) {
        auto conf = TypeCast<PrismaticJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    else if (type == GetTypeID<PulleyJointConf>()) {
        auto conf = TypeCast<PulleyJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    if (type == GetTypeID<RevoluteJointConf>()) {
        auto conf = TypeCast<RevoluteJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    if (type == GetTypeID<RopeJointConf>()) {
        auto conf = TypeCast<RopeJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    else if (type == GetTypeID<TargetJointConf>()) {
        auto conf = TypeCast<TargetJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    else if (type == GetTypeID<WheelJointConf>()) {
        auto conf = TypeCast<WheelJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
    else if (type == GetTypeID<WeldJointConf>()) {
        auto conf = TypeCast<WeldJointConf>(joint);
        EntityUI(conf, bodyRange);
        joint = conf;
    }
}

static void EntityUI(World& world, JointID e)
{
    ImGui::IdContext idCtx(static_cast<int>(e.get()));
    const auto bodyRange = GetBodyRange(world);
    auto joint = GetJoint(world, e);
    EntityUI(joint, bodyRange);
    if (GetJoint(world, e) != joint) {
        SetJoint(world, e, joint);
    }
    if (ImGui::Button("Destroy", ImVec2(-1, 0))) {
        Destroy(world, e);
    }
}

static void EntityUI(Contact& contact)
{
    ImGui::Columns(4, "BodyShapeChildColumns", false);
    ImGui::SetColumnWidth(0, 30);
    ImGui::SetColumnWidth(1, 40);
    ImGui::SetColumnWidth(2, 40);
    ImGui::SetColumnWidth(3, 40);
    ImGui::TextUnformatted("Side");
    ImGui::NextColumn();
    ImGui::TextUnformatted("Body");
    ImGui::NextColumn();
    ImGui::TextUnformatted("Shape");
    ImGui::NextColumn();
    ImGui::TextUnformatted("Child");
    ImGui::NextColumn();
    ImGui::TextUnformatted("A");
    ImGui::NextColumn();
    ImGui::Text("%u", to_underlying(GetBodyA(contact)));
    ImGui::NextColumn();
    ImGui::Text("%u", to_underlying(GetShapeA(contact)));
    ImGui::NextColumn();
    ImGui::Text("%u", GetChildIndexA(contact));
    ImGui::NextColumn();
    ImGui::TextUnformatted("B");
    ImGui::NextColumn();
    ImGui::Text("%u", to_underlying(GetBodyB(contact)));
    ImGui::NextColumn();
    ImGui::Text("%u", to_underlying(GetShapeB(contact)));
    ImGui::NextColumn();
    ImGui::Text("%u", GetChildIndexB(contact));
    ImGui::NextColumn();
    ImGui::Columns(1);
    {
        auto v = IsEnabled(contact);
        if (ImGui::Checkbox("Enabled", &v)) {
            if (v) {
                SetEnabled(contact);
            }
            else {
                UnsetEnabled(contact);
            }
        }
    }
    {
        auto v = IsImpenetrable(contact);
        if (ImGui::Checkbox("Impenetrable", &v)) {
            if (v) {
                SetImpenetrable(contact);
            }
            else {
                UnsetImpenetrable(contact);
            }
        }
    }
    {
        auto v = IsActive(contact);
        if (ImGui::Checkbox("Active", &v)) {
            if (v) {
                SetIsActive(contact);
            }
            else {
                UnsetIsActive(contact);
            }
        }
    }
    {
        auto v = IsSensor(contact);
        if (ImGui::Checkbox("Sensor", &v)) {
            if (v) {
                SetSensor(contact);
            }
            else {
                UnsetIsSensor(contact);
            }
        }
    }
    {
        auto v = NeedsFiltering(contact);
        if (ImGui::Checkbox("Needs filtering", &v)) {
            if (v) {
                FlagForFiltering(contact);
            }
            else {
                UnflagForFiltering(contact);
            }
        }
    }
    {
        auto v = NeedsUpdating(contact);
        if (ImGui::Checkbox("Needs updating", &v)) {
            if (v) {
                FlagForUpdating(contact);
            }
            else {
                UnflagForUpdating(contact);
            }
        }
    }
    {
        auto val = static_cast<float>(GetRestitution(contact));
        if (ImGui::InputFloat("Restitution", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetRestitution(contact, val);
        }
    }
    {
        auto val = static_cast<float>(GetFriction(contact));
        if (ImGui::InputFloat("Friction", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetFriction(contact, val);
        }
    }
    {
        auto val = static_cast<float>(Real{GetTangentSpeed(contact) / MeterPerSecond});
        if (ImGui::InputFloat("Belt Speed", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetTangentSpeed(contact, val * MeterPerSecond);
        }
    }
    {
        auto v = HasValidToi(contact)? GetToi(contact): std::numeric_limits<Real>::quiet_NaN();
        if (ImGui::InputFloat("TOI", &v, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            if (v >= 0 && v <= 1) {
                SetToi(contact, v);
            }
            else {
                UnsetToi(contact);
            }
        }
    }
    {
        auto val = static_cast<int>(GetToiCount(contact));
        if (ImGui::InputInt("TOI Count", &val)) {
            SetToiCount(contact, static_cast<Contact::substep_type>(val));
        }
    }
}

static void EntityUI(World& world, ContactID contactId)
{
    ImGui::IdContext idCtx(static_cast<int>(to_underlying(contactId)));
    ImGui::ItemWidthContext itemWidthCtx(50); // 50

    auto contact = GetContact(world, contactId);
    EntityUI(contact);
    if (GetContact(world, contactId) != contact) {
        try {
            SetContact(world, contactId, contact);
        }
        catch (const std::invalid_argument& ex) {
            ui->message = std::string("Invalid setting: ") + ex.what();
        }
        catch (const std::out_of_range& ex) {
            ui->message = std::string("Out of range: ") + ex.what();
        }
    }
    if (IsTouching(world, contactId)) {
        EntityUI(GetManifold(world, contactId));
    }
}

static void AddBodyUI(World& world)
{
    const auto button_sz = ImVec2(-1, 0);
    if (ImGui::Button("Create", button_sz)) {
        const auto id = CreateBody(world);
        auto fixtures = FixtureSet{};
        fixtures.insert(std::make_pair(id, InvalidShapeID));
        g_testSuite->GetTest()->SetSelectedFixtures(fixtures);
        const auto saved = g_testSuite->GetTest()->GetSelectedBodies();
        saved.count(id);
    }
}

static void CollectionUI(World& world, const World::Bodies& bodies,
                         const BodySet& selectedBodies, const FixtureSet& selectedFixtures)
{
    ImGui::IdContext idCtx("Bodies");
    AddBodyUI(world);
    for (const auto& e: bodies) {
        const auto typeName = ToString(GetType(world, e));
        const auto flags = IsWithin(selectedBodies, e)? ImGuiTreeNodeFlags_DefaultOpen: 0;
        if (ImGui::TreeNodeEx(reinterpret_cast<const void*>(e.get()), flags,
                              "Body %u (%s)", e.get(), typeName)) {
            EntityUI(world, e, selectedFixtures);
            ImGui::TreePop();
        }
    }
}

static void CollectionUI(World& world, const World::Joints& joints)
{
    ImGui::IdContext idCtx("Joints");
    for (const auto& jointID: joints) {
        ImGui::IdContext ctx(to_underlying(jointID));
        const auto flags = 0;
        if (ImGui::TreeNodeEx(reinterpret_cast<const void*>(jointID.get()), flags,
                              "Joint %u (%s)",
                              jointID.get(), Test::ToName(GetType(world, jointID)))) {
            EntityUI(world, jointID);
            ImGui::TreePop();
        }
    }
}

static void CollectionUI(World& world, const World::BodyJoints& joints)
{
    ImGui::IdContext idCtx("BodyJointsRange");
    ImGui::ItemWidthContext itemWidthCtx(130);
    for (const auto& e: joints) {
        const auto bodyID = std::get<BodyID>(e);
        const auto jointID = std::get<JointID>(e);
        if (bodyID != InvalidBodyID) {
            ImGui::Text("Joint %u (%s Other-body=%u)", to_underlying(jointID),
                        Test::ToName(GetType(world, jointID)), to_underlying(bodyID));
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("World ID of joint and world ID of other associated body.");
            }
        }
        else {
            ImGui::Text("Joint %u", to_underlying(jointID));
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("World ID of joint.");
            }
        }
    }
}

static void CollectionUI(World& world, const World::Contacts& contacts, bool interactive)
{
    ImGui::IdContext idCtx("ContactsRange");
    if (interactive) {
        for (const auto& ct: contacts) {
            const auto key = std::get<ContactKey>(ct);
            const auto contactID = std::get<ContactID>(ct);
            if (ImGui::TreeNodeEx(reinterpret_cast<const void*>(contactID.get()), 0,
                                  "Contact %u (%u,%u%s)", contactID.get(),
                                  key.GetMin(), key.GetMax(),
                                  (IsTouching(world, contactID)? ",touching": ""))) {
                EntityUI(world, contactID);
                ImGui::TreePop();
            }
        }
    }
    else {
        for (const auto& ct: contacts) {
            const auto key = std::get<ContactKey>(ct);
            const auto contactID = std::get<ContactID>(ct);
            ImGui::Text("Contact %u (%u,%u%s)", contactID.get(), key.GetMin(), key.GetMax(),
                        IsTouching(world, contactID)? ",touching": "");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("World ID of contact (and associated min & max tree keys).");
            }
        }
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
    const auto selShapes = false;

    ImGui::PushStyleVar(ImGuiStyleVar_IndentSpacing, ImGui::GetFontSize()*1);
    {
        if (ImGui::TreeNodeEx("Shapes", selShapes? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Shapes (%hu)", test->GetWorld().GetShapeRange())) {
            ShapesUI(test->GetWorld());
            ImGui::TreePop();
        }
    }
    {
        const auto bodies = GetBodies(test->GetWorld());
        if (ImGui::TreeNodeEx("Bodies", selBodies? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Bodies (%lu)", size(bodies))) {
            CollectionUI(test->GetWorld(), bodies, selectedBodies, selectedFixtures);
            ImGui::TreePop();
        }
    }
    {
        const auto joints = GetJoints(test->GetWorld());
        if (ImGui::TreeNodeEx("Joints", selJoints? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Joints (%lu)", size(joints))) {
            CollectionUI(test->GetWorld(), joints);
            ImGui::TreePop();
        }
    }
    {
        const auto contacts = GetContacts(test->GetWorld());
        if (ImGui::TreeNodeEx("Contacts", selContacts? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Contacts (%lu)", size(contacts))) {
            CollectionUI(test->GetWorld(), contacts);
            ImGui::TreePop();
        }
    }
    ImGui::PopStyleVar();
}

static bool UserInterface()
{
    auto shouldQuit = false;
    
    if (ui->showAboutTest) {
        // Note: Use ImGuiCond_Appearing to set the position on first appearance of Test
        //   About info and allow later relocation by user. This is preferred over using
        //   another condition like ImGuiCond_Once, since user could move this window out
        //   of viewport and otherwise having no visual way to recover it.
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Appearing);
        ImGui::SetNextWindowSize(ImVec2(261, 136), ImGuiCond_Once);
        
        // Note: without ImGuiWindowFlags_AlwaysAutoResize, ImGui adds a handle icon
        //   which allows manual resizing but stops automatic resizing.
        ImGui::WindowContext window("About This Test", &ui->showAboutTest,
                                    ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize);
        AboutTestUI();
    }

    if (ui->showMenu) {
        ImGui::SetNextWindowPos(ImVec2(float(g_camera.m_width - menuWidth - 10), 10));
        ImGui::SetNextWindowSize(ImVec2(float(menuWidth), float(g_camera.m_height - 20)));
        ImGui::WindowContext window("Testbed Controls", &ui->showMenu,
                                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse);
        shouldQuit = MenuUI();
    }
    
    if (ui->showEntities) {
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(240, 700), ImGuiCond_FirstUseEver);
        ImGui::WindowContext window("Entity Editor", &ui->showEntities,
                                    ImGuiWindowFlags_HorizontalScrollbar|ImGuiWindowFlags_NoCollapse);
        ModelEntitiesUI();
    }

    if (!ui->message.empty()) {
        if (Test::AlertUser("Alert!", "Operation rejected.\n\nReason: %s.\n\n",
                            ui->message.c_str())) {
            ui->message = std::string{};
        }
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

static std::string InitializeOpenglLoader()
{
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
    return std::string(gl3wInit()? "gl3wInit": "");
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
    return [](){
        const auto result = glewInit();
        return (result != GLEW_OK)? std::string(reinterpret_cast<const char*>(glewGetErrorString(result))): std::string{};
    }();
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
    return std::string(gladLoadGL() == 0? "gladLoadGL": "");
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD2)
    // glad2 recommend using the windowing library loader instead of the (optionally) bundled one.
    return std::string(gladLoadGL(glfwGetProcAddress) == 0? "gladLoadGL(glfwGetProcAddress)": "");
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING2)
    glbinding::Binding::initialize();
    return std::string{};
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING3)
    glbinding::initialize([](const char* name) { return (glbinding::ProcAddress)glfwGetProcAddress(name); });
    return std::string{};
#else
    // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is likely to requires some form of initialization.
    return std::string{};
#endif
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
        std::fprintf(stderr,
                     "Warning: overriding previously installed GLFW error callback function.\n");
    }

    if (glfwInit() == 0)
    {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return EXIT_FAILURE;
    }

    const auto buildVersion = GetVersion();
    const auto buildDetails = GetBuildDetails();
    
    char title[64];
    std::sprintf(title, "PlayRho Testbed Version %d.%d.%d",
                 buildVersion.major, buildVersion.minor, buildVersion.revision);

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    const auto mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, title,
                                             nullptr, nullptr);
    if (!mainWindow) {
        std::fprintf(stderr, "Failed to open GLFW main window.\n");
        glfwTerminate();
        return EXIT_FAILURE;
    }

    UiState userInterface;
    ::ui = &userInterface;

    glfwMakeContextCurrent(mainWindow);
    glfwSwapInterval(1); // Control the frame rate. One draw per monitor refresh.

    const auto monitor = glfwGetPrimaryMonitor();
    const auto vidmode = monitor? glfwGetVideoMode(monitor): static_cast<const GLFWvidmode*>(nullptr);
    refreshRate = vidmode? vidmode->refreshRate: decltype(vidmode->refreshRate){0};

    glfwSetScrollCallback(mainWindow, ScrollCallback);
    glfwSetWindowSizeCallback(mainWindow, ResizeWindow);
    glfwSetKeyCallback(mainWindow, KeyCallback);
    glfwSetMouseButtonCallback(mainWindow, MouseButton);
    glfwSetCursorPosCallback(mainWindow, MouseMotion);
    glfwSetScrollCallback(mainWindow, ScrollCallback);
    glfwSetCharCallback(mainWindow, ImGui_ImplGlfw_CharCallback);

    const auto err = InitializeOpenglLoader();
    if (!err.empty()) {
        std::fprintf(stderr, "OpenGL loader failed to initialize (%s)!\n", err.c_str());
        return EXIT_FAILURE;
    }

    std::printf("PlayRho %d.%d.%d (%s), OpenGL %s, GLSL %s\n",
                buildVersion.major, buildVersion.minor, buildVersion.revision, buildDetails.c_str(),
                glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));
    std::printf("Primary monitor refresh rate: %d Hz\n", refreshRate);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    const auto imguiContext = ImGui::CreateContext();
    CreateUI(mainWindow);
    
    auto time1 = glfwGetTime();
    auto frameTime = 0.0;
    auto fps = 0.0;

    glClearColor(0.3f, 0.3f, 0.3f, 1.f);
    {
        DebugDraw drawer(g_camera);
        while (!glfwWindowShouldClose(mainWindow))
        {
            glfwPollEvents();
            glViewport(0, 0, g_camera.m_width, g_camera.m_height);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDisable(GL_DEPTH_TEST);

            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

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

            ImGui::Render();

            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwMakeContextCurrent(mainWindow);
            glfwSwapBuffers(mainWindow);
        }
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext(imguiContext);

    glfwDestroyWindow(mainWindow);
    glfwTerminate();

    return 0;
}
