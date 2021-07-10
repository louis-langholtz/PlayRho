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

#include <cmath> // for std::nextafter

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

namespace {

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
auto shapeTranslateValue = Length2{};
auto shapeScaleValue = Vec2{Real(1), Real(1)};
auto shapeRotateValue = 0_deg;
constexpr char shapeTranslateButtonName[] = "Translate";
constexpr char shapeScaleButtonName[] = "Scale";
constexpr char shapeRotateButtonName[] = "Rotate";
constexpr char createShapeButtonName[] = "Create Shape";
constexpr char createBodyButtonName[] = "Create Body";
constexpr char createJointButtonName[] = "Create Joint";

auto GetMaxTranslation()
{
    return (neededSettings & (1u << Test::NeedMaxTranslation))?
        testSettings.maxTranslation: settings.maxTranslation;
}

auto GetDeltaTime()
{
    return (neededSettings & (1u << Test::NeedDeltaTime))? testSettings.dt: settings.dt;
}

} // namespace

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

static bool InputReal(const char* label, Real& var, Real step = 0, Real step_fast = 0,
                      const char* format = "%.3f", ImGuiInputTextFlags flags = 0)
{
    if constexpr (sizeof(Real) <= sizeof(float)) {
        auto val = static_cast<float>(var);
        if (ImGui::InputFloat(label, &val, step, step_fast, format, flags)) {
            var = static_cast<Real>(val);
            return true;
        }
    }
    else {
        auto val = static_cast<double>(var);
        if (ImGui::InputDouble(label, &val, step, step_fast, format, flags)) {
            var = static_cast<Real>(val);
            return true;
        }
    }
    return false;
}

static bool InputReals(const char* label, Vec2& var, const char* format = "%.3f",
                       ImGuiInputTextFlags flags = 0)
{
    if constexpr (sizeof(Real) <= sizeof(float)) {
        float vals[2] = {static_cast<float>(var[0]), static_cast<float>(var[1])};
        if (ImGui::InputFloat2(label, vals, format, flags)) {
            var = Vec2{static_cast<Real>(vals[0]), static_cast<Real>(vals[1])};
            return true;
        }
    }
    else {
        double vals[2] = {static_cast<double>(var[0]), static_cast<double>(var[1])};
        if (ImGui::InputDouble2(label, vals, format, flags)) {
            var = Vec2{static_cast<Real>(vals[0]), static_cast<Real>(vals[1])};
            return true;
        }
    }
    return false;
}

static void EntityUI(bool& variable, const char* title)
{
    ImGui::Checkbox(title, &variable);
}

template <class T>
static auto LengthUI(T& variable, const char* title, const char* fmt = "%.3f")
-> decltype(Real(variable / 1_m) == Real())
{
    auto value = Real{variable / 1_m};
    if (InputReal(title, value, 0, 0, fmt)) {
        variable = value * 1_m;
        return true;
    }
    return false;
}

static bool LengthUI(Length2& variable, const char* title, const char* fmt = "%f")
{
    auto vals = Vec2{Real{variable[0] / 1_m}, Real{variable[1] / 1_m}};
    if (InputReals(title, vals, fmt)) {
        variable = Length2{vals[0] * 1_m, vals[1] * 1_m};
        return true;
    }
    return false;
}

static bool AngleUI(Angle& variable, const char* title, const char* fmt = "%f")
{
    auto v = Real{variable / 1_deg};
    if (InputReal(title, v, 0, 0, fmt)) {
        variable = v * 1_deg;
        return true;
    }
    return false;
}

static bool MassUI(Mass& variable, const char* title, const char* fmt = "%f")
{
    auto v = Real{variable / 1_kg};
    if (InputReal(title, v, 0, 0, fmt)) {
        variable = v * 1_kg;
        return true;
    }
    return false;
}

static void EntityUI(UnitVec& variable, const char* title, const char* fmt = "%f")
{
    auto val = GetAngle(variable);
    if (AngleUI(val, title, fmt)) {
        variable = UnitVec::Get(val);
    }
}

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

static bool ChangeType(Shape& object, TypeID newType)
{
    const auto oldType = GetType(object);
    if (oldType == newType) {
        return true;
    }
    // Try to copy as much data from the old type as possible...
    if (newType == GetTypeID<ChainShapeConf>()) {
        auto conf = ChainShapeConf{};
        object = conf;
        return true;
    }
    if (newType == GetTypeID<DiskShapeConf>()) {
        auto conf = DiskShapeConf{};
        if (const auto count = GetChildCount(object); count > 0) {
            const auto proxy = GetChild(object, 0);
            conf.vertexRadius = proxy.GetVertexRadius();
        }
        object = conf;
        return true;
    }
    if (newType == GetTypeID<EdgeShapeConf>()) {
        auto conf = EdgeShapeConf{};
        if (const auto count = GetChildCount(object); count > 0) {
            const auto proxy = GetChild(object, 0);
            conf.vertexRadius = proxy.GetVertexRadius();
        }
        object = conf;
        return true;
    }
    if (newType == GetTypeID<MultiShapeConf>()) {
        auto conf = MultiShapeConf{};
        const auto count = GetChildCount(object);
        for (auto childIdx = static_cast<ChildCounter>(0); childIdx < count; ++childIdx) {
            const auto proxy = GetChild(object, childIdx);
            auto vertexSet = VertexSet{};
            const auto vertexCount = proxy.GetVertexCount();
            for (auto i = static_cast<decltype(vertexCount)>(0); i < vertexCount; ++i) {
                vertexSet.add(proxy.GetVertex(i));
            }
            conf.children.push_back(ConvexHull::Get(vertexSet));
        }
        object = conf;
        return true;
    }
    if (newType == GetTypeID<PolygonShapeConf>()) {
        auto conf = PolygonShapeConf{};
        if (const auto count = GetChildCount(object); count > 0) {
            const auto proxy = GetChild(object, 0);
            conf.vertexRadius = proxy.GetVertexRadius();
            conf.Set(Span<const Length2>{proxy.GetVertices().begin(), proxy.GetVertexCount()});
        }
        object = conf;
        return true;
    }
    return false;
}

static bool ChangeType(Body& object, BodyType newType)
{
    const auto oldType = GetType(object);
    if (oldType == newType) {
        return true;
    }
    SetType(object, newType);
    return false;
}

static bool ChangeType(GearJointConf::TypeData& object, std::size_t newType)
{
    switch (newType) {
    case 1u:
        object = GearJointConf::PrismaticData{};
        return true;
    case 2u:
        object = GearJointConf::RevoluteData{};
        return true;
    case 0u:
        break;
    }
    object = std::monostate{};
    return true;
}

static bool ChangeType(Joint& object, TypeID newType)
{
    const auto oldType = GetType(object);
    if (oldType == newType) {
        return true;
    }
    if (newType == GetTypeID<DistanceJointConf>()) {
        auto conf = DistanceJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<FrictionJointConf>()) {
        auto conf = FrictionJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<GearJointConf>()) {
        auto conf = GearJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<MotorJointConf>()) {
        auto conf = MotorJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<PrismaticJointConf>()) {
        auto conf = PrismaticJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<PulleyJointConf>()) {
        auto conf = PulleyJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<RevoluteJointConf>()) {
        auto conf = RevoluteJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<RopeJointConf>()) {
        auto conf = RopeJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<TargetJointConf>()) {
        auto conf = TargetJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<WheelJointConf>()) {
        auto conf = WheelJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    if (newType == GetTypeID<WeldJointConf>()) {
        auto conf = WeldJointConf{};
        conf.bodyA = GetBodyA(object);
        conf.bodyB = GetBodyB(object);
        object = conf;
        return true;
    }
    return false;
}

static void ChangeTypeUI(Shape& object)
{
    const auto type = GetType(object);
    auto itemCurrentIdx = static_cast<std::ptrdiff_t>(size(Test::shapeTypeToNameMap));
    const auto found = Test::shapeTypeToNameMap.find(type);
    if (found != end(Test::shapeTypeToNameMap)) {
        itemCurrentIdx = std::distance(begin(Test::shapeTypeToNameMap), found);
    }
    const auto typeName = Test::ToName(type);
    static auto flags = ImGuiComboFlags(0);
    if (ImGui::BeginCombo("##ShapeTypeSelectionCombo", typeName, flags)) {
        const auto first = begin(Test::shapeTypeToNameMap);
        const auto last = end(Test::shapeTypeToNameMap);
        auto pos = static_cast<std::ptrdiff_t>(0);
        for (auto iter = first; iter != last; ++iter) {
            const bool isSelected = (itemCurrentIdx == pos);
            const auto label = iter->second;
            if (ImGui::Selectable(label, isSelected)) {
                itemCurrentIdx = pos;
                ChangeType(object, iter->first);
            }
            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (isSelected)
                ImGui::SetItemDefaultFocus();
            ++pos;
        }
        ImGui::EndCombo();
    }
}

static void ChangeTypeUI(Body& object)
{
    const auto type = GetType(object);
    auto itemCurrentIdx = int(type);
    if (ImGui::BeginCombo("##BodyTypeSelectionCombo", Test::ToName(type), ImGuiComboFlags(0))) {
        for (auto i = 0; i < 3; ++i) {
            const auto bodyType = BodyType(i);
            const bool isSelected = (itemCurrentIdx == i);
            if (ImGui::Selectable(Test::ToName(bodyType), isSelected)) {
                itemCurrentIdx = i;
                ChangeType(object, bodyType);
            }
            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
}

static void ChangeTypeUI(Joint& object)
{
    const auto type = GetType(object);
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
                ChangeType(object, iter->first);
            }
            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (isSelected)
                ImGui::SetItemDefaultFocus();
            ++pos;
        }
        ImGui::EndCombo();
    }
}

static void ChangeTypeUI(GearJointConf::TypeData& object)
{
    static const char* names[] = {
        "unset", //
        Test::ToName(GetTypeID<PrismaticJointConf>()), //
        Test::ToName(GetTypeID<RevoluteJointConf>()),
    };
    auto itemCurrentIdx = object.index();
    if (ImGui::BeginCombo("##GearJointTypeSelectionCombo", names[itemCurrentIdx],
                          ImGuiComboFlags(0))) {
        for (auto i = std::size_t{0}; i < size(names); ++i) {
            const bool isSelected = (itemCurrentIdx == i);
            if (ImGui::Selectable(names[i], isSelected)) {
                itemCurrentIdx = i;
                ChangeType(object, i);
            }
            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
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
    if (neededSettings & (0x1u << Test::NeedDeltaTime)) {
        auto frequency = 1.0f / testSettings.dt;
        const auto max = 1.0f / testSettings.minDt;
        const auto min = 1.0f / testSettings.maxDt;
        const auto useSci = abs(frequency) < 1.0f;
        if (const auto ratio = max / min; ratio <= 100.0f) {
            ImGui::SliderFloat("Frequency", &frequency, min, max, (useSci? "%.2e Hz": "%.2f Hz"));
        }
        else {
            ImGui::InputFloat("Frequency", &frequency, 0.0f, 0.0f, (useSci? "%.2e Hz": "%.2f Hz"));
        }
        testSettings.dt = 1.0f / std::clamp(frequency, min, max);
    }
    else {
        auto frequency = 1.0f / settings.dt;
        const auto max = 1.0f / settings.minDt;
        const auto min = 1.0f / settings.maxDt;
        const auto useSci = abs(frequency) < 1.0f;
        if (const auto ratio = max / min; ratio <= 100.0f) {
            ImGui::SliderFloat("Frequency", &frequency, min, max, (useSci? "%.2e Hz": "%.2f Hz"));
        }
        else {
            ImGui::InputFloat("Frequency", &frequency, 0.0f, 0.0f, (useSci? "%.2e Hz": "%.2f Hz"));
        }
        settings.dt = 1.0f / std::clamp(frequency, min, max);
    }
    const auto dt = (neededSettings & (0x1u << Test::NeedDeltaTime))? testSettings.dt: settings.dt;
    if (ImGui::IsItemHovered()) {
        std::ostringstream os;
        os << "Simulating " << dt << " seconds every step.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    
    ImGui::SliderInt("Vel. Iter.", &settings.regVelocityIterations, 0, 100);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Maximum number of velocity iterations per step.");
    }
    
    ImGui::SliderInt("Pos. Iter.", &settings.regPositionIterations, 0, 100);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Maximum number of position iterations per step.");
    }
}

static void AdvancedStepOptionsUI()
{
    const auto defaultLinearSlop = static_cast<float>(Real{DefaultLinearSlop / Meter});

    if (neededSettings & (0x1u << Test::NeedDeltaTime)) {
        if (const auto ratio = testSettings.maxDt / testSettings.minDt; ratio <= 100.0f) {
            ImGui::SliderFloat("Sim Time", &testSettings.dt,
                               testSettings.minDt, testSettings.maxDt, "%.2e s");
        }
        else {
            ImGui::InputFloat("Sim Time", &testSettings.dt, 0.0f, 0.0f, "%.2e s");
        }
        testSettings.dt = std::clamp(testSettings.dt, testSettings.minDt, testSettings.maxDt);
    }
    else {
        if (const auto ratio = settings.maxDt / settings.minDt; ratio <= 100.0f) {
            ImGui::SliderFloat("Sim Time", &settings.dt,
                               settings.minDt, settings.maxDt, "%.2e s");
        }
        else {
            ImGui::InputFloat("Sim Time", &settings.dt, 0.0f, 0.0f, "%.2e s");
        }
        settings.dt = std::clamp(settings.dt, settings.minDt, settings.maxDt);
    }
    const auto dt = GetDeltaTime();
    if (ImGui::IsItemHovered()) {
        std::ostringstream os;
        os << "Simulating " << dt << " seconds every step.";
        os << " This is inversely tied to the frequency.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    
    if (neededSettings & (0x1u << Test::NeedMaxTranslation)) {
        ImGui::LabelText("Max Translation", "%.2e m", testSettings.maxTranslation);
    }
    else {
        auto value = settings.maxTranslation;
        if (ImGui::InputFloat("Max Translation", &value, 0.0f, 0.0f, "%.1f m",
                              ImGuiInputTextFlags_EnterReturnsTrue)) {
            settings.maxTranslation = std::max(value, 0.0f);
        }
    }
    if (ImGui::IsItemHovered()) {
        const auto maxTranslation = GetMaxTranslation();
        const auto maxLinearVelocity = maxTranslation / dt;
        std::ostringstream os;
        os << "Max. translation in meters allowed per step.";
        os << " This should be no less than zero.";
        os << " At its current setting and the current simulation time," \
              " this establishes a maximum linear velocity of ";
        os << maxLinearVelocity << " m/s.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    {
        auto value = settings.maxRotation;
        ImGui::SliderFloat("Max Rotation", &value, 0.0f, 179.0f, "%.1f °");
        settings.maxRotation = std::clamp(value, 0.0f, 179.0f);
    }
    if (ImGui::IsItemHovered()) {
        std::ostringstream os;
        const auto maxRotationalVelocity = settings.maxRotation / dt;
        os << "Max. rotation in degrees allowed per step.";
        os << " This should be no less than zero and less than 180°.";
        os << " At its current setting and the current simulation time, this establishes a max";
        os << " rotational velocity of " << maxRotationalVelocity << " °/s.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    
    const auto neededLinearSlop = !!(neededSettings & (0x1u << Test::NeedLinearSlopField));
    if (neededLinearSlop) {
        ImGui::LabelText("Linear Slop", "%.2e m", testSettings.linearSlop);
    }
    else {
        auto value = settings.linearSlop;
        if (ImGui::InputFloat("Linear Slop", &value, 0.0f, 0.0f, "%.2e m",
                              ImGuiInputTextFlags_EnterReturnsTrue)) {
            settings.linearSlop = std::max(value, 0.0f);
        }
    }
    const auto linearSlop = neededLinearSlop? testSettings.linearSlop: settings.linearSlop;
    const auto targetDepth = 3 * linearSlop;
    if (ImGui::IsItemHovered()) {
        std::ostringstream os;
        os << "The amount of \"slop\" to allow for in various length-related calculations.";
        os << " This should be greater than or equal to zero.";
        os << " Usually this should be below the visual threshold of scaling used in visualizing the simulation.";
        os << " The current setting results in a TOI-phase target depth of ";
        os << std::scientific << std::setprecision(2) << targetDepth << " m.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }

    {
        auto value = settings.angularSlop;
        ImGui::SliderFloat("Angular Slop", &value, 1.0f, 179.0f, "%.1f °");
        settings.angularSlop = std::clamp(value, 1.0f, 179.0f);
    }
    if (ImGui::IsItemHovered()) {
        std::ostringstream os;
        os << "The amount of \"slop\" to allow for in various angle-related calculations.";
        os << " This should be greater than zero and less than 180°.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }

    {
        auto value = settings.maxLinearCorrection;
        if (ImGui::InputFloat("Max Lin Correct", &value, 0.0f, 0.0f, "%.2f m")) {
            settings.maxLinearCorrection = std::max(value, settings.linearSlop);
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Maximum linear correction. Should be greater than the linear slop value.",
                           tooltipWrapWidth);
    }

    {
        auto value = settings.maxAngularCorrection;
        ImGui::SliderFloat("Max Ang Correct", &value, 1.0f, 179.0f, "%.1f °");
        settings.maxAngularCorrection = std::clamp(value, 1.0f, 179.0f);
    }
    if (ImGui::IsItemHovered()) {
        std::ostringstream os;
        os << "Maximum angular correction in degrees.";
        os << " This should be greater than zero and significantly less than 180°.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }

    ImGui::InputFloat("AABB Exten.", &settings.aabbExtension, 0.0f, 0.0f, "%.1e m");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Linear amount to extend all Abstract Aligned Bounding Boxes (AABBs) by.");
    }
    
    if (ImGui::CollapsingHeader("Reg-Phase Processing", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::IdContext idContext{"Reg-Phase Processing"};
        ImGui::SliderInt("Vel Iters", &settings.regVelocityIterations, 0, 100);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Maximum number of regular-phase velocity iterations per step.");
        }
        ImGui::SliderInt("Pos Iters", &settings.regPositionIterations, 0, 100);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Maximum number of regular-phase position iterations per step.");
        }
        {
            auto value = -settings.regMinSeparation;
            ImGui::SliderFloat("Max Overlap", &value, 0.0f, 5 * defaultLinearSlop);
            settings.regMinSeparation = -value;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Maximum allowable overlap between interacting bodies for regular "
                               "phase position resolution to be considered good enough for ending "
                               "further iterations. This should be greater than zero. The more it "
                               "is, the more noticable overlap may be but the faster steps will be."
                               " This is the negative of the \"regMinSeparation\" step setting.",
                               tooltipWrapWidth);
        }
        ImGui::SliderInt("Resol Rate", &settings.regPosResRate, 0, 100, "%.0f %%");
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("This is the % of overlap that will"
                               " be resolved per position iteration.", tooltipWrapWidth);
        }
        ImGui::Checkbox("Allow Sleeping", &settings.enableSleep);
        ImGui::InputFloat("Still To Sleep", &settings.minStillTimeToSleep);
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("The min. time in seconds (in simulated time) that a body"
                               " must be still for before it will be put to sleep.",
                               tooltipWrapWidth);
        }
        ImGui::Checkbox("Warm Starting", &settings.enableWarmStarting);
    }
    if (ImGui::CollapsingHeader("TOI-Phase Processing", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::IdContext idContext{"TOI-Phase Processing"};
        ImGui::Checkbox("Perform Continuous", &settings.enableContinuous);
        ImGui::SliderInt("Vel Iters", &settings.toiVelocityIterations, 0, 100);
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Maximum number of TOI-phase velocity iterations per step.",
                               tooltipWrapWidth);
        }
        ImGui::SliderInt("Pos Iters", &settings.toiPositionIterations, 0, 100);
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Maximum number of TOI-phase position iterations per step.",
                               tooltipWrapWidth);
        }
        settings.tolerance = std::min(settings.tolerance, targetDepth);
        ImGui::SliderFloat("Tolerance", &settings.tolerance, 0.0f, targetDepth, "%.2e m");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("+/- Tolerance from target depth. This must not exceed the target "
                              "depth (which currently is %.2e m)", targetDepth);
        }
        {
            auto value = -settings.toiMinSeparation;
            ImGui::SliderFloat("Max Overlap", &value, 0.0f, 5 * defaultLinearSlop);
            settings.toiMinSeparation = -value;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Maximum allowable overlap between interacting bodies for TOI "
                               "phase position resolution to be considered good enough for ending "
                               "further iterations. This should be greater than zero. The more it "
                               "is, the more noticable overlap may be but the faster steps will be."
                               "This is the negative of the \"toiMinSeparation\" step setting.",
                               tooltipWrapWidth);
        }
        ImGui::SliderInt("Resol Rate", &settings.toiPosResRate, 0, 100, "%.0f %%");
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("This is the % of overlap that will"
                               " be resolved per position iteration.", tooltipWrapWidth);
        }
        ImGui::SliderInt("Max Sub Steps", &settings.maxSubSteps, 0, 200);
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Max # of of sub steps that should be tried in resolving"
                               " collisions at a particular time of impact.", tooltipWrapWidth);
        }
        ImGui::SliderInt("Max Root Iters", &settings.maxToiRootIters, 0, 200);
        if (ImGui::IsItemHovered()) {
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
    const auto bsoOpen = ImGui::CollapsingHeader("Basic Step Options",
                                                 ImGuiTreeNodeFlags_DefaultOpen);
    if (ImGui::IsItemHovered()) {
        std::ostringstream os;
        os << "These are basic per-\"step\" options. ";
        os << "One step of the simulation is performed for every display refresh.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    if (bsoOpen) {
        BasicStepOptionsUI();
    }
    const auto asoOpen = ImGui::CollapsingHeader("Advanced Step Options");
    if (ImGui::IsItemHovered()) {
        std::ostringstream os;
        os << "These are advanced per-\"step\" options. ";
        os << "One step of the simulation is performed for every display refresh.";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    if (asoOpen) {
        AdvancedStepOptionsUI();
    }
    ImGui::PopItemWidth();

    const auto outputOptsOpen = ImGui::CollapsingHeader("Output Options",
                                                        ImGuiTreeNodeFlags_DefaultOpen);
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Options for changing what's drawn in the main window.",
                           tooltipWrapWidth);
    }
    if (outputOptsOpen) {
        OutputOptionsUI();
    }

    const auto windowOptsOpen = ImGui::CollapsingHeader("Windows", ImGuiTreeNodeFlags_DefaultOpen);
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Options for opening or closing additional windows.", tooltipWrapWidth);
    }
    if (windowOptsOpen) {
        ImGui::Checkbox("About Test", &ui->showAboutTest);
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Shows a window that provides information about the current test.",
                               tooltipWrapWidth);
        }
        ImGui::Checkbox("Step Statistics", &ui->showStats);
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Shows a window that provides statistics for the current test.",
                               tooltipWrapWidth);
        }
        ImGui::Checkbox("Entity Editor", &ui->showEntities);
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Shows a window for creating, reading, updating, and destroying "
                               "PlayRho entities for the current test simulation.",
                               tooltipWrapWidth);
        }
    }
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Checkbox("Pause", &settings.pause);
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("\"Pauses\" the simulation by overriding the simulation time per step"
                           " with a value of zero until un-paused. This can also be toggled by"
                           " pressing the 'P' key. Press the Single Step button when paused, to "
                           " execute a single step.", tooltipWrapWidth);
    }
    
    if (ImGui::Button("Single Step", button_sz)) {
        settings.singleStep = !settings.singleStep;
    }
    if (ImGui::IsItemHovered()) {
        std::ostringstream os;
        os << "Executes a single step of the current test when the simulation is otherwise paused";
        if (settings.pause) {
            os << " - like it is now";
        }
        os << ".";
        ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
    }
    if (ImGui::Button("Restart", button_sz)) {
        g_testSuite->RestartTest();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Restarts the current test. This can also be invoked by pressing the 'R' key.",
                           tooltipWrapWidth);
    }
    if (ImGui::Button("Quit", button_sz)) {
        shouldQuit = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Quits the application. This can also be invoked by pressing the 'ESC' key.",
                           tooltipWrapWidth);
    }

    ImGui::PopAllowKeyboardFocus();
    
    return shouldQuit;
}

static void EntityUI(BodyID& id, BodyCounter bodyRange, const char* title)
{
    auto val = static_cast<int>(to_underlying(id));
    ImGui::SliderInt(title, &val, 0, int(bodyRange) - 1);
    if (val >= 0 && val < static_cast<int>(bodyRange)) {
        id = BodyID(static_cast<BodyID::underlying_type>(val));
    }
}

static bool EntityUI(Sweep& sweep)
{
    auto changed = false;
    ImGui::TextUnformatted("Sweep Info...");
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("The sweep represents current and end positions of the body if "
                           "unimpeded the remainder of the step, its local center of mass, and a "
                           "factor of how far along it's been moved so far during the step.",
                           tooltipWrapWidth);
    }
    constexpr auto totalWidth = 180.0f;
    constexpr auto colWidth = 52.0f;
    const auto angPosColWidths = std::initializer_list<float>{colWidth, colWidth};
    if (LengthUI(sweep.pos0.linear, "Lin. Pos. 0")) {
        changed = true;
    }
    {
        const auto normalized = GetNormalized(sweep.pos0.angular);
        ImGui::StyleVarContext itemSpacingCtx(ImGuiStyleVar_ItemSpacing, ImVec2(2,2));
        ImGui::StyleVarContext framePaddingCtx(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
        ImGui::ColumnsContext cc(3, "ColsAngPos0", false);
        ImGui::SetColumnWidths(totalWidth, angPosColWidths);
        {
            ImGui::ItemWidthContext itemWidthCtx(colWidth);
            if (AngleUI(sweep.pos0.angular, "##AngPos0")) {
                changed = true;
            }
        }
        ImGui::NextColumn();
        {
            ImGui::ItemWidthContext itemWidthCtx(colWidth);
            if (ImGui::Button("Norm.##0", ImVec2(-1, 0))) {
                sweep.pos0.angular = normalized;
                changed = true;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Press to change the angle to %.2f° and to normalize it to an "
                                  "angle between -180° and +180°.",
                                  static_cast<float>(Real(normalized/1_deg)));
            }
        }
        ImGui::NextColumn();
        ImGui::Text("Ang. Pos. 0");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Angular position 0 in degrees (%.2f° normalized).",
                              static_cast<float>(Real(normalized/1_deg)));
        }
        ImGui::NextColumn();
    }
    if (LengthUI(sweep.pos1.linear, "Lin. Pos. 1")) {
        changed = true;
    }
    {
        const auto normalized = GetNormalized(sweep.pos1.angular);
        ImGui::StyleVarContext itemSpacingCtx(ImGuiStyleVar_ItemSpacing, ImVec2(2, 2));
        ImGui::StyleVarContext framePaddingCtx(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
        ImGui::ColumnsContext cc(3, "ColsAngPos1", false);
        ImGui::SetColumnWidths(totalWidth, angPosColWidths);
        {
            ImGui::ItemWidthContext itemWidthCtx(colWidth);
            if (AngleUI(sweep.pos1.angular, "##AngPos1")) {
                changed = true;
            }
        }
        ImGui::NextColumn();
        {
            ImGui::ItemWidthContext itemWidthCtx(colWidth);
            if (ImGui::Button("Norm.##1", ImVec2(-1, 0))) {
                sweep.pos1.angular = normalized;
                changed = true;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Press to change the angle to %.2f° and to normalize it to an "
                                  "angle between -180° and +180°.",
                                  static_cast<float>(Real(normalized/1_deg)));
            }
        }
        ImGui::NextColumn();
        ImGui::Text("Ang. Pos. 1");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Angular position 1 in degrees (%.2f° normalized).",
                              static_cast<float>(Real(normalized/1_deg)));
        }
        ImGui::NextColumn();
    }
    {
        auto location = sweep.GetLocalCenter();
        if (LengthUI(location, "Loc. Mass Ctr.")) {
            SetLocalCenter(sweep, location);
            changed = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Local center of mass in meters.", tooltipWrapWidth);
        }
    }
    {
        ImGui::ColumnsContext cc(2, "ColsAlpha0", false);
        ImGui::SetColumnWidths(totalWidth, {colWidth * 2 + 4});
        {
            const auto min = 0.0f;
            const auto max = std::nextafter(1.0f, min);
            auto value = static_cast<float>(sweep.GetAlpha0());
            ImGui::ItemWidthContext itemWidthCtx(colWidth * 2 + 4);
            if (ImGui::SliderFloat("##Alpha0", &value, min, max, "%.2f")) {
                value = std::min(value, max);
                sweep.Advance0(Real(value));
                changed = true;
            }
        }
        ImGui::NextColumn();
        ImGui::Text("Alpha 0");
        ImGui::NextColumn();
    }
    return changed;
}

static bool AlmostGE(float value, float limit)
{
    return (value > limit) || AlmostEqual(value, limit);
}

static void EntityUI(Body& body)
{
    {
        const auto transformation = GetTransformation(body);
        auto location = GetLocation(transformation);
        if (LengthUI(location, "Lin. Transform")) {
            SetTransformation(body, Transformation{location, transformation.q});
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Linear transformation in meters.", tooltipWrapWidth);
        }
    }
    {
        const auto transformation = GetTransformation(body);
        auto angle = GetAngle(GetDirection(transformation));
        if (AngleUI(angle, "Ang. Transform", "%f")) {
            SetTransformation(body, Transformation{transformation.p, UnitVec::Get(angle)});
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Angular transformation in degrees. "
                               "This is the directional/normalized angle of the body.",
                               tooltipWrapWidth);
        }
    }
    {
        const auto dt = GetDeltaTime();
        const auto maxTranslation = GetMaxTranslation();
        const auto maxLinearVelocity = maxTranslation / dt;
        const auto maxAngularVelocity = settings.maxRotation / dt;
        const auto velocity = GetVelocity(body);
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(velocity.linear) / MeterPerSecond});
        vals[1] = static_cast<float>(Real{GetY(velocity.linear) / MeterPerSecond});
        const auto maxLinearHit = AlmostGE(abs(vals[0]), maxLinearVelocity) ||
                                  AlmostGE(abs(vals[1]), maxLinearVelocity);
        {
            const auto color = maxLinearHit?
                ImVec4(1.0f, 0.0f, 0.0f, 1.0f): ImGui::GetStyle().Colors[ImGuiCol_Text];
            ImGui::StyleColorContext colorContext(ImGuiCol_Text, color);
            if (ImGui::InputFloat2("Lin. Veloc.", vals, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                SetVelocity(body, LinearVelocity2{vals[0] * 1_mps, vals[1] * 1_mps});
            }
        }
        if (ImGui::IsItemHovered()) {
            std::ostringstream os;
            os << "Linear velocity in meters/second.";
            os << "The current maximum allowable linear velocity ";
            os << "(based on the per-step simulation time of ";
            os << dt << "s and the per-step max translation of " << maxTranslation;
            os << "m) is " << maxLinearVelocity << " m/s.";
            if (maxLinearHit) {
                os << " Linear velocity is currently capped by this max.";
            }
            ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
        }
        auto val = static_cast<float>(Real{velocity.angular / DegreePerSecond});
        const auto maxAngularHit = AlmostGE(abs(val), maxAngularVelocity);
        {
            const auto color = maxAngularHit?
                ImVec4(1.0f, 0.0f, 0.0f, 1.0f): ImGui::GetStyle().Colors[ImGuiCol_Text];
            ImGui::StyleColorContext colorContext(ImGuiCol_Text, color);
            if (ImGui::InputFloat("Ang. Veloc.", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                SetVelocity(body, val * DegreePerSecond);
            }
        }
        if (ImGui::IsItemHovered()) {
            std::ostringstream os;
            os << "Angular velocity in degrees/second.";
            os << "The current maximum allowable angular velocity ";
            os << "(based on the per-step simulation time of ";
            os << dt << "s and the per-step max rotation of " << settings.maxRotation;
            os << "m) is " << maxAngularVelocity << " °/s.";
            if (maxAngularHit) {
                os << " Angular velocity is currently capped by this max.";
            }
            ImGui::ShowTooltip(os.str(), tooltipWrapWidth);
        }
    }
    {
        const auto acceleration = GetAcceleration(body);
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(acceleration.linear) / MeterPerSquareSecond});
        vals[1] = static_cast<float>(Real{GetY(acceleration.linear) / MeterPerSquareSecond});
        if (ImGui::InputFloat2("Lin. Accel.", vals, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetAcceleration(body, LinearAcceleration2{vals[0] * 1_mps2, vals[1] * 1_mps2});
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Linear acceleration in meters/second².", tooltipWrapWidth);
        }
        auto val = static_cast<float>(Real{acceleration.angular / DegreePerSquareSecond});
        if (ImGui::InputFloat("Ang. Accel.", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            SetAcceleration(body, val * DegreePerSquareSecond);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::ShowTooltip("Angular acceleration in degrees/second².", tooltipWrapWidth);
        }
    }
    ImGui::Spacing();
    {
        auto sweep = GetSweep(body);
        if (EntityUI(sweep)) {
            SetSweep(body, sweep);
        }
    }
    ImGui::Spacing();
    {
        auto v = IsImpenetrable(body);
        if (ImGui::Checkbox("Bullet", &v)) {
            if (v)
                SetImpenetrable(body);
            else
                UnsetImpenetrable(body);
        }
    }
    ImGui::SameLine();
    {
        auto v = !IsFixedRotation(body);
        if (ImGui::Checkbox("Rotatable", &v)) {
            SetFixedRotation(body, !v);
        }
    }
    {
        auto v = IsSleepingAllowed(body);
        if (ImGui::Checkbox("Sleepable", &v)) {
            SetSleepingAllowed(body, v);
        }
    }
    ImGui::SameLine();
    {
        auto v = IsAwake(body);
        if (ImGui::Checkbox("Awake", &v)) {
            if (v) {
                SetAwake(body);
            }
            else {
                UnsetAwake(body);
            }
        }
    }
    {
        auto v = IsEnabled(body);
        if (ImGui::Checkbox("Enabled", &v)) {
            SetEnabled(body, v);
        }
    }
}

static void EntityUI(Body& body, const MassData& computed)
{
    {
        auto v = GetMass(body);
        if (MassUI(v, "Mass (kg)", "%.2e")) {
            SetMass(body, v);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Mass of the body. This can be different than the computed mass of %.2e kg.",
                              double(Real(computed.mass / 1_kg)));
        }
    }
    {
        auto v = Real{GetRotInertia(body) / (1_kg * 1_m2 / Square(1_rad))};
        if (InputReal("Rot. Inertia (kg·m²)", v, 0, 0, "%.2e")) {
            SetRotInertia(body, RotInertia{v * (1_kg * 1_m2 / Square(1_rad))});
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Rotational inertia of the body. This can be different than the computed rotational inertia of %.2e kg·m².",
                              double(Real(computed.I / (1_kg * 1_m2 / Square(1_rad)))));
        }
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
        ImGui::ColumnsContext cc(3, "VertexColumns", false);
        ImGui::SetColumnWidths(140, {15, 68, 68});
        for (auto i = static_cast<VertexCounter>(0); i < numVertices; ++i) {
            const auto vertex = proxy.GetVertex(i);
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

template <class T>
static auto DensityUI(T& shape) -> decltype(SetDensity(shape, GetDensity(shape)))
{
    auto val = static_cast<float>(Real{GetDensity(shape) * SquareMeter / Kilogram});
    if (ImGui::InputFloat("Density (kg/m²)", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
        SetDensity(shape, val * Kilogram / SquareMeter);
    }
}

template <class T>
static auto FrictionUI(T& shape) -> decltype(SetFriction(shape, GetFriction(shape)))
{
    auto val = static_cast<float>(GetFriction(shape));
    if (ImGui::InputFloat("Friction", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
        SetFriction(shape, val);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Friction for the shape. Value must be non-negative!",
                           tooltipWrapWidth);
    }
}

template <class T>
static auto RestitutionUI(T& shape) -> decltype(SetRestitution(shape, GetRestitution(shape)))
{
    auto val = static_cast<float>(GetRestitution(shape));
    if (ImGui::InputFloat("Restitution", &val, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
        SetRestitution(shape, val);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Restitution/bounciness for the shape. Value must be finite!",
                           tooltipWrapWidth);
    }
}

template <class T>
static auto SensorUI(T& shape) -> decltype(SetSensor(shape, IsSensor(shape)))
{
    ImGui::StyleVarContext framePaddingCtx(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
    if (auto v = IsSensor(shape); ImGui::Checkbox("Sensor", &v)) {
        SetSensor(shape, v);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Whether or not this acts as a sensor."
                           " Sensors detect collisions but don't participate"
                           " in their resolution - i.e. bodies will pass right through"
                           " bodies having just sensor shapes.",
                           tooltipWrapWidth);
    }
}

template <class T>
static auto FilterUI(T& shape) -> decltype(SetFilter(shape, GetFilter(shape)))
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
        SetFilter(shape, newFilterData);
    }
}

template <class T>
static auto ChildrenUI(T& shape) -> decltype(GetChildCount(shape), EntityUI(GetChild(shape, ChildCounter{})))
{
    const auto childCount = GetChildCount(shape);
    if (ImGui::TreeNodeEx("ShapeChildren", 0, "Children (%u)", childCount)) {
        ImGui::IdContext idCtx("ShapeChildrenCtx");
        for (auto i = ChildCounter(0); i < childCount; ++i) {
            if (ImGui::TreeNodeEx(reinterpret_cast<const void*>(i), 0, "Child %u", i)) {
                EntityUI(GetChild(shape, i));
                ImGui::TreePop();
            }
        }
        ImGui::TreePop();
    }
}

template <class T>
static auto GeneralShapeUI(T& shape) ->
decltype(DensityUI(shape), FrictionUI(shape), RestitutionUI(shape), SensorUI(shape),
         FilterUI(shape), ChildrenUI(shape),
         Translate(shape, shapeTranslateValue),
         Scale(shape, shapeScaleValue),
         Rotate(shape, UnitVec::Get(shapeRotateValue)))
{
    {
        ImGui::ItemWidthContext itemWidthCtx(60);
        //const auto vertexRadius = GetVertexRadius(shape);
        DensityUI(shape);
        ImGui::LabelText("Computed Mass (kg)", "%.2e",
                         static_cast<double>(Real{GetMassData(shape).mass / 1_kg}));
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Shape's computed mass based on its density and area.");
        }
        FrictionUI(shape);
        RestitutionUI(shape);
    }
    ImGui::Spacing();
    SensorUI(shape);
    ImGui::Spacing();
    FilterUI(shape);
    ImGui::Spacing();
    ChildrenUI(shape);
    {
        ImGui::StyleVarContext itemSpacingCtx(ImGuiStyleVar_ItemSpacing, ImVec2(1, 2));
        ImGui::ColumnsContext cc(3, "TranslateScaleRotateColumns", false);
        if (ImGui::Button(shapeTranslateButtonName, ImVec2(-1, 0))) {
            Translate(shape, shapeTranslateValue);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Press to translate this shape %fm horizontally and %fm vertically.",
                              static_cast<float>(Real{GetX(shapeTranslateValue)/1_m}),
                              static_cast<float>(Real{GetY(shapeTranslateValue)/1_m}));
        }
        ImGui::NextColumn();
        if (ImGui::Button(shapeScaleButtonName, ImVec2(-1, 0))) {
            Scale(shape, shapeScaleValue);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Press to scale this shape %fx horizontally and %fx vertically.",
                              static_cast<float>(GetX(shapeScaleValue)),
                              static_cast<float>(GetY(shapeScaleValue)));
        }
        ImGui::NextColumn();
        if (ImGui::Button(shapeRotateButtonName, ImVec2(-1, 0))) {
            Rotate(shape, UnitVec::Get(shapeRotateValue));
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Press to rotate this shape %f degrees counter-clockwise.",
                              static_cast<float>(Real{shapeRotateValue/1_deg}));
        }
        ImGui::NextColumn();
    }
}

static void EntityUI(DiskShapeConf& shape)
{
    {
        ImGui::ItemWidthContext itemWidthCtx(100);
        LengthUI(shape.location, "Location");
    }
    {
        ImGui::ItemWidthContext itemWidthCtx(60);
        LengthUI(shape.vertexRadius, "Radius (m)");
    }
}

static void EntityUI(EdgeShapeConf& shape)
{
    {
        ImGui::ItemWidthContext itemWidthCtx(60);
        LengthUI(shape.vertexRadius, "Vertex Radius (m)");
    }
    {
        //ImGui::ItemWidthContext itemWidthCtx(60);
        const auto location = shape.GetVertexA();
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(location) / Meter});
        vals[1] = static_cast<float>(Real{GetY(location) / Meter});
        ImGui::ItemWidthContext itemWidthCtx(100);
        if (ImGui::InputFloat2("Vertex A", vals, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            shape.Set(Length2{vals[0] * 1_m, vals[1] * 1_m}, shape.GetVertexB());
        }
    }
    {
        const auto location = shape.GetVertexB();
        float vals[2];
        vals[0] = static_cast<float>(Real{GetX(location) / Meter});
        vals[1] = static_cast<float>(Real{GetY(location) / Meter});
        ImGui::ItemWidthContext itemWidthCtx(100);
        if (ImGui::InputFloat2("Vertex B", vals, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            shape.Set(shape.GetVertexA(), Length2{vals[0] * 1_m, vals[1] * 1_m});
        }
    }
}

static void EntityUI(PolygonShapeConf& shape)
{
    {
        ImGui::ItemWidthContext itemWidthCtx(60);
        LengthUI(shape.vertexRadius, "Vertex Radius (m)");
    }
}

static void EntityUI(Shape& shape)
{
    const auto type = GetType(shape);
    if (type == GetTypeID<PolygonShapeConf>()) {
        auto conf = TypeCast<PolygonShapeConf>(shape);
        EntityUI(conf);
        shape = conf;
    }
    else if (type == GetTypeID<DiskShapeConf>()) {
        auto conf = TypeCast<DiskShapeConf>(shape);
        EntityUI(conf);
        shape = conf;
    }
    else if (type == GetTypeID<EdgeShapeConf>()) {
        auto conf = TypeCast<EdgeShapeConf>(shape);
        EntityUI(conf);
        shape = conf;
    }
    GeneralShapeUI(shape);
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
    ImGui::ItemWidthContext itemWidthCtx(70);
    auto shape = GetShape(world, shapeId);
    ChangeTypeUI(shape);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Shows shape's current type and changes it if new selection made.");
    }
    ImGui::SameLine();
    ImGui::Text("Shape's Type");
    try {
        EntityUI(shape);
    }
    catch (const std::invalid_argument& ex) {
        ui->message = std::string("Invalid shape setting: ") + ex.what();
    }
    if (GetShape(world, shapeId) != shape) {
        SetShape(world, shapeId, shape);
    }
    if (ImGui::Button("Destroy", ImVec2(-1, 0))) {
        Destroy(world, shapeId);
    }
}

[[maybe_unused]] static void TransformationMatrixUI()
{
    const auto y = ImGui::GetCursorPosY();
    {
        ImGui::StyleVarContext itemSpacingCtx(ImGuiStyleVar_ItemSpacing, ImVec2(2,2));
        ImGui::StyleVarContext framePaddingCtx(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
        ImGui::ColumnsContext columnsCtx(2, "TransformationMatrix", false);
        const auto columnWidth = 40.0f;
        ImGui::SetColumnWidth(0, columnWidth);
        ImGui::SetColumnWidth(1, columnWidth);
        {
            ImGui::ItemWidthContext itemWidthCtx(columnWidth);
            auto val = static_cast<float>(shapeTransformationMatrix[0][0]);
            if (ImGui::InputFloat("##00", &val)) {
                shapeTransformationMatrix[0][0] = val;
            }
        }
        ImGui::NextColumn();
        {
            ImGui::ItemWidthContext itemWidthCtx(columnWidth);
            auto val = static_cast<float>(shapeTransformationMatrix[0][1]);
            if (ImGui::InputFloat("##01", &val)) {
                shapeTransformationMatrix[0][1] = val;
            }
        }
        ImGui::NextColumn();
        {
            ImGui::ItemWidthContext itemWidthCtx(columnWidth);
            auto val = static_cast<float>(shapeTransformationMatrix[1][0]);
            if (ImGui::InputFloat("##10", &val)) {
                shapeTransformationMatrix[1][0] = val;
            }
        }
        ImGui::NextColumn();
        {
            ImGui::ItemWidthContext itemWidthCtx(columnWidth);
            auto val = static_cast<float>(shapeTransformationMatrix[1][1]);
            if (ImGui::InputFloat("##11", &val)) {
                shapeTransformationMatrix[1][1] = val;
            }
        }
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
}

static void ShapesUI(World& world)
{
    ImGui::IdContext idCtx("WorldShapes");
    LengthUI(shapeTranslateValue, "Translate Amount");
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Amount that's applied on pressing a shape's translate button.",
                           tooltipWrapWidth);
    }
    InputReals("Scale Amount", shapeScaleValue);
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Amount that's applied on pressing a shape's scale button.",
                           tooltipWrapWidth);
    }
    AngleUI(shapeRotateValue, "Rotate Amount");
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("Amount that's applied on pressing a shape's rotate button.",
                           tooltipWrapWidth);
    }
    ImGui::Spacing();
    const auto numShapes = world.GetShapeRange();
    for (auto i = static_cast<ShapeCounter>(0); i < numShapes; ++i) {
        ImGui::IdContext ctx(i);
        const auto shapeId = ShapeID(i);
        if (ImGui::TreeNodeEx(reinterpret_cast<const void*>(i), 0,
                              "Shape %u (%s)", i,
                              Test::ToName(GetType(world, shapeId)))) {
            EntityUI(world, shapeId);
            ImGui::TreePop();
        }
    }
}

static void AttachShapeUI(World& world, BodyID bodyId, const std::vector<ShapeID>& shapeIds)
{
    static auto itemCurrentIdx = 0u;
    static auto flags = ImGuiComboFlags(0);
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
    itemCurrentIdx = itemCurrentIdx % size(available);
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
    ImGui::IdContext idCtx(to_underlying(bodyId));
    ImGui::ItemWidthContext itemWidthCtx(100);
    auto body = GetBody(world, bodyId);
    ChangeTypeUI(body);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Shows body's current type and changes it if new selection made.");
    }
    ImGui::SameLine();
    ImGui::Text("Body's Type");
    const auto shapes = GetShapes(world, bodyId);
    EntityUI(body);
    EntityUI(body, ComputeMassData(world, shapes));
    const auto& curBody = GetBody(world, bodyId);
    if (curBody != body) {
        const auto curType = GetType(curBody);
        const auto newType = GetType(body);
        const auto resetMassData = curType != newType && newType == BodyType::Dynamic && //
                                   GetInvMass(body) == InvMass{} && //
                                   GetInvRotInertia(body) == InvRotInertia{};
        SetBody(world, bodyId, body);
        if (resetMassData) {
            ResetMassData(world, bodyId);
        }
    }
    if (ImGui::TreeNodeEx("BodyShapes", 0, "Shapes (%lu)", size(shapes))) {
        CollectionUI(world, shapes, bodyId, selectedFixtures);
        ImGui::TreePop();
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
    AngleUI(conf.referenceAngle, "Ref. Angle (°)", "%.1e");
    ImGui::LabelText("Limit State", "%s", ToString(GetLimitState(conf)));
    ImGui::LabelText("Motor Impulse (N·m·s)", "%.1e",
                     static_cast<double>(Real{GetAngularMotorImpulse(conf) / NewtonMeterSecond}));
    EntityUI(conf.enableLimit, "Enable Limit");
    {
        auto v = static_cast<float>(Real{GetAngularLowerLimit(conf) / 1_deg});
        if (ImGui::InputFloat("Lower Limit (°)", &v, 0, 0, "%.2f")) {
            SetAngularLimits(conf, v * 1_deg, GetAngularUpperLimit(conf));
        }
    }
    {
        auto v = static_cast<float>(Real{GetAngularUpperLimit(conf) / 1_deg});
        if (ImGui::InputFloat("Upper Limit (°)", &v, 0, 0, "%.2f")) {
            SetAngularLimits(conf, GetAngularLowerLimit(conf), v * 1_deg);
        }
    }
    EntityUI(conf.enableMotor, "Enable Motor");
    {
        auto v = static_cast<float>(Real{GetMotorSpeed(conf) / DegreePerSecond});
        if (ImGui::InputFloat("Motor Speed (°/sec)", &v, 0, 0, "%.2f")) {
            SetMotorSpeed(conf, v * DegreePerSecond);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMaxMotorTorque(conf) / NewtonMeter});
        if (ImGui::InputFloat("Max Mot. Torq. (N·m)", &v)) {
            SetMaxMotorTorque(conf, v * NewtonMeter);
        }
    }
    LengthUI(conf.localAnchorA, "Loc. Anchor A");
    LengthUI(conf.localAnchorB, "Loc. Anchor B");
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
}

static void EntityUI(PrismaticJointConf& conf, BodyCounter bodyRange)
{
    ImGui::LabelText("Limit State", "%s", ToString(GetLimitState(conf)));
    ImGui::LabelText("Motor Impulse (N·s)", "%.1e",
                     static_cast<double>(Real{GetLinearMotorImpulse(conf) / NewtonSecond}));
    ImGui::LabelText("Ref. Angle (°)", "%.1e",
                     static_cast<double>(Real{GetReferenceAngle(conf) / Degree}));
    EntityUI(conf.enableLimit, "Enable Limit");
    {
        auto v = static_cast<float>(Real{GetLinearLowerLimit(conf) / Meter});
        if (ImGui::InputFloat("Lower Limit (m)", &v, 0, 0, "%.2f")) {
            SetLinearLimits(conf, v * Meter, GetLinearUpperLimit(conf));
        }
    }
    {
        auto v = static_cast<float>(Real{GetLinearUpperLimit(conf) / Meter});
        if (ImGui::InputFloat("Upper Limit (m)", &v, 0, 0, "%.2f")) {
            SetLinearLimits(conf, GetLinearLowerLimit(conf), v * Meter);
        }
    }
    EntityUI(conf.enableMotor, "Enable Motor");
    {
        auto v = static_cast<float>(Real{GetMotorSpeed(conf) / DegreePerSecond});
        if (ImGui::InputFloat("Motor Speed (°/sec)", &v)) {
            SetMotorSpeed(conf, v * DegreePerSecond);
        }
    }
    {
        auto v = static_cast<float>(Real{GetMaxMotorForce(conf) / Newton});
        if (ImGui::InputFloat("Max. Motor Force (N)", &v)) {
            SetMaxMotorForce(conf, v * Newton);
        }
    }
    LengthUI(conf.localAnchorA, "Loc. Anchor A");
    LengthUI(conf.localAnchorB, "Loc. Anchor B");
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
}

static void EntityUI(DistanceJointConf& conf, BodyCounter bodyRange)
{
    // All settings implemented here...
    LengthUI(conf.length, "Length (m)");
    {
        auto v = static_cast<float>(Real{GetFrequency(conf) / Hertz});
        if (ImGui::InputFloat("Frequency (Hz)", &v))
        {
            SetFrequency(conf, v * Hertz);
        }
    }
    InputReal("Damping Ratio", conf.dampingRatio);
    LengthUI(conf.localAnchorA, "Loc. Anchor A");
    LengthUI(conf.localAnchorB, "Loc. Anchor B");
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
}

static void EntityUI(PulleyJointConf& conf, BodyCounter bodyRange)
{
    LengthUI(conf.lengthA, "Length A (m)");
    LengthUI(conf.lengthB, "Length B (m)");
    InputReal("Ratio", conf.ratio);
    LengthUI(conf.groundAnchorA, "Ground Anchor A");
    LengthUI(conf.groundAnchorB, "Ground Anchor B");
    LengthUI(conf.localAnchorA, "Loc. Anchor A");
    LengthUI(conf.localAnchorB, "Loc. Anchor B");
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
}

static void EntityUI(TargetJointConf& conf, BodyCounter bodyRange)
{
    LengthUI(conf.target, "Target Loc. (m)");
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
    LengthUI(conf.localAnchorB, "Loc. Anchor B");
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "Not Used A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
}

static void EntityUI(GearJointConf& conf, BodyCounter bodyRange)
{
    const auto typeAC = GetTypeAC(conf);
    const auto typeNameAC = (typeAC != GetTypeID<void>())? Test::ToName(typeAC): "unset";
    const auto hoverMessageAC = [&]() {
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Data for a %s joint between bodies A (%u) and C (%u).", typeNameAC,
                              to_underlying(conf.bodyA), to_underlying(conf.bodyC));
        }
    };
    if (ImGui::TreeNodeEx("TypeDataAC", 0, "A-C Type Data (%s)", typeNameAC)) {
        hoverMessageAC();
        ChangeTypeUI(conf.typeDataAC);
        if (std::holds_alternative<GearJointConf::RevoluteData>(conf.typeDataAC)) {
            auto& data = std::get<GearJointConf::RevoluteData>(conf.typeDataAC);
            AngleUI(data.referenceAngle, "Ref. Angle (°)");
            if (ImGui::IsItemHovered()) {
                ImGui::ShowTooltip("Reference angle in degrees. This is the initial angular "
                                   "difference from body C to body A.", tooltipWrapWidth);
            }
        }
        if (std::holds_alternative<GearJointConf::PrismaticData>(conf.typeDataAC)) {
            auto& data = std::get<GearJointConf::PrismaticData>(conf.typeDataAC);
            LengthUI(data.localAnchorA, "Loc. Anchor C");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Anchor local to body C (%u).", to_underlying(conf.bodyC));
            }
            LengthUI(data.localAnchorB, "Loc. Anchor A");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Anchor local to body A (%u).", to_underlying(conf.bodyA));
            }
            EntityUI(data.localAxis, "Local Axis (°)");
        }
        ImGui::TreePop();
    }
    else {
        hoverMessageAC();
    }

    const auto typeBD = GetTypeBD(conf);
    const auto typeNameBD = (typeBD != GetTypeID<void>())? Test::ToName(typeBD): "unset";
    const auto hoverMessageBD = [&]() {
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Data for a %s joint between bodies B (%u) and D (%u).", typeNameBD,
                              to_underlying(conf.bodyB), to_underlying(conf.bodyD));
        }
    };
    if (ImGui::TreeNodeEx("TypeDataBD", 0, "B-D Type Data (%s)", typeNameBD)) {
        hoverMessageBD();
        ChangeTypeUI(conf.typeDataBD);
        if (std::holds_alternative<GearJointConf::RevoluteData>(conf.typeDataBD)) {
            auto& data = std::get<GearJointConf::RevoluteData>(conf.typeDataBD);
            AngleUI(data.referenceAngle, "Ref. Angle (°)");
            if (ImGui::IsItemHovered()) {
                ImGui::ShowTooltip("Reference angle in degrees. This is the initial angular "
                                   "difference from body D to body B.", tooltipWrapWidth);
            }
        }
        if (std::holds_alternative<GearJointConf::PrismaticData>(conf.typeDataBD)) {
            auto& data = std::get<GearJointConf::PrismaticData>(conf.typeDataBD);
            LengthUI(data.localAnchorA, "Loc. Anchor D");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Anchor local to body D (%u).", to_underlying(conf.bodyD));
            }
            LengthUI(data.localAnchorB, "Loc. Anchor B");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Anchor local to body B (%u).", to_underlying(conf.bodyB));
            }
            EntityUI(data.localAxis, "Local Axis (°)");
        }
        ImGui::TreePop();
    }
    else {
        hoverMessageBD();
    }

    InputReal("Constant", conf.constant, 0, 0, "%.3f");
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("An offset applied along with the ratio during position constraint "
                           "solving. A non-finite value skips position constraint solving.",
                           tooltipWrapWidth);
    }
    InputReal("Ratio", conf.ratio, 0, 0, "%.3f");
    if (ImGui::IsItemHovered()) {
        ImGui::ShowTooltip("The gearing ratio between bodies A & B.", tooltipWrapWidth);
    }
    InputReal("\"Mass\"", conf.mass, 0, 0, "%.3f");

    InputReals("JvAC", conf.JvAC);
    InputReals("JvBD", conf.JvBD);
    LengthUI(conf.JwA, "JwA (m)");
    LengthUI(conf.JwB, "JwB (m)");
    LengthUI(conf.JwC, "JwC (m)");
    LengthUI(conf.JwD, "JwD (m)");

    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
    EntityUI(conf.bodyC, bodyRange, "ID of Body C");
    EntityUI(conf.bodyD, bodyRange, "ID of Body D");
}

static void EntityUI(WheelJointConf& conf, BodyCounter bodyRange)
{
    EntityUI(conf.enableMotor, "Enable Motor");
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
    LengthUI(conf.localAnchorA, "Loc. Anchor A");
    LengthUI(conf.localAnchorB, "Loc. Anchor B");
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
}

static void EntityUI(WeldJointConf& conf, BodyCounter bodyRange)
{
    AngleUI(conf.referenceAngle, "Ref. Angle (°)", "%.1e");
    {
        auto v = static_cast<float>(Real{GetFrequency(conf) / Hertz});
        if (ImGui::InputFloat("Frequency (Hz)", &v))
        {
            SetFrequency(conf, v * Hertz);
        }
    }
    InputReal("Damping Ratio", conf.dampingRatio);
    LengthUI(conf.localAnchorA, "Loc. Anchor A");
    LengthUI(conf.localAnchorB, "Loc. Anchor B");
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
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
    LengthUI(conf.localAnchorA, "Loc. Anchor A");
    LengthUI(conf.localAnchorB, "Loc. Anchor B");
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
}

static void EntityUI(RopeJointConf& conf, BodyCounter bodyRange)
{
    ImGui::LabelText("Limit State", "%s", ToString(GetLimitState(conf)));
    LengthUI(conf.length, "Length (m)");
    {
        auto v = static_cast<float>(Real{GetMaxLength(conf) / Meter});
        if (ImGui::InputFloat("Max. Length (m)", &v))
        {
            SetMaxLength(conf, v * Meter);
        }
    }
    LengthUI(conf.localAnchorA, "Loc. Anchor A");
    LengthUI(conf.localAnchorB, "Loc. Anchor B");
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
}

static void EntityUI(MotorJointConf& conf, BodyCounter bodyRange)
{
    LengthUI(conf.rA, "Relative A (m)");
    LengthUI(conf.rB, "Relative B (m)");
    LengthUI(conf.linearError, "Lin. Error (m)", "%.2e");
    AngleUI(conf.angularError, "Ang. Error (°)", "%.2e");
    LengthUI(conf.linearOffset, "Lin. Offset (m)");
    AngleUI(conf.angularOffset, "Ang. Offset (°)", "%.2f");
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
    InputReal("Correction Factor", conf.correctionFactor);
    EntityUI(conf.collideConnected, "Collide Connected");
    EntityUI(conf.bodyA, bodyRange, "ID of Body A");
    EntityUI(conf.bodyB, bodyRange, "ID of Body B");
}

static void EntityUI(Joint& joint, BodyCounter bodyRange)
{
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
    ImGui::ItemWidthContext itemWidthCtx(100);
    const auto bodyRange = GetBodyRange(world);
    auto joint = GetJoint(world, e);
    ChangeTypeUI(joint);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Shows joint's current type and changes it if new selection made.");
    }
    ImGui::SameLine();
    ImGui::Text("Joint's Type");
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
        auto v = HasValidToi(contact)? static_cast<double>(GetToi(contact)): std::numeric_limits<double>::quiet_NaN();
        if (ImGui::InputDouble("TOI", &v, 0, 0, "%f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            if (v >= 0 && v <= 1) {
                SetToi(contact, static_cast<Real>(v));
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

static void CollectionUI(World& world, const World::Bodies& bodies,
                         const BodySet& selectedBodies, const FixtureSet& selectedFixtures)
{
    ImGui::IdContext idCtx("Bodies");
    for (const auto& e: bodies) {
        const auto typeName = Test::ToName(GetType(world, e));
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
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("World ID of joint and world ID of other associated body.");
            }
        }
        else {
            ImGui::Text("Joint %u", to_underlying(jointID));
            if (ImGui::IsItemHovered()) {
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
    auto& world = test->GetWorld();

    ImGui::StyleVarContext indentSpacingCtx(ImGuiStyleVar_IndentSpacing, ImGui::GetFontSize()*1);
    ImGui::Spacing();
    {
        static auto shape = Shape{PolygonShapeConf{}};
        ImGui::ItemWidthContext itemWidthCtx(80);
        ChangeTypeUI(shape);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Selects type of shape that's created on pressing the %s button.",
                              createShapeButtonName);
        }
        ImGui::SameLine();
        const auto button_sz = ImVec2(-1, 0);
        if (ImGui::Button(createShapeButtonName, button_sz)) {
            CreateShape(world, shape);
        }
        if (ImGui::TreeNodeEx("Shapes", selShapes? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Shapes (%hu)", world.GetShapeRange())) {
            ShapesUI(world);
            ImGui::TreePop();
        }
    }
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    {
        static auto body = Body{};
        ImGui::ItemWidthContext itemWidthCtx(80);
        ChangeTypeUI(body);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Selects type of body that's created on pressing the %s button.",
                              createBodyButtonName);
        }
        ImGui::SameLine();
        if (ImGui::Button(createBodyButtonName, ImVec2(-1, 0))) {
            const auto id = CreateBody(world, body);
            const auto fixtures = FixtureSet{std::make_pair(id, InvalidShapeID)};
            g_testSuite->GetTest()->SetSelectedFixtures(fixtures);

        }
        const auto bodies = GetBodies(world);
        if (ImGui::TreeNodeEx("Bodies", selBodies? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Bodies (%lu)", size(bodies))) {
            CollectionUI(world, bodies, selectedBodies, selectedFixtures);
            ImGui::TreePop();
        }
    }
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    {
        static auto joint = Joint{RevoluteJointConf{}};
        ImGui::ItemWidthContext itemWidthCtx(80);
        ChangeTypeUI(joint);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Selects type of joint that's created on pressing the %s button.",
                              createJointButtonName);
        }
        ImGui::SameLine();
        if (ImGui::Button(createJointButtonName, ImVec2(-1, 0))) {
            CreateJoint(world, joint);
        }
        const auto joints = GetJoints(world);
        if (ImGui::TreeNodeEx("Joints", selJoints? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Joints (%lu)", size(joints))) {
            CollectionUI(world, joints);
            ImGui::TreePop();
        }
    }
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    {
        ImGui::TextWrapped("Contacts cannot be created nor destroyed by users.");
        const auto contacts = GetContacts(world);
        if (ImGui::TreeNodeEx("Contacts", selContacts? ImGuiTreeNodeFlags_DefaultOpen: 0,
                              "Contacts (%lu)", size(contacts))) {
            CollectionUI(world, contacts);
            ImGui::TreePop();
        }
    }
    ImGui::Spacing();
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
    std::ostringstream stream;
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

static void SetupGlfwWindowHints()
{
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

    if (glfwSetErrorCallback(GlfwErrorCallback)) {
        std::fprintf(stderr,
                     "Warning: overriding previously installed GLFW error callback function.\n");
    }
    if (glfwInit() == 0) {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return EXIT_FAILURE;
    }

    const auto buildVersion = GetVersion();
    const auto buildDetails = GetBuildDetails();
    
    char title[64];
    std::sprintf(title, "PlayRho Testbed Version %d.%d.%d",
                 buildVersion.major, buildVersion.minor, buildVersion.revision);

    SetupGlfwWindowHints();

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
        while (!glfwWindowShouldClose(mainWindow)) {
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
            if (!UserInterface()) {
	            glfwSetWindowShouldClose(mainWindow, GL_TRUE);
            }
            Simulate(drawer);

            // Measure speed
            const auto time2 = glfwGetTime();
            const auto timeElapsed = time2 - time1;
            constexpr auto alpha = 0.9;
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
