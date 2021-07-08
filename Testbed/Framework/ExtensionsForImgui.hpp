//
//  ExtensionsForImgui.hpp
//  Testbed
//
//  Created by Louis D. Langholtz on 4/27/21.
//

#ifndef ExtensionsForImgui_hpp
#define ExtensionsForImgui_hpp

#include <initializer_list>
#include <string>

#include "imgui.h"

namespace ImGui {

struct WindowContext
{
public:
    WindowContext(const char* name, bool* p_open = nullptr, ImGuiWindowFlags flags = 0)
    {
        Begin(name, p_open, flags);
    }

    WindowContext(const char* name, bool* p_open, const ImVec2& size_first_use,
                  float bg_alpha = -1.0f, ImGuiWindowFlags flags = 0)
    {
        if (size_first_use.x != 0.0f || size_first_use.y != 0.0f)
            SetNextWindowSize(size_first_use, ImGuiCond_FirstUseEver);
        if (bg_alpha >= 0.0f)
            SetNextWindowBgAlpha(bg_alpha);
        if (bg_alpha == 0)
            flags |= ImGuiWindowFlags_NoBackground;
        Begin(name, p_open, flags);
    }

    ~WindowContext()
    {
        End();
    }
};

class ColumnsContext
{
public:
    ColumnsContext(int count = 1, const char* id = nullptr, bool border = true):
    m_before_count(GetColumnsCount())
    {
        Columns(count, id, border);
    }
    ~ColumnsContext()
    {
        Columns(m_before_count);
    }
private:
    int m_before_count;
};

struct TextWrapPosContext
{
    TextWrapPosContext(float wrap_pos_x = 0.0f)
    {
        PushTextWrapPos(wrap_pos_x);
    }
    ~TextWrapPosContext()
    {
        PopTextWrapPos();
    }
};

struct TooltipContext
{
    TooltipContext()
    {
        BeginTooltip();
    }

    ~TooltipContext()
    {
        EndTooltip();
    }
};

struct IdContext
{
    IdContext(const char* key) { PushID(key); }
    IdContext(const char* key_begin, const char* key_end) { PushID(key_begin, key_end); }
    IdContext(const void* key) { PushID(key); }
    IdContext(int key) { PushID(key); }
    ~IdContext() { PopID(); }
};

struct ItemWidthContext
{
    ItemWidthContext(float item_width)
    {
        PushItemWidth(item_width);
    }
    ~ItemWidthContext()
    {
        PopItemWidth();
    }
};

struct GroupContext
{
    GroupContext() { BeginGroup(); }
    ~GroupContext() { EndGroup(); }
};

struct PopupModalContext
{
    bool is_open = false;

    PopupModalContext(const char* name, bool* p_open = nullptr)
    {
        is_open = BeginPopupModal(name, p_open);
    }

    ~PopupModalContext()
    {
        if (is_open) {
            EndPopup();
        }
    }

    explicit operator bool() const noexcept
    {
        return is_open;
    }
};

struct StyleVarContext
{
    StyleVarContext(ImGuiStyleVar idx, const ImVec2& val)
    {
        PushStyleVar(idx, val);
    }

    StyleVarContext(ImGuiStyleVar idx, float val)
    {
        PushStyleVar(idx, val);
    }

    ~StyleVarContext()
    {
        PopStyleVar();
    }
};

struct StyleColorContext
{
    StyleColorContext(ImGuiCol idx, ImU32 color)
    {
        PushStyleColor(idx, color);
    }

    StyleColorContext(ImGuiCol idx, const ImVec4& color)
    {
        PushStyleColor(idx, color);
    }

    ~StyleColorContext()
    {
        PopStyleColor();
    }
};

IMGUI_API void Value(const char* prefix, unsigned long v);
IMGUI_API void Value(const char* prefix, double v, const char* float_format = nullptr);

void SetColumnWidths(float remainingWidth, std::initializer_list<float> widths);

inline void TextUnformatted(const std::string& str)
{
    ImGui::TextUnformatted(str.c_str(), str.c_str() + str.length());
}

inline void TextWrappedUnformatted(const std::string& str)
{
    ImGui::TextWrapPosContext ctxt;
    ImGui::TextUnformatted(str.c_str(), str.c_str() + str.length());
}

inline void ShowTooltip(const std::string& str, float wrap_pos_x = 0.0f)
{
    TooltipContext ctx;
    TextWrapPosContext twpc(wrap_pos_x);
    TextUnformatted(str);
}

IMGUI_API bool InputDouble2(const char* label, double v[2], const char* format = "%.6f",
                            ImGuiInputTextFlags flags = 0);

} // namespace ImGui

#endif /* ExtensionsForImgui_hpp */
