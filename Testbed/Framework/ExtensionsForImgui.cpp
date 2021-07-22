/*
 * Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "ExtensionsForImgui.hpp"

#include "imgui_internal.h"

void ImGui::SetColumnWidths(float remainingWidth, std::initializer_list<float> widths)
{
    const auto numColumns = GetColumnsCount();
    auto column = decltype(numColumns){0};
    for (auto width: widths)
    {
        SetColumnWidth(column, width);
        remainingWidth -= width;
        ++column;
    }
    const auto remainingColumns = numColumns - column;
    const auto widthPerRemainingColumn = remainingWidth / remainingColumns;
    for (auto i = column; i < numColumns; ++i)
    {
        SetColumnWidth(i, widthPerRemainingColumn);
    }
}

void ImGui::Value(const char* prefix, unsigned long v)
{
    Text("%s: %lu", prefix, v);
}

void ImGui::Value(const char* prefix, double v, const char* float_format)
{
    if (float_format)
    {
        char fmt[64];
        ImFormatString(fmt, IM_ARRAYSIZE(fmt), "%%s: %s", float_format);
        Text(fmt, prefix, v);
    }
    else
    {
        Text("%s: %.3f", prefix, v);
    }
}

bool ImGui::InputDouble2(const char* label, double v[2], const char* format, ImGuiInputTextFlags flags)
{
    return InputScalarN(label, ImGuiDataType_Double, v, 2, NULL, NULL, format, flags);
}
