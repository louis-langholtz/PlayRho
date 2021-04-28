//
//  ExtensionsForImgui.cpp
//  Testbed
//
//  Created by Louis D. Langholtz on 4/27/21.
//

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
