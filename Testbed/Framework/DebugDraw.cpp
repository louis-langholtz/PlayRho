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

#include "DebugDraw.hpp"

#if defined(__APPLE_CC__)
#include <OpenGL/gl3.h>
#else
#include <GL/glew.h>
#endif

#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

#include "RenderGL3.h"

namespace playrho {

namespace {

TextAlign ToNative(Drawer::TextAlign align) noexcept
{
    return static_cast<TextAlign>(align);
}

static void sCheckGLError()
{
    const auto errCode = glGetError();
    if (errCode != GL_NO_ERROR)
    {
        fprintf(stderr, "OpenGL error = %d\n", errCode);
        assert(false);
    }
}

// Prints shader compilation errors
static void sPrintLog(GLuint object)
{
    GLint log_length = 0;
    if (glIsShader(object))
        glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else if (glIsProgram(object))
        glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else
    {
        fprintf(stderr, "printlog: Not a shader or a program\n");
        return;
    }
    
    if (log_length < 0)
    {
        fprintf(stderr, "printlog: got negative log length??\n");
        return;
    }
    
    char* log = (char*)std::malloc(static_cast<std::size_t>(log_length));
    
    if (glIsShader(object))
        glGetShaderInfoLog(object, log_length, nullptr, log);
    else if (glIsProgram(object))
        glGetProgramInfoLog(object, log_length, nullptr, log);
    
    fprintf(stderr, "%s", log);
    std::free(log);
}

static GLuint sCreateShaderFromString(const char* source, GLenum type)
{
    GLuint res = glCreateShader(type);
    const char* sources[] = { source };
    glShaderSource(res, 1, sources, nullptr);
    glCompileShader(res);
    GLint compile_ok = GL_FALSE;
    glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);
    if (compile_ok == GL_FALSE)
    {
        fprintf(stderr, "Error compiling shader of type %d!\n", type);
        sPrintLog(res);
        glDeleteShader(res);
        return 0;
    }
    
    return res;
}

static GLuint sCreateShaderProgram(const char* vs, const char* fs)
{
    GLuint vsId = sCreateShaderFromString(vs, GL_VERTEX_SHADER);
    GLuint fsId = sCreateShaderFromString(fs, GL_FRAGMENT_SHADER);
    assert(vsId != 0 && fsId != 0);
    
    GLuint programId = glCreateProgram();
    glAttachShader(programId, vsId);
    glAttachShader(programId, fsId);
    glBindFragDataLocation(programId, 0, "color");
    glLinkProgram(programId);
    
    glDeleteShader(vsId);
    glDeleteShader(fsId);
    
    GLint status = GL_FALSE;
    glGetProgramiv(programId, GL_LINK_STATUS, &status);
    assert(status != GL_FALSE);
    
    return programId;
}

} // namespace


Length2D ConvertScreenToWorld(const Camera& camera, const Coord2D ps)
{
    const auto w = float(camera.m_width);
    const auto h = float(camera.m_height);
    const auto u = ps.x / w;
    const auto v = (h - ps.y) / h;
    
    const auto ratio = w / h;
    const auto extents = Coord2D{ratio * 25.0f, 25.0f} * camera.m_zoom;
    
    const auto lower = camera.m_center - extents;
    const auto upper = camera.m_center + extents;
    
    const auto x = Real{((1 - u) * lower.x + u * upper.x)};
    const auto y = Real{((1 - v) * lower.y + v * upper.y)};
    return Length2D{x * Meter, y * Meter};
}

AABB ConvertScreenToWorld(const Camera& camera)
{
    const auto w = float(camera.m_width);
    const auto h = float(camera.m_height);
    
    const auto ratio = w / h;
    const auto extents = Coord2D{ratio * 25.0f, 25.0f} * camera.m_zoom;
    
    const auto lower = camera.m_center - extents;
    const auto upper = camera.m_center + extents;
    
    return AABB{
        Length2D{Real{lower.x} * Meter, Real{lower.y} * Meter},
        Length2D{Real{upper.x} * Meter, Real{upper.y} * Meter}
    };
}

Coord2D ConvertWorldToScreen(const Camera& camera, const Length2D pw)
{
    const auto w = float(camera.m_width);
    const auto h = float(camera.m_height);
    const auto ratio = w / h;
    const auto extents = Coord2D{ratio * 25.0f, 25.0f} * camera.m_zoom;
    
    const auto lower = camera.m_center - extents;
    const auto upper = camera.m_center + extents;
    
    const auto u = (float(Real{GetX(pw) / Meter}) - lower.x) / (upper.x - lower.x);
    const auto v = (float(Real{GetY(pw) / Meter}) - lower.y) / (upper.y - lower.y);
    
    return Coord2D{u * w, (float(1) - v) * h};
}

// Convert from world coordinates to normalized device coordinates.
// http://www.songho.ca/opengl/gl_projectionmatrix.html
ProjectionMatrix GetProjectionMatrix(const Camera& camera, float zBias)
{
    const auto w = float(camera.m_width);
    const auto h = float(camera.m_height);
    const auto ratio = w / h;
    const auto extents = Coord2D{ratio * 25.0f, 25.0f} * camera.m_zoom;
    
    const auto lower = camera.m_center - extents;
    const auto upper = camera.m_center + extents;
    
    return ProjectionMatrix{{
        2.0f / (upper.x - lower.x), // 0
        0.0f, // 1
        0.0f, // 2
        0.0f, // 3
        0.0f, // 4
        2.0f / (upper.y - lower.y), // 5
        0.0f, // 6
        0.0f, // 7
        0.0f, // 8
        0.0f, // 9
        1.0f, // 10
        0.0f, // 11
        -(upper.x + lower.x) / (upper.x - lower.x), // 12
        -(upper.y + lower.y) / (upper.y - lower.y), // 13
        zBias, // 14
        1.0f
    }};
}

struct GLRenderPoints
{
    GLRenderPoints()
    {
        static constexpr char vs[] = \
        "#version 330\n"
        "uniform mat4 projectionMatrix;\n"
        "layout(location = 0) in vec2 v_position;\n"
        "layout(location = 1) in vec4 v_color;\n"
        "layout(location = 2) in float v_size;\n"
        "out vec4 f_color;\n"
        "void main(void)\n"
        "{\n"
        "    f_color = v_color;\n"
        "    gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
        "    gl_PointSize = v_size;\n"
        "}\n";
        
        static constexpr char fs[] = \
        "#version 330\n"
        "in vec4 f_color;\n"
        "out vec4 color;\n"
        "void main(void)\n"
        "{\n"
        "    color = f_color;\n"
        "}\n";
        
        m_programId = sCreateShaderProgram(vs, fs);
        m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
        m_vertexAttribute = 0;
        m_colorAttribute = 1;
        m_sizeAttribute = 2;
        
        // Generate
        glGenVertexArrays(1, &m_vaoId);
        glGenBuffers(3, m_vboIds);
        
        glBindVertexArray(m_vaoId);
        glEnableVertexAttribArray(m_vertexAttribute);
        glEnableVertexAttribArray(m_colorAttribute);
        glEnableVertexAttribArray(m_sizeAttribute);
        
        // Vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
        glVertexAttribPointer(m_sizeAttribute, 1, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_sizes), m_sizes, GL_DYNAMIC_DRAW);

        sCheckGLError();
        
        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        
        m_count = 0;
    }
    
    ~GLRenderPoints()
    {
        if (m_vaoId)
        {
            glDeleteVertexArrays(1, &m_vaoId);
            glDeleteBuffers(2, m_vboIds);
            m_vaoId = 0;
        }
        
        if (m_programId)
        {
            glDeleteProgram(m_programId);
            m_programId = 0;
        }
    }
    
    void Vertex(Camera& camera, const Coord2D& v, const Color& c, float size)
    {
        if (m_count == e_maxVertices)
            Flush(camera);
        
        m_vertices[m_count] = v;
        m_colors[m_count] = c;
        m_sizes[m_count] = size;
        ++m_count;
    }
    
    void Flush(Camera& camera)
    {
        if (m_count == 0)
            return;
        
        glUseProgram(m_programId);
        
        const auto proj = GetProjectionMatrix(camera, 0.0f);
        
        glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj.m);
        
        glBindVertexArray(m_vaoId);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<unsigned>(m_count) * sizeof(m_vertices[0]), m_vertices);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<unsigned>(m_count) * sizeof(m_colors[0]), m_colors);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<unsigned>(m_count) * sizeof(m_sizes[0]), m_sizes);

        glEnable(GL_PROGRAM_POINT_SIZE);
        glDrawArrays(GL_POINTS, 0, m_count);
        glDisable(GL_PROGRAM_POINT_SIZE);

        sCheckGLError();
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);
        
        m_count = 0;
    }
    
    enum { e_maxVertices = 512 };
    Coord2D m_vertices[e_maxVertices];
    Color m_colors[e_maxVertices];
    float m_sizes[e_maxVertices];

    GLsizei m_count;
    
    GLuint m_vaoId;
    GLuint m_vboIds[3];
    GLuint m_programId;
    GLint m_projectionUniform;
    GLuint m_vertexAttribute;
    GLuint m_colorAttribute;
    GLuint m_sizeAttribute;
};

//
struct GLRenderLines
{
    GLRenderLines()
    {
        const char* vs = \
        "#version 330\n"
        "uniform mat4 projectionMatrix;\n"
        "layout(location = 0) in vec2 v_position;\n"
        "layout(location = 1) in vec4 v_color;\n"
        "out vec4 f_color;\n"
        "void main(void)\n"
        "{\n"
        "    f_color = v_color;\n"
        "    gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
        "}\n";
        
        const char* fs = \
        "#version 330\n"
        "in vec4 f_color;\n"
        "out vec4 color;\n"
        "void main(void)\n"
        "{\n"
        "    color = f_color;\n"
        "}\n";
        
        m_programId = sCreateShaderProgram(vs, fs);
        m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
        m_vertexAttribute = 0;
        m_colorAttribute = 1;
        
        // Generate
        glGenVertexArrays(1, &m_vaoId);
        glGenBuffers(2, m_vboIds);
        
        glBindVertexArray(m_vaoId);
        glEnableVertexAttribArray(m_vertexAttribute);
        glEnableVertexAttribArray(m_colorAttribute);
        
        // Vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);
        
        sCheckGLError();
        
        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        
        m_count = 0;
    }
    
    ~GLRenderLines()
    {
        if (m_vaoId)
        {
            glDeleteVertexArrays(1, &m_vaoId);
            glDeleteBuffers(2, m_vboIds);
        }
        
        if (m_programId)
        {
            glDeleteProgram(m_programId);
        }
    }
    
    void Vertex(Camera& camera, const Coord2D& v, const Color& c)
    {
        if (m_count == e_maxVertices)
            Flush(camera);
        
        m_vertices[m_count] = v;
        m_colors[m_count] = c;
        ++m_count;
    }
    
    void Flush(Camera& camera)
    {
        if (m_count == 0)
            return;
        
        glUseProgram(m_programId);
        
        const auto proj = GetProjectionMatrix(camera, 0.1f);
        
        glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj.m);
        
        glBindVertexArray(m_vaoId);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<unsigned>(m_count) * sizeof(m_vertices[0]), m_vertices);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<unsigned>(m_count) * sizeof(m_colors[0]), m_colors);
        
        glDrawArrays(GL_LINES, 0, m_count);
        
        sCheckGLError();
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);
        
        m_count = 0;
    }
    
    enum { e_maxVertices = 2 * 512 };
    Coord2D m_vertices[e_maxVertices];
    Color m_colors[e_maxVertices];
    
    GLsizei m_count;
    
    GLuint m_vaoId;
    GLuint m_vboIds[2];
    GLuint m_programId;
    GLint m_projectionUniform;
    GLuint m_vertexAttribute;
    GLuint m_colorAttribute;
};

//
struct GLRenderTriangles
{
    GLRenderTriangles()
    {
        const char* vs = \
            "#version 330\n"
            "uniform mat4 projectionMatrix;\n"
            "layout(location = 0) in vec2 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "out vec4 f_color;\n"
            "void main(void)\n"
            "{\n"
            "    f_color = v_color;\n"
            "    gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
            "}\n";

        const char* fs = \
            "#version 330\n"
            "in vec4 f_color;\n"
            "out vec4 color;\n"
            "void main(void)\n"
            "{\n"
            "    color = f_color;\n"
            "}\n";

        m_programId = sCreateShaderProgram(vs, fs);
        m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
        m_vertexAttribute = 0;
        m_colorAttribute = 1;

        // Generate
        glGenVertexArrays(1, &m_vaoId);
        glGenBuffers(2, m_vboIds);

        glBindVertexArray(m_vaoId);
        glEnableVertexAttribArray(m_vertexAttribute);
        glEnableVertexAttribArray(m_colorAttribute);

        // Vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

        sCheckGLError();

        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        m_count = 0;
    }

    ~GLRenderTriangles()
    {
        if (m_vaoId)
        {
            glDeleteVertexArrays(1, &m_vaoId);
            glDeleteBuffers(2, m_vboIds);
        }

        if (m_programId)
        {
            glDeleteProgram(m_programId);
        }
    }

    void Vertex(Camera& camera, const Coord2D& v, const Color& c)
    {
        if (m_count == e_maxVertices)
            Flush(camera);

        m_vertices[m_count] = v;
        m_colors[m_count] = c;
        ++m_count;
    }

    void Flush(Camera& camera)
    {
        if (m_count == 0)
            return;
        
        glUseProgram(m_programId);
        
        const auto proj = GetProjectionMatrix(camera, 0.2f);
        
        glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj.m);
        
        glBindVertexArray(m_vaoId);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<unsigned>(m_count) * sizeof(m_vertices[0]), m_vertices);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<unsigned>(m_count) * sizeof(m_colors[0]), m_colors);
        
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArrays(GL_TRIANGLES, 0, m_count);
        glDisable(GL_BLEND);
        
        sCheckGLError();
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);
        
        m_count = 0;
    }
    
    enum { e_maxVertices = 3 * 512 };
    Coord2D m_vertices[e_maxVertices];
    Color m_colors[e_maxVertices];

    GLsizei m_count;

    GLuint m_vaoId;
    GLuint m_vboIds[2];
    GLuint m_programId;
    GLint m_projectionUniform;
    GLuint m_vertexAttribute;
    GLuint m_colorAttribute;
};

DebugDraw::DebugDraw(Camera& camera):
    m_camera(camera),
    m_cosInc{std::cos((2 * Pi) / m_circleParts)},
    m_sinInc{std::sin((2 * Pi) / m_circleParts)}
{
    m_points = new GLRenderPoints;
    m_lines = new GLRenderLines;
    m_triangles = new GLRenderTriangles;
}

DebugDraw::~DebugDraw() noexcept
{
    delete m_triangles;
    delete m_lines;
    delete m_points;
}

void DebugDraw::DrawTriangle(const Length2D& p1, const Length2D& p2, const Length2D& p3, const Color& color)
{
    const auto c1 = Coord2D{static_cast<float>(StripUnit(GetX(p1))), static_cast<float>(StripUnit(GetY(p1)))};
    const auto c2 = Coord2D{static_cast<float>(StripUnit(GetX(p2))), static_cast<float>(StripUnit(GetY(p2)))};
    const auto c3 = Coord2D{static_cast<float>(StripUnit(GetX(p3))), static_cast<float>(StripUnit(GetY(p3)))};
    m_triangles->Vertex(m_camera, c1, color);
    m_triangles->Vertex(m_camera, c2, color);
    m_triangles->Vertex(m_camera, c3, color);
}

void DebugDraw::DrawSegment(const Length2D& p1, const Length2D& p2, const Color& color)
{
    const auto c1 = Coord2D{static_cast<float>(StripUnit(GetX(p1))), static_cast<float>(StripUnit(GetY(p1)))};
    const auto c2 = Coord2D{static_cast<float>(StripUnit(GetX(p2))), static_cast<float>(StripUnit(GetY(p2)))};
    m_lines->Vertex(m_camera, c1, color);
    m_lines->Vertex(m_camera, c2, color);
}

void DebugDraw::DrawPoint(const Length2D& p, float size, const Color& color)
{
    const auto c = Coord2D{static_cast<float>(StripUnit(GetX(p))), static_cast<float>(StripUnit(GetY(p)))};
    m_points->Vertex(m_camera, c, color, size);
}

void DebugDraw::Flush()
{
    m_triangles->Flush(m_camera);
    m_lines->Flush(m_camera);
    m_points->Flush(m_camera);
}

void DebugDraw::DrawPolygon(const Length2D* vertices, size_type vertexCount, const Color& color)
{
    auto p1 = vertices[vertexCount - 1];
    for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
    {
        const auto p2 = vertices[i];
        DrawSegment(p1, p2, color);
        p1 = p2;
    }
}

void DebugDraw::DrawCircle(const Length2D& center, Length radius, const Color& color)
{
    auto r1 = Vec2(1, 0);
    auto v1 = center + radius * r1;
    for (auto i = decltype(m_circleParts){0}; i < m_circleParts; ++i)
    {
        const auto r2 = Vec2{
            m_cosInc * GetX(r1) - m_sinInc * GetY(r1), m_sinInc * GetX(r1) + m_cosInc * GetY(r1)
        };
        const auto v2 = center + radius * r2;
        DrawSegment(v1, v2, color);
        r1 = r2;
        v1 = v2;
    }
}

void DebugDraw::DrawSolidPolygon(const Length2D* vertices, size_type vertexCount, const Color& color)
{
    for (auto i = decltype(vertexCount){1}; i < vertexCount - 1; ++i)
    {
        DrawTriangle(vertices[0], vertices[i], vertices[i+1], color);
    }
}

void DebugDraw::DrawSolidCircle(const Length2D& center, Length radius, const Color& color)
{
    const auto v0 = center;
    auto r1 = Vec2(m_cosInc, m_sinInc);
    auto v1 = center + radius * r1;
    for (auto i = decltype(m_circleParts){0}; i < m_circleParts; ++i)
    {
        // Perform rotation to avoid additional trigonometry.
        const auto r2 = Vec2{
            m_cosInc * GetX(r1) - m_sinInc * GetY(r1), m_sinInc * GetX(r1) + m_cosInc * GetY(r1)
        };
        const auto v2 = center + radius * r2;
        DrawTriangle(v0, v1, v2, color);
        r1 = r2;
        v1 = v2;
    }
}

void DebugDraw::DrawString(int x, int y, TextAlign align, const char *string, ...)
{
    const auto h = float(m_camera.m_height);

    char buffer[512];

    va_list arg;
    va_start(arg, string);
    vsnprintf(buffer, sizeof(buffer), string, arg);
    va_end(arg);

    AddGfxCmdText(float(x), h - float(y), ToNative(align), buffer, SetRGBA(230, 153, 153, 255));
}

void DebugDraw::DrawString(const Length2D& pw, TextAlign align, const char *string, ...)
{
    const auto ps = ConvertWorldToScreen(m_camera, pw);
    const auto h = float(m_camera.m_height);

    char buffer[512];

    va_list arg;
    va_start(arg, string);
    vsprintf(buffer, string, arg);
    va_end(arg);

    AddGfxCmdText(ps.x, h - ps.y, ToNative(align), buffer, SetRGBA(230, 153, 153, 255));
}

Length2D DebugDraw::GetTranslation() const
{
    return Length2D{
        Real(m_camera.m_center.x) * Meter,
        Real(m_camera.m_center.y) * Meter
    };
}

void DebugDraw::SetTranslation(Length2D value)
{
    m_camera.m_center = Coord2D{
        static_cast<float>(Real{GetX(value) / Meter}),
        static_cast<float>(Real{GetY(value) / Meter})
    };
}
    
}
