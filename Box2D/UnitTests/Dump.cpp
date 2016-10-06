/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "gtest/gtest.h"
#include <Box2D/Box2D.h>

#include <unistd.h>

using namespace box2d;

static void open_stderr_as_stdout()
{
	fclose(stdout);
	stdout = stderr;
}

TEST(Dump, EmptyWorld)
{
	std::stringstream buf;
	buf << "Vec2 g\\(0\\.000000000000000e\\+00f, -9\\.800000190734863e\\+00f\\);\n";
	buf << "m_world->SetGravity\\(g\\);\n";
	buf << "Body\\*\\* bodies = \\(Body\\*\\*\\)alloc\\(0 \\* sizeof\\(Body\\*\\)\\);\n";
	buf << "Joint\\*\\* joints = \\(Joint\\*\\*\\)alloc\\(0 \\* sizeof\\(Joint\\*\\)\\);\n";
	buf << "free\\(joints\\);\n";
	buf << "free\\(bodies\\);\n";
	buf << "joints = nullptr;\n";
	buf << "bodies = nullptr;\n";

	EXPECT_EXIT({ open_stderr_as_stdout(); World world; Dump(world); exit(0); }, ::testing::ExitedWithCode(0), buf.str());
}

TEST(Dump, OneBodyWorld)
{
	std::stringstream buf;
	buf << "Vec2 g\\(0\\.000000000000000e\\+00f, -9\\.800000190734863e\\+00f\\);\n";
	buf << "m_world->SetGravity\\(g\\);\n";
	buf << "Body\\*\\* bodies = \\(Body\\*\\*\\)alloc\\(1 \\* sizeof\\(Body\\*\\)\\);\n";
	buf << "{\n";
	buf << "  BodyDef bd;\n";
	buf << "  bd\\.type = BodyType\\(0\\);\n";
	buf << "  bd\\.position = Vec2\\(0\\.000000000000000e\\+00f, 0\\.000000000000000e\\+00f\\);\n";
	buf << "  bd\\.angle = 0\\.000000000000000e\\+00f;\n";
	buf << "  bd\\.linearVelocity = Vec2\\(0\\.000000000000000e\\+00f, 0\\.000000000000000e\\+00f);\n";
	buf << "  bd\\.angularVelocity = 0\\.000000000000000e\\+00f;\n";
	buf << "  bd\\.linearDamping = 0\\.000000000000000e\\+00f;\n";
	buf << "  bd\\.angularDamping = 0\\.000000000000000e\\+00f;\n";
	buf << "  bd\\.allowSleep = bool\\(1\\);\n";
	buf << "  bd\\.awake = bool\\(1\\);\n";
	buf << "  bd\\.fixedRotation = bool\\(0\\);\n";
	buf << "  bd\\.bullet = bool\\(1\\);\n";
	buf << "  bd\\.active = bool\\(1\\);\n";
	buf << "  bodies\\[0\\] = m_world->CreateBody\\(bd\\);\n";
	buf << "\n";
	buf << "}\n";
	buf << "Joint\\*\\* joints = \\(Joint\\*\\*\\)alloc\\(0 \\* sizeof\\(Joint\\*\\)\\);\n";
	buf << "free\\(joints\\);\n";
	buf << "free\\(bodies\\);\n";
	buf << "joints = nullptr;\n";
	buf << "bodies = nullptr;\n";
	EXPECT_EXIT({ open_stderr_as_stdout(); World world; world.CreateBody(BodyDef{}); Dump(world); exit(0); }, ::testing::ExitedWithCode(0), buf.str());
}