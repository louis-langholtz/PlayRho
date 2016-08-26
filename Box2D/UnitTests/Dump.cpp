//
//  Dump.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 8/25/16.
//
//

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