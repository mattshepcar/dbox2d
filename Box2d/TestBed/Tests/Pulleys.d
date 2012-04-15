/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

module box2d.testbed.tests.pulleys;

import box2d.all;
import box2d.testbed.framework.test;
import std.typecons;

class Pulleys : Test
{
public:
	this()
	{
		float32 y = 16.0f;
		float32 L = 12.0f;
		float32 a = 1.0f;
		float32 b = 2.0f;

		b2Body ground;
		{
			b2BodyDef bd;
			ground = m_world.CreateBody(bd);

			auto edge = scoped!b2EdgeShape;
			edge.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			//ground.CreateFixture(&shape, 0.0f);

			auto circle = scoped!b2CircleShape;
			circle.m_radius = 2.0f;

			circle.m_p.Set(-10.0f, y + b + L);
			ground.CreateFixture(circle, 0.0f);

			circle.m_p.Set(10.0f, y + b + L);
			ground.CreateFixture(circle, 0.0f);
		}

		{

			auto shape = scoped!b2PolygonShape;
			shape.SetAsBox(a, b);

			b2BodyDef bd;
			bd.type = b2BodyType.b2_dynamicBody;

			//bd.fixedRotation = true;
			bd.position.Set(-10.0f, y);
			b2Body body1 = m_world.CreateBody(bd);
			body1.CreateFixture(shape, 5.0f);

			bd.position.Set(10.0f, y);
			b2Body body2 = m_world.CreateBody(bd);
			body2.CreateFixture(shape, 5.0f);

			auto pulleyDef = scoped!b2PulleyJointDef;
			b2Vec2 anchor1 = {-10.0f, y + b};
			b2Vec2 anchor2 = {10.0f, y + b};
			b2Vec2 groundAnchor1 = {-10.0f, y + b + L};
			b2Vec2 groundAnchor2 = {10.0f, y + b + L};
			pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.5f);

			m_joint1 = cast(b2PulleyJoint)m_world.CreateJoint(pulleyDef);
		}
	}

	override void Keyboard(ubyte key)
	{
		switch (key)
		{
		default:
			break;
		}
	}

	override void Step(Settings settings)
	{
		super.Step(settings);

		float32 ratio = m_joint1.GetRatio();
		float32 L = m_joint1.GetCurrentLengthA() + ratio * m_joint1.GetCurrentLengthB();
		m_debugDraw.DrawString(5, m_textLine, "L1 + %4.2f * L2 = %4.2f", cast(float) ratio, cast(float) L);
		m_textLine += 15;
	}

	static Test Create()
	{
		return new Pulleys;
	}

	b2PulleyJoint m_joint1;
}
