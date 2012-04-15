/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

module box2d.testbed.tests.bullettest;

import box2d.all;
import box2d.testbed.framework.test;

class BulletTest : Test
{
public:

	this()
	{
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 0.0f);
			b2Body b = m_world.CreateBody(bd);

			auto edge = new b2EdgeShape;
			edge.Set(b2Vec2(-10.0f, 0.0f), b2Vec2(10.0f, 0.0f));
			b.CreateFixture(edge, 0.0f);

			auto shape = new b2PolygonShape;
			shape.SetAsBox(0.2f, 1.0f, b2Vec2(0.5f, 1.0f), 0.0f);
			b.CreateFixture(shape, 0.0f);
		}

		{
			b2BodyDef bd;
			bd.type = b2BodyType.b2_dynamicBody;
			bd.position.Set(0.0f, 4.0f);

			auto box = new b2PolygonShape;
			box.SetAsBox(2.0f, 0.1f);

			m_body = m_world.CreateBody(bd);
			m_body.CreateFixture(box, 1.0f);

			box.SetAsBox(0.25f, 0.25f);

			//m_x = RandomFloat(-1.0f, 1.0f);
			m_x = 0.20352793f;
			bd.position.Set(m_x, 10.0f);
			bd.bullet = true;

			m_bullet = m_world.CreateBody(bd);
			m_bullet.CreateFixture(box, 100.0f);

			m_bullet.SetLinearVelocity(b2Vec2(0.0f, -50.0f));
		}
	}

	void Launch()
	{
		m_body.SetTransform(b2Vec2(0.0f, 4.0f), 0.0f);
		m_body.SetLinearVelocity(b2Vec2_zero);
		m_body.SetAngularVelocity(0.0f);

		m_x = RandomFloat(-1.0f, 1.0f);
		m_bullet.SetTransform(b2Vec2(m_x, 10.0f), 0.0f);
		m_bullet.SetLinearVelocity(b2Vec2(0.0f, -50.0f));
		m_bullet.SetAngularVelocity(0.0f);

		b2_gjkCalls = 0;
		b2_gjkIters = 0;
		b2_gjkMaxIters = 0;

		b2_toiCalls = 0;
		b2_toiIters = 0;
		b2_toiMaxIters = 0;
		b2_toiRootIters = 0;
		b2_toiMaxRootIters = 0;
	}

	override void Step(Settings settings)
	{
		super.Step(settings);

		if (b2_gjkCalls > 0)
		{
			m_debugDraw.DrawString(5, m_textLine, "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
				b2_gjkCalls, b2_gjkIters / cast(float32)b2_gjkCalls, b2_gjkMaxIters);
			m_textLine += 15;
		}

		if (b2_toiCalls > 0)
		{
			m_debugDraw.DrawString(5, m_textLine, "toi calls = %d, ave toi iters = %3.1f, max toi iters = %d",
				b2_toiCalls, b2_toiIters / cast(float32)b2_toiCalls, b2_toiMaxRootIters);
			m_textLine += 15;

			m_debugDraw.DrawString(5, m_textLine, "ave toi root iters = %3.1f, max toi root iters = %d",
				b2_toiRootIters / cast(float32)b2_toiCalls, b2_toiMaxRootIters);
			m_textLine += 15;
		}

		if (m_stepCount % 60 == 0)
		{
			Launch();
		}
	}

	static Test Create()
	{
		return new BulletTest;
	}

	b2Body m_body;
	b2Body m_bullet;
	float32 m_x;
}
