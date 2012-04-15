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

module box2d.testbed.tests.gears;

import box2d.all;
import box2d.testbed.framework.test;

import std.typecons;

class Gears : Test
{
public:
	this()
	{
		b2Body ground;
		{
			b2BodyDef bd;
			ground = m_world.CreateBody(bd);

			auto shape = scoped!b2EdgeShape;
			shape.Set(b2Vec2(50.0f, 0.0f), b2Vec2(-50.0f, 0.0f));
			ground.CreateFixture(shape, 0.0f);
		}

		{
			auto circle1 = scoped!b2CircleShape;
			circle1.m_radius = 1.0f;

			auto box = scoped!b2PolygonShape;
			box.SetAsBox(0.5f, 5.0f);

			auto circle2 = scoped!b2CircleShape;
			circle2.m_radius = 2.0f;
			
			b2BodyDef bd1;
			bd1.type = b2BodyType.b2_staticBody;
			bd1.position.Set(10.0f, 9.0f);
			b2Body body1 = m_world.CreateBody(bd1);
			body1.CreateFixture(circle1, 5.0f);

			b2BodyDef bd2;
			bd2.type = b2BodyType.b2_dynamicBody;
			bd2.position.Set(10.0f, 8.0f);
			b2Body body2 = m_world.CreateBody(bd2);
			body2.CreateFixture(box, 5.0f);

			b2BodyDef bd3;
			bd3.type = b2BodyType.b2_dynamicBody;
			bd3.position.Set(10.0f, 6.0f);
			b2Body body3 = m_world.CreateBody(bd3);
			body3.CreateFixture(circle2, 5.0f);

			auto jd1 = scoped!b2RevoluteJointDef;
			jd1.Initialize(body2, body1, bd1.position);
			b2Joint joint1 = m_world.CreateJoint(jd1);

			auto jd2 = scoped!b2RevoluteJointDef;
			jd2.Initialize(body2, body3, bd3.position);
			b2Joint joint2 = m_world.CreateJoint(jd2);

			auto jd4 = scoped!b2GearJointDef;
			jd4.bodyA = body1;
			jd4.bodyB = body3;
			jd4.joint1 = joint1;
			jd4.joint2 = joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			m_world.CreateJoint(jd4);
		}

		{
			auto circle1 = scoped!b2CircleShape;
			circle1.m_radius = 1.0f;

			auto circle2 = scoped!b2CircleShape;
			circle2.m_radius = 2.0f;
			
			auto box = scoped!b2PolygonShape;
			box.SetAsBox(0.5f, 5.0f);

			b2BodyDef bd1;
			bd1.type = b2BodyType.b2_dynamicBody;
			bd1.position.Set(-3.0f, 12.0f);
			b2Body body1 = m_world.CreateBody(bd1);
			body1.CreateFixture(circle1, 5.0f);

			auto jd1 = scoped!b2RevoluteJointDef;
			jd1.bodyA = ground;
			jd1.bodyB = body1;
			jd1.localAnchorA = ground.GetLocalPoint(bd1.position);
			jd1.localAnchorB = body1.GetLocalPoint(bd1.position);
			jd1.referenceAngle = body1.GetAngle() - ground.GetAngle();
			m_joint1 = cast(b2RevoluteJoint)m_world.CreateJoint(jd1);

			b2BodyDef bd2;
			bd2.type = b2BodyType.b2_dynamicBody;
			bd2.position.Set(0.0f, 12.0f);
			b2Body body2 = m_world.CreateBody(bd2);
			body2.CreateFixture(circle2, 5.0f);

			auto jd2 = scoped!b2RevoluteJointDef;
			jd2.Initialize(ground, body2, bd2.position);
			m_joint2 = cast(b2RevoluteJoint)m_world.CreateJoint(jd2);

			b2BodyDef bd3;
			bd3.type = b2BodyType.b2_dynamicBody;
			bd3.position.Set(2.5f, 12.0f);
			b2Body body3 = m_world.CreateBody(bd3);
			body3.CreateFixture(box, 5.0f);

			auto jd3 = scoped!b2PrismaticJointDef;
			jd3.Initialize(ground, body3, bd3.position, b2Vec2(0.0f, 1.0f));
			jd3.lowerTranslation = -5.0f;
			jd3.upperTranslation = 5.0f;
			jd3.enableLimit = true;

			m_joint3 = cast(b2PrismaticJoint)m_world.CreateJoint(jd3);

			auto jd4 = scoped!b2GearJointDef;
			jd4.bodyA = body1;
			jd4.bodyB = body2;
			jd4.joint1 = m_joint1;
			jd4.joint2 = m_joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			m_joint4 = cast(b2GearJoint)m_world.CreateJoint(jd4);

			auto jd5 = scoped!b2GearJointDef;
			jd5.bodyA = body2;
			jd5.bodyB = body3;
			jd5.joint1 = m_joint2;
			jd5.joint2 = m_joint3;
			jd5.ratio = -1.0f / circle2.m_radius;
			m_joint5 = cast(b2GearJoint)m_world.CreateJoint(jd5);
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

		float32 ratio, value;
		
		ratio = m_joint4.GetRatio();
		value = m_joint1.GetJointAngle() + ratio * m_joint2.GetJointAngle();
		m_debugDraw.DrawString(5, m_textLine, "theta1 + %4.2f * theta2 = %4.2f", cast(float) ratio, cast(float) value);
		m_textLine += 15;

		ratio = m_joint5.GetRatio();
		value = m_joint2.GetJointAngle() + ratio * m_joint3.GetJointTranslation();
		m_debugDraw.DrawString(5, m_textLine, "theta2 + %4.2f * delta = %4.2f", cast(float) ratio, cast(float) value);
		m_textLine += 15;
	}

	static Test Create()
	{
		return new Gears;
	}

	b2RevoluteJoint m_joint1;
	b2RevoluteJoint m_joint2;
	b2PrismaticJoint m_joint3;
	b2GearJoint m_joint4;
	b2GearJoint m_joint5;
}
