/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

module box2d.testbed.tests.car;

import box2d.common.b2Math;
import box2d.collision.shapes.b2EdgeShape;
import box2d.collision.shapes.b2CircleShape;
import box2d.collision.shapes.b2PolygonShape;
import box2d.dynamics.b2Body;
import box2d.dynamics.b2Fixture;
import box2d.dynamics.b2RevoluteJoint;
import box2d.dynamics.b2WheelJoint;
import box2d.testbed.framework.test;

// This is a fun demo that shows off the wheel joint
class Car : Test
{
public:
	this()
	{		
		m_hz = 4.0f;
		m_zeta = 0.7f;
		m_speed = 50.0f;

		b2Body ground;
		{
			b2BodyDef bd;
			ground = m_world.CreateBody(bd);

			auto shape = new b2EdgeShape;

			b2FixtureDef fd;
			fd.shape = shape;
			fd.density = 0.0f;
			fd.friction = 0.6f;

			shape.Set(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));
			ground.CreateFixture(fd);

			float32[10] hs = [0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f];

			float32 x = 20.0f, y1 = 0.0f, dx = 5.0f;

			foreach(y2; hs)
			{
				shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground.CreateFixture(fd);
				y1 = y2;
				x += dx;
			}

			foreach(y2; hs)
			{
				shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground.CreateFixture(fd);
				y1 = y2;
				x += dx;
			}

			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground.CreateFixture(fd);

			x += 80.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground.CreateFixture(fd);

			x += 40.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 10.0f, 5.0f));
			ground.CreateFixture(fd);

			x += 20.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground.CreateFixture(fd);

			x += 40.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x, 20.0f));
			ground.CreateFixture(fd);
		}

		// Teeter
		{
			b2BodyDef bd;
			bd.position.Set(140.0f, 1.0f);
			bd.type = b2BodyType.b2_dynamicBody;
			b2Body b = m_world.CreateBody(bd);

			auto box = new b2PolygonShape;
			box.SetAsBox(10.0f, 0.25f);
			b.CreateFixture(box, 1.0f);

			auto jd = new b2RevoluteJointDef;
			jd.Initialize(ground, b, b.GetPosition());
			jd.lowerAngle = -8.0f * b2_pi / 180.0f;
			jd.upperAngle = 8.0f * b2_pi / 180.0f;
			jd.enableLimit = true;
			m_world.CreateJoint(jd);

			b.ApplyAngularImpulse(100.0f);
		}

		// Bridge
		{
			int32 N = 20;
			auto shape = new b2PolygonShape;
			shape.SetAsBox(1.0f, 0.125f);

			b2FixtureDef fd;
			fd.shape = shape;
			fd.density = 1.0f;
			fd.friction = 0.6f;

			auto jd = new b2RevoluteJointDef;

			b2Body prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
				b2BodyDef bd;
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(161.0f + 2.0f * i, -0.125f);
				b2Body b = m_world.CreateBody(bd);
				b.CreateFixture(fd);

				b2Vec2 anchor = {160.0f + 2.0f * i, -0.125f};
				jd.Initialize(prevBody, b, anchor);
				m_world.CreateJoint(jd);

				prevBody = b;
			}

			b2Vec2 anchor = {160.0f + 2.0f * N, -0.125f};
			jd.Initialize(prevBody, ground, anchor);
			m_world.CreateJoint(jd);
		}

		// Boxes
		{
			auto box = new b2PolygonShape;
			box.SetAsBox(0.5f, 0.5f);

			b2Body b;
			b2BodyDef bd;
			bd.type = b2BodyType.b2_dynamicBody;

			bd.position.Set(230.0f, 0.5f);
			b = m_world.CreateBody(bd);
			b.CreateFixture(box, 0.5f);

			bd.position.Set(230.0f, 1.5f);
			b = m_world.CreateBody(bd);
			b.CreateFixture(box, 0.5f);

			bd.position.Set(230.0f, 2.5f);
			b = m_world.CreateBody(bd);
			b.CreateFixture(box, 0.5f);

			bd.position.Set(230.0f, 3.5f);
			b = m_world.CreateBody(bd);
			b.CreateFixture(box, 0.5f);

			bd.position.Set(230.0f, 4.5f);
			b = m_world.CreateBody(bd);
			b.CreateFixture(box, 0.5f);
		}

		// Car
		{
			auto chassis = new b2PolygonShape;
			b2Vec2[8] vertices;
			vertices[0].Set(-1.5f, -0.5f);
			vertices[1].Set(1.5f, -0.5f);
			vertices[2].Set(1.5f, 0.0f);
			vertices[3].Set(0.0f, 0.9f);
			vertices[4].Set(-1.15f, 0.9f);
			vertices[5].Set(-1.5f, 0.2f);
			chassis.Set(vertices);

			auto circle = new b2CircleShape;
			circle.m_radius = 0.4f;

			b2BodyDef bd;
			bd.type = b2BodyType.b2_dynamicBody;
			bd.position.Set(0.0f, 1.0f);
			m_car = m_world.CreateBody(bd);
			m_car.CreateFixture(chassis, 1.0f);

			b2FixtureDef fd;
			fd.shape = circle;
			fd.density = 1.0f;
			fd.friction = 0.9f;

			bd.position.Set(-1.0f, 0.35f);
			m_wheel1 = m_world.CreateBody(bd);
			m_wheel1.CreateFixture(fd);

			bd.position.Set(1.0f, 0.4f);
			m_wheel2 = m_world.CreateBody(bd);
			m_wheel2.CreateFixture(fd);

			auto jd = new b2WheelJointDef;
			b2Vec2 axis = {0.0f, 1.0f};

			jd.Initialize(m_car, m_wheel1, m_wheel1.GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring1 = cast(b2WheelJoint)m_world.CreateJoint(jd);

			jd.Initialize(m_car, m_wheel2, m_wheel2.GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring2 = cast(b2WheelJoint)m_world.CreateJoint(jd);
		}
	}

	override void Keyboard(ubyte key)
	{
		switch (key)
		{
		case 'a':
			m_spring1.SetMotorSpeed(m_speed);
			break;

		case 's':
			m_spring1.SetMotorSpeed(0.0f);
			break;

		case 'd':
			m_spring1.SetMotorSpeed(-m_speed);
			break;

		case 'q':
			m_hz = b2Max(0.0f, m_hz - 1.0f);
			m_spring1.SetSpringFrequencyHz(m_hz);
			m_spring2.SetSpringFrequencyHz(m_hz);
			break;

		case 'e':
			m_hz += 1.0f;
			m_spring1.SetSpringFrequencyHz(m_hz);
			m_spring2.SetSpringFrequencyHz(m_hz);
			break;

		default:
			break;
		}
	}

	override void Step(Settings settings)
	{
		m_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		m_textLine += 15;
		m_debugDraw.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_textLine += 15;

		settings.viewCenter.x = m_car.GetPosition().x;
		super.Step(settings);
	}

	static Test Create()
	{
		return new Car;
	}

	b2Body m_car;
	b2Body m_wheel1;
	b2Body m_wheel2;

	float32 m_hz;
	float32 m_zeta;
	float32 m_speed;
	b2WheelJoint m_spring1;
	b2WheelJoint m_spring2;
}
