/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

module box2d.rope.b2Rope;

import box2d.common.b2Math;
import box2d.common.b2Draw;

struct b2RopeDef
{	
	///
	b2Vec2[] vertices;

	///
	float32[] masses;

	///
	b2Vec2 gravity = {0.0f, 0.0f};

	///
	float32 damping = 0.1f;

	/// Stretching stiffness
	float32 k2 = 0.9f;

	/// Bending stiffness. Values above 0.5 can make the simulation blow up.
	float32 k3 = 0.1f;
};

/// 
class b2Rope
{
public:
	///
	void Initialize(const b2RopeDef def)
	{
		assert(def.vertices.length >= 3);
		m_ps.length = def.vertices.length;
		m_p0s.length = def.vertices.length;
		m_vs.length = def.vertices.length;
		m_ims.length = def.vertices.length;

		foreach(i; 0 .. m_ps.length)
		{
			m_ps[i] = def.vertices[i];
			m_p0s[i] = def.vertices[i];
			m_vs[i].SetZero();

			float32 m = def.masses[i];
			if (m > 0.0f)
			{
				m_ims[i] = 1.0f / m;
			}
			else
			{
				m_ims[i] = 0.0f;
			}
		}

		m_Ls.length = m_ps.length - 1; 

		foreach(i; 0 .. m_Ls.length)
		{
			b2Vec2 p1 = m_ps[i];
			b2Vec2 p2 = m_ps[i+1];
			m_Ls[i] = b2Distance(p1, p2);
		}

		m_as.length = m_ps.length - 2;
		foreach(i; 0 .. m_as.length)
		{
			b2Vec2 p1 = m_ps[i];
			b2Vec2 p2 = m_ps[i + 1];
			b2Vec2 p3 = m_ps[i + 2];

			b2Vec2 d1 = p2 - p1;
			b2Vec2 d2 = p3 - p2;

			float32 a = b2Cross(d1, d2);
			float32 b = b2Dot(d1, d2);

			m_as[i] = b2Atan2(a, b);
		}

		m_gravity = def.gravity;
		m_damping = def.damping;
		m_k2 = def.k2;
		m_k3 = def.k3;
	}

	///
	void Step(float32 h, int32 iterations)
	{
		if (h == 0.0)
		{
			return;
		}

		float32 d = exp(- h * m_damping);

		foreach(i; 0 .. m_ps.length)
		{
			m_p0s[i] = m_ps[i];
			if (m_ims[i] > 0.0f)
			{
				m_vs[i] += h * m_gravity;
			}
			m_vs[i] *= d;
			m_ps[i] += h * m_vs[i];

		}

		foreach(i; 0 .. iterations)
		{
			SolveC2();
			SolveC3();
			SolveC2();
		}

		float32 inv_h = 1.0f / h;
		foreach(i; 0 .. m_vs.length)
		{
			m_vs[i] = inv_h * (m_ps[i] - m_p0s[i]);
		}
	}

	///
	int32 GetVertexCount() const
	{
		return m_ps.length;
	}

	///
	const(b2Vec2[]) GetVertices() const
	{
		return m_ps;
	}

	///
	void Draw(b2Draw draw) const
	{
		b2Color c = b2Color(0.4f, 0.5f, 0.7f);

		foreach(i; 0 .. m_ps.length - 1)
		{
			draw.DrawSegment(m_ps[i], m_ps[i+1], c);
		}
	}

	///
	void SetAngle(float32 angle)
	{
		foreach(ref a; m_as)
			a = angle;
	}

private:

	void SolveC2()
	{
		foreach(i; 0 .. m_ps.length - 1)
		{
			b2Vec2 p1 = m_ps[i];
			b2Vec2 p2 = m_ps[i + 1];

			b2Vec2 d = p2 - p1;
			float32 L = d.Normalize();

			float32 im1 = m_ims[i];
			float32 im2 = m_ims[i + 1];

			if (im1 + im2 == 0.0f)
			{
				continue;
			}

			float32 s1 = im1 / (im1 + im2);
			float32 s2 = im2 / (im1 + im2);

			p1 -= m_k2 * s1 * (m_Ls[i] - L) * d;
			p2 += m_k2 * s2 * (m_Ls[i] - L) * d;

			m_ps[i] = p1;
			m_ps[i + 1] = p2;
		}
	}
	void SolveC3()
	{
		foreach(i; 0 .. m_ps.length - 2)
		{
			b2Vec2 p1 = m_ps[i];
			b2Vec2 p2 = m_ps[i + 1];
			b2Vec2 p3 = m_ps[i + 2];

			float32 m1 = m_ims[i];
			float32 m2 = m_ims[i + 1];
			float32 m3 = m_ims[i + 2];

			b2Vec2 d1 = p2 - p1;
			b2Vec2 d2 = p3 - p2;

			float32 L1sqr = d1.LengthSquared();
			float32 L2sqr = d2.LengthSquared();

			if (L1sqr * L2sqr == 0.0f)
			{
				continue;
			}

			float32 a = b2Cross(d1, d2);
			float32 b = b2Dot(d1, d2);

			float32 angle = b2Atan2(a, b);

			b2Vec2 Jd1 = (-1.0f / L1sqr) * d1.Skew();
			b2Vec2 Jd2 = (1.0f / L2sqr) * d2.Skew();

			b2Vec2 J1 = -Jd1;
			b2Vec2 J2 = Jd1 - Jd2;
			b2Vec2 J3 = Jd2;

			float32 mass = m1 * b2Dot(J1, J1) + m2 * b2Dot(J2, J2) + m3 * b2Dot(J3, J3);
			if (mass == 0.0f)
			{
				continue;
			}

			mass = 1.0f / mass;

			float32 C = angle - m_as[i];

			while (C > b2_pi)
			{
				angle -= 2 * b2_pi;
				C = angle - m_as[i];
			}

			while (C < -b2_pi)
			{
				angle += 2.0f * b2_pi;
				C = angle - m_as[i];
			}

			float32 impulse = - m_k3 * mass * C;

			p1 += (m1 * impulse) * J1;
			p2 += (m2 * impulse) * J2;
			p3 += (m3 * impulse) * J3;

			m_ps[i] = p1;
			m_ps[i + 1] = p2;
			m_ps[i + 2] = p3;
		}
	}

	b2Vec2[] m_ps;
	b2Vec2[] m_p0s;
	b2Vec2[] m_vs;

	float32[] m_ims;

	float32[] m_Ls;
	float32[] m_as;

	b2Vec2 m_gravity;
	float32 m_damping;

	float32 m_k2;
	float32 m_k3;
}
