/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

module box2d.dynamics.b2MouseJoint;

import box2d.common.b2Math;
import box2d.dynamics.b2Joint;
import box2d.dynamics.b2TimeStep;

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
class b2MouseJointDef : b2JointDef
{
	this()
	{
		type = b2JointType.e_mouseJoint;
	}

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	b2Vec2 target = {0.0f, 0.0f};

	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	float32 maxForce = 0.0f;

	/// The response speed.
	float32 frequencyHz = 5.0f;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float32 dampingRatio = 0.7f;
};

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
class b2MouseJoint : b2Joint
{
public:

	/// Implements b2Joint.
	override b2Vec2 GetAnchorA() const
	{
		return m_targetA;
	}

	/// Implements b2Joint.
	override b2Vec2 GetAnchorB() const
	{
		return m_bodyB.GetWorldPoint(m_localAnchorB);
	}

	/// Implements b2Joint.
	override b2Vec2 GetReactionForce(float32 inv_dt) const
	{
		return inv_dt * m_impulse;
	}

	/// Implements b2Joint.
	override float32 GetReactionTorque(float32 inv_dt) const
	{
		return inv_dt * 0.0f;
	}

	/// Use this to update the target point.
	void SetTarget(b2Vec2 target)
	{
		if (m_bodyB.IsAwake() == false)
		{
			m_bodyB.SetAwake(true);
		}
		m_targetA = target;
	}
	b2Vec2 GetTarget() const
	{
		return m_targetA;
	}

	/// Set/get the maximum force in Newtons.
	void SetMaxForce(float32 force)
	{
		m_maxForce = force;
	}
	float32 GetMaxForce() const
	{
		return m_maxForce;
	}

	/// Set/get the frequency in Hertz.
	void SetFrequency(float32 hz)
	{
		m_frequencyHz = hz;
	}
	float32 GetFrequency() const
	{
		return m_frequencyHz;
	}

	/// Set/get the damping ratio (dimensionless).
	void SetDampingRatio(float32 ratio)
	{
		m_dampingRatio = ratio;
	}
	float32 GetDampingRatio() const
	{
		return m_dampingRatio;
	}

	/// The mouse joint does not support dumping.
	override void Dump() { b2Log("Mouse joint dumping is not supported.\n"); }

	/// Implement b2Joint::ShiftOrigin
	override void ShiftOrigin(b2Vec2 newOrigin)
	{
		m_targetA -= newOrigin;
	}

protected:
	package this(b2MouseJointDef def)
	{
		assert(def.target.IsValid());
		assert(b2IsValid(def.maxForce) && def.maxForce >= 0.0f);
		assert(b2IsValid(def.frequencyHz) && def.frequencyHz >= 0.0f);
		assert(b2IsValid(def.dampingRatio) && def.dampingRatio >= 0.0f);

		super(def);

		m_targetA = def.target;
		m_localAnchorB = b2MulT(m_bodyB.GetTransform(), m_targetA);

		m_maxForce = def.maxForce;
		m_impulse.SetZero();

		m_frequencyHz = def.frequencyHz;
		m_dampingRatio = def.dampingRatio;

		m_beta = 0.0f;
		m_gamma = 0.0f;
	}

	override void InitVelocityConstraints(b2SolverData data)
	{
		m_indexB = m_bodyB.m_islandIndex;
		m_localCenterB = m_bodyB.m_sweep.localCenter;
		m_invMassB = m_bodyB.m_invMass;
		m_invIB = m_bodyB.m_invI;

		b2Vec2 cB = data.positions[m_indexB].c;
		float32 aB = data.positions[m_indexB].a;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float32 wB = data.velocities[m_indexB].w;

		b2Rot qB = b2Rot(aB);

		float32 mass = m_bodyB.GetMass();

		// Frequency
		float32 omega = 2.0f * b2_pi * m_frequencyHz;

		// Damping coefficient
		float32 d = 2.0f * mass * m_dampingRatio * omega;

		// Spring stiffness
		float32 k = mass * (omega * omega);

		// magic formulas
		// gamma has units of inverse mass.
		// beta has units of inverse time.
		float32 h = data.step.dt;
		assert(d + h * k > b2_epsilon);
		m_gamma = h * (d + h * k);
		if (m_gamma != 0.0f)
		{
			m_gamma = 1.0f / m_gamma;
		}
		m_beta = h * k * m_gamma;

		// Compute the effective mass matrix.
		m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		b2Mat22 K;
		K.ex.x = m_invMassB + m_invIB * m_rB.y * m_rB.y + m_gamma;
		K.ex.y = -m_invIB * m_rB.x * m_rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = m_invMassB + m_invIB * m_rB.x * m_rB.x + m_gamma;

		m_mass = K.GetInverse();

		m_C = cB + m_rB - m_targetA;
		m_C *= m_beta;

		// Cheat with some damping
		wB *= 0.98f;

		if (data.step.warmStarting)
		{
			m_impulse *= data.step.dtRatio;
			vB += m_invMassB * m_impulse;
			wB += m_invIB * b2Cross(m_rB, m_impulse);
		}
		else
		{
			m_impulse.SetZero();
		}

		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
	override void SolveVelocityConstraints(b2SolverData data)
	{
		b2Vec2 vB = data.velocities[m_indexB].v;
		float32 wB = data.velocities[m_indexB].w;

		// Cdot = v + cross(w, r)
		b2Vec2 Cdot = vB + b2Cross(wB, m_rB);
		b2Vec2 impulse = b2Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));

		b2Vec2 oldImpulse = m_impulse;
		m_impulse += impulse;
		float32 maxImpulse = data.step.dt * m_maxForce;
		if (m_impulse.LengthSquared() > maxImpulse * maxImpulse)
		{
			m_impulse *= maxImpulse / m_impulse.Length();
		}
		impulse = m_impulse - oldImpulse;

		vB += m_invMassB * impulse;
		wB += m_invIB * b2Cross(m_rB, impulse);

		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
	override bool SolvePositionConstraints(b2SolverData data)
	{
		return true;
	}

	b2Vec2 m_localAnchorB;
	b2Vec2 m_targetA;
	float32 m_frequencyHz;
	float32 m_dampingRatio;
	float32 m_beta;
	
	// Solver shared
	b2Vec2 m_impulse;
	float32 m_maxForce;
	float32 m_gamma;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterB;
	float32 m_invMassB;
	float32 m_invIB;
	b2Mat22 m_mass;
	b2Vec2 m_C;
}
