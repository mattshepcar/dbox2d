/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

module box2d.dynamics.b2MotorJoint;

import box2d.common.b2Math;
import box2d.dynamics.b2Body;
import box2d.dynamics.b2Joint;
import box2d.dynamics.b2TimeStep;

/// Motor joint definition.
class b2MotorJointDef : b2JointDef
{
	this()
	{
		type = b2JointType.e_motorJoint;
	}

	/// Initialize the bodies and offsets using the current transforms.
	void Initialize(b2Body bA, b2Body bB)
	{
		bodyA = bA;
		bodyB = bB;
		b2Vec2 xB = bodyB.GetPosition();
		linearOffset = bodyA.GetLocalPoint(xB);

		float32 angleA = bodyA.GetAngle();
		float32 angleB = bodyB.GetAngle();
		angularOffset = angleB - angleA;
	}

	/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
	b2Vec2 linearOffset = {0.0f, 0.0f};

	/// The bodyB angle minus bodyA angle in radians.
	float32 angularOffset = 0.0f;
	
	/// The maximum motor force in N.
	float32 maxForce = 1.0f;

	/// The maximum motor torque in N-m.
	float32 maxTorque = 1.0f;

	/// Position correction factor in the range [0,1].
	float32 correctionFactor = 0.3f;
};

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
class b2MotorJoint : b2Joint
{
public:
	override b2Vec2 GetAnchorA() const
	{
		return m_bodyA.GetPosition();	
	}
	override b2Vec2 GetAnchorB() const
	{
		return m_bodyB.GetPosition();
	}

	override b2Vec2 GetReactionForce(float32 inv_dt) const
	{
		return inv_dt * m_linearImpulse;
	}
	override float32 GetReactionTorque(float32 inv_dt) const
	{
		return inv_dt * m_angularImpulse;
	}

	/// Set/get the target linear offset, in frame A, in meters.
	void SetLinearOffset(b2Vec2 linearOffset)
	{
		if (linearOffset.x != m_linearOffset.x || linearOffset.y != m_linearOffset.y)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_linearOffset = linearOffset;
		}
	}
	b2Vec2 GetLinearOffset() const
	{
		return m_linearOffset;
	}

	/// Set/get the target angular offset, in radians.
	void SetAngularOffset(float32 angularOffset)
	{
		if (angularOffset != m_angularOffset)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_angularOffset = angularOffset;
		}
	}
	float32 GetAngularOffset() const
	{
		return m_angularOffset;
	}

	/// Set the maximum friction force in N.
	void SetMaxForce(float32 force)
	{
		assert(b2IsValid(force) && force >= 0.0f);
		m_maxForce = force;
	}

	/// Get the maximum friction force in N.
	float32 GetMaxForce() const
	{
		return m_maxForce;
	}

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(float32 torque)
	{
		assert(b2IsValid(torque) && torque >= 0.0f);
		m_maxTorque = torque;
	}

	/// Get the maximum friction torque in N*m.
	float32 GetMaxTorque() const
	{
		return m_maxTorque;
	}

	/// Dump to b2Log
	override void Dump()
	{
		int32 indexA = m_bodyA.m_islandIndex;
		int32 indexB = m_bodyB.m_islandIndex;

		b2Log("  b2MotorJointDef jd;\n");
		b2Log("  jd.bodyA = bodies[%d];\n", indexA);
		b2Log("  jd.bodyB = bodies[%d];\n", indexB);
		b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
		b2Log("  jd.linearOffset.Set(%.15lef, %.15lef);\n", m_linearOffset.x, m_linearOffset.y);
		b2Log("  jd.angularOffset = %.15lef;\n", m_angularOffset);
		b2Log("  jd.maxForce = %.15lef;\n", m_maxForce);
		b2Log("  jd.maxTorque = %.15lef;\n", m_maxTorque);
		b2Log("  jd.correctionFactor = %.15lef;\n", m_correctionFactor);
		b2Log("  joints[%d] = m_world.CreateJoint(&jd);\n", m_index);
	}

protected:
	package this(const b2MotorJointDef def)
	{
		super(def);
		m_linearOffset = def.linearOffset;
		m_angularOffset = def.angularOffset;

		m_linearImpulse.SetZero();
		m_angularImpulse = 0.0f;

		m_maxForce = def.maxForce;
		m_maxTorque = def.maxTorque;
		m_correctionFactor = def.correctionFactor;
	}

	override void InitVelocityConstraints(b2SolverData data)
	{
		m_indexA = m_bodyA.m_islandIndex;
		m_indexB = m_bodyB.m_islandIndex;
		m_localCenterA = m_bodyA.m_sweep.localCenter;
		m_localCenterB = m_bodyB.m_sweep.localCenter;
		m_invMassA = m_bodyA.m_invMass;
		m_invMassB = m_bodyB.m_invMass;
		m_invIA = m_bodyA.m_invI;
		m_invIB = m_bodyB.m_invI;

		b2Vec2 cA = data.positions[m_indexA].c;
		float32 aA = data.positions[m_indexA].a;
		b2Vec2 vA = data.velocities[m_indexA].v;
		float32 wA = data.velocities[m_indexA].w;

		b2Vec2 cB = data.positions[m_indexB].c;
		float32 aB = data.positions[m_indexB].a;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float32 wB = data.velocities[m_indexB].w;

		b2Rot qA = b2Rot(aA);
		b2Rot qB = b2Rot(aB);

		// Compute the effective mass matrix.
		m_rA = b2Mul(qA, -m_localCenterA);
		m_rB = b2Mul(qB, -m_localCenterB);

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		float32 mA = m_invMassA, mB = m_invMassB;
		float32 iA = m_invIA, iB = m_invIB;

		b2Mat22 K;
		K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
		K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

		m_linearMass = K.GetInverse();

		m_angularMass = iA + iB;
		if (m_angularMass > 0.0f)
		{
			m_angularMass = 1.0f / m_angularMass;
		}

		m_linearError = cB + m_rB - cA - m_rA - b2Mul(qA, m_linearOffset);
		m_angularError = aB - aA - m_angularOffset;

		if (data.step.warmStarting)
		{
			// Scale impulses to support a variable time step.
			m_linearImpulse *= data.step.dtRatio;
			m_angularImpulse *= data.step.dtRatio;

			b2Vec2 P = {m_linearImpulse.x, m_linearImpulse.y};
			vA -= mA * P;
			wA -= iA * (b2Cross(m_rA, P) + m_angularImpulse);
			vB += mB * P;
			wB += iB * (b2Cross(m_rB, P) + m_angularImpulse);
		}
		else
		{
			m_linearImpulse.SetZero();
			m_angularImpulse = 0.0f;
		}

		data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;
		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
	override void SolveVelocityConstraints(b2SolverData data)
	{
		b2Vec2 vA = data.velocities[m_indexA].v;
		float32 wA = data.velocities[m_indexA].w;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float32 wB = data.velocities[m_indexB].w;

		float32 mA = m_invMassA, mB = m_invMassB;
		float32 iA = m_invIA, iB = m_invIB;

		float32 h = data.step.dt;
		float32 inv_h = data.step.inv_dt;

		// Solve angular friction
		{
			float32 Cdot = wB - wA + inv_h * m_correctionFactor * m_angularError;
			float32 impulse = -m_angularMass * Cdot;

			float32 oldImpulse = m_angularImpulse;
			float32 maxImpulse = h * m_maxTorque;
			m_angularImpulse = b2Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_angularImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Solve linear friction
		{
			b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA) + inv_h * m_correctionFactor * m_linearError;

			b2Vec2 impulse = -b2Mul(m_linearMass, Cdot);
			b2Vec2 oldImpulse = m_linearImpulse;
			m_linearImpulse += impulse;

			float32 maxImpulse = h * m_maxForce;

			if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
			{
				m_linearImpulse.Normalize();
				m_linearImpulse *= maxImpulse;
			}

			impulse = m_linearImpulse - oldImpulse;

			vA -= mA * impulse;
			wA -= iA * b2Cross(m_rA, impulse);

			vB += mB * impulse;
			wB += iB * b2Cross(m_rB, impulse);
		}

		data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;
		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
	override bool SolvePositionConstraints(b2SolverData data)
	{
		return true;
	}

	// Solver shared
	b2Vec2 m_linearOffset;
	float32 m_angularOffset;
	b2Vec2 m_linearImpulse;
	float32 m_angularImpulse;
	float32 m_maxForce;
	float32 m_maxTorque;
	float32 m_correctionFactor;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	b2Vec2 m_linearError;
	float32 m_angularError;
	float32 m_invMassA;
	float32 m_invMassB;
	float32 m_invIA;
	float32 m_invIB;
	b2Mat22 m_linearMass;
	float32 m_angularMass;
}
