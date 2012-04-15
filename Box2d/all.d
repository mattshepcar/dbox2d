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

module box2d.all;

/**
\mainpage Box2D API Documentation

\section intro_sec Getting Started

For documentation please see http://box2d.org/documentation.html

For discussion please visit http://box2d.org/forum
*/

// These include files constitute the main Box2D API

public import box2d.common.b2Math;
public import box2d.common.b2Draw;
public import box2d.common.b2Timer;

public import box2d.collision.shapes.b2CircleShape;
public import box2d.collision.shapes.b2EdgeShape;
public import box2d.collision.shapes.b2ChainShape;
public import box2d.collision.shapes.b2PolygonShape;

public import box2d.collision.b2BroadPhase;
public import box2d.collision.b2Distance;
public import box2d.collision.b2DynamicTree;
public import box2d.collision.b2TimeOfImpact;

public import box2d.dynamics.b2Body;
public import box2d.dynamics.b2Fixture;
public import box2d.dynamics.b2WorldCallbacks;
public import box2d.dynamics.b2TimeStep;
public import box2d.dynamics.b2World;
public import box2d.dynamics.b2Contact;
public import box2d.dynamics.b2Joint;

public import box2d.dynamics.b2DistanceJoint;
public import box2d.dynamics.b2FrictionJoint;
public import box2d.dynamics.b2GearJoint;
public import box2d.dynamics.b2MotorJoint;
public import box2d.dynamics.b2MouseJoint;
public import box2d.dynamics.b2PrismaticJoint;
public import box2d.dynamics.b2PulleyJoint;
public import box2d.dynamics.b2RevoluteJoint;
public import box2d.dynamics.b2RopeJoint;
public import box2d.dynamics.b2WeldJoint;
public import box2d.dynamics.b2WheelJoint;

public import box2d.rope.b2Rope;
