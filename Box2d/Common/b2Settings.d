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

module box2d.common.b2Settings;

import std.stdio;

alias byte		int8;
alias short		int16;
alias int		int32;
alias ubyte		uint8;
alias ushort	uint16;
alias uint		uint32;
alias float		float32;
alias double	float64;

const b2_maxFloat	= float.max;
const b2_epsilon	= float.epsilon;
const b2_pi			= 3.14159265359f;

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
const b2_maxManifoldPoints = 2;

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
const b2_maxPolygonVertices	= 8;

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
const b2_aabbExtension = 0.1f;

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
const b2_aabbMultiplier	= 2.0f;

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
const b2_linearSlop	= 0.005f;

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
const b2_angularSlop = 2.0f / 180.0f * b2_pi;

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
const b2_polygonRadius = 2.0f * b2_linearSlop;

/// Maximum number of sub-steps per contact in continuous physics simulation.
const b2_maxSubSteps = 8;


// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
const b2_maxTOIContacts	= 32;

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
const b2_velocityThreshold = 1.0f;

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
const b2_maxLinearCorrection = 0.2f;

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
const b2_maxAngularCorrection = 8.0f / 180.0f * b2_pi;

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
const b2_maxTranslation	= 2.0f;
const b2_maxTranslationSquared = b2_maxTranslation * b2_maxTranslation;

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
const b2_maxRotation = 0.5f * b2_pi;
const b2_maxRotationSquared	= b2_maxRotation * b2_maxRotation;

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
const b2_baumgarte		= 0.2f;
const b2_toiBaugarte	= 0.75f;


// Sleep

/// The time that a body must be still before it will go to sleep.
const b2_timeToSleep = 0.5f;

/// A body cannot sleep if its linear velocity is above this tolerance.
const b2_linearSleepTolerance = 0.01f;

/// A body cannot sleep if its angular velocity is above this tolerance.
const b2_angularSleepTolerance = 2.0f / 180.0f * b2_pi;

// Memory Allocation

/// Implement this function to use your own memory allocator.
void* b2Alloc(int32 size);

/// If you implement b2Alloc, you should also implement this function.
void b2Free(void* mem);

/// Logging function.
alias writefln b2Log;

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
struct b2Version
{
	int32 major;		///< significant changes
	int32 minor;		///< incremental changes
	int32 revision;		///< bug fixes
}

/// Current version.
b2Version b2_version;
