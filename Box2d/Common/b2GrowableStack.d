/*
* Copyright (c) 2010 Erin Catto http://www.box2d.org
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

module box2d.common.b2GrowableStack;
import box2d.common.b2Settings;

/// This is a growable LIFO stack with an initial capacity of N.
/// If the stack size exceeds the initial capacity, the heap is used
/// to increase the size of the stack.
struct b2GrowableStack(T)
{
	void Push(T element)
	{		
		if (m_count == m_stack.length)
            m_stack.length *= 2;		
		m_stack[m_count++] = element;
	}

	T Pop()
	{
		assert(m_count > 0);
		return m_stack[--m_count];
	}

	int32 GetCount()
	{
		return m_count;
	}

	T[] m_stack;
	int32 m_count = 0;
}

b2GrowableStack!(T) b2MakeGrowableStack(T)(T[] array)
{
	b2GrowableStack!(T) r = {array, 0};
	return r;
}
