import std.stdio : writeln, writefln; 
import derelict.opengl3.gl3; 
import derelict.glfw3.glfw3;
import std.conv;

import box2d.all;
import box2d.testbed.framework.test;
import box2d.testbed.tests.addpair;
import box2d.testbed.tests.applyforce;
import box2d.testbed.tests.bodytypes;
import box2d.testbed.tests.breakable;
import box2d.testbed.tests.bullettest;
import box2d.testbed.tests.car;
import box2d.testbed.tests.cantilever;
import box2d.testbed.tests.chain;
import box2d.testbed.tests.charactercollision;
import box2d.testbed.tests.gears;
import box2d.testbed.tests.pyramid;
import box2d.testbed.tests.bridge;
import box2d.testbed.tests.dominos;
import box2d.testbed.tests.motorjoint;
import box2d.testbed.tests.pulleys;
import box2d.testbed.tests.rope;
import box2d.testbed.tests.ropejoint;
import box2d.testbed.tests.pinball;

import graphics.opengl.shader;
import graphics.opengl.imm2d;

Test g_Test;

bool running = true;
extern(C) int OnClose(GLFWwindow window) 
{
	running = false; 
	return 0;
}

extern(C) void OnKey(GLFWwindow window, int key, int state)
{
	if (state == GLFW_PRESS)
		g_Test.Keyboard(cast(ubyte)key);
	else
		g_Test.KeyboardUp(cast(ubyte)key);
}

struct Color
{
	float r, g, b;

	this(float _r, float _g, float _b)
	{
		r = _r;
		g = _g;
		b = _b;
	}
};

void main() 
{ 
	//try {
	DerelictGL3.load(); 
	DerelictGLFW3.load(); 

	if(!glfwInit()) 
		throw new Exception("glfwInit failure: " ~ to!string(glfwErrorString(glfwGetError()))); 
	scope(exit) glfwTerminate(); 

	glfwOpenWindowHint(GLFW_OPENGL_VERSION_MAJOR, 3); 
	glfwOpenWindowHint(GLFW_OPENGL_VERSION_MINOR, 3); 
	glfwOpenWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); 
	glfwOpenWindowHint(GLFW_FSAA_SAMPLES, 4);

	auto window = glfwOpenWindow(0, 0, GLFW_WINDOWED, "Hello DerelictGLFW3", null); 
	if(!window) 
		throw new Exception("Failed to create window."); 

	DerelictGL3.reload(); 

	writefln("OpenGL version string: %s", to!string(glGetString(GL_VERSION))); 
	writefln("OpenGL renderer string: %s", to!string(glGetString(GL_RENDERER))); 
	writefln("OpenGL vendor string: %s", to!string(glGetString(GL_VENDOR))); 

	g_Test = new Pyramid;
	Settings settings;

	settings.hz = 150.0f;
	settings.velocityIterations = 8;
	settings.positionIterations = 3;

	//settings.enableContinuous = 1;

	float now = glfwGetTime();

	auto vshader = new VShader(import("vshader.glsl"));
	auto fshader = new FShader(import("fshader.glsl"));
	auto program = new Program(vshader, fshader);

	//GLuint vao;
	//glGenVertexArrays(1, &vao);
	//glBindVertexArray(vao);
	//GLuint vbo;
	//glGenBuffers(1, &vbo);
	//glBindBuffer(GL_ARRAY_BUFFER, vbo);
	//glBufferData(GL_ARRAY_BUFFER, verts.length * verts[0].sizeof, verts.ptr, GL_STATIC_DRAW);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, null); 
	//glEnableVertexAttribArray(0);

	g_Imm2d = new Imm2d;

	int lastmousebut = 0;

    glfwSwapInterval(1);
	glfwSetWindowCloseCallback(&OnClose);
	//glfwSetCharCallback(&OnKey);
	glfwSetKeyCallback(&OnKey);
	while (running)
	{
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		glViewport(0, 0, width, height);

		glClearColor(.2,.1,.3,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		program.Bind();		

		int mousex, mousey;
		glfwGetMousePos(window, &mousex, &mousey);
		b2Vec2 mousepos = b2Vec2(mousex * (2.0f / width) - .5f, 
								 1.5f - mousey * (2.0f / height));
		mousepos *= 10.0f;
		int mousebut = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1);
		int mousebutdif = mousebut ^ lastmousebut;
		if (lastmousebut & mousebutdif)
		{
			g_Test.MouseUp(mousepos);
		}
		else if (mousebut & mousebutdif)
		{
			if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
				g_Test.ShiftMouseDown(mousepos);
			else
				g_Test.MouseDown(mousepos);
		}
		g_Test.MouseMove(mousepos);
		lastmousebut = mousebut;		

		float then = now;
		now = glfwGetTime();
		float timeStep = now - then;

		g_Test.Step(settings);

		g_Imm2d.Draw();

		glfwSwapBuffers();
		glfwPollEvents();
		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			running = false;		
	}
}
