#include <iostream>
#include <fstream>
#include <sstream>
#include <GL/glut.h>
#include "ParticleSystem.h"
#include "Timer.h"
#include "Vector3.h"
#include "IRenderable.h"
#include "ppm.h"
#include <sstream>
#include <cstdio>
#include "ClothMesh.h"
#include "ConvexifierMesh.h"
#include "extgl.h"
#include "Mesh.h"
#include <vector>
#include <algorithm>
#include <functional>
#include <string>
#include <iterator>
using namespace std;
using ajg::physics::Vector3;

ajg::physics::ParticleSystem particleSystem;
std::vector<ajg::physics::IRenderable*> objects;
Timer timer;
int v_height,v_width;
bool pause = false;
bool sphere = false;
float theta = 0;
float phi = 0;
float rad = 7;
int frame = 0;
const int nprogs = 3;
int cur_prog = 1;

GLuint vp,fp;
GLuint fprogs[10];
float mx,my;
GLuint tex[2];

ajg::physics::ClothMesh* cm =0;
bool use_programs = true;

bool PRINT_FRAME = false;
bool OUTPUT_FILE = false;


struct pixel
{
	float r,g,b,a;
};

struct image
{
	int width;
	int height;
	vector<pixel> pixels;
};

image MakeTexture1()
{
	image i;
	i.width = 256;
	i.height = 256;
	i.pixels.resize(256*256);
	for(int j=0;j<256;j++)
	{
		for(int k=0;k<256;k++)
		{
			pixel p;
			p.a = 1.0;
			p.r = 0.5*sin(j*2.0) + 0.5;
			p.g = 0.5*sin(j*2.0) + 0.5;
			p.b = 0.5*sin(j*2.0) + 0.5;

			i.pixels[j*256+k]=p;
		}
	}
	return i;
}

image MakeTexture2()
{
	image i;
	i.width = 256;
	i.height = 256;
	i.pixels.resize(256*256);
	for(int j=0;j<256;j++)
	{
		for(int k=0;k<256;k++)
		{
			pixel p;
			p.a = 1.0;
			p.r = 0.5*cos(j*4.0)*sin(k*4.0) + 0.5;
			p.g = 0.5*sin(j*4.0)*sin(k*4.0) + 0.5;
			p.b = 0.5*sin(j*4.0)*sin(k*4.0) + 0.5;

			i.pixels[j*256+k]=p;
		}
	}
	return i;
}
void DisplayFunc()
{

	switch(glGetError())
	{
		case GL_NO_ERROR: break;
		case GL_INVALID_ENUM: std::cout<<"An unacceptable value is specified for an enumerated argument. The offending function is ignored, having no side effect other than to set the error flag.  \n";
		case GL_INVALID_VALUE: std::cout<<"A numeric argument is out of range. The offending function is ignored, having no side effect other than to set the error flag.  \n";
		case GL_INVALID_OPERATION: std::cout<<"The specified operation is not allowed in the current state. The offending function is ignored, having no side effect other than to set the error flag.  \n";
		case GL_STACK_OVERFLOW: std::cout<<"This function would cause a stack overflow. The offending function is ignored, having no side effect other than to set the error flag.  \n";
		case GL_STACK_UNDERFLOW: std::cout<<"This function would cause a stack underflow. The offending function is ignored, having no side effect other than to set the error flag.  \n";
		case GL_OUT_OF_MEMORY: std::cout<<"There is not enough memory left to execute the function. The state of OpenGL is undefined, except for the state of the error flags, after this error is recorded.\n";	
	}

	float time = 0;
	while(time < 1.0 / 60.0)
	{
		time = timer.GetElapsedTime();
	}
	timer.Reset();

	static float frameTime;
	static int count = 0;
	count++;
	frameTime += time;
	
	if(frameTime >= 1.0)
	{
		std::cout << "Framerate: " << count / frameTime << std::endl;
		count = 0;
		frameTime = 0;
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float eyex = rad * sinf(theta) * cosf(phi);
	float eyey = rad * sinf(phi);
	float eyez = rad * cosf(theta) * cosf(phi);
	gluLookAt(eyex,eyey,eyez,0,0,0,0,1,0);

	float pos[]={0,0,4,1};
	pos[0] = mx;
	pos[1] = my;

	glLightfv(GL_LIGHT0,GL_POSITION,pos);

	float dark[]={0.2f,0.2f,0.2f,1};
	float red[]={0.7f,0.0f,0.0f,1};
	float black[]={0,0,0,1};
	float white[]={0.8f,0.8f,0.8f,1};
	float one[]={1,1,1,1};
	glLightfv(GL_LIGHT0,GL_AMBIENT,dark);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,red);

	glMateriali(GL_FRONT,GL_SHININESS,128);
	glMaterialfv(GL_FRONT,GL_AMBIENT,red);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,red);
	glMaterialfv(GL_FRONT,GL_SPECULAR,black);
//	glMaterialfv(GL_FRONT,GL_SPECULAR,white);

//	glLightf(GL_LIGHT0,GL_CONSTANT_ATTENUATION,1);
//	glLightf(GL_LIGHT0,GL_LINEAR_ATTENUATION,0);
//	glLightf(GL_LIGHT0,GL_QUADRATIC_ATTENUATION ,0.02);

	if(!pause)
	{
		particleSystem.Update(1.0 / 60.0);
		
		if(cm) cm->Update(1.0 / 100.0);
	}
	
	float red2[]={1,0,0,1};
//	glMaterialfv(GL_FRONT,GL_DIFFUSE,red2);
	std::for_each(objects.begin(),objects.end(),std::mem_fun(&ajg::physics::IRenderable::Render));

	glutSwapBuffers();
	glutPostRedisplay();
	
	if(OUTPUT_FILE || PRINT_FRAME)
	{
		PRINT_FRAME = false;
		static int frame = 0;
		ostringstream out;
		out << "movie/cloth" << frame << ".ppm";
		frame++;
		FILE* fp;
		fopen_s(&fp, out.str().c_str(), "w");
		
		DumpPPM(fp,0,v_width,v_height);
		fclose(fp);
	}
}

void ReshapeFunc(int width, int height)
{
	if(height==0)
		height=1;

	v_height=height;
	v_width=width;

	glViewport(0,0,width,height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45,float(width)/float(height),0.1,10);
	glMatrixMode(GL_MODELVIEW);
}

void KeyboardFunc(unsigned char key, int x, int y) 
{
 	switch(key) 
	{
	case 'a':
		theta += 0.1;
		break;
	case 'd':
		theta -= 0.1;
		break;
	case 'w':
		phi += 0.1;
		break;
	case 's':
		phi -= 0.1;
		break;
	case 'q': exit(0);
		break;
	case ' ':
		pause = !pause;
		break;
	case 'l':
		glPolygonMode(GL_FRONT,GL_LINE);
		break;
	case 'f':
		glPolygonMode(GL_FRONT,GL_FILL);
		break;
	case 'o':
		sphere = !sphere;
		break;
	case 'p':
		use_programs = !use_programs;
		break;
		
	case 'k':
		PRINT_FRAME = true;
		break;
	default:
		if(isdigit(key))
		{
			if(use_programs)
			{
				int n = key - '0';
				if(n == 0)
				{
					glDisable(GL_VERTEX_PROGRAM_ARB);
					glDisable(GL_FRAGMENT_PROGRAM_ARB);
				}
				else if(n <= nprogs)
				{
					cur_prog = n;	
					glBindProgramARB(GL_FRAGMENT_PROGRAM_ARB,fprogs[cur_prog]);

					glEnable(GL_VERTEX_PROGRAM_ARB);
					glEnable(GL_FRAGMENT_PROGRAM_ARB);
				}
			}
		}
		break;
	}
}


void MouseFunc(int button, int state, int x, int y)
{
	y=v_height-y;
		
	double model[16];
	double proj[16];
	int view[4];

	glGetDoublev(GL_MODELVIEW_MATRIX,model);
	glGetDoublev(GL_PROJECTION_MATRIX,proj);
	glGetIntegerv(GL_VIEWPORT,view);
	double ox,oy,oz;
	double wx,wy,wz;
	gluProject(0,0,0,model,proj,view,&wx,&wy,&wz);
	gluUnProject(x,y,wz,model,proj,view,&ox,&oy,&oz);
	static Vector3 position,oldPosition;
	static bool clicked = false;
	
	oz = 0;
	Vector3 click(ox,oy,oz);
	return;
	std::cerr<<click<<std::endl;
	if (state == 0)  //button down
	{
		if (button == GLUT_LEFT_BUTTON) 
		{
			oldPosition = position;
			//position = Vector3(4*(x-v_width/2)/double(v_width),4*(y-v_height/2)/double(v_height),0);
			position = click;
			clicked = ! clicked;
			if(!clicked)
			{
				Vector3 right = position - oldPosition;
				right[1] = right[2] = 0;
				Vector3 up = position - oldPosition;
				up[0] = up[2] = 0;
				Vector3 out = right.Cross(up);
				out.Normalize();
				out *= right.Magnitude();
				up = -out;
				position = oldPosition;
				cout<<position<<endl<<right<<endl<<up<<endl;
				objects.push_back(cm = new ajg::physics::ClothMesh(position,position + right,position + right + up,position + up,10,10));
			}
		
		}
		else if (button == GLUT_MIDDLE_BUTTON) 
		{
		
		}
		else if (button == GLUT_RIGHT_BUTTON) 
		{
		}
	}
	else if (state == 1) //button up
	{
		if (button == GLUT_LEFT_BUTTON) 
		{
			
		}
		else if(button==GLUT_MIDDLE_BUTTON)
		{

		}
		else if(button==GLUT_RIGHT_BUTTON)
		{
		
		}
	}
}


void MotionFunc(int x, int y)
{
	y=v_height-y;
	mx = x;
	my = y;
	mx = 8*float(x-v_width/2)/float(v_width);
	my = 8*float(y-v_height/2)/float(v_height);
}

int main(int argc,char **argv)
{	
	glutInit(&argc,argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA); // set display mode
	glutInitWindowSize(400,300); // set window size
	glutInitWindowPosition(0,0); // set window position on screen
	glutCreateWindow("Cloth"); // set window title
	
	glutMotionFunc(MotionFunc);
	glutMouseFunc(MouseFunc); // register the mouse action function
	glutKeyboardFunc(KeyboardFunc); // register the keyboard action function
	glutDisplayFunc(DisplayFunc); //register the redraw function
	glutReshapeFunc(ReshapeFunc);
	extgl_Initialize();
	
	if (!extgl_Extensions.ARB_vertex_program)
	{
		cout << "ARB_vertex_program is not supported" << endl;
		use_programs = false;
	}
	if(!extgl_Extensions.ARB_fragment_program)
	{
		cout <<"ARB_fragment_program is not supported" <<endl;
		use_programs = false;
	}

	glShadeModel(GL_SMOOTH);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
//	glEnable(GL_TEXTURE_2D);

	if(use_programs)
	{
		glEnable(GL_VERTEX_PROGRAM_ARB);
		glEnable(GL_FRAGMENT_PROGRAM_ARB);

		glGenProgramsARB(1,&vp);
		glGenProgramsARB(1,&fp);
		glBindProgramARB(GL_VERTEX_PROGRAM_ARB,vp);
		glBindProgramARB(GL_FRAGMENT_PROGRAM_ARB,fp);

		int pos;

		string vprog;
		ifstream vpin("vp.txt");
		copy(istreambuf_iterator<char>(vpin),istreambuf_iterator<char>(),back_inserter(vprog));
		glProgramStringARB(GL_VERTEX_PROGRAM_ARB,GL_PROGRAM_FORMAT_ASCII_ARB,(GLsizei)vprog.find("END")+3,vprog.c_str());
		glGetIntegerv(GL_PROGRAM_ERROR_POSITION_ARB,&pos);
		if(pos != -1)
		{
			cout << glGetString(GL_PROGRAM_ERROR_STRING_ARB) << endl;
			cout << "Invalid vertex program"<<endl;
			cout << "Error at " << pos << endl;
			exit(0);
		}

		for(int i=1;i<=nprogs;i++)
		{
			glGenProgramsARB(1,&fprogs[i]);
			glBindProgramARB(GL_FRAGMENT_PROGRAM_ARB,fprogs[i]);

			string fprog;
			char file[1000];
			sprintf_s(file,"fp%d.txt",i);
			ifstream fpin(file);
			copy(istreambuf_iterator<char>(fpin),istreambuf_iterator<char>(),back_inserter(fprog));
			glProgramStringARB(GL_FRAGMENT_PROGRAM_ARB,GL_PROGRAM_FORMAT_ASCII_ARB,(GLsizei)fprog.find("END")+3,fprog.c_str());
			glGetIntegerv(GL_PROGRAM_ERROR_POSITION_ARB,&pos);
			if(pos != -1)
			{
				cout << glGetString(GL_PROGRAM_ERROR_STRING_ARB) << endl;
				cout << "Invalid fragment program"<<endl;
				cout << "Error at "<<pos<<endl;
				exit(0);
			}
		}
		glBindProgramARB(GL_FRAGMENT_PROGRAM_ARB,fprogs[cur_prog]);
	}

	glGenTextures(1,tex);
	image image1 = MakeTexture1();
	glBindTexture(GL_TEXTURE_2D, tex[0]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, 4, image1.width, image1.height, 0, GL_RGBA, GL_FLOAT, &image1.pixels[0]);
	

	Vector3 position(-2.5,2.5,0);
	Vector3 right(5,0,0);
	Vector3 up(0,0,5);
	objects.push_back(cm = new ajg::physics::ClothMesh(position,position + right,position + right + up,position + up,40,40));

	timer.Reset();
	glutMainLoop();

	if(use_programs)
	{
		glDeleteProgramsARB(1,&vp);
		glDeleteProgramsARB(1,&fp);
	}
	return 0;
}
