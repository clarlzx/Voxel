#include <iostream>
#include "mesh.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#define M_PI 3.141592654
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#endif

using namespace std;

//#define M_PI 3.141592654

myObjType myObj;

// global variable

bool m_Smooth = FALSE;
bool m_Highlight = TRUE;
bool m_DrawBoundary = FALSE;
bool m_DrawVoxels = FALSE;
bool m_DrawObject = TRUE;
GLfloat angle = 0;   /* in degrees */
GLfloat angle2 = 0;   /* in degrees */
GLfloat zoom = 1.0;
int mouseButton = 0;
int moving, startx, starty;

#define NO_OBJECT 4;
int current_object = 0;

using namespace std;

void setupLighting()
{
	glShadeModel(GL_SMOOTH);
	glEnable(GL_NORMALIZE);

	// Lights, material properties
    GLfloat	ambientProperties[]  = {0.7f, 0.7f, 0.7f, 1.0f};
	GLfloat	diffuseProperties[]  = {0.8f, 0.8f, 0.8f, 1.0f};
    GLfloat	specularProperties[] = {1.0f, 1.0f, 1.0f, 1.0f};
	GLfloat lightPosition[] = {-100.0f,100.0f,100.0f,1.0f};
	
    glClearDepth( 1.0 );

	glLightfv( GL_LIGHT0, GL_POSITION, lightPosition);
	
    glLightfv( GL_LIGHT0, GL_AMBIENT, ambientProperties);
    glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuseProperties);
    glLightfv( GL_LIGHT0, GL_SPECULAR, specularProperties);
    glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 0.0);

	// Default : lighting
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

}



void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	float red[] = { 1.0f, 0.0f, 0.0f, 1.0f };

	if (m_DrawBoundary) {
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);

		glPushMatrix();
		glTranslatef(0.0, 0.0, 0.01); // Translate in z-direction to avoid z-fighting with faces
		gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);
		glRotatef(angle2, 1.0, 0.0, 0.0);
		glRotatef(angle, 0.0, 1.0, 0.0);
		glScalef(zoom, zoom, zoom);
		myObj.drawBoundaries();
		glPopMatrix();
	}

	float mat_specular[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	float mat_ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	float mat_ambient_color[] = { 0.8f, 0.8f, 0.2f, 1.0f };
	float mat_diffuse[] = { 0.1f, 0.5f, 0.8f, 1.0f };
	float shininess = 20;
	float no_highlight[] = { 0.0f, 0.0f, 0.0f, 0.0f };

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

	if (m_Highlight) {
		glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
		glMaterialf(GL_FRONT, GL_SHININESS, shininess);
	}
	else {
		glMaterialfv(GL_FRONT, GL_SPECULAR, no_highlight);
		glMaterialf(GL_FRONT, GL_SHININESS, 0);
	}

	if (m_DrawVoxels) {
		glPushMatrix();
		gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);
		glRotatef(angle2, 1.0, 0.0, 0.0);
		glRotatef(angle, 0.0, 1.0, 0.0);
		glScalef(zoom, zoom, zoom);
		myObj.drawVoxels();
		glPopMatrix();
	}

	if (m_DrawObject) {
		glPushMatrix();
		gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);
		glRotatef(angle2, 1.0, 0.0, 0.0);
		glRotatef(angle, 0.0, 1.0, 0.0);
		glScalef(zoom, zoom, zoom);
		myObj.draw(m_Smooth);
		glPopMatrix();
	}

	glutSwapBuffers ();
}




void keyboard (unsigned char key, int x, int y)
{
	char filename[256];
	int newVoxelDepth;
	switch (key) {
	case 'p':
	case 'P':
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		break;			
	case 'w':
	case 'W':
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		break;			
	case 'v':
	case 'V':
		glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
		break;			
	case 's':
	case 'S':
		m_Smooth = !m_Smooth;
		break;
	case 'h':
	case 'H':
		m_Highlight = !m_Highlight;
		break;
	case 'b':
	case 'B':
		m_DrawBoundary = !m_DrawBoundary;
		break;
	case 'd':
	case 'D':
		m_DrawObject = !m_DrawObject;
		break;
	case 'm':
	case 'M':
		cout << "Voxel depth:";
		cin >> newVoxelDepth;
		myObj.setVoxelDepth(newVoxelDepth);
		break;
	case 'x':
	case 'X':
		m_DrawVoxels = !m_DrawVoxels;
		break;
	case 'o':
	case 'O':
		cout << "Enter the filename you want to write (only .obj):";
		cin >> filename;
		myObj.writeFile(filename);
		break;
	case '1':
	case '2':
	case '3':
	case '4':
		current_object = key - '1';
		break;

	case 'Q':
	case 'q':
		exit(0);
	break;

	default:
	break;
	}

	glutPostRedisplay();
}

void
mouse(int button, int state, int x, int y)
{
  if (state == GLUT_DOWN) {
	mouseButton = button;
    moving = 1;
    startx = x;
    starty = y;
  }
  if (state == GLUT_UP) {
	mouseButton = button;
    moving = 0;
  }
}

void motion(int x, int y)
{
  if (moving) {
	if(mouseButton==GLUT_LEFT_BUTTON)
	{
		angle = angle + (x - startx);
		angle2 = angle2 + (y - starty);
	}
	else zoom += ((y-starty)*0.001);
    startx = x;
    starty = y;
	glutPostRedisplay();
  }
  
}

int main(int argc, char **argv)
{
	int fileType;
	char filename[255];
	cout<<"CS3242 "<< endl<< endl;

	cout << "1: Read OBJ" << endl;
	cout << "2: Read STL" << endl;
	cout << "Enter the number of the file type to read:";
	cin >> fileType;
	myObj.checkFileType(fileType);

	cout << "Enter the filename you want to open:";
	cin >> filename;
	myObj.readFile(filename);



	//cout << "1-4: Draw different objects"<<endl;
	cout << "S: Toggle Smooth Shading"<<endl;
	cout << "H: Toggle Highlight"<<endl;
	cout << "B: Draw Boundary Edges" << endl;
	cout << "W: Draw Wireframe"<<endl;
	cout << "P: Draw Polygon"<<endl;
	cout << "V: Draw Vertices"<<endl;
	cout << "M: Set Voxel Depth (Recommend < 7)" << endl;
	cout << "X: Draw Voxels (Default depth = 4)" << endl;
	cout << "D: Draw Object" << endl;
	cout << "O: Write File" << endl;
	cout << "Q: Quit" <<endl<< endl;

	cout << "Left mouse click and drag: rotate the object"<<endl;
	cout << "Right mouse click and drag: zooming"<<endl;

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize (600, 600);
	glutInitWindowPosition (50, 50);
	glutCreateWindow ("CS3242 Assignment 3");
	glClearColor (1.0,1.0,1.0, 1.0);
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	setupLighting();
	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST); 
	glDepthMask(GL_TRUE);

    glMatrixMode(GL_PROJECTION);
    gluPerspective( /* field of view in degree */ 40.0,
  /* aspect ratio */ 1.0,
    /* Z near */ 1.0, /* Z far */ 80.0);
	glMatrixMode(GL_MODELVIEW);
	glutMainLoop();

	return 0;
}
