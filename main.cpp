#include "GL/freeglut.h"
#include <cmath>
#include <iostream>
#include <set>
#include <sstream>
#include <vector>
#include "extra.h"
#include "Ray.h"
#include "camera.h"
#include <Eigen/Geometry>
#include <surface_mesh/Surface_mesh.h>
#include "headers\Deformablemesh.h"
#include "headers\TriangleIntersect.h"


using namespace std;
using namespace surface_mesh;
using namespace Eigen;

//Surface mesh and related color array
Surface_mesh *mesh;
DeformableMesh *deformableMesh;
vector<Vector3f> colors;
Vector3f hoveredIntersectionPoint = Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);

//States to select triangles for mesh deformation or rotate them
bool isModeDeformedSelection = false;
bool isModeFixedSelecton = false;
bool isModeLoop = false;
bool isModeRotateX = false;
bool isModeRotateY = false;
bool isModeRotateZ = false;
float selectionRadius = 0.2;

//Angles for rotation
int xAxisAngle = 0;
int yAxisAngle = 0;
int zAxisAngle = 0;

//Vectors to store triangles currently listed for deformation
vector<int> deformedSelectedVertices;
vector<int> fixedSelectedVertices;
void deform(Vector3f angles);

// You will need more global variables to implement color and position changes
int colorCounter = 0;

float left_right = 1.0f;
float up_down = 1.0f;

bool spin = false;
float angle = 0.0f;

bool colorChangeFlag = false;
bool isChanging = false;
int nextColor = 0;


//Define the camera and mouse related constants
Camera camera;
bool gMousePressed = false;
int width = 600;
int height = 600;

//Axis display list
GLuint gAxisList;

//Display text in openGL
void displayString(float x, float y, string &text, Vector3f color) {

	glMatrixMode(GL_PROJECTION);
	glPushMatrix(); 
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glDisable(GL_DEPTH_TEST);

	glDisable(GL_LIGHTING);
	glColor3f(color.x(), color.y(), color.z());
	glRasterPos2f(x, y);

	for (int i = 0; i < text.size();i++) {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,text[i]);
	} 
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST); // Turn depth testing back on
	

	glMatrixMode(GL_PROJECTION);
	glPopMatrix(); 
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

bool isBacking = false;
void loopAngleUpdate(int x) {

	if (xAxisAngle == 360) {
		isBacking = true;
	}
	else if (xAxisAngle == 0) {
		isBacking = false;
	}

	if (isModeLoop) {
		xAxisAngle = isBacking ? xAxisAngle - 5 : xAxisAngle + 5;
		deform(Vector3f(xAxisAngle, yAxisAngle, zAxisAngle));
		glutPostRedisplay();
	}

	glutTimerFunc(100, loopAngleUpdate, 0);
}

//Function to monitor rotation
void setAngleForRotation(int incr) {
	if (isModeRotateX) {
		xAxisAngle += incr;

		if (xAxisAngle < 0) {
			xAxisAngle += 360;
		} else if (xAxisAngle > 360) {
			xAxisAngle -= 360;
		}
	}
	else if (isModeRotateY) {

		yAxisAngle += incr;

		if (yAxisAngle < 0) {
			yAxisAngle += 360;
		}
		else if (yAxisAngle > 360) {
			yAxisAngle -= 360;
		}
	}
	else if (isModeRotateZ) {

		zAxisAngle += incr;

		if (zAxisAngle < 0) {
			zAxisAngle += 360;
		}
		else if (zAxisAngle > 360) {
			zAxisAngle -= 360;
		}
	}

	deform(Vector3f(xAxisAngle, yAxisAngle, zAxisAngle));

}

//Scroll event handler
void mouseWheel(int button, int dir, int x, int y) {

	isModeLoop = false;

	//Scroll up
	if (dir > 0) {
		setAngleForRotation(5);
	}
	//Scroll down
	else {
		setAngleForRotation(-5);
	}
}

//Called when mouse is moved without any buttons pressed
void passiveMouseFunc(int x, int y) {

	Ray r = camera.generateRay(x, y);
	float dist;
	hoveredIntersectionPoint = TriangleIntersect::intersect(r, 0.001,dist,mesh);

	Surface_mesh::Face_container container = deformableMesh->mesh.faces();
	Surface_mesh::Face_iterator face_iter;
	Surface_mesh::Vertex_around_face_circulator vafc, vafc_end;
	int f_i = 0;

	for (face_iter = deformableMesh->mesh.faces_begin(); face_iter != mesh->faces_end(); ++face_iter) {
		vafc = mesh->vertices(*face_iter);
		vafc_end = vafc;

		do {
			Surface_mesh::Vertex v = *vafc;
			Vector3f p = mesh->position(v);

			float dist = (p - hoveredIntersectionPoint).norm();
			if (dist < selectionRadius) {
				if (isModeDeformedSelection) {
					if (find(deformedSelectedVertices.begin(), deformedSelectedVertices.end(), v.idx()) == deformedSelectedVertices.end()) {
						deformedSelectedVertices.push_back(v.idx());
					}
				}
				else if (isModeFixedSelecton) {
					if (find(fixedSelectedVertices.begin(), fixedSelectedVertices.end(), v.idx()) == fixedSelectedVertices.end()) {
						fixedSelectedVertices.push_back(v.idx());
					}
				}
			}
		} while (++vafc != vafc_end);
		f_i++;
	}
	
	glutPostRedisplay();
}

//  Called when mouse button is pressed.
void mouseFunc(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		gMousePressed = true;

		switch (button)
		{
		case GLUT_LEFT_BUTTON:
			camera.MouseClick(Camera::LEFT, x, y);
			break;
		case GLUT_MIDDLE_BUTTON:
			camera.MouseClick(Camera::MIDDLE, x, y);
			break;
		case GLUT_RIGHT_BUTTON:
			camera.MouseClick(Camera::RIGHT, x, y);
		default:
			break;
		}
	}
	else
	{
		camera.MouseRelease(x, y);
		gMousePressed = false;
	}
}

// Called when mouse is moved while button pressed.
void motionFunc(int x, int y)
{
	camera.MouseDrag(x, y);

	glutPostRedisplay();
}

//This function is called whenever a "Normal" key is released
void keyboardUpFunc(unsigned char key, int x, int y) {
	switch (key)
	{
	case 'd':
		isModeDeformedSelection = false;
		break;
	case 'i':
		isModeFixedSelecton = false;
		break;
	case 'x':
		isModeRotateX = false;
		break;
	case 'y':
		isModeRotateY = false;
		break;
	case 'z':
		isModeRotateZ = false;
		break;
	}
}
// This function is called whenever a "Normal" key press is received.
void keyboardFunc(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27: // Escape key
		exit(0);
		break;
	case 'r':
		deformedSelectedVertices.clear();
		fixedSelectedVertices.clear();

		for (int i = 0; i < colors.size(); i++) {
			colors[i] = Vector3f(0.7, 0.7, 0.7);
		}
		break;
	case 'd':
		isModeDeformedSelection = isModeFixedSelecton ? false : true;
		break;
	case 'i':
		isModeFixedSelecton = isModeDeformedSelection ? false : true;
		break;
	case 'x':
		isModeRotateX = true;
		break;
	case 'y':
		isModeRotateY = true;
		break;
	case 'z':
		isModeRotateZ = true;
		break;
	case 'l':
		isModeLoop = !isModeLoop;
		break;
	default:
		cout << "Unhandled key press " << key << "." << endl;
	}

	// this will refresh the screen so that the user sees the color change
	glutPostRedisplay();
}

// This function is called whenever a "Special" key press is received.
// Right now, it's handling the arrow keys.
void specialFunc(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_UP:
		// add code to change light position
		up_down = up_down + 0.5;
		break;
	case GLUT_KEY_DOWN:
		// add code to change light position
		up_down = up_down - 0.5;
		break;
	case GLUT_KEY_LEFT:
		// add code to change light position
		left_right = left_right - 0.5;
		break;
	case GLUT_KEY_RIGHT:
		// add code to change light position
		left_right = left_right + 0.5;
		break;
	}

	// this will refresh the screen so that the user sees the light position
	glutPostRedisplay();
}

// rotates shapes by 1 degree every 25 milliseconds
void spinTimer(int value) {
	if (spin) {
		angle++;
		if (angle > 360) {  // angles kept small for float precision issues
			angle -= 360;
		}
	}
	glutPostRedisplay();
	glutTimerFunc(25, spinTimer, 0);
}


// Draw the axes
void makeDisplayLists()
{
	gAxisList = glGenLists(1);

	// Compile the display lists

	glNewList(gAxisList, GL_COMPILE);
	{
		// Save current state of OpenGL
		glPushAttrib(GL_ALL_ATTRIB_BITS);

		// This is to draw the axes when the mouse button is down
		glDisable(GL_LIGHTING);
		glLineWidth(3);
		glPushMatrix();
		glScaled(5.0, 5.0, 5.0);
		glBegin(GL_LINES);
		glColor4f(1, 0.5, 0.5, 1); glVertex3d(0, 0, 0); glVertex3d(1, 0, 0);
		glColor4f(0.5, 1, 0.5, 1); glVertex3d(0, 0, 0); glVertex3d(0, 1, 0);
		glColor4f(0.5, 0.5, 1, 1); glVertex3d(0, 0, 0); glVertex3d(0, 0, 1);

		glColor4f(0.5, 0.5, 0.5, 1);
		glVertex3d(0, 0, 0); glVertex3d(-1, 0, 0);
		glVertex3d(0, 0, 0); glVertex3d(0, -1, 0);
		glVertex3d(0, 0, 0); glVertex3d(0, 0, -1);

		glEnd();
		glPopMatrix();

		glPopAttrib();
	}
	glEndList();

}

// This function is responsible for displaying the object.
void drawScene(void)
{
	int i;

	// Clear the rendering window
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);  // Current matrix affects objects positions
	glLoadIdentity();              // Initialize to the identity

	// Define specular color and shininess
	GLfloat specColor[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat shininess[] = { 100.0 };

	// Note that the specular color and shininess can stay constant
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specColor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	// Set light properties

	// Light color (RGBA)
	GLfloat Lt0diff[] = { 1.0,1.0,1.0,1.0 };
	// Light position
	GLfloat Lt0pos[] = { left_right, up_down, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_DIFFUSE, Lt0diff);
	glLightfv(GL_LIGHT0, GL_POSITION, Lt0pos);

	camera.ApplyModelview();

	glRotatef(angle, 0, 1, 0);

	// This GLUT method draws a teapot.  You should replace
	// it with code which draws the object you loaded.
	// glutSolidTeapot(1.0);

	glEnable(GL_COLOR_MATERIAL);

	Surface_mesh::Face_container container = deformableMesh->mesh.faces();
	Surface_mesh::Face_iterator face_iter;
	Surface_mesh::Vertex_around_face_circulator vafc, vafc_end;
	int f_i = 0;

	for (face_iter = deformableMesh->mesh.faces_begin(); face_iter != mesh->faces_end(); ++face_iter) {
		vafc = mesh->vertices(*face_iter);
		vafc_end = vafc;

		glBegin(GL_TRIANGLES);

		do {
			Surface_mesh::Vertex v = *vafc;
			Vector3f p = mesh->position(v);
			Vector3f n = mesh->compute_vertex_normal(v);

			glNormal3d(n.x(),n.y(), n.z());
			glColor3f(colors[f_i].x(), colors[f_i].y(), colors[f_i].z());
			glVertex3d(p.x(), p.y(), p.z());

		} while (++vafc != vafc_end);

		glEnd();


		vafc = mesh->vertices(*face_iter);
		vafc_end = vafc;

		do {
			Surface_mesh::Vertex v = *vafc;
			Vector3f p = mesh->position(v);

			glPushMatrix();

			float dist = (p - hoveredIntersectionPoint).norm();
			if (dist < selectionRadius) {
				glColor3f(1, 0, 1);
			}
			else if (find(deformedSelectedVertices.begin(), deformedSelectedVertices.end(), v.idx()) != deformedSelectedVertices.end()) {
				glColor3f(0, 0, 1);
			}
			else if (find(fixedSelectedVertices.begin(), fixedSelectedVertices.end(), v.idx()) != fixedSelectedVertices.end()) {
				glColor3f(0, 1, 0);
			}
			else {
				glColor3f(1, 1, 1);
			}
			glTranslatef(p.x(), p.y(), p.z());
			glutSolidSphere(0.02, 5, 5);
			glPopMatrix();

		} while (++vafc != vafc_end);


		f_i++;
	}

	glDisable(GL_COLOR_MATERIAL);

	displayString(-0.9, 0.85, "X " + to_string(xAxisAngle),Vector3f(1,1,1));
	displayString(-0.9, 0.75, "Y " + to_string(yAxisAngle),Vector3f(1,1,1));
	displayString(-0.9, 0.65, "Z " + to_string(zAxisAngle),Vector3f(1,1,1));
	displayString(-0.9, -0.75, "Handles " + to_string(deformedSelectedVertices.size()),Vector3f(0,0,1));
	displayString(-0.9, -0.85, "Fixed " + to_string(fixedSelectedVertices.size()),Vector3f(0,1,0));

	glPushMatrix();
	glTranslated(camera.GetCenter().x(), camera.GetCenter().y(), camera.GetCenter().z());
	glCallList(gAxisList);
	glPopMatrix();

	// Dump the image to the screen.
	glutSwapBuffers();
}

// Initialize OpenGL's rendering modes
void initRendering()
{
	//Surface_mesh mesh;
	//mesh->read("plane_4x4.obj");
	//DeformableMesh dm = DeformableMesh(mesh);

	//// instantiate a Surface_mesh object
	//Surface_mesh mesh;

	//// read a mesh specified as the first command line argument
	//mesh->read(argv[1]);

	//// eqn 6 test
	//vector<int> fixed_ids;
	//vector<int> handle_ids;

	//for (int i = 0; i < 4; i++) {
	//	fixed_ids.push_back(i);
	//}
	//
	//for (int i = 12; i < 16; i++) {
	//	handle_ids.push_back(i);
	//}
	//
	//VectorXf init = VectorXf::Zero(mesh->vertices_size());
	//VectorXf out;
	//eqn6(mesh, fixed_ids, handle_ids, init, 0.52f, out);
	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);   // Depth testing must be turned on
	glEnable(GL_LIGHTING);     // Enable lighting calculations
	glEnable(GL_LIGHT0);       // Turn on light #0.
	glEnable(GL_NORMALIZE);
}

// Called when the window is resized
// w, h - width and height of the window in pixels.
 void reshapeFunc(int w, int h)
    {

	    width = w;
		height = h;
        camera.SetDimensions(w,h);

        camera.SetViewport(0,0,w,h);
        camera.ApplyViewport();

        // Set up a perspective view, with square aspect ratio
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        camera.SetPerspective(50);
        camera.ApplyPerspective();
    }

void loadInput(int argc, char **argv)
{
	cout << "Reading " << argv[1] << endl;
	Surface_mesh *m = new Surface_mesh();
	m->read(argv[1]);
	deformableMesh = new DeformableMesh(*m);

	mesh = &deformableMesh->mesh;

	for (int i = 0; i < mesh->n_faces(); i++) {
		colors.push_back(Vector3f(0.7, 0.7, 0.7));
	}
}

//Function to deform the vertices (DOM & JOHSI, yall do stuff here)
void deform(Vector3f angles) {
	
	VectorXf init = VectorXf::Zero(mesh->vertices_size(), 1);
	cout << init.rows() << " " << init.cols() << endl;

	deformableMesh->deform_mesh(deformedSelectedVertices, fixedSelectedVertices, init, (angles.x() * M_PI / 180));
	//eqn6(mesh, fixed_ids, handle_ids, init, 0.52f, out);
}

// Main routine.
// Set up OpenGL, define the callbacks and start the main loop
int main(int argc, char** argv)
{
	cout << "Reading " << argv[1] << endl;
	loadInput(argc,argv);
	glutInit(&argc, argv);

	// We're going to animate it, so double buffer 
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// Initial parameters for window position and size
	glutInitWindowPosition(60, 60);
	glutInitWindowSize(width, height);

	camera.SetDimensions(600, 600);

	camera.SetDistance(10);
	camera.SetCenter(Vector3f(0, 0, 0));

	glutCreateWindow("Assignment 0");

	// Initialize OpenGL parameters.
	initRendering();

	// Set up callback functions for key presses
	glutKeyboardFunc(keyboardFunc); // Handles "normal" ascii symbols
	glutSpecialFunc(specialFunc);   // Handles "special" keyboard keys
	glutKeyboardUpFunc(keyboardUpFunc);

	 // Set up the callback function for resizing windows
	glutReshapeFunc(reshapeFunc);

	// Call this whenever window needs redrawing
	glutDisplayFunc(drawScene);

	glutTimerFunc(25, spinTimer, 0);

	//Setup callback function for mouse & movement
	glutMouseFunc(mouseFunc);
	glutMotionFunc(motionFunc);
	glutPassiveMotionFunc(passiveMouseFunc);
	glutMouseWheelFunc(mouseWheel);

	//Setup for loop function
	glutTimerFunc(100, loopAngleUpdate, 0);
	
	//Draw the axes
	makeDisplayLists();

	// Start the main loop.  glutMainLoop never returns.
	glutMainLoop();

	return 0;	// This line is never reached.
}