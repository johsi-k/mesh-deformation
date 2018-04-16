#include "GL/freeglut.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>
#include "extra.h"
#include "Ray.h"
#include "camera.h"
#include <Eigen/Geometry>
#include <surface_mesh/Surface_mesh.h>
#include "..\headers\eqn6.h"
#include "headers\DeformableMesh.h"


using namespace std;
using namespace surface_mesh;
using namespace Eigen;

//Surface mesh and related color array
Surface_mesh mesh;
vector<Vector3f> colors;
int hoveredTriangleIndex = INT_MAX;

//States to select triangles for mesh deformation or rotate them
bool isModeDeformedSelection = false;
bool isModeInterpolationSelection = false;
bool isModeRotateX = false;
bool isModeRotateY = false;
bool isModeRotateZ = false;

//Angles for rotation
int xAxisAngle = 0;
int yAxisAngle = 0;
int zAxisAngle = 0;

//Vectors to store triangles currently listed for deformation
vector<int> deformedSelectionTriangles;
vector<int> interpolationSelectionTriangles;

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

//Axis display list
GLuint gAxisList;

Matrix3f generateMatrix3fFromVectors(Vector3f a, Vector3f b, Vector3f c) {
	Matrix <float,3,3,ColMajor> M;
	M <<
		a.x(), a.y(), a.z(),
		b.x(), b.y(), b.z(),
		c.x(), c.y(), c.z();
	return M;
}

int intersect(Ray r, int tmin) {
	int index = INT_MAX;
	float t = FLT_MAX;

	Surface_mesh::Face_container container = mesh.faces();
	Surface_mesh::Face_iterator face_iter;
	Surface_mesh::Vertex_around_face_circulator vafc, vafc_end;

	int f_i = 0;

	for (face_iter = mesh.faces_begin(); face_iter != mesh.faces_end(); ++face_iter) {
		vafc = mesh.vertices(*face_iter);
		vafc_end = vafc;

		Vector3f points[3];
		int i = 0;
		do {
			Surface_mesh::Vertex v = *vafc;
			points[i] = mesh.position(v);
			i++;

		} while (++vafc != vafc_end);

		Matrix3f t_m = generateMatrix3fFromVectors(points[0] - points[1], points[0] - points[2], points[0] - r.getOrigin());
		Matrix3f beta_m = generateMatrix3fFromVectors(points[0] - r.getOrigin(), points[0] - points[2], r.getDirection());
		Matrix3f gamma_m = generateMatrix3fFromVectors(points[0] - points[1], points[0] - r.getOrigin(), r.getDirection());

		Matrix3f A = generateMatrix3fFromVectors(points[0] - points[1], points[0] - points[2], r.getDirection());

		float beta = beta_m.determinant() / A.determinant();
		float gamma = gamma_m.determinant() / A.determinant();

		f_i++;

		if (beta < 0.0f || gamma < 0.0f) {
			continue;
		}

		if (beta + gamma > 1.0f) {
			continue;
		}

		float alpha = 1 - beta - gamma;
		float t_face = t_m.determinant() / A.determinant();

		if (t_face < t && t_face >= tmin) {
			t = t_face;
			index = f_i-1;
		}

	}
	return index;
}

//TODO: fill in rotation method
void rotate(Vector3f angle) {

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

	rotate(Vector3f(xAxisAngle, yAxisAngle, zAxisAngle));
}

//Scroll event handler
void mouseWheel(int button, int dir, int x, int y) {
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
	hoveredTriangleIndex = intersect(r, 0.001);

	if (hoveredTriangleIndex < INT_MAX) {
		if (isModeDeformedSelection) {
			if (std::find(deformedSelectionTriangles.begin(), deformedSelectionTriangles.end(), hoveredTriangleIndex) == deformedSelectionTriangles.end()) {
				if (std::find(interpolationSelectionTriangles.begin(), interpolationSelectionTriangles.end(), hoveredTriangleIndex) == interpolationSelectionTriangles.end()) {
					deformedSelectionTriangles.push_back(hoveredTriangleIndex);
					colors[hoveredTriangleIndex] = Vector3f(0, 0, 1);
				}
			}
		}
		else if (isModeInterpolationSelection) {
			if (std::find(interpolationSelectionTriangles.begin(), interpolationSelectionTriangles.end(), hoveredTriangleIndex) == interpolationSelectionTriangles.end()) {
				if (std::find(deformedSelectionTriangles.begin(), deformedSelectionTriangles.end(), hoveredTriangleIndex) == deformedSelectionTriangles.end()) {
					interpolationSelectionTriangles.push_back(hoveredTriangleIndex);
					colors[hoveredTriangleIndex] = Vector3f(0, 1, 0);
				}
			}
		}
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
		isModeInterpolationSelection = false;
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
		deformedSelectionTriangles.clear();
		interpolationSelectionTriangles.clear();

		for (int i = 0; i < colors.size(); i++) {
			colors[i] = Vector3f(0.7, 0.7, 0.7);
		}
		break;
	case 'd':
		isModeDeformedSelection = isModeInterpolationSelection ? false : true;
		break;
	case 'i':
		isModeInterpolationSelection = isModeDeformedSelection ? false : true;
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

	Surface_mesh::Face_container container = mesh.faces();
	Surface_mesh::Face_iterator face_iter;
	Surface_mesh::Vertex_around_face_circulator vafc, vafc_end;
	int f_i = 0;

	for (face_iter = mesh.faces_begin(); face_iter != mesh.faces_end(); ++face_iter) {
		vafc = mesh.vertices(*face_iter);
		vafc_end = vafc;

		glBegin(GL_TRIANGLES);

		do {
			Surface_mesh::Vertex v = *vafc;
			Vector3f p = mesh.position(v);
			Vector3f n = mesh.compute_vertex_normal(v);

			glNormal3d(n.x(),n.y(), n.z());
			if (hoveredTriangleIndex == f_i) {
				glColor3f(1, 0.5, 0);
			}
			else {
				glColor3f(colors[f_i].x(), colors[f_i].y(), colors[f_i].z());
			}
			glVertex3d(p.x(), p.y(), p.z());

		} while (++vafc != vafc_end);

		glEnd();

		f_i++;
	}

	glDisable(GL_COLOR_MATERIAL);

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
	Surface_mesh mesh;
	mesh.read("plane_4x4.obj");
	DeformableMesh dm = DeformableMesh(mesh);
	dm.test();

	//// instantiate a Surface_mesh object
	//Surface_mesh mesh;

	//// read a mesh specified as the first command line argument
	//mesh.read(argv[1]);

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
	//VectorXf init = VectorXf::Zero(mesh.vertices_size());
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
	// load the OBJ file here
	mesh.read(argv[1]);

	for (int i = 0; i < mesh.n_faces(); i++) {
		colors.push_back(Vector3f(0.7, 0.7, 0.7));
	}
}

//Function to deform the vertices (DOM & JOHSI, yall do stuff here)
void deform() {
	vector<int> fixed_ids;
	vector<int> handle_ids;

	for (int i = 0; i < 4; i++) {
		fixed_ids.push_back(i);
	}
	
	for (int i = 12; i < 16; i++) {
		handle_ids.push_back(i);
	}
	
	VectorXf init = VectorXf::Zero(mesh.vertices_size());
	VectorXf out;
	eqn6(mesh, fixed_ids, handle_ids, init, 0.52f, out);
}


// Main routine.
// Set up OpenGL, define the callbacks and start the main loop
int main(int argc, char** argv)
{
	loadInput(argc,argv);
	deform();
	glutInit(&argc, argv);

	// We're going to animate it, so double buffer 
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// Initial parameters for window position and size
	glutInitWindowPosition(60, 60);
	glutInitWindowSize(360, 360);

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
	
	//Draw the axes
	makeDisplayLists();

	// Start the main loop.  glutMainLoop never returns.
	glutMainLoop();

	return 0;	// This line is never reached.
}