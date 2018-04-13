#include "GL/freeglut.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>
#include "vecmath.h"
#include <surface_mesh/Surface_mesh.h>
#include "..\headers\eqn6.h"


using namespace std;
using namespace surface_mesh;

// Globals

//// This is the list of points (3D vectors)
//vector<Vector3f> vecv;
//
//// This is the list of normals (also 3D vectors)
//vector<Vector3f> vecn;
//
//// This is the list of faces (indices into vecv and vecn)
//vector<vector<unsigned>> vecf;
//
//
//// You will need more global variables to implement color and position changes
//int colorCounter = 0;
//
//float left_right = 1.0f;
//float up_down = 1.0f;
//
//bool spin = false;
//float angle = 0.0f;
//
//bool colorChangeFlag = false;
//bool isChanging = false;
//int nextColor = 0;
//
//GLfloat diffColors[4][4] = { { 0.5, 0.5, 0.9, 1.0 },
//{ 0.9, 0.5, 0.5, 1.0 },
//{ 0.5, 0.9, 0.3, 1.0 },
//{ 0.3, 0.8, 0.9, 1.0 } };
//
//int colorIndex = 0;
//GLfloat currentColor[4] = { 0.5, 0.5, 0.9, 1.0 };
//
//// These are convenience functions which allow us to call OpenGL 
//// methods on Vec3d objects
//inline void glVertex(const Vector3f &a)
//{
//	glVertex3fv(a);
//}
//
//inline void glNormal(const Vector3f &a)
//{
//	glNormal3fv(a);
//}
//
//
//// This function is called whenever a "Normal" key press is received.
//void keyboardFunc(unsigned char key, int x, int y)
//{
//	switch (key)
//	{
//	case 27: // Escape key
//		exit(0);
//		break;
//	case 'c':
//		// add code to change color here
//		if (!isChanging) {  // when color is not in the process of transitioning
//			colorChangeFlag = true;
//		}
//		break;
//	case 'r':
//		spin = !spin;
//		break;
//	default:
//		cout << "Unhandled key press " << key << "." << endl;
//	}
//
//	// this will refresh the screen so that the user sees the color change
//	glutPostRedisplay();
//}
//
//// This function is called whenever a "Special" key press is received.
//// Right now, it's handling the arrow keys.
//void specialFunc(int key, int x, int y)
//{
//	switch (key)
//	{
//	case GLUT_KEY_UP:
//		// add code to change light position
//		up_down = up_down + 0.5;
//		break;
//	case GLUT_KEY_DOWN:
//		// add code to change light position
//		up_down = up_down - 0.5;
//		break;
//	case GLUT_KEY_LEFT:
//		// add code to change light position
//		left_right = left_right - 0.5;
//		break;
//	case GLUT_KEY_RIGHT:
//		// add code to change light position
//		left_right = left_right + 0.5;
//		break;
//	}
//
//	// this will refresh the screen so that the user sees the light position
//	glutPostRedisplay();
//}
//
//// rotates shapes by 1 degree every 25 milliseconds
//void spinTimer(int value) {
//	if (spin) {
//		angle++;
//		if (angle > 360) {  // angles kept small for float precision issues
//			angle -= 360;
//		}
//	}
//	glutPostRedisplay();
//	glutTimerFunc(25, spinTimer, 0);
//}
//
//
//float num_steps = 100;
//float t;
//unsigned int iter;
//GLfloat diff0, diff1, diff2, diff3;
//
//void colorTimer(int value) {
//	if (colorChangeFlag) {
//		iter = 0;
//		colorChangeFlag = false;
//		isChanging = true;
//	}
//	if (isChanging) {
//		iter += 1;
//		t = iter / num_steps;
//		//cout << "t: " << t << endl;
//
//		int startIndex = colorIndex;
//		int endIndex = (colorIndex + 1) % 4;
//		
//		diff0 = (diffColors[endIndex][0] - diffColors[startIndex][0]);
//		diff1 = (diffColors[endIndex][1] - diffColors[startIndex][1]);
//		diff2 = (diffColors[endIndex][2] - diffColors[startIndex][2]);
//		diff3 = (diffColors[endIndex][3] - diffColors[startIndex][3]);
//
//		currentColor[0] = diffColors[startIndex][0] + diff0 * t;
//		currentColor[1] = diffColors[startIndex][1] + diff1 * t;
//		currentColor[2] = diffColors[startIndex][2] + diff2 * t;
//		currentColor[3] = diffColors[startIndex][3] + diff3 * t;
//
//		//cout << diff0 << " " << diff1 << " " << diff2 << " " << diff3 << endl;
//
//		if (iter >= num_steps) {
//			colorIndex = endIndex;
//			isChanging = false;
//		}
//	}
//
//glutPostRedisplay();
//glutTimerFunc(25, colorTimer, 0);
//}
//
//// This function is responsible for displaying the object.
//void drawScene(void)
//{
//	int i;
//
//	// Clear the rendering window
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//	glMatrixMode(GL_MODELVIEW);  // Current matrix affects objects positions
//	glLoadIdentity();              // Initialize to the identity
//
//	// Position the camera at [0,0,5], looking at [0,0,0],
//	// with [0,1,0] as the up direction.
//	gluLookAt(0.0, 0.0, 5.0,
//		0.0, 0.0, 0.0,
//		0.0, 1.0, 0.0);
//
//	// Set material properties of object
//
//	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, currentColor);
//
//	// Define specular color and shininess
//	GLfloat specColor[] = { 1.0, 1.0, 1.0, 1.0 };
//	GLfloat shininess[] = { 100.0 };
//
//	// Note that the specular color and shininess can stay constant
//	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specColor);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
//
//	// Set light properties
//
//	// Light color (RGBA)
//	GLfloat Lt0diff[] = { 1.0,1.0,1.0,1.0 };
//	// Light position
//	GLfloat Lt0pos[] = { left_right, up_down, 5.0f, 1.0f };
//
//	glLightfv(GL_LIGHT0, GL_DIFFUSE, Lt0diff);
//	glLightfv(GL_LIGHT0, GL_POSITION, Lt0pos);
//
//	glRotatef(angle, 0, 1, 0);
//
//	// This GLUT method draws a teapot.  You should replace
//	// it with code which draws the object you loaded.
//	// glutSolidTeapot(1.0);
//
//	for (unsigned int iter = 0; iter < vecf.size(); iter++) {
//		int a = vecf[iter][0];
//		int c = vecf[iter][1];
//		int d = vecf[iter][2];
//		int f = vecf[iter][3];
//		int g = vecf[iter][4];
//		int i = vecf[iter][5];
//
//		glBegin(GL_TRIANGLES);
//		glNormal3d(vecn[c - 1][0], vecn[c - 1][1], vecn[c - 1][2]);
//		glVertex3d(vecv[a - 1][0], vecv[a - 1][1], vecv[a - 1][2]);
//		glNormal3d(vecn[f - 1][0], vecn[f - 1][1], vecn[f - 1][2]);
//		glVertex3d(vecv[d - 1][0], vecv[d - 1][1], vecv[d - 1][2]);
//		glNormal3d(vecn[i - 1][0], vecn[i - 1][1], vecn[i - 1][2]);
//		glVertex3d(vecv[g - 1][0], vecv[g - 1][1], vecv[g - 1][2]);
//		glEnd();
//	}
//
//	int main(int /*argc*/, char** argv)
//	{
//		// instantiate a Surface_mesh object
//		Surface_mesh mesh;
//		// read a mesh specified as the first command line argument
//		mesh.read(argv[1]);
//		// ...
//		// do fancy stuff with the mesh
//		// ...
//		// write the mesh to the file specified as second argument
//		mesh.write(argv[2]);
//		return 0;
//	}
//
//
//
//	// Dump the image to the screen.
//	glutSwapBuffers();
//}
//
//// Initialize OpenGL's rendering modes
//void initRendering()
//{
//	glEnable(GL_DEPTH_TEST);   // Depth testing must be turned on
//	glEnable(GL_LIGHTING);     // Enable lighting calculations
//	glEnable(GL_LIGHT0);       // Turn on light #0.
//}
//
//// Called when the window is resized
//// w, h - width and height of the window in pixels.
//void reshapeFunc(int w, int h)
//{
//	// Always use the largest square viewport possible
//	if (w > h) {
//		glViewport((w - h) / 2, 0, h, h);
//	}
//	else {
//		glViewport(0, (h - w) / 2, w, w);
//	}
//
//	// Set up a perspective view, with square aspect ratio
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	// 50 degree fov, uniform aspect ratio, near = 1, far = 100
//	gluPerspective(50.0, 1.0, 1.0, 100.0);
//}
//
//void loadInput()
//{
//	// load the OBJ file here
//
//	const int MAX_BUFFER_SIZE = 256;
//	char buffer[MAX_BUFFER_SIZE];
//
//	string s;
//	Vector3f v;
//
//	while (!cin.fail()) {
//
//		// read a single line from char array
//		cin.getline(buffer, MAX_BUFFER_SIZE);
//
//		// put contents of line into stringstream object
//		stringstream ss(buffer);
//
//		// read type from stringstream into string s
//		ss >> s;
//
//		if (s == "v") {
//			// read floats from stringstream into Vector3f v
//			ss >> v[0] >> v[1] >> v[2];
//			vecv.push_back(Vector3f(v[0], v[1], v[2]));
//		}
//		else if (s == "vn") {
//			ss >> v[0] >> v[1] >> v[2];
//			vecn.push_back(Vector3f(v[0], v[1], v[2]));
//		}
//		else if (s == "f") {
//			char slash;
//			int a, b, c, d, e, f, g, h, i;
//
//			ss >> a >> slash >> b >> slash >> c >> d >> slash >> e >> slash >> f >> g >> slash >> h >> slash >> i;
//
//			vector<unsigned> tempvec;
//			tempvec.push_back(a);
//			tempvec.push_back(c);
//			tempvec.push_back(d);
//			tempvec.push_back(f);
//			tempvec.push_back(g);
//			tempvec.push_back(i);
//
//			vecf.push_back(tempvec);
//		}
//	}
//}
//
//
//// Main routine.
//// Set up OpenGL, define the callbacks and start the main loop
//int main(int argc, char** argv)
//{
//	loadInput();
//
//	glutInit(&argc, argv);
//
//	// We're going to animate it, so double buffer 
//	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
//
//	// Initial parameters for window position and size
//	glutInitWindowPosition(60, 60);
//	glutInitWindowSize(360, 360);
//	glutCreateWindow("Assignment 0");
//
//	// Initialize OpenGL parameters.
//	initRendering();
//
//	// Set up callback functions for key presses
//	glutKeyboardFunc(keyboardFunc); // Handles "normal" ascii symbols
//	glutSpecialFunc(specialFunc);   // Handles "special" keyboard keys
//
//	 // Set up the callback function for resizing windows
//	glutReshapeFunc(reshapeFunc);
//
//	// Call this whenever window needs redrawing
//	glutDisplayFunc(drawScene);
//
//	glutTimerFunc(25, spinTimer, 0);
//
//	glutTimerFunc(25, colorTimer, 0);
//
//	// Start the main loop.  glutMainLoop never returns.
//	glutMainLoop();
//
//	return 0;	// This line is never reached.
//}




int main(int argc, char** argv)
{
	// instantiate a Surface_mesh object
	Surface_mesh mesh;

	// read a mesh specified as the first command line argument
	mesh.read(argv[1]);

	// eqn 6 test
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

	return 0;
}