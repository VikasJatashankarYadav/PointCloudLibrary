

#include <iostream>
#include <GL/freeglut.h>

#include "ModelCamera.h"

using namespace strivision;

template <typename Real>
struct Point {
	Real x, y, z;
	Real r, g, b;
};

template <class PointType>
struct PointCloud {
	std::vector<PointType> data;
};

std::shared_ptr<PointCloud<Point<Real>>> cloud;

std::shared_ptr<ModelCamera<Real>> camera = nullptr;

bool init() {
	glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
	glEnable(GL_DEPTH_TEST);

	camera = std::make_shared<ModelCamera<Real>>();

	cloud = std::make_shared<PointCloud<Point<Real>>>();
	const std::size_t N = 4096;
	cloud->data.resize(N);

	for ( std::size_t i = 0; i < N; i++ ) {
		Point<Real>& p = cloud->data[i];

		p.x = ((double) rand() / (RAND_MAX));
		p.y = ((double) rand() / (RAND_MAX));
		p.z = ((double) rand() / (RAND_MAX));

		p.r = p.z;
		p.g = p.z;
		p.b = p.z;
	}

	return true;
}

inline bool RenderPointCloud(const std::shared_ptr<PointCloud<Point<Real>>>& cloud) {
	if ( cloud == nullptr ) return false;

	glPointSize(2.0f);
	glBegin(GL_POINTS);
	for ( std::size_t i = 0; i < cloud->data.size(); i++ ) {
		const Point<Real>& p = cloud->data[i];
		glColor3f(p.r, p.g, p.b);
		glVertex3f(p.x, p.y, p.z);
	}
	glEnd();
	return true;
}

void render() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
		glLoadIdentity();
		glMultMatrixf(camera->getViewMatrix().data());

		glBegin(GL_LINES);
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(1.0f, 0.0f, 0.0f);
			glColor3f(0.0f, 1.0f, 0.0f);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(0.0f, 1.0f, 0.0f);
			glColor3f(0.0f, 0.0f, 1.0f);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(0.0f, 0.0f, 1.0f);
		glEnd();

		RenderPointCloud(cloud);

	glPopMatrix();
	glFlush();
}

void reshape(GLint width, GLint height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(65.0, (float)width / height, 0.1f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
}

void mouseFunc(int button, int state, int x, int y) {
	if ( button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN ) camera->bPan = true;
	if ( button == GLUT_MIDDLE_BUTTON && state == GLUT_UP ) camera->bPan = false;
	if ( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN ) camera->bRotate = true;
	if ( button == GLUT_LEFT_BUTTON && state == GLUT_UP ) camera->bRotate = false;
	if ( button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN ) camera->bZoom = true;
	if ( button == GLUT_RIGHT_BUTTON && state == GLUT_UP ) camera->bZoom = false;

	camera->mouse_last = {x, y};
	camera->mouse_cur = {x, y};
	glutPostRedisplay();
}

void mouseMotionFunc(int x, int y) {
	camera->mouse_cur = {x, y};

	const double panSen = 1.0;
	const double rotSen = 1.0;
	const double zoomSen = 0.1;

	Vector2<int32_t> diff = camera->mouse_last - camera->mouse_cur;
	if ( camera->bPan ) camera->pan(diff.x() * panSen, diff.y() * panSen);
	if ( camera->bRotate ) camera->rotate(diff.x()* rotSen, diff.y() * rotSen);
	if ( camera->bZoom ) camera->zoom(diff.y() * zoomSen);

	camera->mouse_last = camera->mouse_cur;
	glutPostRedisplay();
}

int main(int argc, char** argv) {
	glutInit (&argc, argv);
	glutInitWindowSize(800, 600);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("Multisample");
	if ( init() ==  false ) return 1;
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouseFunc);
	glutMotionFunc(mouseMotionFunc);
	glutMainLoop();
	return 0;
}
