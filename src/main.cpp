#include "main.h"
#include <math.h>

using namespace std;


//****************************************************
// Some Classes
//****************************************************
class Viewport {
public:
    Viewport(): mousePos(0.0,0.0) { orientation = identity3D(); };
	int w, h; // width and height
	vec2 mousePos;
    mat4 orientation;
};


//#define MAX_CART_SPEED 1000
//#define CART_SPEED	1

#define INITIAL_PE 	 0
#define INITIAL_VELOCITY		 150
#define MAX_VELOCITY	10000		
#define MIN_VELOCITY	0.1
#define TIMESTEP 	(1.0/MAX_VELOCITY)
#define VELOCITY_SCALE	0.01
#define H_SCALE		1000

//double TIMESTEP = 1.0/MAX_VELOCITY;
int velocity_sign = 1;
double velocity = INITIAL_VELOCITY;
double PE = INITIAL_PE;

bool s_pressed = false;
//****************************************************
// Global Variables
//****************************************************
Viewport viewport;
UCB::ImageSaver * imgSaver;
int frameCount = 0;
double t = 0;
SplineCoaster *coaster;
enum {VIEW_FIRSTPERSON, VIEW_THIRDPERSON, VIEW_SIDE, VIEW_MAX};
int viewMode = VIEW_THIRDPERSON;
bool inv = false;

//<begin> From as9
Mesh *mesh;
Skeleton *skel;	//NO SKELETON
Animation *anim;

// these variables track which joint is under IK, and where
int ikJoint;
double ikDepth;

// ui modes
bool playanim = false;
int ik_mode = IK_CCD;

// setup the model view matrix for mesh rendering

//May not need
/**
void setupView() {
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    glTranslatef(0,0,-3);
    applyMat4(viewport.orientation);
}
**/
//From as9 </begin>

//-------------------------------------------------------------------------------
/// Called to update the screen at 30 fps.
void frameTimer(int value) {
    frameCount++;
    glutPostRedisplay();
    glutTimerFunc(1000/30, frameTimer, 1);
}


// A simple helper function to load a mat4 into opengl
void applyMat4(mat4 &m) {
	double glmat[16];
	int idx = 0;
	for (int j = 0; j < 4; j++) 
		for (int i = 0; i < 4; i++)
			glmat[idx++] = m[i][j];
	glMultMatrixd(glmat);
}

mat4 getBasis(double t, bool inv) {
	SplinePoint sp = coaster->sample(t);
	vec3 forward = coaster->sampleForward(t);
	vec3 up = coaster->sampleUp(t);
	if(inv) up = -1 * up;
	vec3 x = forward ^ up;
	vec3 origin = sp.point;
	forward = forward.normalize();
	up = up.normalize();
	x = x.normalize();
	return mat4(vec4(forward[VX], up[VX], x[VX], origin[VX]), vec4(forward[VY], up[VY], x[VY], origin[VY]), vec4(forward[VZ], up[VZ], x[VZ], origin[VZ]), vec4(0, 0, 0, 1));
}


mat4 getCameraBasis(double t){
	SplinePoint sp = coaster->sample(t);
	vec3 forward = coaster->sampleForward(t);
	vec3 up = coaster->sampleUp(t);
	vec3 x = forward ^ up;
	vec3 origin = sp.point;
	forward = forward.normalize();
	up = up.normalize();
	x = x.normalize();
	return mat4(vec4(x[VX], up[VX], -forward[VX], origin[VX]), vec4(x[VY], up[VY], -forward[VY], origin[VY]), vec4(x[VZ], up[VZ], -forward[VZ], origin[VZ]), vec4(0, 0, 0, 1));
}



// replace this with something cooler!
void drawMesh(double size, const vec3& meshcolor) {
	glColor3f(meshcolor[VX],meshcolor[VY],meshcolor[VZ]);
	//glutSolidTeapot(size);
	mesh->render();
}

void drawSkeleton(double size, const vec3& skelcolor) {
	glColor3f(skelcolor[VX],skelcolor[VY],skelcolor[VZ]);
	//glutSolidTeapot(size);
	if (!playanim) { // if not playing an animation draw the skeleton
        skel->render(ikJoint); 
    }
}


void drawMeshAndSkeleton(const vec3& meshcolor, const vec3& skelcolor, double t){
	glPushMatrix();
	mat4 meshBasis = getBasis(t, inv);
	applyMat4(meshBasis);
	glRotatef(90,0,1,0);
	glTranslatef(0,1.9,0);
	drawMesh(1, meshcolor);
	drawSkeleton(1, skelcolor); 
	skel->render(ikJoint);
	glPopMatrix();
}


void display() {

	//Clear Buffers
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);					// indicate we are specifying camera transformations
	glLoadIdentity();							// make sure transformation is "zero'd"
	
    if (viewMode == VIEW_THIRDPERSON) {
        glTranslatef(0,-5,-50);
        applyMat4(viewport.orientation);
    }else if (viewMode == VIEW_FIRSTPERSON){
		glTranslatef(0, -3, -2);
		mat4 basis = getCameraBasis(t).inverse();
		applyMat4(basis);
	}else if (viewMode == VIEW_SIDE){
		glRotatef(90,0,1,0);
		glTranslatef(6,-1.8,0);
		mat4 basis = getCameraBasis(t).inverse();
		applyMat4(basis);
	}
    coaster->renderWithDisplayList(100,.3,3,.2,0);
	
	SplinePoint sp = coaster->sample(t);
	//vec3 forward = coaster->sampleForward(t);
	//vec3 up = coaster->sampleUp(t);
	//vec3 x = forward ^ up;
	vec3 origin = sp.point;
	//forward = forward.normalize();
	//up = up.normalize();
	//x = x.normalize();
	
	PE = INITIAL_PE + H_SCALE * origin[VY];
	velocity = (PE > 0) ? (INITIAL_VELOCITY - sqrt(PE)) : (INITIAL_VELOCITY + sqrt(PE * -1));
	
	
	if(abs(velocity) < MIN_VELOCITY)
		velocity_sign = -1 * velocity_sign;
	
	//velocity = velocity;
	
	double displacement = 0; //velocity * TIMESTEP
	vec3 prev_point = origin;
	//cout<<"hit_______"<<endl;
	while (displacement < velocity * VELOCITY_SCALE){
		//cout<<"happened"<<endl;
		t = t + velocity_sign * 3 * TIMESTEP;
		vec3 point = coaster->sample(t).point;
		displacement += sqrt((prev_point - point) * (prev_point - point));
		prev_point = point;
	}
	
	
	if(t > 1){
		inv = !inv;
		t = t - floor(t);
	}
	
	drawMeshAndSkeleton(vec3(1.0, 1.0, 0.0), vec3(1.0, 1.0, 1.0), t);
	
	//Now that we've drawn on the buffer, swap the drawing buffer and the displaying buffer.
	glutSwapBuffers();
	
	if(s_pressed){
		imgSaver->saveFrame();
	}

}



//-------------------------------------------------------------------------------
/// \brief	Called when the screen gets resized.
/// This gives you the opportunity to set up all the relevant transforms.
///
void reshape(int w, int h) {
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, ((double)w / MAX(h, 1)), 1.0, 100.0);
	//glOrtho(-10,10,-10,10,1,100);

    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


//-------------------------------------------------------------------------------
/// Called to handle keyboard events.
void myKeyboardFunc (unsigned char key, int x, int y) {
	switch (key) {
		case 27:			// Escape key
			exit(0);
			break;
        case 'S':
        case 's':
    	    //s_pressed = !s_pressed;
            break;
		case 'U':
		case 'u':
			imgSaver->saveFrame();
			break;
        case 'V':
        case 'v':
            viewMode = (viewMode+1)%VIEW_MAX;
            break;
	}
}

//-------------------------------------------------------------------------------
/// Called whenever the mouse moves while a button is pressed
void myActiveMotionFunc(int x, int y) {

    // Rotate viewport orientation proportional to mouse motion
    vec2 newMouse = vec2((double)x / glutGet(GLUT_WINDOW_WIDTH),(double)y / glutGet(GLUT_WINDOW_HEIGHT));
    vec2 diff = (newMouse - viewport.mousePos);
    double len = diff.length();
    if (len > .001) {
        vec3 axis = vec3(diff[1]/len, diff[0]/len, 0);
        viewport.orientation = rotation3D(axis, 180 * len) * viewport.orientation;
    }

    //Record the mouse location for drawing crosshairs
    viewport.mousePos = newMouse;

    //Force a redraw of the window.
    glutPostRedisplay();
}


//-------------------------------------------------------------------------------
/// Called whenever the mouse moves without any buttons pressed.
void myPassiveMotionFunc(int x, int y) {
	ikJoint = -1;
	
	//setJointsByAnimation(x);  NO SKELETON
	
    //Record the mouse location for drawing crosshairs
    viewport.mousePos = vec2((double)x / glutGet(GLUT_WINDOW_WIDTH),(double)y / glutGet(GLUT_WINDOW_HEIGHT));

    //Force a redraw of the window.
    glutPostRedisplay();
}




//-------------------------------------------------------------------------------
/// Initialize the environment
int main(int argc,char** argv) {
	//cout<<"TIMESTEP "<<TIMESTEP<<endl;
	//Initialize OpenGL
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);

	//Set up global variables
	viewport.w = 600;
	viewport.h = 600;

	coaster = new SplineCoaster("track.trk");

/**
	if (argc < 2) {
	    cout << "USAGE: coaster coaster.trk" << endl;
	    return -1;
    } else {
        // Create the coaster
        coaster = new SplineCoaster("helix.trk");
        if (coaster->bad()) {
            cout << "Coaster file appears to not have a proper coaster in it" << endl;
            return -1;
        }
    }
**/
	//Initialize the screen capture class to save BMP captures
	//in the current directory, with the prefix "coaster"
	imgSaver = new UCB::ImageSaver("./", "coaster");

	//Create OpenGL Window
	glutInitWindowSize(viewport.w,viewport.h);
	glutInitWindowPosition(0,0);
	glutCreateWindow("CS184 Framework");

	//Register event handlers with OpenGL.
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(myKeyboardFunc);
	glutMotionFunc(myActiveMotionFunc);
	glutPassiveMotionFunc(myPassiveMotionFunc);
    frameTimer(0);

    glClearColor(.4,.2,1,0);

    // set some lights
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .2f, .5f, .5f };
       float pos[4] = { 0, 5, -5, 0 };
       glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT1, GL_POSITION, pos);
       glEnable(GL_LIGHT1);
    }
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .5f, .2f, .5f };
       float pos[4] = { 5, 0, -5, 0 };
       glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT2, GL_POSITION, pos);
       glEnable(GL_LIGHT2);
    }
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    // load a mesh
    mesh = new Mesh();
    mesh->loadFile("Model1.obj");
	// load a matching skeleton
    skel = new Skeleton();
    skel->loadPinocchioFile("skeleton.out");
    mesh->centerAndScale(*skel, 4);
    // load the correspondence between skeleton and mesh
    skel->initBoneWeights("attachment.out", *mesh);
    skel->updateSkin(*mesh);
    // start a new animation
    anim = new Animation();


	//And Go!
	glutMainLoop();
}
