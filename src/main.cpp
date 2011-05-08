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
#define INITIAL_VELOCITY		 10
#define MAX_VELOCITY	10000		
#define MIN_VELOCITY	0.1
#define TIMESTEP 	(1.0/MAX_VELOCITY)
#define VELOCITY_SCALE	0.01
#define H_SCALE		1000
#define FRONTFOOT1 14 //right front
#define FRONTFOOT2 17 //left front
#define FRONTKNEE1 13//right
#define FRONTKNEE2 16//left
#define BACKFOOT1 7 //right back
#define BACKFOOT2 11 //left back
#define BACKANKLE1 6 //right
#define BACKANKLE2 10 //left
#define BACKKNEE1 5 //right
#define BACKKNEE2 9 //left
#define HIP1 4 //right
#define HIP2 8 //left
#define SHOULDER1 12//right
#define SHOULDER2 15//left
#define SPINE 1
#define PELVIS 2
#define HEAD 3
#define PI 3.14159265

//double TIMESTEP = 1.0/MAX_VELOCITY;
int velocity_sign = 1;
double velocity = INITIAL_VELOCITY;
double PE = INITIAL_PE;

bool s_pressed = false;
bool p_pressed = false;
//****************************************************
// Global Variables
//****************************************************
GLuint skyboxtexture;
Viewport viewport;
UCB::ImageSaver * imgSaver;
int frameCount = 0;
double t = 0;
SplineCoaster *coaster;
enum {VIEW_FIRSTPERSON, VIEW_THIRDPERSON, VIEW_SIDE1, VIEW_SIDE2, VIEW_MAX};
int viewMode = VIEW_THIRDPERSON;
bool inv = false;

//<begin> From as9
Mesh *mesh;
Skeleton *skel;	//NO SKELETON
Animation *anim;

// these variables track which joint is under IK, and where
int ikJoint;
double ikDepth;
vec3 FF1Initial;
vec3 FF2Initial;
vec3 BF1Initial;
vec3 BF2Initial;
vec3 FK1Initial;
vec3 FK2Initial;
vec3 BK1Initial;
vec3 BA1Initial;
vec3 BK2Initial;
vec3 BA2Initial;
vec3 H1Initial;
vec3 H2Initial;
vec3 S1Initial;
vec3 S2Initial;
vec3 SpInitial;
vec3 PInitial;
vec3 HeInitial;


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

// setup the model view matrix for mesh rendering
void setupView() {
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    glTranslatef(0,0,-3);
    applyMat4(viewport.orientation);
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

mat4 getSkyBoxBasisForTrackCoord(double t, bool inv){
	SplinePoint sp = coaster->sample(t);
	vec3 forward = coaster->sampleForward(t);
	vec3 up = coaster->sampleUp(t);
	if(inv) up = -1 * up;
	vec3 x = forward ^ up;
//	vec3 origin = sp.point;
	forward = forward.normalize();
	up = up.normalize();
	x = x.normalize();
	return mat4(vec4(forward[VX], up[VX], x[VX], 0), vec4(forward[VY], up[VY], x[VY], 0), vec4(forward[VZ], up[VZ], x[VZ], 0), vec4(0, 0, 0, 1));
}


mat4 getCameraBasis(double t, bool inv){
	SplinePoint sp = coaster->sample(t);
	vec3 forward = coaster->sampleForward(t);
	vec3 up = coaster->sampleUp(t);
	if(inv) up = -1 * up;
	vec3 x = forward ^ up;
	vec3 origin = sp.point;
	forward = forward.normalize();
	up = up.normalize();
	x = x.normalize();
	return mat4(vec4(x[VX], up[VX], -forward[VX], origin[VX]), vec4(x[VY], up[VY], -forward[VY], origin[VY]), vec4(x[VZ], up[VZ], -forward[VZ], origin[VZ]), vec4(0, 0, 0, 1));
}


void drawSkyBox(){
	
    // Enable/Disable features
    glPushAttrib(GL_ENABLE_BIT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_BLEND);
    // Just in case we set all vertices to white.
    glColor4f(1,1,1,1);
    // Render the front quad
    glBindTexture(GL_TEXTURE_2D, skyboxtexture);


	glBegin(GL_QUADS);
		// Front Face
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
		// Back Face
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
		// Top Face
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
		// Bottom Face
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		// Right face
		glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		// Left Face
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
	glEnd();
	
	glPopAttrib();
//	glPopMatrix();
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
	if (!p_pressed) { // if not playing an animation draw the skeleton
        skel->render(ikJoint); 
    }
}


void drawMeshAndSkeleton(const vec3& meshcolor, const vec3& skelcolor, double t){
	glPushMatrix();
	mat4 meshBasis = getBasis(t, inv);
	applyMat4(meshBasis);
	glRotatef(90,0,1,0);

	vector<Joint> jArray = skel->getJointArray();
	int root = skel->getRoot();

	double xInterp1 = (30*PI)*t;
	double xInterp2 = xInterp1+PI;
	xInterp1 = 0.4*sin(xInterp1);
	xInterp2 = 0.4*sin(xInterp2);
	double yInterp1 = (30*PI)*t;
	double yInterp2 = ((30*PI)*t)+PI;
	if(cos(yInterp1)>0){
		yInterp1 = 0.4*cos(yInterp1);
	} else {
		yInterp1 = 0;
	}
	if(cos(yInterp2)>0){
		yInterp2 = 0.4*cos(yInterp2);
	} else {
		yInterp2 = 0;
	}

	vec3 point = coaster->sample(t).point;
	vec3 forwardPoint = coaster->sample(t+(0.015)).point;
	vec3 backPoint = coaster->sample(t-(0.01)).point;
	vec3 pNormal = coaster->sampleUp(t);
	vec3 fNormal = coaster->sampleUp(t+0.015);
	vec3 bNormal = coaster->sampleUp(t-0.01);
	double fAngle = acos((fNormal*pNormal)/(fNormal.length()*pNormal.length()));
	double bAngle = acos((bNormal*pNormal)/(bNormal.length()*pNormal.length()));
	double addFront = 15*(sin(fAngle))*(meshBasis.inverse()*forwardPoint - meshBasis.inverse()*point)[1];
	double addBack = 15*(sin(bAngle))*(meshBasis.inverse()*point-meshBasis.inverse()*backPoint)[1];
	if(inv == true){
		addFront = -addFront;
		addBack = -addBack;
	}

	vec3 targetFF1 = vec3(FF1Initial[0],FF1Initial[1]+xInterp1,FF1Initial[2]-(yInterp1+addFront));
	vec3 targetFK1 = vec3(FK1Initial[0],FK1Initial[1]+(xInterp1/2),FK1Initial[2]-((yInterp1+addFront)/2));
	vec3 targetFF2 = vec3(FF2Initial[0],FF2Initial[1]+xInterp2,FF2Initial[2]-(yInterp2+addFront));
	vec3 targetFK2 = vec3(FK2Initial[0],FK2Initial[1]+(xInterp2/2),FK2Initial[2]-((yInterp2+addFront)/2));

	//vec3 targetBF1 = vec3(BF1Initial[0],BF1Initial[1]+xInterp2,BF1Initial[2]-yInterp2);
	//vec3 targetBF2 = vec3(BF2Initial[0],BF2Initial[1]+xInterp1,BF2Initial[2]-yInterp1);
	vec3 targetBK1 = vec3(BK1Initial[0],BK1Initial[1]+(xInterp2/2),BK1Initial[2]-(yInterp2/2)+addBack);
	vec3 targetBK2 = vec3(BK2Initial[0],BK2Initial[1]+(xInterp1/2),BK2Initial[2]-(yInterp1/2)+addBack);
	vec3 targetBA1 = vec3(BA1Initial[0],BA1Initial[1]+xInterp2,BA1Initial[2]-yInterp2+addBack);
	vec3 targetBA2 = vec3(BA2Initial[0],BA2Initial[1]+xInterp1,BA2Initial[2]-yInterp1+addBack);

	skel->inverseKinematics(FRONTKNEE1, targetFK1, ik_mode);
	skel->inverseKinematics(FRONTFOOT1, targetFF1, ik_mode);
	skel->inverseKinematics(FRONTKNEE2, targetFK2, ik_mode);
	skel->inverseKinematics(FRONTFOOT2, targetFF2, ik_mode);

	//skel->inverseKinematics(BACKFOOT1, targetBF1, ik_mode);
	skel->inverseKinematics(BACKANKLE1, targetBA1, ik_mode);
	skel->inverseKinematics(BACKKNEE1, targetBK1, ik_mode);
	skel->inverseKinematics(HIP1, H1Initial, ik_mode);
	
	//skel->inverseKinematics(BACKFOOT2, targetBF2, ik_mode);
	skel->inverseKinematics(BACKANKLE2, targetBA2, ik_mode);
	skel->inverseKinematics(BACKKNEE2, targetBK2, ik_mode);
	skel->inverseKinematics(HIP2, H2Initial, ik_mode);

	skel->inverseKinematics(SHOULDER1, S1Initial, ik_mode);
	skel->inverseKinematics(SHOULDER2, S2Initial, ik_mode);
	skel->inverseKinematics(PELVIS, PInitial, ik_mode);
	skel->inverseKinematics(SPINE, SpInitial, ik_mode);

	//vec3 HeTarget = vec3(HeInitial[0]+(0.5*sin(20*PI*t)),HeInitial[1],HeInitial[2]+(0.5*sin(20*PI*t)));
	//skel->inverseKinematics(HEAD, HeTarget, ik_mode);

	glTranslatef(0,1,0);
	glRotatef(90,1,0,0);

	drawMesh(1, meshcolor);
	drawSkeleton(1, skelcolor); 
	skel->updateSkin(*mesh);

	glPopMatrix();
	/*glPushMatrix();
	meshBasis = getBasis((t+(-0.01)+(0.01*xInterp2)), inv);
	applyMat4(meshBasis);
	glutSolidTeapot(1);
	glPopMatrix();*/
}


void display() {

	//Clear Buffers
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);					// indicate we are specifying camera transformations
	glLoadIdentity();							// make sure transformation is "zero'd"
	
	
	
    if (viewMode == VIEW_THIRDPERSON) {
	
		glPushMatrix();
		glTranslatef(0, 0, -1);
		applyMat4(viewport.orientation);
		drawSkyBox();
		glPopMatrix();
		
        glTranslatef(0,-5,-50);
        applyMat4(viewport.orientation);
    }else if (viewMode == VIEW_FIRSTPERSON){
		glPushMatrix();
		glTranslatef(0, 0, -1);
		mat4 basisSky = getSkyBoxBasisForTrackCoord(t, inv).inverse();
		applyMat4(basisSky);
		drawSkyBox();
		glPopMatrix();
		
		
		glTranslatef(0, -1, -2);
		mat4 basis = getCameraBasis(t, inv).inverse();
		applyMat4(basis);
	}else if (viewMode == VIEW_SIDE1){
		glPushMatrix();
		glTranslatef(0, 0, -1);
		glRotatef(90,0,1,0);
		mat4 basisSky = getSkyBoxBasisForTrackCoord(t, inv).inverse();
		applyMat4(basisSky);
		drawSkyBox();
		glPopMatrix();
		
		
		glRotatef(90,0,1,0);
		glTranslatef(6,-0.5,0);
		mat4 basis = getCameraBasis(t, inv).inverse();
		applyMat4(basis);
		//applyMat4(viewport.orientation);
	}else if (viewMode == VIEW_SIDE2){
		glPushMatrix();
		glTranslatef(0, 0, -1);
		glRotatef(270,0,1,0);
		mat4 basisSky = getSkyBoxBasisForTrackCoord(t, inv).inverse();
		applyMat4(basisSky);
		drawSkyBox();
		glPopMatrix();
		
		glRotatef(270,0,1,0);
		glTranslatef(-8,1.5,0);
		mat4 basis = getCameraBasis(t, inv).inverse();
		applyMat4(basis);
		//applyMat4(viewport.orientation);
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
	
	//PE = INITIAL_PE + H_SCALE * origin[VY];
	velocity = INITIAL_VELOCITY;//(PE > 0) ? (INITIAL_VELOCITY - sqrt(PE)) : (INITIAL_VELOCITY + sqrt(PE * -1));
	
	
	if(abs(velocity) < MIN_VELOCITY)
		velocity_sign = -1 * velocity_sign;
	
	//velocity = velocity;
	
	double displacement = 0; //velocity * TIMESTEP
	vec3 prev_point = origin;
	//cout<<"hit_______"<<endl;
	double prevT = t;
	while ((displacement < velocity * VELOCITY_SCALE)){
		//cout<<"happened"<<endl;
		t = t + velocity_sign * 3 * TIMESTEP;
		if(abs(sin(30*PI*t)) >= 0.98){
			break;
		}
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
    	    s_pressed = !s_pressed;
            break;
		case 'U':
		case 'u':
			imgSaver->saveFrame();
			break;
		case 'P':
		case 'p':
			p_pressed = !p_pressed;
			break;
        case 'V':
        case 'v':
            viewMode = (viewMode+1)%VIEW_MAX;
            break;
	}
}

void myMouseFunc(int button, int state, int x, int y) {
	/*if (viewMode == VIEW_SIDE){
		setupView();
		ikJoint = skel->pickJoint(ikDepth, vec2(x,y));
	}*/
}


//-------------------------------------------------------------------------------
/// Called whenever the mouse moves while a button is pressed
void myActiveMotionFunc(int x, int y) {
	if (ikJoint != -1 && !playanim) { // if a joint is selected for ik and we're not in animation playback mode, do ik
		vec3 target = skel->getPos(vec2(x,y), ikDepth);
        skel->inverseKinematics(ikJoint, target, ik_mode);
        skel->updateSkin(*mesh);
    } else if (true){//viewMode == VIEW_THIRDPERSON){ // else mouse movements update the view
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
	}

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
	glutMouseFunc(myMouseFunc);
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
    mesh->loadFile("ant2.obj");
	// load a matching skeleton
    skel = new Skeleton();
    skel->loadPinocchioFile("skeleton.out");
    mesh->centerAndScale(*skel, 4);
    // load the correspondence between skeleton and mesh
    skel->initBoneWeights("attachment.out", *mesh);
    skel->updateSkin(*mesh);
    // start a new animation
    anim = new Animation();

	loadTexture("earth.png", skyboxtexture);
	
	vector<Joint> initialJoints = skel->getJointArray();
	FF1Initial = initialJoints[FRONTFOOT1].posn;
	FF2Initial = initialJoints[FRONTFOOT2].posn;
	FK1Initial = initialJoints[FRONTKNEE1].posn;
	FK2Initial = initialJoints[FRONTKNEE2].posn;

	H1Initial = initialJoints[HIP1].posn;
	H2Initial = initialJoints[HIP2].posn;

	//BF1Initial = vec3(FF1Initial[0],FF1Initial[1]-2,FF1Initial[2]);
	BA1Initial = vec3(FF1Initial[0],FF1Initial[1]-2,FF1Initial[2]-0.2);
	BK1Initial = initialJoints[BACKKNEE1].posn;
	//BF2Initial = vec3(FF2Initial[0],FF2Initial[1]-2,FF2Initial[2]);
	BA2Initial = vec3(FF2Initial[0],FF2Initial[1]-2,FF2Initial[2]-0.2);
	BK2Initial = initialJoints[BACKKNEE2].posn;

	S1Initial = initialJoints[SHOULDER1].posn;
	S2Initial = initialJoints[SHOULDER2].posn;
	SpInitial = initialJoints[SPINE].posn;
	PInitial = initialJoints[PELVIS].posn;
	HeInitial = initialJoints[HEAD].posn;

	//And Go!
	glutMainLoop();
}
