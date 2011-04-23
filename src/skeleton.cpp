#include "skeleton.h"

#include <fstream>
#include <string>
#include <sstream>

void Animation::setJoints(vector<Joint> &joints, double frame) {
    // TODO: Set joints by interpolating frames
	int curFrame = frame;
	cout << frame << endl;
	for(int i=0; i<joints.size(); i++) {
		//sets the orientation of each joint by interpolating between adjacent frames, with the interp value as a 
		//decimal from 0 to 1
		joints[i].orient = orientations[curFrame][i].nlerp(orientations[curFrame+1][i], frame-curFrame);
		//need to figure out how to change positions of joints to correct position (i think)
	}
}


// note: if you only do one ik implementation, you can ignore the "method" argument here.
void Skeleton::inverseKinematics(int j, vec3 pos, int method) {
    // get chain of joints going back to root
    vector<Joint*> chain = getChain(j);
    // note you can use this function after changing joint orientations in the chain to find the new joint positions & frames:
    //    void updateChainFrames(vector<Joint*> &chain);
    for(int i=0; i<(chain.size()-1); i++ ){
		vec3 end = chain[0]->posn;
		vec3 jparent = chain[i+1]->posn;
		vec3 dirBone = end-jparent; //direction of the bone currently in terms of the parent's local space
		dirBone=dirBone.normalize();
		vec3 goalDir = pos-jparent; //direction from the parent to the goal in the parent's local space
		goalDir=goalDir.normalize();
		if((goalDir == vec3(0)) || (dirBone == vec3(0))){
			cout << "passing zero vectors to rotation" << endl;
		}
		quat rotation= rotation.getRotation(dirBone,goalDir);
		rotation=rotation.normalize();
		quat localRotation = chain[i]->l2w.conjugate() * rotation * chain[i]->l2w;
		localRotation=localRotation.normalize();
		chain[i]->orient = chain[i]->orient*localRotation;
		chain[i]->orient = chain[i]->orient.normalize();
		updateChainFrames(chain);
	}
}


void Skeleton::updateSkin(Mesh &mesh) {
    updateJointPosnAndFrame(root);

    vector<Vert> &meshVerts = mesh.verts;
    for (vector<Vert>::iterator it = meshVerts.begin(); it != meshVerts.end(); ++it) {
		
	vec3 newpos = vec3(0);
	vector<BoneWeight> &weights = it->weights;
	for(vector<BoneWeight>::iterator it2 = weights.begin(); it2 != weights.end(); it2++){
		vec3 pos_temp = joints[it2->joint].localToWorld(it2->jointLocalPos);	
		newpos += it2->weight * pos_temp; 	
	}
	it->p = newpos;

    }
}







void Skeleton::saveRestPose() {
    restPose.resize(joints.size());
    for (size_t i = 0; i < joints.size(); i++) {
        restPose[i] = joints[i].orient;
    }
}
void Skeleton::resetPose() {
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i].orient = restPose[i];
    }
    updateJoints();
}

// Load a skeleton file
void Skeleton::loadPinocchioFile(const char *skelfile) {
    clear();
    ifstream f(skelfile);

    string line;
	while (getline(f,line)) {
		if (line.empty())
			continue;
		stringstream ss(line);
		int id, parent;
        vec3 posn;
        if (ss >> id >> posn[0] >> posn[1] >> posn[2] >> parent) {
            assert(id == int(joints.size()));
            joints.push_back(Joint(parent, posn));
            if (parent == -1)
                root = id;
        }
    }

    // init lengths and children array
    for (size_t i = 0; i < joints.size(); i++) {
        int parent = joints[i].parent;
        if (parent > -1) {
            joints[i].length = (joints[i].posn - joints[parent].posn).length();
            joints[parent].children.push_back(int(i));
        }
    }

    updateOrientationAndFrameFromPosn(root);

    saveRestPose();
}
// Load a bone weight file (attaching the skeleton to a mesh)
void Skeleton::initBoneWeights(const char *skinfile, Mesh &mesh, double threshWeight) {
    ifstream f(skinfile);

    vector<Vert> &meshVerts = mesh.verts;
    updateJointPosnAndFrame(root);

    // expect one line per vertex
    string line;
    size_t vertind = 0;
	while (getline(f,line)) {
		if (line.empty())
			continue;
		stringstream ss(line);
        
        vector<BoneWeight> &weights = meshVerts[vertind].weights; 

        int id = 0;
        double weight;
        double totalWeight = 0;
        while (ss >> weight) {
            if (id == root) id++;
            if (weight > threshWeight) {
                totalWeight += weight;
                weights.push_back(BoneWeight(id, weight, joints[id].worldToLocal(meshVerts[vertind].p)));
            }
            id++;
        }
        
        assert(totalWeight > 0); // double check you didn't throw away too much ...

        // renormalize weights so they sum to 1 after discarding all small weights.
        for (vector<BoneWeight>::iterator it = weights.begin(); it != weights.end(); ++it) {
            it->weight /= totalWeight;
        }

        vertind++;
    }
}

// offset and scale joint positions; useful when adjusting mesh scale/offset
void Skeleton::offsetAndScale(vec3 offset, double scale) {
    for (vector<Joint>::iterator it = joints.begin(); it != joints.end(); ++it) {
        it->posn += offset;
        it->posn *= scale;
        it->length *= scale;
    }
}

// update joint positions and frames using joint orientations
void Skeleton::updateJointPosnAndFrame(int joint, quat acc) {
    Joint &j = joints[joint];
    acc = acc * j.orient;
    j.l2w = acc;

    if (j.parent != -1) {
        Joint &p = joints[j.parent];
        vec3 bone(0,j.length,0);
        bone = acc.rotate(bone);
        vec3 oldposn = j.posn;
        j.posn = p.posn + bone;
    }
    
    for (vector<int>::iterator it = j.children.begin(); it != j.children.end(); ++it) {
        int ch = *it;
        updateJointPosnAndFrame(ch, acc);
    }
}
// update joint orientations and frames using joint positions
void Skeleton::updateOrientationAndFrameFromPosn(int joint, quat acc) {
    Joint &j = joints[joint];

    if (j.parent != -1) {
        Joint &p = joints[j.parent]; // parent has orientation and frame set
        vec3 boneDir = p.worldToLocal(j.posn);
        j.orient = quat::getRotation(vec3(0,1,0), boneDir);
        acc = acc * j.orient;
    }
    j.l2w = acc;

    for (vector<int>::iterator it = j.children.begin(); it != j.children.end(); ++it) {
        int ch = *it;
        updateOrientationAndFrameFromPosn(ch, acc);
    }
}

void Skeleton::render(int highlightChainToJoint) {
    updateJointPosnAndFrame(root);

    glDisable(GL_LIGHTING); // just draw plain colored lines
    glDisable(GL_DEPTH_TEST); // make it show through the mesh

    if (highlightChainToJoint) {
        glColor3d(1,0,0);
        glLineWidth(10);
        glBegin(GL_LINE_STRIP);
        vector<Joint*> chain = getChain(highlightChainToJoint);
        for (size_t i = 0; i < chain.size(); i++) {
            glVertex3dv(&chain[i]->posn[0]);
        }
        glEnd();
    }
    
    glColor3d(1,1,0);
    glLineWidth(3);
    glBegin(GL_LINES);
    for (size_t i = 0; i < joints.size(); i++) {
        vec3 posn = joints[i].posn;
        int parent = joints[i].parent;
        if (parent > -1) {
            vec3 pposn = joints[parent].posn;
            glVertex3dv(&posn[0]);
            glVertex3dv(&pposn[0]);
        }
    }
    glEnd();

    glPointSize(10);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < joints.size(); i++) {
        glVertex3dv(&joints[i].posn[0]);
    }
    glEnd();

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
}

// choose a joint within the selection radius of the mouse
int Skeleton::pickJoint(double &depth, vec2 mouse, double selectionRadius) {
    double modelview[16], projection[16];
    int viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    mouse[1] = viewport[3] - mouse[1];

    int bestJoint = -1;
    double bestDist = selectionRadius*selectionRadius;
    for (size_t i = 0; i < joints.size(); ++i) {
        vec3 p = joints[i].posn;
        vec2 s;
        double sz;
        gluProject(p[0], p[1], p[2], 
            modelview, projection, viewport, 
            &s[0], &s[1], &sz);
        vec2 diff = mouse - s;
        
        if (diff.length2() <= bestDist) {
            bestDist = diff.length2();
            bestJoint = int(i);
            depth = sz;
        }
    }

    return bestJoint;
}
// project a screen pos into the world at the given depth 
vec3 Skeleton::getPos(vec2 mouse, double depth) {
    double modelview[16], projection[16];
    int viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    mouse[1] = viewport[3] - mouse[1];

    double x,y,z;
    gluUnProject(mouse[0], mouse[1], depth, 
                 modelview, projection, viewport,
                 &x, &y, &z);
    return vec3(x,y,z);
}

// get chain of joints from j to root, useful for ik
vector<Joint*> Skeleton::getChain(int j) {
    vector<Joint*> chain;
    int nextj = j;
    while (nextj != -1) {
        chain.push_back(&joints[nextj]);
        nextj = joints[nextj].parent;
    }
    return chain;
}
// update just the positions and frames in the given chain
void Skeleton::updateChainFrames(vector<Joint*> &chain) {
    quat acc;
    // root is at end of chain; go from root to front
    for (int i = int(chain.size())-1; i >= 0; i--) {
        Joint &j = *chain[i];
        acc = acc * j.orient;
        j.l2w = acc;
        if (j.parent != -1) {
            Joint &p = joints[j.parent];
            vec3 bone(0,j.length,0);
            bone = acc.rotate(bone);
            j.posn = p.posn + bone;
        }
    }
}
