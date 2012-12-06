/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

// xbox controller includes
// No MFC
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
//#include <MMSystem.h>
#include <Xinput.h>
#include <limits.h>

/// September 2006: VehicleDemo is work in progress, this file is mostly just a placeholder
/// This VehicleDemo file is very early in development, please check it later
/// One todo is a basic engine model:
/// A function that maps user input (throttle) into torque/force applied on the wheels
/// with gears etc.
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

//
// By default, Bullet Vehicle uses Y as up axis.
// You can override the up axis, for example Z-axis up. Enable this define to see how to:
//#define FORCE_ZAXIS_UP 1
//

#ifdef FORCE_ZAXIS_UP
		int rightIndex = 0; 
		int upIndex = 2; 
		int forwardIndex = 1;
		btVector3 wheelDirectionCS0(0,0,-1);
		btVector3 wheelAxleCS(1,0,0);
#else
		int rightIndex = 0;
		int upIndex = 1;
		int forwardIndex = 2;
		btVector3 wheelDirectionCS0(0,-1,0);
		btVector3 wheelAxleCS(-1,0,0);
#endif

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging

#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"
#include "VehicleDemo.h"

const int maxProxies = 32766;
const int maxOverlap = 65535;

///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts
float	gEngineForce = 0.f;
float	gBreakingForce = 0.f;

float	maxEngineForce = 2500.f;//this should be engine/velocity dependent
float	maxBreakingForce = 200.f;

float	gVehicleSteering = 0.f;
float	steeringIncrement = .1f;
float	steeringClamp = .25f;
float	wheelRadius = 0.5f;
float	wheelWidth = 0.4f;
float	wheelFriction = 10;//BT_LARGE_FLOAT;
float	suspensionStiffness = 20.f;
float	suspensionDamping = 2.3f;
float	suspensionCompression = 4.4f;
float	rollInfluence = 0.1f;//1.0f;

int score = 0;


btScalar suspensionRestLength(0.6);

#define CUBE_HALF_EXTENTS 1

////////////////////////////////////




VehicleDemo::VehicleDemo()
:
m_carChassis(0),
m_indexVertexArrays(0),
m_vertices(0),
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
m_maxCameraDistance(10.f),
hornSound(NULL),
idleSound(NULL),
accelSound(NULL)
{
	m_vehicle = 0;
	m_wheelShape = 0;
	m_cameraPosition = btVector3(30,30,30);
    m_debugMode |= btIDebugDraw::DBG_NoHelpText;
    resetInput();
}

VehicleDemo::~VehicleDemo()
{
		//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;

	delete m_vehicleRayCaster;

	delete m_vehicle;

	delete m_wheelShape;

	delete m_chassisShape;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

}

void VehicleDemo::initPhysics()
{
	
#ifdef FORCE_ZAXIS_UP
	m_cameraUp = btVector3(0,0,1);
	m_forwardAxis = 1;
#endif

	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	m_collisionShapes.push_back(groundShape);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
#ifdef FORCE_ZAXIS_UP
	m_dynamicsWorld->setGravity(btVector3(0,0,-10));
#endif 

	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
btTransform tr;
tr.setIdentity();

//either use heightfield or triangle mesh
#define  USE_TRIMESH_GROUND 1
#ifdef USE_TRIMESH_GROUND
	int i;

const float TRIANGLE_SIZE=20.f;

	//create a triangle-mesh ground
	int vertStride = sizeof(btVector3);
	int indexStride = 3*sizeof(int);

	const int NUM_VERTS_X = 20;
	const int NUM_VERTS_Y = 20;
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
	
	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	m_vertices = new btVector3[totalVerts];
	int*	gIndices = new int[totalTriangles*3];

	

	for ( i=0;i<NUM_VERTS_X;i++)
	{
		for (int j=0;j<NUM_VERTS_Y;j++)
		{
			float wl = .2f;
			//height set to zero, but can also use curved landscape, just uncomment out the code
			float height = 0.f;//20.f*sinf(float(i)*wl)*cosf(float(j)*wl);
#ifdef FORCE_ZAXIS_UP
			m_vertices[i+j*NUM_VERTS_X].setValue(
				(i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
				(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE,
				height
				);

#else
			m_vertices[i+j*NUM_VERTS_X].setValue(
				(i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
				height,
				(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
#endif

		}
	}

	int index=0;
	for ( i=0;i<NUM_VERTS_X-1;i++)
	{
		for (int j=0;j<NUM_VERTS_Y-1;j++)
		{
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
		}
	}
	
	/*m_indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
		gIndices,
		indexStride,
		totalVerts,(btScalar*) &m_vertices[0].x(),vertStride);
		*/
	m_indexVertexArrays = ObjectMan::GetId("environment")->GetIndexVertexArray();
	bool useQuantizedAabbCompression = true;
	groundShape = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression);
	
	tr.setOrigin(btVector3(0,-4.5f,0));

#else
	//testing btHeightfieldTerrainShape
	int width=128;
	int length=128;
	unsigned char* heightfieldData = new unsigned char[width*length];
	{
		for (int i=0;i<width*length;i++)
		{
			heightfieldData[i]=0;
		}
	}

	char*	filename="heightfield128x128.raw";
	FILE* heightfieldFile = fopen(filename,"r");
	if (!heightfieldFile)
	{
		filename="../../heightfield128x128.raw";
		heightfieldFile = fopen(filename,"r");
	}
	if (heightfieldFile)
	{
		int numBytes =fread(heightfieldData,1,width*length,heightfieldFile);
		//btAssert(numBytes);
		if (!numBytes)
		{
			printf("couldn't read heightfield at %s\n",filename);
		}
		fclose (heightfieldFile);
	}
	

	btScalar maxHeight = 20000.f;
	
	bool useFloatDatam=false;
	bool flipQuadEdges=false;

	btHeightfieldTerrainShape* heightFieldShape = new btHeightfieldTerrainShape(width,length,heightfieldData,maxHeight,upIndex,useFloatDatam,flipQuadEdges);;
	groundShape = heightFieldShape;
	
	heightFieldShape->setUseDiamondSubdivision(true);

	btVector3 localScaling(20,20,20);
	localScaling[upIndex]=1.f;
	groundShape->setLocalScaling(localScaling);

	tr.setOrigin(btVector3(0,-64.5f,0));

#endif //
    
	m_collisionShapes.push_back(groundShape);

	//create ground object
	btRigidBody* groundBody = localCreateRigidBody(0,tr,groundShape);
    
/*
#ifdef FORCE_ZAXIS_UP
//   indexRightAxis = 0; 
//   indexUpAxis = 2; 
//   indexForwardAxis = 1; 
	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,2.f, 0.5f));
	btCompoundShape* compound = new btCompoundShape();
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,0,1));
#else
	btCollisionShape* chassisShape = new btBoxShape(btVector3(.8f, 0.3f,1.2f));
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,1,0));

	btTriangleIndexVertexArray* carMeshArray = ObjectMan::GetId("car")->GetIndexVertexArray();
	m_chassisShape = new btBvhTriangleMeshShape(carMeshArray, true);
/*
#endif
*/
	btTriangleIndexVertexArray* carMeshArray = ObjectMan::GetId("car")->GetIndexVertexArray();
	btCollisionShape* chassisShape = new btConvexTriangleMeshShape(carMeshArray);
	m_chassisShape = new btBvhTriangleMeshShape(carMeshArray, true);
	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(0, 0, 0));

	compound->addChildShape(localTrans,chassisShape);

	tr.setOrigin(btVector3(0.0f,0.45f,-0.1f));

	m_carChassis = localCreateRigidBody(800,tr,compound);//chassisShape);
	//m_carChassis->setDamping(0.2,0.2);
	
	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
	
	clientResetScene();

	/// create vehicle
	{
		
		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);
		
		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);
		
		m_dynamicsWorld->addVehicle(m_vehicle);

		float connectionHeight = 1.2f;

	
		bool isFrontWheel=true;

		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

#ifdef FORCE_ZAXIS_UP
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif

		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif

		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif //FORCE_ZAXIS_UP
		isFrontWheel = false;
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}


    // Setup a cube of boxes
    createCube(0, -10, 0, 10, 10, 10, 5);
    createCube(-25, -24, -3, 5, 5, 5);
    // Setup some other random cubes...
    for (int i = 0; i < 4; i++) {
        createCube(-100 + rand()/(double)RAND_MAX * 200, -24, -100 + rand()/(double)RAND_MAX * 200, 5, 5, 5);
    }
	
	setCameraDistance(26.f);

    // And then somewhere after you construct the world:
    m_dynamicsWorld->setInternalTickCallback(tickCallback);

}


//to be implemented by the demo
void VehicleDemo::renderme()
{
	updateCamera();

	btScalar m[16];
	int i;


	btVector3 wheelColor(0.1,0.1,0.1);

	btVector3	worldBoundsMin,worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);

	bool inAir = false;

	for (i=0;i<m_vehicle->getNumWheels();i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i,true);
		//draw wheels (cylinders)
		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
		m_shapeDrawer->drawOpenGL(m,m_wheelShape,wheelColor,getDebugMode(),worldBoundsMin,worldBoundsMax);

		btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
		wheel.m_frictionSlip = wheelFriction;
	}

	btDefaultMotionState* myMotionState = (btDefaultMotionState*)m_vehicle->getRigidBody()->getMotionState();
	myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);

	btVector3 wireColor(1.f,1.0f,0.5f); //wants deactivation

	//m_shapeDrawer->drawOpenGL(m, m_chassisShape, wireColor, getDebugMode(), worldBoundsMin, worldBoundsMax, TextureMan::GetInstance()->Get("car"));
	
	//glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	//glDisable(GL_COLOR_MATERIAL);
	//glDisable(GL_TEXTURE_GEN_T);
	//glDisable(GL_TEXTURE_GEN_R);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat lightPosition[] = {50.0f, 30.0f, 50.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
			
	DrawObject(m, CAR);

	// UI TEXT CODE
	{
		// Calculate Score
		int ao = 0;
		btCollisionObjectArray objects = m_dynamicsWorld->getCollisionObjectArray();
		for(int i=0;i<objects.size();i++)
			if(objects[i]->isActive())
				ao++;
		score += (ao-1);
		// Draw the Score to the screen
		DemoApplication::printw2d(20, 20, "SCORE: %i", score);
		DemoApplication::printw2d(20, 40, "SKID VALUE: %i", m_vehicle->getWheelInfo(0).m_skidInfo);
		DemoApplication::printw2d(20, 60, "wheelFriction: %f", wheelFriction);
		DemoApplication::printw2d(20, 80, "SPEED: %f km/h", m_vehicle->getCurrentSpeedKmHour());
	}

	DemoApplication::renderme((btCollisionObject*)m_vehicle->getRigidBody());
	//  Uncomment this line to see the convex hull rendered with the car
	//DemoApplication::renderme();
}

void VehicleDemo::DrawObject(btScalar* mat, string id)
{
	glPushMatrix();
	glMultMatrixf(mat);
	Geometry* geo = ObjectMan::GetId(id);
	geo->Draw();
	glPopMatrix();
}

void VehicleDemo::clientMoveAndDisplay()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    float delta = getDeltaTimeMicroseconds();
    float dt = delta * 0.000001f;
    //printf("Delta: %f\n", dt);
    
    // Speed cap - 200 Km/H
    btVector3 velocity = m_vehicle->getRigidBody()->getLinearVelocity();
    btScalar speed = velocity.length();
    btScalar maxSpeed = 75.0f;
    /* Appears to break things.
    if (speed > maxSpeed) {
        velocity *= speed / maxSpeed;
        speed = velocity.length();
        m_vehicle->getRigidBody()->setLinearVelocity(velocity);
    }
    */
    
    // Update vibration state, currently purely based on speed.
    if (speed > 25.0f) {
        setVibrate(min((speed - 25.0f) / (maxSpeed - 25.0f), 1.0f));
    } else {
        setVibrate(0.0f);
    }

	// Get direction of wheel
    if (steering != 0.0f) {
        gVehicleSteering = min(steeringClamp * fabs(steering), max(-steeringClamp * fabs(steering), m_vehicle->getSteeringValue(0) - steering * steeringIncrement * 4 * dt));
    } else {
        gVehicleSteering = m_vehicle->getSteeringValue(0);
        if (gVehicleSteering < 0) {
            gVehicleSteering += steeringIncrement * dt;
        } else {
            gVehicleSteering -= steeringIncrement * dt;
        }
    }
    gEngineForce = accel * maxEngineForce * (boost > 0.0f ? boost * 5.0f : 1.0f);
    gBreakingForce = brake * maxBreakingForce;
    if (speed < 5.0f && accel < 1.0f) {
        gEngineForce = brake * maxEngineForce * -0.5f;
    }

	{
		int wheelIndex = 2;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 3;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);


		wheelIndex = 0;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		wheelIndex = 1;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);

	}

    // If the sounds haven't been initialized, init them and load things
    if (hornSound == NULL) {
        // Initialize BASS
        //printf("glut hwnd: %i\n", FindWindowA("glut", NULL));
        BASS_Init(-1, 44100, 0, FindWindowA("glut", NULL), 0); // initialize default output device
        hornSound = BASS_StreamCreateFile(false, "horn.mp3", 0, 0, 0); // create a stream to play an audio file
        idleSound = BASS_StreamCreateFile(false, "idle.mp3", 0, 0, 0);
        accelSound = BASS_StreamCreateFile(false, "accel.mp3", 0, 0, 0);
    }

    // Play sounds based on the input
    if (accel > 0.f) {
        BASS_ChannelPlay(accelSound, false);
        BASS_ChannelStop(idleSound);
    } else {
        BASS_ChannelPlay(idleSound, false);
        BASS_ChannelSetPosition(accelSound, 0, BASS_POS_BYTE);
        BASS_ChannelStop(accelSound);
    }
    if (horn) {
        BASS_ChannelPlay(hornSound, true);
        horn = false;
    }


	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 2;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
		

//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_FEEDBACK

	}

	
	




#ifdef USE_QUICKPROF 
#ifndef NDEBUG
        btProfiler::beginBlock("render"); 
#endif
#endif //USE_QUICKPROF 


	renderme(); 

#ifndef NDEBUG
	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
#endif

#ifdef USE_QUICKPROF 
#ifndef NDEBUG
        btProfiler::endBlock("render"); 
#endif
#endif 
	

	glFlush();
	glutSwapBuffers();

    // Reset the Input from this cycle
    pollInput();
}



void VehicleDemo::displayCallback(void) 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

#ifndef NDEBUG
//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
#endif

	glFlush();
	glutSwapBuffers();
}



void VehicleDemo::clientResetScene()
{
	gVehicleSteering = 0.f;
	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0,0,0));
	m_carChassis->setAngularVelocity(btVector3(0,0,0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());
	if (m_vehicle)
	{
		m_vehicle->resetSuspension();
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i,true);
		}
	}

}



void VehicleDemo::specialKeyboardUp(int key, int x, int y)
{
   switch (key) 
    {
    case GLUT_KEY_UP :
		{
            forward = 0.f;
            accel = 0.f;
			//gEngineForce = 0.f;
		break;
		}
	case GLUT_KEY_DOWN :
		{
            brake = 0.f;
			//gBreakingForce = 0.f; 
            //gEngineForce = 0.f;
		break;
		}
    case GLUT_KEY_LEFT:
        steering = 0.0f;
        break;
    case GLUT_KEY_RIGHT:
        steering = 0.0f;
        break;
	default:
		DemoApplication::specialKeyboardUp(key,x,y);
        break;
    }

}


void VehicleDemo::specialKeyboard(int key, int x, int y)
{

//	printf("key = %i x=%i y=%i\n",key,x,y);

    switch (key) 
    {
    case GLUT_KEY_LEFT : 
		{
            steering = -1.0f;
			//gVehicleSteering += steeringIncrement;
			//if (	gVehicleSteering > steeringClamp)
			//		gVehicleSteering = steeringClamp;

		break;
		}
    case GLUT_KEY_RIGHT : 
		{
            steering = 1.0f;
			//gVehicleSteering -= steeringIncrement;
			//if (	gVehicleSteering < -steeringClamp)
			//		gVehicleSteering = -steeringClamp;

		break;
		}
    case GLUT_KEY_UP :
		{
            forward = 1.0f;
            accel = 1.0f;
			//gEngineForce = maxEngineForce;
			//gBreakingForce = 0.f;
		break;
		}
	case GLUT_KEY_DOWN :
		{		
            brake = 1.0f;
            /*
            // If stopped, apply a backwards force
            printf("Current Speed: %f\n", m_vehicle->getCurrentSpeedKmHour());
            if (m_vehicle->getCurrentSpeedKmHour() > 0.001) {
			    gBreakingForce = maxBreakingForce; 
			    gEngineForce = 0.f;
            } else {
                gEngineForce = maxEngineForce * -.5f;
                gBreakingForce = 0.f;
            }
            */
		break;
		}
	default:
		DemoApplication::specialKeyboard(key,x,y);
        break;
    }

//	glutPostRedisplay();


}



void	VehicleDemo::updateCamera()
{
	
//#define DISABLE_CAMERA 1
#ifdef DISABLE_CAMERA
	DemoApplication::updateCamera();
	return;
#endif //DISABLE_CAMERA

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	btTransform chassisWorldTrans;

	//look at the vehicle
	m_carChassis->getMotionState()->getWorldTransform(chassisWorldTrans);
	m_cameraTargetPosition = chassisWorldTrans.getOrigin();

	//interpolate the camera height
#ifdef FORCE_ZAXIS_UP
	m_cameraPosition[2] = (15.0*m_cameraPosition[2] + m_cameraTargetPosition[2] + m_cameraHeight)/16.0;
#else
	m_cameraPosition[1] = (15.0*m_cameraPosition[1] + m_cameraTargetPosition[1] + m_cameraHeight)/16.0;
#endif 

	btVector3 camToObject = m_cameraTargetPosition - m_cameraPosition;

	//keep distance between min and max distance
	float cameraDistance = camToObject.length();
	float correctionFactor = 0.f;
	if (cameraDistance < m_minCameraDistance)
	{
		correctionFactor = 0.15*(m_minCameraDistance-cameraDistance)/cameraDistance;
	}
	if (cameraDistance > m_maxCameraDistance)
	{
		correctionFactor = 0.15*(m_maxCameraDistance-cameraDistance)/cameraDistance;
	}
	m_cameraPosition -= correctionFactor*camToObject;
	
	  btScalar aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
        glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);

         glMatrixMode(GL_MODELVIEW);
         glLoadIdentity();

    gluLookAt(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2],
                      m_cameraTargetPosition[0],m_cameraTargetPosition[1], m_cameraTargetPosition[2],
                          m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());



}

void VehicleDemo::createCube(btScalar x, btScalar y, btScalar z, btScalar xCount, btScalar yCount, btScalar zCount, btScalar scaling) {
	//create a few dynamic rigidbodies
	// Re-using the same collision is better for memory usage and performance

	btBoxShape* colShape = new btBoxShape(btVector3(scaling*1,scaling*1,scaling*1));
	//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
	m_collisionShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();
    
	btScalar	mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass,localInertia);

	float start_x = x - xCount/2;
	float start_y = y;
	float start_z = z - zCount/2;

	for (int k=0;k<yCount;k++)
	{
		for (int i=0;i<xCount;i++)
		{
			for(int j = 0;j<zCount;j++)
			{
				startTransform.setOrigin(scaling*btVector3(
									btScalar(2.0*i + start_x),
									btScalar(20+2.0*k + start_y),
									btScalar(2.0*j + start_z)));

			
				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
				btRigidBody* body = new btRigidBody(rbInfo);
                body->setActivationState(WANTS_DEACTIVATION);

				m_dynamicsWorld->addRigidBody(body);
			}
		}
	}
}

void VehicleDemo::keyboardCallback(unsigned char key, int x, int y) {
	switch(key){
	case 'h':
		horn = true;
		break;
	case '+':
		wheelFriction += 1;
		break;
	case '-':
		wheelFriction -= 1;
		break;
    case 'b':
        boost = 1.0f;
        break;
	default:
		DemoApplication::keyboardCallback(key, x, y);
		break;
	}
}

void VehicleDemo::keyboardUpCallback(unsigned char key, int x, int y) {
    switch (key) {
    case 'b':
        boost = 0.0f;
        break;
    default:
        DemoApplication::keyboardUpCallback(key, x, y);
        break;
    }
}

// Tell the compiler to load XInput.lib for the xbox controller support
#pragma comment(lib, "XInput.lib")

void VehicleDemo::pollInput() {
    // Read the dpad using xinput, as the joystick code doesn't return the dpad data for some reason.
    XINPUT_STATE controllerState;
    ZeroMemory(&controllerState, sizeof(XINPUT_STATE));
    DWORD result = XInputGetState(0, &controllerState);

    // If no controller detected
    if (result != ERROR_SUCCESS) {
        return;
    }

    // Accelerate and brake
    if (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_A) {
        accel = 1.0f;
    } else {
        accel = 0.0f;
    }
    if (controllerState.Gamepad.wButtons & (XINPUT_GAMEPAD_B | XINPUT_GAMEPAD_X)) {
        brake = 1.0f;
    } else {
        brake = 0.0f;
    }

    // Boost
    if (controllerState.Gamepad.bRightTrigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD) {
        boost = controllerState.Gamepad.bRightTrigger / 255.0f;
    } else {
        boost = 0.0f;
    }

    // Horn!
    if (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_Y) {
        horn = true;
    }

    // Steering
    if (controllerState.Gamepad.sThumbLX > XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE || controllerState.Gamepad.sThumbLX < -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) {
        steering = controllerState.Gamepad.sThumbLX / (float)SHRT_MAX;
    } else {
        steering = 0.0f;
    }
}

void VehicleDemo::setVibrate(float percent) {
    // Xbox code aided by: http://www.codeproject.com/Articles/26949/Xbox-360-Controller-Input-in-C-with-XInput

    // Create a Vibraton State
    XINPUT_VIBRATION vibration;

    // Zeroise the Vibration
    ZeroMemory(&vibration, sizeof(XINPUT_VIBRATION));

    // Set the Vibration Values
    vibration.wLeftMotorSpeed = percent * USHRT_MAX;
    vibration.wRightMotorSpeed = percent * USHRT_MAX;

    // Vibrate the controller
    XInputSetState(0, &vibration);
}

void VehicleDemo::tickCallback(btDynamicsWorld *world, btScalar timeStep) {
    //Assume world->stepSimulation or world->performDiscreteCollisionDetection has been called
    /*
	int numManifolds = world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
		const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());
	
		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance()<0.f)
			{
				const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;
			}
		}
	}
    printf("%i things collided\n", numManifolds);
    */
}
