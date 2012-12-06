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
#ifndef VEHICLE_DEMO_H
#define VEHICLE_DEMO_H

class btVehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;

#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

#include "GlutDemoApplication.h"
#include "ObjectMan.h"
#include "Geometry.h"

// Bass sound library
#include "bass.h"


///VehicleDemo shows how to setup and use the built-in raycast vehicle
class VehicleDemo : public GlutDemoApplication
{
	public:

	btRigidBody* m_carChassis;
	btCollisionShape* m_chassisShape;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;

	
	btRaycastVehicle::btVehicleTuning	m_tuning;
	btVehicleRaycaster*	m_vehicleRayCaster;
	btRaycastVehicle*	m_vehicle;
	btCollisionShape*	m_wheelShape;

	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;


	VehicleDemo();

	virtual ~VehicleDemo();

	virtual void clientMoveAndDisplay();

	virtual void	clientResetScene();

	virtual void displayCallback();
	
	///a very basic camera following the vehicle
	virtual void updateCamera();

	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	void renderme();

	void initPhysics();

	static DemoApplication* Create()
	{
		VehicleDemo* demo = new VehicleDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

    void keyboardCallback(unsigned char key, int x, int y);

private:

    void createCube(btScalar x, btScalar y, btScalar z, btScalar xCount, btScalar yCount, btScalar zCount);

    // Horrible practice to not pull this into another class, but its easier to just reuse the code they gave us for now.
    // So here is the input manager's code, essentially.
	float forward, back, steering, accel, brake;
    bool horn;

    // This should poll any input and update the state methods that need them.
    void pollInput();

    // This is called at the start only. At runtime, input states should be updated either by the polling of the joystick or the keyboard events
    void resetInput() {
        forward = 0;
        back = 0;
        steering = 0;
        accel = 0;
        brake = 0;
        horn = false;
    }

    void setVibrate(float percent = 0.0f);

    HSTREAM hornSound;
    HSTREAM idleSound;
    HSTREAM accelSound;
};

#endif //VEHICLE_DEMO_H


