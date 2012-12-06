
#include "VehicleDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"
GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{

        VehicleDemo* vehicleDemo = new VehicleDemo;

        vehicleDemo->initPhysics(); 
		vehicleDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,640,480,"Racer - CS 455", vehicleDemo);
}

