//	Class:		InputMan
//	Updated:	11/26/12
//	Project:	CS 455 Game - Racer
//
//	Acts as manager for handling input.

#ifndef INPUTMAN_H
#define INPUTMAN_H

#include <iostream>
#include "main.h"

using namespace std;

// Methods used for key recognition
GLvoid GLKeyDown(unsigned char key, int x, int y);
GLvoid GLKeyUp(unsigned char key, int x, int y);
GLvoid SpecialKeys(int key, int x, int y);
GLvoid SpecialKeysUp(int key, int x, int y);

class InputMan
{
private:
	static InputMan* instance;
	float forward, back, left, right, accel, brake;
	InputMan();
	void HandleKeyboardInput();
	void ResetInput();

public:
	static InputMan* GetInstance();
	static void UpdateInput();
	static float GetMovementForward();
	static float GetMovementBackward();
	static float GetMovementLeft();
	static float GetMovementRight();
	static bool GetAccel();
	static bool GetBrake();
};

#endif