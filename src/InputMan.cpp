//	Class:		InputMan
//	Updated:	11/26/12
//	Project:	CS 455 Game - Racer
//
//	Acts as manager for handling input.

#include "../inc/InputMan.h"

InputMan* InputMan::instance = NULL;
bool specialKeys[1000] = {0};
int fullscreen = 0;

InputMan::InputMan()
{
	glutKeyboardFunc(GLKeyDown);
	glutKeyboardUpFunc(GLKeyUp);
	glutSpecialFunc(SpecialKeys);
	glutSpecialUpFunc(SpecialKeysUp);
}

InputMan* InputMan::GetInstance()
{
	if(instance == NULL)
	{
		instance = new InputMan();
	}
	return instance;
}

void InputMan::UpdateInput()
{
	GetInstance()->ResetInput();	
	GetInstance()->HandleKeyboardInput();
}

float InputMan::GetMovementForward()
{
	return GetInstance()->forward;
}

float InputMan::GetMovementBackward()
{
	return GetInstance()->back;
}

float InputMan::GetMovementLeft()
{
	return GetInstance()->left;
}

float InputMan::GetMovementRight()
{
	return GetInstance()->right;
}

bool InputMan::GetAccel()
{
	return GetInstance()->accel;
}

bool InputMan::GetBrake()
{
	return GetInstance()->brake;
}

void InputMan::HandleKeyboardInput(){
	if(specialKeys[GLUT_KEY_LEFT])
	{
		std::cout << "Key left pressed" << std::endl;
		left = 1;
	}
	if(specialKeys[GLUT_KEY_RIGHT])
	{
		std::cout << "Key right pressed" << std::endl;
		right = 1;
	}
	if(specialKeys[GLUT_KEY_UP])
	{
		std::cout << "Key up pressed" << std::endl;
		forward = 1;
	}
	if(specialKeys[GLUT_KEY_DOWN])
	{
		std::cout << "Key down pressed" << std::endl;
		back = 1;
	}
	if(specialKeys[(int)' '])
	{
		cout << "Spacebar pressed" << endl;
		accel = 1;
	}
}

void InputMan::ResetInput()
{
	forward = 0;
	back = 0;
	left = 0;
	right = 0;
	accel = 0;
	brake = 0;
}

GLvoid SpecialKeys(int key, int x, int y){
	switch(key){
		case GLUT_KEY_LEFT:
			specialKeys[GLUT_KEY_LEFT] = 1;
			break;
		case GLUT_KEY_RIGHT:
			specialKeys[GLUT_KEY_RIGHT] = 1;
			break;
		case GLUT_KEY_UP:
			specialKeys[GLUT_KEY_UP] = 1;
			break;
		case GLUT_KEY_DOWN:
			specialKeys[GLUT_KEY_DOWN] = 1;
			break;
		case KEYBOARD_SPACE:
			specialKeys[KEYBOARD_SPACE] = 1;
			break;
		default:
			break;
	}
}

GLvoid SpecialKeysUp(int key, int x, int y){
	switch(key){
		case GLUT_KEY_LEFT:
			specialKeys[GLUT_KEY_LEFT] = 0;
			break;
		case GLUT_KEY_RIGHT:
			specialKeys[GLUT_KEY_RIGHT] = 0;
			break;
		case GLUT_KEY_UP:
			specialKeys[GLUT_KEY_UP] = 0;
			break;
		case GLUT_KEY_DOWN:
			specialKeys[GLUT_KEY_DOWN] = 0;
			break;
		case KEYBOARD_SPACE:
			specialKeys[KEYBOARD_SPACE] = 0;
			break;
		default:
			break;
	}
}

GLvoid GLKeyDown(unsigned char key, int x, int y)
{
	// if "esc" is pressed close
	if(key == KEYBOARD_ESC)
		throw NORMAL_EXIT_GLUT_LOOP;

	// if "f" key is pressed go into fullscreen mode
	if(key == KEYBOARD_F){
		if(fullscreen){
			fullscreen = 0;
			glutReshapeWindow(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);
			glutPositionWindow(100,100);
		}
		else{
			glutFullScreen();
			fullscreen = 1;
		}
		return;
	}

	SpecialKeys((int)key, x, y);
}

GLvoid GLKeyUp(unsigned char key, int x, int y)
{	
	SpecialKeysUp((int)key, x, y);
}