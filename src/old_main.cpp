//	Class:		main
//	Updated:	11/18/12
//	Project:	CS 455 Game - Racer
//
//	Program entrance for Racer game

#include "../inc/main.h"

int windowWidth = DEFAULT_WINDOW_WIDTH;
int windowHeight = DEFAULT_WINDOW_HEIGHT;
char* windowName = PROGRAM_NAME;

// renamed to old_main as to not interfere with main.
int old_main(int argc, char* argv[]){	

	cout << PROGRAM_NAME << " - " << VERSION << endl
		<< "BYU CS 455 - Fall 2012" << endl << endl
		<< "Created by:" << endl
		<< "\tJosh Davis" << endl
		<< "\tTyler Gill" << endl
		<< "\tMorgan Strong" << endl
		<< "\tJames Williams" << endl << endl;
	
	glutInit(&argc, argv);
	/* setting up double buffering rbg and depth for this */
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(windowWidth, windowHeight);
	glutInitWindowPosition(100, 100);
	/* this name needs to match the name of the window in DrawGLScene */
	glutCreateWindow(windowName);

	InitGL();

	/* the draw function will draw once and no more */
	glutDisplayFunc(DrawGLScene);
	/* this is the idle function it gets called as much as possible */
	glutIdleFunc(IdleGLScene);
	/* This is called everytime the window is altered */
	glutReshapeFunc(ReSizeGLScene);
	/* this gets called on a keyboard event */
	//glutKeyboardFunc(GLKeyDown);

	//glutSpecialFunc(SpecialKeys);

	//glutSpecialUpFunc(SpecialKeysUp);

	// Game initialization
	ObjectMan* man = ObjectMan::GetInstance();
	InputMan* inman = InputMan::GetInstance();

	cout << "Running Game..." << endl << endl;

	/* Gets the loop rolling */
	try{
		/* 
		 * This loop wont return ever so we are going to play some tricks
		 * to make it look like it exits when it is actually done. That is why
		 * we use the try catch.
		 */
		// TODO: Override the x button on the window.
		glutMainLoop();
	}
	catch(const char* msg){
		printf(msg);

		HWND hwnd = FindWindow("glut", windowName);
		ShowWindow(hwnd, SW_HIDE);
		Sleep(500);
	}

	return 0;
}


/* insert your draw code in this function */
GLvoid DrawGLScene(){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	/* This is your normal draw code. Put your code here. This should be kept no matter the OS you are using */
	glViewport(0, 0, windowWidth, windowHeight);
	glTranslatef(0.0f, -1.0f, -5.0f);

	vector<Geometry*> models = ObjectMan::GetObjects();
	
	for(int i = 0; i < models.size(); i++)
	{
		models[i]->Draw();
	}

	glFlush();
	glutSwapBuffers();
}

/* checks for joystick input then draws */
GLvoid IdleGLScene(){
	InputMan::UpdateInput();
	ObjectMan::UpdateObjectsWithInput();
	DrawGLScene();
}



/*
 * This function handles all normal key presses on the keyboard. If you need
 * to capture special keys like, ctrl, shift, F1, F2, F..., or arrow keys use
 * the special keys function
 */



#ifdef WIN32

/* some joystick example code. Operating systems other than windows will need other options for the joy stick also */
GLvoid PollJoyStick(){
	JOYINFOEX stuff;
	ZeroMemory(&stuff, sizeof(stuff));
	stuff.dwSize = sizeof(stuff);
	stuff.dwFlags |= JOY_RETURNALL;
	joyGetPosEx(JOYSTICKID1, &stuff);

	/* axis forcing them between 0 and 1 you need to check for dead zones */
	float axisY = (float)(stuff.dwYpos - 32767.0f) / 32768;
	float axisX = (float)(stuff.dwXpos - 32767.0f) / 32768;
	float axisZ = (float)(stuff.dwZpos - 32767.0f) / 32768;
	float axisR = (float)(stuff.dwRpos - 32767.0f) / 32768;
	/* 
	 * the buttons are set up on powers of 2
	 * i.e. button 1 = 1 or (2 ^ (button - 1)) or (2 ^ 0)
	 * button 2 = 2 or (2 ^ (2 - 1)) or (2 ^ 1)
	 * button 3 = 4 or (2 ^ (3 - 1)) or (2 ^ 2)
	 * etc...
	 * You can use flags to test them
	 * e.g. (buttons & 0x00000001) or (buttons & 1) would give me true if 1 is pressed and false other wise or
	 * (buttons & 0x00000004) or (buttons & 4) would give me true if button 3 is pressed
	 * This allows the same int to pack multiple button pressed into one unsigned int
	 * so buttons would be 3 if buttons 1 and 2 were pressed. You can do (buttons & 1) would give true
	 * and (buttons & 2) would give true but (buttons & 8) would give false;
	 */
	unsigned int buttons = stuff.dwButtons;

	/*std::cout << "My joystick outputs: " << std::endl
		<< "Axis Data: " << std::endl
		<< "\tAxisY: " << axisY << std::endl
		<< "\tAxisX: " << axisX << std::endl
		<< "\tAxisZ: " << axisZ << std::endl
		<< "\tAxisR: " << axisR << std::endl << std::endl
		<< "Button Data:" << std::endl
		<< "\tMy Button Data is: " << buttons << std::endl;*/
}

#endif // WIN32


/* 
 * important GL initialization. You can mess with these,
 * but you don't need to for any of the projects
 */
GLvoid InitGL(){
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(0.5f, 0.5f, 0.5f, 0.5f);				// grey Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_CULL_FACE);
	glShadeModel(GL_SMOOTH);
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glEnable(GL_COLOR_MATERIAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

/* this gets called everytime your window resizes or moves */
GLvoid ReSizeGLScene(int width, int height){
	if(height == 0)
		height = 1;
	glViewport(0,0,width,height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(45.0f, (GLfloat)width/(GLfloat)height, 0.1f, 2000.0f);

	windowWidth = width;
	windowHeight = height;
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

#ifdef WIN32
/* Operating systems different from windows will need other options for loading bitmaps */
bool NeHeLoadBitmap(LPTSTR szFileName, GLuint &texid)					// Creates Texture From A Bitmap File
{
	HBITMAP hBMP;														// Handle Of The Bitmap
	BITMAP	BMP;														// Bitmap Structure
	glGenTextures(1, &texid);											// Create The Texture
	hBMP=(HBITMAP)LoadImage(GetModuleHandle(NULL), szFileName, IMAGE_BITMAP, 0, 0, LR_CREATEDIBSECTION | LR_LOADFROMFILE );
	if (!hBMP)															// Does The Bitmap Exist?
		return FALSE;													// If Not Return False
	GetObject(hBMP, sizeof(BMP), &BMP);									// Get The Object
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);								// Pixel Storage Mode (Word Alignment / 4 Bytes)
	glBindTexture(GL_TEXTURE_2D, texid);								// Bind To The Texture ID
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// Linear Min Filter
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);	// Linear Mag Filter
	glTexImage2D(GL_TEXTURE_2D, 0, 3, BMP.bmWidth, BMP.bmHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, BMP.bmBits);
	DeleteObject(hBMP);													// Delete The Object
	return TRUE;														// Loading Was Successful
}

#endif
