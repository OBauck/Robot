#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include "interface.h"
#include "geometry.h"

#define KEYMASK ButtonPressMask | KeyPressMask | KeyReleaseMask | ButtonReleaseMask | PointerMotionMask


#define LEFT_MOUSE_BUTTON 	1
#define RIGHT_MOUSE_BUTTON 	3
#define MOUSE_WHEEL		2
#define MOUSE_WHEEL_FORWARD	4
#define MOUSE_WHEEL_BACKWARD	5

Display *display;
Window window;
XEvent event;
bool button = 0;
bool buttonR = 0;
Point3D position = Point3D(XSTART, YSTART, ZSTART);
int xzero = 0;
int yzero = 0;
 
void windowInit()
{
    int s;
    /* open connection with the server */
    display = XOpenDisplay(NULL);
    if (display == NULL)
    {
        fprintf(stderr, "Cannot open display\n");
        exit(1);
    }
 
    s = DefaultScreen(display);
 
    /* create window */
    window = XCreateSimpleWindow(display, RootWindow(display, s), 10, 10, 500, 500, 1,
                           BlackPixel(display, s), WhitePixel(display, s));
 
    /* select kind of events we are interested in */
    XSelectInput(display, window, KEYMASK);
 
    /* map (show) the window */
    XMapWindow(display, window);

    //do not detect autorepeating events from keyboard
    XAutoRepeatOff(display);
    printf("Display open\n");
}

void checkEvent(Manipulator *man){
        XNextEvent (display, & event);
        switch(event.type)
		{
			case MotionNotify:
				if(button){
					position.x -= event.xmotion.x - xzero;
					position.y -= event.xmotion.y - yzero;
					xzero = event.xmotion.x;
					yzero = event.xmotion.y;
					//printf("xpos: %d\t ypos: %d\n", xpos, ypos);
					man->goToPosition(position);
				}	
				break;
			case ButtonPress:
				if(event.xkey.keycode == LEFT_MOUSE_BUTTON)
				{
					button = 1;
					xzero = event.xbutton.x;
					yzero = event.xbutton.y;
				}
				if(event.xkey.keycode == MOUSE_WHEEL_FORWARD)
				{
					position.z -= 10;
					man->goToPosition(position);
				}
				if(event.xkey.keycode == MOUSE_WHEEL_BACKWARD)
				{
					position.z += 10;
					man->goToPosition(position);
				}
			
				printf( "KeyPress: %d\n", event.xkey.keycode );
				break;
			case ButtonRelease:
				if(event.xkey.keycode == LEFT_MOUSE_BUTTON)
					button = 0;
				break;
		}
}
