//
//  ViewController.m
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 7/27/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#import "ViewController.h"
#include "Common.h"

int hog_main(int argc, char **argv);

@implementation ViewController

const float FRAMERATE = 1.0f/30.0f;

- (void)viewDidLoad {
	[super viewDidLoad];

	NSArray *arguments = [[NSProcessInfo processInfo] arguments];
	// use -objectAtIndex: to obtain an element of the array
	// and -count to obtain the number of elements in the array
	char **args = new char*[[arguments count]];
	int len = [arguments count];
	for (int x = 0; x < len; x++)
	{
		int nextLen = [[arguments objectAtIndex:x] length];
		args[x] = new char[nextLen + 1];
		strncpy(args[x], [[arguments objectAtIndex:x] cStringUsingEncoding:NSASCIIStringEncoding], nextLen);
		args[x][nextLen] = 0;
	}
	
	hog_main([arguments count], args);
	
	
	
	
	[NSTimer scheduledTimerWithTimeInterval:FRAMERATE
									 target:self
								   selector:@selector(onTick:)
								   userInfo:nil
									repeats:YES];

	pRecContext pContextInfo = getCurrentContext();
	initialConditions(pContextInfo);
	HandleWindowEvent(pContextInfo, kWindowCreated);
//	initialConditions(pContextInfo);
	drawingView.display = &pContextInfo->display;
//	self.drawingView.display = pContextInfo->display;
}


- (void)setRepresentedObject:(id)representedObject {
	[super setRepresentedObject:representedObject];

	// Update the view, if already loaded.
}


-(void)onTick:(NSTimer *)timer {
	pRecContext pContextInfo = getCurrentContext();
	pContextInfo->display.StartFrame();
	for (int x = 0; x < pContextInfo->numPorts; x++)
	{
		setViewport(pContextInfo, x);
		//if (pContextInfo->drawing)
		{
			// set projection
			HandleFrame(pContextInfo, x);
		}
	}
	pContextInfo->display.EndFrame();
	[drawingView setNeedsDisplay:YES];
	if (getTextBuffer() != 0)
		[messageField setStringValue:[NSString stringWithUTF8String:getTextBuffer()]];
}

-(tButtonType)getButton:(NSEvent *)event
{
	switch ([event type])// == NSEventTypeLeftMouseUp)
	{
		case NSEventTypeLeftMouseDown: return kLeftButton; break;
		case NSEventTypeLeftMouseUp: return kLeftButton; break;
		case NSEventTypeRightMouseDown: return kRightButton; break;
		case NSEventTypeRightMouseUp:  return kRightButton; break;
		case NSEventTypeLeftMouseDragged:  return kLeftButton; break;
		case NSEventTypeRightMouseDragged:  return kRightButton; break;
	}
	return kLeftButton;
}

-(void)mouseDown:(NSEvent *)event
{
	tButtonType bType = [self getButton:event];
	NSPoint curPoint = [event locationInWindow];
	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
	HandleMouse(getCurrentContext(), p, bType, kMouseDown);
//	int viewport = [drawingView getViewport:curPoint];
//	HandleMouseClick(getCurrentContext(), viewport, curPoint.x, curPoint.y, p, bType, kMouseDown);
}
	


-(void)mouseUp:(NSEvent *)event
{
	tButtonType bType = [self getButton:event];
	NSPoint curPoint = [event locationInWindow];
	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
	HandleMouse(getCurrentContext(), p, bType, kMouseUp);
//	int viewport = [drawingView getViewport:curPoint];
//	HandleMouseClick(getCurrentContext(), viewport, curPoint.x, curPoint.y, p, bType, kMouseUp);
}

-(void)mouseDragged:(NSEvent *)event
{
	tButtonType bType = [self getButton:event];
	NSPoint curPoint = [event locationInWindow];
	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
	HandleMouse(getCurrentContext(), p, bType, kMouseDrag);
//	int viewport = [drawingView getViewport:curPoint];
//	HandleMouseClick(getCurrentContext(), viewport, curPoint.x, curPoint.y, p, bType, kMouseDrag);
}


//-(void)keyDown:(NSEvent *)event
//{
//	NSString *characters;
//	characters = [event characters];
//	NSLog(characters);
//}

-(BOOL)acceptsFirstResponder
{
	return YES;
}

- (void)keyDown:(NSEvent *)event
{
	NSString *characters;
	characters = [event characters];
//	printf("%c : %d!\n", [characters characterAtIndex:0], [characters characterAtIndex:0]);
	bool shift = false;
	if (isupper([characters characterAtIndex:0]))
		shift = true;
	DoKeyboardCommand(getCurrentContext(), [characters characterAtIndex:0], shift?kShiftDown:kNoModifier, false, false);
}

@end
