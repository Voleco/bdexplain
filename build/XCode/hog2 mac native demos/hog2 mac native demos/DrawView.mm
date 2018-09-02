//
//  DrawView.m
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 7/30/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#import "DrawView.h"
#include "CocoaHOG.h"

@implementation DrawView


-(id)initWithCoder:(NSCoder *)coder
{
	self = [super initWithCoder:coder];
	if (self) {
		[self getHeights];
	}
	return self;
}

- (id)initWithFrame:(NSRect)frame
{
	self = [super initWithFrame:frame];
	if (self) {
		[self getHeights];
	}
	return self;
}


-(void)getHeights
{
	height = self.frame.size.height;
	width = self.frame.size.width;
	pRecContext context = getCurrentContext();
	context->windowHeight = height;
	context->windowWidth = width;
	
	double limit = std::min(self.bounds.size.width, self.bounds.size.height);
	xoffset = limit/2+(width-limit)/2;
	yoffset = limit/2+(height-limit)/2;
	xscale = limit/2.0;
	yscale = limit/2.0;
}

- (void)drawRect:(NSRect)rect
{
	[self getHeights];
	CGContextRef context = [NSGraphicsContext currentContext].CGContext;
	if (_display == 0)
	{
		CGContextSetRGBFillColor(context, 1, 0, 0, 1.0);
		CGContextFillRect(context, CGRectMake(0, 0, width, height));
		return;
	}

	CGContextSetRGBFillColor(context, 0, 0, 0, 1.0);
	CGContextFillRect(context, CGRectMake(0, 0, width, height));
	
	for (int x = 0; x < _display->backgroundDrawCommands.size(); x++)
	{
		[self drawCommand:&(_display->backgroundDrawCommands[x])];
	}
	for (int x = 0; x < _display->backgroundLineSegments.size(); x++)
	{
		[self drawSegments:&(_display->backgroundLineSegments[x])];
	}
	
	for (int x = 0; x < _display->drawCommands.size(); x++)
	{
		[self drawCommand:&(_display->drawCommands[x])];
	}
	for (int x = 0; x < _display->lineSegments.size(); x++)
	{
		[self drawSegments:&(_display->lineSegments[x])];
	}
	for (int x = 0; x < _display->text.size(); x++)
	{
		// DRAW TEXT HERE
		NSString *s = [NSString stringWithUTF8String:_display->text[x].s.c_str()];
//		NSLog(s);
		NSMutableParagraphStyle *style = [[NSParagraphStyle defaultParagraphStyle] mutableCopy];
		if (_display->text[x].align == Graphics::textAlignCenter)
			[style setAlignment:NSTextAlignmentCenter];
		else if (_display->text[x].align == Graphics::textAlignLeft)
			[style setAlignment:NSTextAlignmentLeft];
		NSDictionary *textAttributes =
		@{NSFontAttributeName: [NSFont monospacedDigitSystemFontOfSize:_display->text[x].size*height/2.0 weight:1.0],
		  NSForegroundColorAttributeName: [NSColor colorWithRed:_display->text[x].c.r green:_display->text[x].c.g blue:_display->text[x].c.b alpha:1.0],
		  NSParagraphStyleAttributeName: style
		  };
		NSPoint p;
		p.x = [self hogToScreenX:_display->text[x].loc.x viewport:_display->text[x].viewport];
		p.y = [self hogToScreenY:_display->text[x].loc.y viewport:_display->text[x].viewport];
		//NSLog(@"%f %f", p.x, p.y);
		if (_display->text[x].align == Graphics::textAlignLeft)
			[s drawAtPoint:p withAttributes:textAttributes];
		else
			[s drawInRect:CGRectMake(p.x-width, p.y-_display->text[x].size*height/4.0, 2*width, _display->text[x].size*height/2.0) withAttributes:textAttributes];
	}

}

-(void)drawSegments:(Graphics::Display::segments*)seg
{
	CGContextRef context = [NSGraphicsContext currentContext].CGContext;
	Graphics::Display::segments &s = *seg;
	int port = s.viewport;
	if (s.points.size() == 0)
		return;
	CGContextSetRGBStrokeColor(context, s.c.r, s.c.g, s.c.b, 1.0);
	CGContextSetLineWidth(context, s.size);
	CGContextMoveToPoint(context, [self hogToScreenX:s.points[0].x viewport:port], [self hogToScreenY:s.points[0].y viewport:port]);
	for (int y = 1; y < s.points.size(); y++)
	{
		CGContextAddLineToPoint(context, [self hogToScreenX:s.points[y].x viewport:port], [self hogToScreenY:s.points[y].y viewport:port]);
	}
	CGContextStrokePath(context);
}

-(void)drawCommand:(Graphics::Display::data*)dat
{
	Graphics::Display::data &d = *dat;
	CGContextRef context = [NSGraphicsContext currentContext].CGContext;
	int port = d.viewport;
	switch (d.what)
	{
		case Graphics::Display::kLine:
		{
			Graphics::Display::lineInfo &o = d.line;
			CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width);
			CGContextSetLineCap(context, kCGLineCapRound);
			CGContextMoveToPoint(context, [self hogToScreenX:o.start.x viewport:port], [self hogToScreenY:o.start.y viewport:port]);
			CGContextAddLineToPoint(context, [self hogToScreenX:o.end.x viewport:port], [self hogToScreenY:o.end.y viewport:port]);
			
			if (o.arrow)
			{
				Graphics::point newEnd = o.end*0.975f+o.start*0.025f;
				Graphics::point p1 = o.end-o.start;
				Graphics::point p2 = o.start;
				p2.z = 1;
				p2 = p1*p2;
				p2.normalise();
				p2 *= (o.end-newEnd).length();
				CGContextMoveToPoint(context, [self hogToScreenX:o.end.x viewport:port], [self hogToScreenY:o.end.y viewport:port]);
				CGContextAddLineToPoint(context, [self hogToScreenX:newEnd.x+p2.x viewport:port], [self hogToScreenY:newEnd.y+p2.y viewport:port]);
				CGContextMoveToPoint(context, [self hogToScreenX:o.end.x viewport:port], [self hogToScreenY:o.end.y viewport:port]);
				CGContextAddLineToPoint(context, [self hogToScreenX:newEnd.x-p2.x viewport:port], [self hogToScreenY:newEnd.y-p2.y viewport:port]);
			}
			CGContextStrokePath(context);
			break;
		}
		case Graphics::Display::kFillRectangle:
		{
			Graphics::Display::drawInfo &o = d.shape;
			CGContextSetRGBFillColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width);
			Graphics::rect &tmp = o.r;
			CGContextFillRect(context, [self makeRect:tmp viewport:port]);
			break;
		}
		case Graphics::Display::kFrameRectangle:
		{
			Graphics::Display::drawInfo &o = d.shape;
			CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width);
			CGContextSetLineCap(context, kCGLineCapRound);
			Graphics::rect &tmp = o.r;
			CGContextStrokeRect(context, [self makeRect:tmp viewport:port]);
			break;
		}
		case Graphics::Display::kFillOval:
		{
			Graphics::Display::drawInfo &o = d.shape;
			CGContextSetRGBFillColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width);
			Graphics::rect &tmp = o.r;
			CGContextFillEllipseInRect(context, [self makeRect:tmp viewport:port]);
			break;
		}
		case Graphics::Display::kFrameOval:
		{
			Graphics::Display::drawInfo &o = d.shape;
			
			CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width);
			CGContextSetLineCap(context, kCGLineCapRound);
			Graphics::rect &tmp = o.r;
			CGContextStrokeEllipseInRect(context, [self makeRect:tmp viewport:port]);
			break;
		}
	}

}


const float epsilon = 0.5f; // in screen pixels
- (CGRect)makeRect:(Graphics::rect)r viewport:(int)v
{
	CGRect result = CGRectMake([self hogToScreenX:r.left viewport:v],
							   [self hogToScreenY:r.top viewport:v],
							   [self hogToScreenX:r.right viewport:v]-[self hogToScreenX:r.left viewport:v],
							   [self hogToScreenY:r.bottom viewport:v]-[self hogToScreenY:r.top viewport:v]);
//							   (r.right-r.left)*xscale,
//							   -((r.bottom-r.top)*yscale));
//							   (r.left*xscale+xoffset),
//							   height-(r.top*yscale+yoffset),
//							   ((r.right-r.left)*xscale),
//							   -((r.bottom-r.top)*yscale));
	result = CGRectInset(result, -epsilon, -epsilon);
	return result;
}

-(CGFloat)hogToScreenX:(CGFloat)x viewport:(int)v
{
	pRecContext pContextInfo = getCurrentContext();
	point3d input(x, 0.f, 0.f);
	point3d result = ViewportToGlobalHOG(pContextInfo, pContextInfo->viewports[v], input);
//	if (v == 1)
//		printf("X:%f -> %f\n", x, ((result.x+1.0))/2.0);
	return ((result.x+1.0)*width)/2.0;
}

-(CGFloat)hogToScreenY:(CGFloat)y viewport:(int)v
{
	pRecContext pContextInfo = getCurrentContext();
	point3d input(0.f, y, 0.f);
	point3d result = ViewportToGlobalHOG(pContextInfo, pContextInfo->viewports[v], input);
//	result.y -= pContextInfo->viewports[v].bounds.bottom;
//	result.y = pContextInfo->viewports[v].bounds.top - result.y;
	//	if (v == 1)
//		printf("Y:%f -> %f\n", y, ((result.y+1.0))/2.0);
	return -((result.y-1.0)*height)/2.0;
}

-(point3d)convertToGlobalHogCoordinate:(NSPoint)currPoint
{
	point3d p;
	p.x = 2.0*currPoint.x/width-1.0;
	p.y = -2.0*currPoint.y/height+1.0;
	return p;
	
//	switch (_display->GetNumViewports())
//	{
//		case 1:
//			p.x = (currPoint.x-xoffset)/xscale;
//			p.y = (height-currPoint.y-yoffset)/yscale;
//			return p;
//		case 2:
//			int v = [self getViewport:currPoint];
//			if (v == 0)
//				p.y = (((height-currPoint.y)-yoffset)/yscale+0.25)/0.75;
//			if (v == 1)
//				p.y = (((height-currPoint.y)-yoffset)/yscale+0.75)/0.25;
//	}
////	(currPoint.x - xscale)/( = (0.75*p.x*xscale+xoffset);
//	p.x = (currPoint.x-xoffset)/(xscale*0.75);
//
//	return p;
	//point3d p = [drawingView convertToHogCoordinate:curPoint];
}

-(int)getViewport:(NSPoint)currPoint
{
	if (_display->GetNumViewports() == 1)
		return 0;
	point3d p;
	p.x = (currPoint.x-xoffset)/xscale;
	p.y = (height-currPoint.y-yoffset)/yscale;
	if (p.y < 2.0*(1.0-0.75))
		return 0;
	return 1;
}

-(BOOL)acceptsFirstResponder
{
	return YES;
}

- (void)keyDown:(NSEvent *)event
{
	[super keyDown:event];
	//	NSString *characters;
//	characters = [event characters];
//	printf("%c : %d\n", [characters characterAtIndex:0], [characters characterAtIndex:0]);
	
}


@end
