package com.example.osgjniclientmultimarker;

import android.content.Context;
import android.graphics.Canvas;
//import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
//import android.hardware.Camera.Size;
import android.util.AttributeSet;
//import android.util.Log;
//import android.view.SurfaceHolder;
//import android.view.SurfaceView;
import android.view.View;

//used to display current selected color
public class DrawView extends View{
	
	Canvas mCanvas;
	Paint paint;
	Rect DrawRect;//the rectangle to display current selected color
	public int mColor;//selected color
	public DrawView(Context context) {
		super(context);
		
		paint=new Paint();
		paint.setStyle(Paint.Style.FILL);

		mColor=0xffffffff;
		paint.setColor(mColor);
		paint.setTextSize(50);
		DrawRect=new Rect();
		adjustFaceRect();

	}
	public DrawView(Context context, AttributeSet attrs) 
	{
		super(context,attrs);
		paint=new Paint();
		paint.setStyle(Paint.Style.FILL);
		mColor=0xffffffff;
		paint.setColor(mColor);
		paint.setTextSize(50);
		DrawRect=new Rect();
		adjustFaceRect();
	}

	//update the color preview
	@Override
	protected void onDraw(Canvas canvas)
	{
		
		mCanvas=canvas;
		paint.setStyle(Paint.Style.FILL);
		paint.setColor(mColor);
		paint.setTextSize(50);
		mCanvas.drawRect(DrawRect, paint);
		
		super.onDraw(canvas);
		invalidate();
	}

	//set the position of DrawRect
	void adjustFaceRect()
	{
		int nLeft,nRight,nTop,nBottom;
		nLeft=10;
		nRight=160;
		nTop=10;
		nBottom=80;
		DrawRect.set(nLeft, nTop, nRight, nBottom);
	}

}
