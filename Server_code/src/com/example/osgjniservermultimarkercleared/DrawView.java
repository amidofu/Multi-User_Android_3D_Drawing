package com.example.osgjniservermultimarkercleared;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.view.View;

public class DrawView extends View{
	
	public Rect faceRect;
	Canvas mCanvas;
	Paint paint;
	Rect DrawRect;
	public int mColor;
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
