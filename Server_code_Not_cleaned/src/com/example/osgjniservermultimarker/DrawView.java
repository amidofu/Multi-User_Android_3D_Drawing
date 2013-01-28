package com.example.osgjniservermultimarker;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.hardware.Camera.Size;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;

public class DrawView extends View{
	
	public Rect faceRect;
	Canvas mCanvas;
	Paint paint;
	Rect DrawRect;
	public int mColor;
	public DrawView(Context context) {
		super(context);
		// TODO Auto-generated constructor stub
		paint=new Paint();
		paint.setStyle(Paint.Style.FILL);
		//paint.setStyle(Paint.Style.STROKE);
		//mColor=0xff000000;
		mColor=0xffffffff;
		paint.setColor(mColor);
		paint.setTextSize(50);
		DrawRect=new Rect();
		adjustFaceRect();
		//mHolder=this.getHolder();
	}
	public DrawView(Context context, AttributeSet attrs) 
	{
		super(context,attrs);
		paint=new Paint();
		paint.setStyle(Paint.Style.FILL);
		//paint.setStyle(Paint.Style.STROKE);
		//mColor=0xff000000;
		mColor=0xffffffff;
		//paint.setColor(Color.CYAN);
		paint.setColor(mColor);
		paint.setTextSize(50);
		DrawRect=new Rect();
		adjustFaceRect();
		//mHolder=this.getHolder();
	}

	@Override
	protected void onDraw(Canvas canvas)
	{
		
		mCanvas=canvas;
		paint.setStyle(Paint.Style.FILL);
		//paint.setStyle(Paint.Style.STROKE);
		//paint.setColor(Color.CYAN);
		paint.setColor(mColor);
		paint.setTextSize(50);
		//mHolder=this.getHolder();
		//mCanvas=mHolder.lockCanvas();
		//mCanvas.drawText("Test Text", 20, 40, paint);
		//faceRect.set(30, 30, 90, 90);
		//adjustFaceRect();
		//mCanvas.drawRect(faceRect, paint);
		//Log.i("Face Center","X: "+DrawRect.centerX()+" , Y: "+DrawRect.centerY());
		mCanvas.drawRect(DrawRect, paint);
		//Log.i("Draw Rect", "OK");
		
		super.onDraw(canvas);
		invalidate();
		//mHolder.unlockCanvasAndPost(mCanvas);
	}

	void adjustFaceRect()
	{
		int nLeft,nRight,nTop,nBottom;
		//nLeft=(faceRect.left+1000)*camSize.width/2000;
		//nRight=(faceRect.right+1000)*camSize.width/2000;
		//nTop=(faceRect.top+1000)*camSize.height/2000;
		//nBottom=(faceRect.bottom+1000)*camSize.height/2000;
		nLeft=10;
		nRight=160;
		nTop=10;
		nBottom=80;
		DrawRect.set(nLeft, nTop, nRight, nBottom);
		//Log.i("Size", "width: "+camSize.width+" , height:"+camSize.height);
	}

}
