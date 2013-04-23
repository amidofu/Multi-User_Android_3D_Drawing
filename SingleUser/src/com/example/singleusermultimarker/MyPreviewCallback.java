package com.example.singleusermultimarker;
import nativeFunctions.NativeLib;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.util.Log;
import android.view.SurfaceHolder;
public class MyPreviewCallback implements PreviewCallback,Runnable{

	SurfaceHolder mHolder;
	int mHeight;
	int mWidth;
	int [] mRGBA;
	long time=0;
	
    private Mat mYuv;
    private Mat mRgba;
    private Mat mRGB;
    
	boolean mThreadRun;
	byte [] mFrame;
	
	public void setSurfaceHolder(SurfaceHolder holder)
	{
		mHolder=holder;
	}
	public void setupPreviewBitmap(int height, int width)
	{
		mHeight=height;
		mWidth=width;
		mRGBA = new int[mWidth * mHeight];
	}
	public void setHeightWidth(int height, int width)
	{
		mHeight=height;
		mWidth=width;
	}
	
	public void setupMat()
	{
		mYuv = new Mat(mHeight + mHeight / 2, mWidth, CvType.CV_8UC1);
        mRgba = new Mat();
        mRGB =new Mat();
	}
	@Override
	public void onPreviewFrame(byte[] data, Camera camera) {		
        synchronized (MyPreviewCallback.this) {
            mFrame = data;
            MyPreviewCallback.this.notify();
        }
	}
	@Override
	public void run() {
		 mThreadRun = true;
	        Log.i("PreviweCallback", "Starting processing thread");
	        while (mThreadRun) {
	            synchronized (MyPreviewCallback.this) {
	                try {
	                    this.wait();
	                    processFrame(mFrame);
	                } catch (InterruptedException e) {
	                    e.printStackTrace();
	                }
	            }
	        }
	}
	
	public void processFrame(byte[] data)
	{
		time=System.currentTimeMillis();
		//Log.i("preview", "preview");
		mYuv.put(0, 0, data);
        Imgproc.cvtColor(mYuv, mRgba, Imgproc.COLOR_YUV420sp2RGB, 4);
		//change to 3-channel
        Imgproc.cvtColor(mRgba, mRGB, Imgproc.COLOR_RGBA2RGB, 3);
        //detect one marker
		NativeLib.MarkerDetection(mRGB.getNativeObjAddr(), mHeight, mWidth, 2);
		// Log.e("time:",": "+(System.currentTimeMillis()-time));
	}

}
