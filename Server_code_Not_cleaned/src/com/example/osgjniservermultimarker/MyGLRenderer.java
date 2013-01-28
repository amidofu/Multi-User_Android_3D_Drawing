package com.example.osgjniservermultimarker;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import nativeFunctions.NativeLib;


import android.opengl.GLSurfaceView.Renderer;
import android.util.Log;
public class MyGLRenderer implements Renderer{

	long time;
	@Override
	public void onDrawFrame(GL10 gl) {
		Log.e("time:",": "+(System.currentTimeMillis()-time));
        NativeLib.OSGRun();
        time=System.currentTimeMillis();
       
	}

	@Override
	public void onSurfaceChanged(GL10 gl, int width, int height) {
		Log.i("OnSurfaceChange","in");
		NativeLib.initOSG(width,height);
	}

	@Override
	public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        gl.glEnable(GL10.GL_DEPTH_TEST);             //Enables Depth Testing
	}
}
