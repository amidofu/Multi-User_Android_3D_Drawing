package com.example.osgjniservermultimarker;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
//used for combine GLSurfaceView and UI
public class MyGLSurfaceView extends GLSurfaceView{

	public MyGLSurfaceView(Context context, AttributeSet attrs) {
		super(context, attrs);
	}
	public MyGLSurfaceView(Context context){
		super(context);
	}

}