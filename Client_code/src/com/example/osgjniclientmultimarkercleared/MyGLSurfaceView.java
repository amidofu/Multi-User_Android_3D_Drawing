package com.example.osgjniclientmultimarkercleared;
import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
//used to combine GLSurface View with other UI
public class MyGLSurfaceView extends GLSurfaceView{

	public MyGLSurfaceView(Context context, AttributeSet attrs) {
		super(context, attrs);
	}
	public MyGLSurfaceView(Context context){
		super(context);
	}

}
