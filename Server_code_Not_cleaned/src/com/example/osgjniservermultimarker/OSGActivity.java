package com.example.osgjniservermultimarker;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.List;
import java.util.Scanner;
import java.util.Timer;
import java.util.TimerTask;




import nativeFunctions.CommonFuncAndConst;
import nativeFunctions.NativeLib;
import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.hardware.Camera;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.Camera.PictureCallback;
import android.hardware.Camera.ShutterCallback;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.text.format.DateFormat;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnTouchListener;
import android.widget.Button;

public class OSGActivity extends Activity implements OnTouchListener{

	//members for sensor related
	private SensorManager mSenMan;
	Sensor Accelerometer;
	Sensor Gyroscope;
	String storageFolder;
	String OSGstorageFolder; 
	private SensorEventListener SELaccelerometer;
	private SensorEventListener SELgyroscope;
	//not used
	private final float[] deltaRotationVector = new float[4];
	//used to calculate the rotation of camera
	private float timestamp;
	
	static final int AxisX=0;
	static final int AxisY=1;
	static final int AxisZ=2;
	
	static final int  TypeRotation=0;
	static final int  TypeTranslation=1;
	static final int  TypeScale=2;
	
	int currentAxis;
	int currentEditType;

	Camera mCamera;
	MyPreviewCallback previewCallback;
	SurfaceView camView;
	int mWidth;
	int mHeight;
	boolean edit;
	//public GLSurfaceView mview;
	MyGLSurfaceView mview;
	//Button home;
	//View view1;
	Button MoveForward;
	Button MoveBack;
	//Button SendGeometry;
	//Button SendScene;
	Button ToggleDrawLine;
	boolean DrawLineStatus;
	Button NewPt;
	Button ToggleEdit;
	boolean EditStatus;
	//Button EditR;
	Button TAxis;
	Timer mCameraTimer;
	Button EditType;
	
	Button DeleteGeom;
	Button saveSMarker;
	
	Button addBox;
	Button setColor;
	ChangeColorUI setColorUI;
	
	Button changeColor;
	float pX,pY;
	public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //storageFolder=new String(Environment.getExternalStorageDirectory()+"/marker_data");
		checkMarkerStorage();
		checkOSGStorage();
        loadSelecteResolution();
        setupCamera();
        setColorUI=new ChangeColorUI(this);
        Log.i("OSGActivity", "start");
        currentAxis=AxisX;
        currentEditType=TypeRotation;
        edit=false;
        //set UI view
        setContentView(R.layout.glsurfaceview);
        
        mview=(MyGLSurfaceView)findViewById(R.id.MyGLSurfaceView);
        mview.setRenderer(new MyGLRenderer());
        mview.setOnTouchListener(this);
        //setContentView(mview);
        //home=(Button)findViewById(R.id.HomeView);
        
        //home.setOnClickListener(PressHome);
        
        MoveForward=(Button)findViewById(R.id.MoveForward);
        MoveBack=(Button)findViewById(R.id.MoveBack);
        
        MoveForwardTouch MFT=new MoveForwardTouch();
        MoveForward.setOnTouchListener(MFT);
        
        MoveBackTouch MBT=new MoveBackTouch();
        MoveBack.setOnTouchListener(MBT);
        
        //SendGeometry=(Button)findViewById(R.id.SendGeometry);
        //SendGeometry.setOnClickListener(pressSendGeometry);
        
        //SendScene=(Button)findViewById(R.id.SendMainScene);
        //SendScene.setOnClickListener(pressSendScene);
        
        ToggleDrawLine=(Button)findViewById(R.id.toggleDrawLine);
        ToggleDrawLine.setOnClickListener(pressToggleDrawLine);
        DrawLineStatus=false;
        
        TAxis=(Button)findViewById(R.id.TAxis);
        TAxis.setOnClickListener(pressTAxis);
        
        NewPt=(Button)findViewById(R.id.NewPt);
        NewPt.setOnClickListener(pressNewPt);
        
        ToggleEdit=(Button)findViewById(R.id.ToggleEdit);
        ToggleEdit.setOnClickListener(pressTEdit);
        EditStatus=false;
        //EditR=(Button)findViewById(R.id.EditR);
        //EditR.setOnClickListener(pressEditR);
	    //compute translation by the accelerometer, not used currently due to accuracy
        
        EditType=(Button)findViewById(R.id.EditType);
        EditType.setOnClickListener(pressEditType);
        
        DeleteGeom=(Button)findViewById(R.id.DeleteGeometry);
        DeleteGeom.setOnClickListener(pressDeleteGeom);
        
        saveSMarker=(Button)findViewById(R.id.saveSMarker);
        saveSMarker.setOnClickListener(pressSaveSMarker);
        
        addBox=(Button)findViewById(R.id.addBox);
        addBox.setOnClickListener(pressAddBox);
        
        setColor=(Button)findViewById(R.id.setColor);
        setColor.setOnClickListener(pressSetColor);
        
        changeColor=(Button)findViewById(R.id.ChangeColor);
        changeColor.setOnClickListener(pressChangeColor);
        
        SELaccelerometer = new SensorEventListener(){
			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {
				// TODO Auto-generated method stub
			}
			@Override
			public void onSensorChanged(SensorEvent event) {
				// TODO Auto-generated method stub
                /*
                float[] values = event.values;
                float x=values[0];
                float y=values[1];
                float z=values[2];
                osgNativeLib.sendAcceleration(x, -z, y, dT)
                */
			}
        	
        };
        
        SELgyroscope=new SensorEventListener(){

			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {
			}

			@Override
			public void onSensorChanged(SensorEvent event) {
				// This timestep's delta rotation to be multiplied by the current rotation
		        // after computing it from the gyro sample data.
		        if (timestamp != 0) {
		        	final float dT = (event.timestamp - timestamp) * CommonFuncAndConst.NS2S;
		            // Axis of the rotation sample, not normalized yet.
		            float axisX = event.values[0];
		            float axisY = event.values[1];
		            float axisZ = event.values[2];

		            // Calculate the angular speed of the sample
		            float omegaMagnitude = (float) Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);

		            // Normalize the rotation vector if it's big enough to get the axis
		            if (omegaMagnitude > CommonFuncAndConst.EPSILON) {
		            	axisX /= omegaMagnitude;
		                axisY /= omegaMagnitude;
		                axisZ /= omegaMagnitude;
		            }
		            float thetaOverTwo = omegaMagnitude * dT / 2.0f;
		            float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
		            float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
		            deltaRotationVector[0] = sinThetaOverTwo * axisX;
		            deltaRotationVector[1] = sinThetaOverTwo * axisY;
		            deltaRotationVector[2] = sinThetaOverTwo * axisZ;
		            deltaRotationVector[3] = cosThetaOverTwo;
		             
		            
		            float angle=thetaOverTwo*2.0f;
		            if(angle>0.01f)
		            	NativeLib.sendRotation(-axisY, axisX,-axisZ, angle);
		            else
		            	NativeLib.sendRotation(1.0f, 0.0f,0.0f, 0.0f);
		            	
		          }
		          timestamp = event.timestamp;
			}
        	
        };
        //setup sensors
        mSenMan = (SensorManager)getSystemService(SENSOR_SERVICE);
        Accelerometer = mSenMan.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Gyroscope=mSenMan.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        //mCameraTimer=new Timer();
        
        new Thread(previewCallback).start();
	}

    void loadSelecteResolution()
    {
    	String path=new String(Environment.getExternalStorageDirectory()+"/marker_data/SelectRes.txt");
    	try {
			FileReader fr=new FileReader(path);
			Scanner scanner=new Scanner(fr);
			
			if(scanner.hasNext())
			{
				if(scanner.hasNextInt())
					mWidth=scanner.nextInt();
				else
					Log.e("read camera param", "width wrong");
				if(scanner.hasNextInt())
					mHeight=scanner.nextInt();
				else
					Log.e("read camera param", "height wrong");
			}
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }
    
    void checkOSGStorage()
    {
    	String path=new String(Environment.getExternalStorageDirectory()+"/osg_data");
    	File storage=new File(path);
    	boolean success=false;
    	if(!storage.exists())
    	{
    		success=storage.mkdir();
        	if(!success)//folder exist
        	{
        		Log.e("OSG folder", "fail to created");
        	}
        	else// folder doesn't exist, create the folder
        	{
        		Log.i("OSG folder", "created");
        		OSGstorageFolder=path; 
        	}
    	}
    	else
    	{
    		OSGstorageFolder=path;
    	}

    }
    
    void checkMarkerStorage()
    {
    	String path=new String(Environment.getExternalStorageDirectory()+"/marker_data");
    	File storage=new File(path);
    	if(!storage.exists())
    	{
    		Log.e("check marker data", "fail");
    		finish();
    	}
    	else
    		storageFolder=path;
    }
	
	/*
	private OnClickListener PressHome =new OnClickListener(){

		@Override
		public void onClick(View v) {
			Log.i("Server", "home pressed");
			int KEY_KP_Space        = 0xFF80;
			NativeLib.sendKeyDown(KEY_KP_Space);
		}
		
	};
	
	private OnClickListener pressSendGeometry =new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			NativeLib.sendGeometry();
		}
		
	};
	
	private OnClickListener pressSendScene = new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			NativeLib.sendMainScene();
		}
		
	};
	*/
	private OnClickListener pressToggleDrawLine=new OnClickListener()
	{

		@Override
		public void onClick(View arg0) {
			// TODO Auto-generated method stub
			
			NativeLib.toggleDrawLine();
			if(DrawLineStatus)
				ToggleDrawLine.setTextColor(Color.BLACK);
			else
				ToggleDrawLine.setTextColor(Color.RED);
			DrawLineStatus=!DrawLineStatus;
		}
		
	};
	
	private OnClickListener pressTAxis=new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			currentAxis=(currentAxis+1)%3;
			switch(currentAxis)
			{
			case AxisX:
				TAxis.setText("Axis X");
				break;
			case AxisY:
				TAxis.setText("Axis Y");
				break;
			case AxisZ:
				TAxis.setText("Axis Z");
				break;
			}
		}
		
	};
	
	private OnClickListener pressEditType=new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			currentEditType=(currentEditType+1)%3;
			switch(currentEditType)
			{
			case TypeRotation:
				EditType.setText("Rotate");
				break;
			case TypeTranslation:
				EditType.setText("Trans");
				break;
			case TypeScale:
				EditType.setText("Scale");
				break;
			}
		}		
	};
	
	private OnClickListener pressNewPt=new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			NativeLib.getNewPt();
		}
		
	};
	private OnClickListener pressTEdit=new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			
			//NativeLib.toggleEdit();
			if(!NativeLib.toggleEdit())
				return;
			edit=!edit;
			if(EditStatus)
				ToggleEdit.setTextColor(Color.BLACK);
			else
				ToggleEdit.setTextColor(Color.RED);
			EditStatus=!EditStatus;
		}
		
	};
	/*
	private OnClickListener pressEditR=new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			NativeLib.EditR(currentAxis, 1.0f);
		}
		
	};
	*/
	private class MoveForwardTouch implements OnTouchListener{

		@Override
		public boolean onTouch(View v, MotionEvent event) {
			//Log.i("press", "forward");
			NativeLib.sendKeyDown('w');
			return true;
		}
	}
	
	private class MoveBackTouch implements OnTouchListener{

		@Override
		public boolean onTouch(View v, MotionEvent event) {
			NativeLib.sendKeyDown('s');
			return true;
		}
	}
	
	private OnClickListener pressDeleteGeom=new OnClickListener()
	{
		@Override
		public void onClick(View v) {
			NativeLib.deleteGeometry();
		}			
	};
	
	private OnClickListener pressSaveSMarker=new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			String s=new String("");
			s=s+storageFolder+"/sMarker.txt";
			NativeLib.saveSMarker(s);
		}
		
	};

	private OnClickListener pressAddBox=new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			NativeLib.addBox();
		}
		
	};
	
	private OnClickListener pressSetColor=new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			setColorUI.show();
		}
		
	};
	
	private OnClickListener pressChangeColor=new OnClickListener()
	{

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			NativeLib.changeColor();
		}
		
	};
    public void onBackPressed(){
		finish();
    }
    
	@Override
	protected void onPause()
	{
		super.onPause();
        //mSenMan.unregisterListener(SELaccelerometer);
        mSenMan.unregisterListener(SELgyroscope);
	}
	@Override
	protected void onResume()
	{
		super.onResume();
        //if use marker, no need to use sensors
		//mSenMan.registerListener(SELaccelerometer, Accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        mSenMan.registerListener(SELgyroscope,Gyroscope, SensorManager.SENSOR_DELAY_NORMAL);
	}
	@Override
	protected void onStop()
	{
    	super.onStop();
        //mSenMan.unregisterListener(SELaccelerometer);
        mSenMan.unregisterListener(SELgyroscope);
	}

	@Override
	public boolean onTouch(View v, MotionEvent event) {
		//int action = event.getAction();
		//long time=event.getEventTime();

		switch(event.getAction())
		{
		case MotionEvent.ACTION_DOWN:
			if(!edit)
			NativeLib.mouseButtonPressEvent(event.getX(), event.getY(), 1);
			else
			{
				pX=event.getX();
				pY=event.getY();
			}
			break;
		case MotionEvent.ACTION_MOVE:
			if(!edit)
			NativeLib.mouseMoveEvent(event.getX(), event.getY());
			else
			{
				NativeLib.Edit(currentEditType, currentAxis, event.getX()-pX);
				pX=event.getX();
				pY=event.getY();
			}
			break;
		case MotionEvent.ACTION_UP:
			NativeLib.mouseButtonReleaseEvent(event.getX(),event.getY(), 1);
			break;
		default:
			break;
		}
		return true;
	}
	
	
	public boolean onCreateOptionsMenu(Menu menu){
		menu.add(Menu.NONE,Menu.FIRST+1,1,"Reset View");
		menu.add(Menu.NONE,Menu.FIRST+2,2,"Reset Rotation");
		menu.add(Menu.NONE,Menu.FIRST+3,3,"Send Geometry");
		menu.add(Menu.NONE,Menu.FIRST+4,4,"Send Scene");
		menu.add(Menu.NONE,Menu.FIRST+5,5,"Send Marker");	
		menu.add(Menu.NONE,Menu.FIRST+6,6,"Load Scene");
		menu.add(Menu.NONE,Menu.FIRST+7,7,"Save Scene");
		return true;
		
	}
	
	public boolean onOptionsItemSelected(MenuItem item)
	{
		switch(item.getItemId())
		{
		case Menu.FIRST+1:
			int KEY_KP_Space        = 0xFF80;
			NativeLib.sendKeyDown(KEY_KP_Space);
			break;
		case Menu.FIRST+2:
			NativeLib.sendKeyDown('r');
			break;
			
		case Menu.FIRST+3:
			NativeLib.sendGeometry();
			break;
			
		case Menu.FIRST+4:
			NativeLib.sendMainScene();
			break;
			
		case Menu.FIRST+5:
			NativeLib.sendSMarker();
			break;
		case Menu.FIRST+6:
			NativeLib.loadScene(OSGstorageFolder);
			break;
		case Menu.FIRST+7:
			NativeLib.saveScene(OSGstorageFolder);
			break;
		}
		return false;
		
	}
	
    public void setupCamera() {

        mCamera=Camera.open();
	    synchronized (this) {
	          if (mCamera != null) {  
	        	  Log.i("setupCamera", "camera opened");
	             Camera.Parameters params = mCamera.getParameters();
	             List<Camera.Size> sizes = params.getSupportedPreviewSizes();       
	             int width=0;
	             int height=0;
	             int mFrameWidth = width;
	             int mFrameHeight = height;

	             // selecting optimal camera preview size
	             {
	                  int  minDiff = Integer.MAX_VALUE;
	                  for (Camera.Size size : sizes) {
	                      if (Math.abs(size.height - height) < minDiff) {
	                         mFrameWidth = size.width;
	                         mFrameHeight = size.height;
	                         minDiff = Math.abs(size.height - height);
	                      }
	                  }
	             }

                 //mFrameWidth=320;
                 //mFrameHeight=240;
                 
                 mFrameWidth=mWidth;
                 mFrameHeight=mHeight;
	             params.setPreviewSize(mFrameWidth, mFrameHeight);
	             //initialize global variables for native functions
	             //NativeLib.InitializedOpenCVGlobalVar(mFrameWidth, mFrameHeight);		
	             NativeLib.InitialozeOpenCVGlobalVarByLoad(storageFolder);
	             Log.i("Init CV Global VR","Ok");
	             mCamera.setParameters(params);		                
		         previewCallback=new MyPreviewCallback();
		   	     mCamera.setPreviewCallback(previewCallback);
			     camView=new SurfaceView(OSGActivity.this);

		   	     previewCallback.setupPreviewBitmap(mFrameHeight, mFrameWidth);
		   	     previewCallback.setHeightWidth(mFrameHeight, mFrameWidth);
		   	     previewCallback.setupMat();   
	             params = mCamera.getParameters();
	              mCamera.startPreview();
	              Log.i("setupCamera", "start preview");
	            }
	        
	    }
        
    }

}
