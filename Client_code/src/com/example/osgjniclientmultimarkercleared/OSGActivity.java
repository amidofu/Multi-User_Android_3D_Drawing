package com.example.osgjniclientmultimarkercleared;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Scanner;

import nativeFunctions.CommonFuncAndConst;
import nativeFunctions.NativeLib;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.graphics.Color;
import android.hardware.Camera;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnTouchListener;
import android.widget.Button;

public class OSGActivity extends Activity implements OnTouchListener{

	static final float DisplacementScalar=80.0f;
	String storageFolder;
	String OSGstorageFolder;
	private final float[] deltaRotationVector = new float[4];
	//used to calculate the rotation of camera
	private float timestamp;
		
	//sensors	
	private SensorManager mSenMan;
	Sensor Accelerometer;
	Sensor Gyroscope;
	private SensorEventListener SELgyroscope;
		

	MyGLSurfaceView mview;
	Button MoveForward;//move camera in OSG forward
	Button MoveBack;// move camera in OSG backward
	Button ResetRotation;//make the up vector of camera in OSG to be (0,0,1) 
	Button SendGeometry;//probably not used
	Button SendScene;//send geometries to other phone
	Button ToggleDrawLine;//toggle the line drawing function
	boolean DrawLineStatus;//record if the line drawing function is on
	Button NewPt;//record new point for the line
	Button toggleEdit;//toggle the geometry transformation mode
	boolean EditStatus;//record if the geometry transformation mode is on
	Button DeleteGeom;//delete selected geometry
	
	Camera mCamera;//phone's camera
	MyPreviewCallback previewCallback;
	SurfaceView camView;//used to get camera preview image
	int mWidth;
	int mHeight;
	byte[] mBuffer;
	byte[] mFrame;
	
	//indicate switch type of geometry transformation 
	static final int  TypeRotation=0;
	static final int  TypeTranslation=1;
	static final int  TypeScale=2;
	//geometry transformation axis
	static final int AxisX=0;
	static final int AxisY=1;
	static final int AxisZ=2;
	
	//record geometry transformation type and axis
	int currentAxis;
	int currentEditType;
	boolean edit;
	
	Button TAxis;//change transformation axis
	Button EditType;//change transformation type
	
	Button saveSMarker;//save marker information if you got the information from another phone
	Button addBox;//create a box in space
	Button setColor;//open the color selection UI
	ChangeColorUI setColorUI;
	
	Button changeColor;//change the color of geometry to selected color
	
	Button fixViewAngle;
	boolean fixVA;
	
    //used for record coordinate of touch screen 
	float pX;
    float pY;
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		
		//check if directory and data of marker's information and camera parameters is existed
		checkMarkerStorage();
		//check if model for OSG is existed
		checkOSGStorage();
		//load camera parameters
		loadSelecteResolution();
		setupCamera();
		setColorUI=new ChangeColorUI(this);

	    currentAxis=AxisX;
	    currentEditType=TypeRotation;
	    edit=false;
	    setContentView(R.layout.glsurfaceview);
	        
	    mview=(MyGLSurfaceView)findViewById(R.id.MyGLSurfaceView2);
	    mview.setRenderer(new MyGLRenderer());
	    mview.setOnTouchListener(this);

	    MoveForward=(Button)findViewById(R.id.MoveForward);
	    MoveBack=(Button)findViewById(R.id.MoveBack);
	    MoveForwardTouch MFT=new MoveForwardTouch();
	    MoveForward.setOnTouchListener(MFT);
	        
	        
	    MoveBackTouch MBT=new MoveBackTouch();
	    MoveBack.setOnTouchListener(MBT);
	    
	    
        ToggleDrawLine=(Button)findViewById(R.id.toggleDrawLine);
        ToggleDrawLine.setOnClickListener(pressToggleDrawLine);
        DrawLineStatus=false;
        
        NewPt=(Button)findViewById(R.id.NewPt);
        NewPt.setOnClickListener(pressNewPt);
        
        toggleEdit=(Button)findViewById(R.id.toggleEdit);
        toggleEdit.setOnClickListener(pressTedit);
        EditStatus=false;
        TAxis=(Button)findViewById(R.id.TAxis);
        TAxis.setOnClickListener(pressTAxis);
        
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
	      
        fixViewAngle=(Button)findViewById(R.id.fixViewAngle);
        fixViewAngle.setOnClickListener(pressFixVA);
        fixVA=false;
	        
	        SELgyroscope=new SensorEventListener(){

				@Override
				public void onAccuracyChanged(Sensor sensor, int accuracy) {
				}

				@SuppressLint({ "FloatMath", "FloatMath" })
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
			            if(angle>0.01f)//check if the rotation is large enough
			            	NativeLib.sendRotation(-axisY, axisX,-axisZ, angle);
			            else
			            	NativeLib.sendRotation(1.0f, 0.0f,0.0f, 0.0f);
			            	
			          }
			          timestamp = event.timestamp;
				}
	        	
	        };
	        mSenMan = (SensorManager)getSystemService(SENSOR_SERVICE);
	        Accelerometer=mSenMan.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
	        Gyroscope=mSenMan.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
	        
	        //Log.i("gyro vendor", Gyroscope.getVendor());
	        
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
	    
		//switch transformation axis
		private OnClickListener pressTAxis=new OnClickListener()
		{

			@Override
			public void onClick(View v) {
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
		
		//switch transformation type
		private OnClickListener pressEditType=new OnClickListener()
		{

			@Override
			public void onClick(View v) {
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
				Log.i("edit", "current type: "+currentEditType);
				
			}		
		};

		private OnClickListener pressToggleDrawLine=new OnClickListener()
		{
			@Override
			public void onClick(View arg0) {
				NativeLib.toggleDrawLine();
				
				if(DrawLineStatus)
					ToggleDrawLine.setTextColor(Color.BLACK);
				else
					ToggleDrawLine.setTextColor(Color.RED);
				DrawLineStatus=!DrawLineStatus;
			}			
		};
		private OnClickListener pressNewPt=new OnClickListener()
		{
			@Override
			public void onClick(View v) {
				NativeLib.getNewPt();
			}	
		};
		private OnClickListener pressTedit=new OnClickListener()
		{
			@Override
			public void onClick(View v) {
				Log.i("edit", "toggled: "+edit);
				
				if(!NativeLib.toggleEdit())
					return;
				edit=!edit;
				if(EditStatus)
					toggleEdit.setTextColor(Color.BLACK);
				else
					toggleEdit.setTextColor(Color.RED);
				EditStatus=!EditStatus;
				
			}			
		};
		
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
				String s=new String("");
				s=s+storageFolder+"/sMarker.txt";
				NativeLib.saveSMarker(s);
			}
			
		};
		
		private OnClickListener pressAddBox=new OnClickListener()
		{

			@Override
			public void onClick(View v) {
				NativeLib.addBox();
			}
			
		};
		
		private OnClickListener pressSetColor=new OnClickListener()
		{

			@Override
			public void onClick(View v) {
				setColorUI.show();
			}
			
		};
		
		private OnClickListener pressChangeColor=new OnClickListener()
		{

			@Override
			public void onClick(View v) {
				NativeLib.changeColor();
			}
			
		};
		
		private OnClickListener pressFixVA=new OnClickListener()
		{

			@Override
			public void onClick(View v) {
				NativeLib.fixViewAngle();
				fixVA=!fixVA;
				if(fixVA)
					fixViewAngle.setTextColor(Color.RED);
				else
					fixViewAngle.setTextColor(Color.BLACK);
			}
			
		};
		
	    @Override
	    public void onBackPressed(){
	        mCamera.stopPreview();
	        mCamera.setPreviewCallback(null);
	        mCamera.release();
	        mCamera = null;
	        
			finish();
	    }
	    
		@Override
		protected void onPause()
		{
			super.onPause();
	        mSenMan.unregisterListener(SELgyroscope);
		}
		@Override
		protected void onResume()
		{
			super.onResume();
	        //if use marker, no need to use sensors
	        mSenMan.registerListener(SELgyroscope,Gyroscope, SensorManager.SENSOR_DELAY_NORMAL);
		}
		@Override
		protected void onStop()
		{
	    	super.onStop();
	        mSenMan.unregisterListener(SELgyroscope);
		}

		
		private class MoveForwardTouch implements OnTouchListener{

			@Override
			public boolean onTouch(View v, MotionEvent event) {
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
		@Override
		public boolean onTouch(View v, MotionEvent event) {
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
					float amount=event.getX()-pX;
					if(Math.abs(amount)<0.001)
						break;
					NativeLib.Edit(currentEditType, currentAxis,amount );
					Log.e("edit type", ""+currentEditType);
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
			menu.add(Menu.NONE,Menu.FIRST+3,3,"Send Scene");	
			menu.add(Menu.NONE,Menu.FIRST+4,4,"Send Marker");	
			menu.add(Menu.NONE,Menu.FIRST+5,5,"Load Scene");
			menu.add(Menu.NONE,Menu.FIRST+6,6,"Save Scene");
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
				NativeLib.sendMainScene();
				break;
				
			case Menu.FIRST+4:
				NativeLib.sendSMarker();
				break;
			case Menu.FIRST+5:
				NativeLib.loadScene(OSGstorageFolder);
				break;
			case Menu.FIRST+6:
				NativeLib.saveScene(OSGstorageFolder);
				break;
			}
			return false;
			
		}
		
	    public void setupCamera() {

	        mCamera=Camera.open();
		    synchronized (this) {
		          if (mCamera != null) {  
		        	  Log.d("setupCamera", "camera opened");
		             Camera.Parameters params = mCamera.getParameters();
		             int mFrameWidth;
		             int mFrameHeight;
	                 mFrameWidth=mWidth;
	                 mFrameHeight=mHeight;
		             params.setPreviewSize(mFrameWidth, mFrameHeight);
		             NativeLib.InitialozeOpenCVGlobalVarByLoad(storageFolder);
		             Log.i("Init CV Global VR","Ok");
		             mCamera.setParameters(params);		                
			         previewCallback=new MyPreviewCallback();
			   	     mCamera.setPreviewCallback(previewCallback);
				     camView=new SurfaceView(OSGActivity.this);
			   	     previewCallback.setupPreviewBitmap(mFrameHeight, mFrameWidth);
			   	     previewCallback.setupMat();   
		             params = mCamera.getParameters();
		              mCamera.startPreview();
		              Log.d("setupCamera", "start preview");
		            }
		        
		    }
	        
	    }



}
