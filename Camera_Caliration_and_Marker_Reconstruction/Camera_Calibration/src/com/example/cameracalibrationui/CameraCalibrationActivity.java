package com.example.cameracalibrationui;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.Scanner;


import nativeFunctions.NativeLib;

import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Environment;
import android.app.Activity;
import android.util.Log;
import android.view.Menu;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

public class CameraCalibrationActivity extends Activity {

	Camera mCamera;
	List<Camera.Size> sizes;
	Camera.Size selectedSize;
	SurfaceView CamSurfaceView;
	
	MyPreviewCallback previewCallback;
	SurfaceHolder mHolder;
	String storagePath;
	String cameraParamOutput;
	String errorMessage;
	Button checkFile;
	Button saveFile;
	Button Calibrate;
	TextView Message;
	int width,height;
	byte [] mBuffer;
	byte [] mFrame;
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera_calibration);
        
        
        CamSurfaceView=(SurfaceView)findViewById(R.id.CamSurfaceView);
        checkFile=(Button)findViewById(R.id.CheckFile);
        checkFile.setOnClickListener(pressCheckFile);
        saveFile=(Button)findViewById(R.id.saveResult);
        saveFile.setOnClickListener(pressSaveFile);
        Calibrate=(Button)findViewById(R.id.Calibrate);
        Calibrate.setOnClickListener(pressCalibrate);
        Message=(TextView)findViewById(R.id.Message);
        storagePath=new String(Environment.getExternalStorageDirectory()+"/marker_data/CamParam.txt");
        loadSelecteResolution();
        Log.i("check","checked");
        setupCamera();
        new Thread(previewCallback).start();
        
        selectedSize=null;
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
					width=scanner.nextInt();
				else
					Log.e("read camera param", "width wrong");
				if(scanner.hasNextInt())
					height=scanner.nextInt();
				else
					Log.e("read camera param", "height wrong");
			}
		} catch (FileNotFoundException e) {
			String s=new String("selected resolution not found");
			showMessage(s);
			e.printStackTrace();
		}
    }

	@Override
	public void onBackPressed()
	{
        mCamera.stopPreview();
        mCamera.setPreviewCallback(null);
        mCamera.release();
        mCamera = null;
        
        this.finish();
	}
	
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.activity_calibrate, menu);
        return true;
    }
    
    
    private OnClickListener pressCheckFile=new OnClickListener()
    {

		@Override
		public void onClick(View v) {
			
	        //startPreview();
	        Log.i("check","checked");
			checkStorage();
		}
    
    };

    private OnClickListener pressSaveFile=new OnClickListener()
    {

		@Override
		public void onClick(View v) {
			NativeLib.saveCameraParam(storagePath);
		}
    	
    };
    
    private OnClickListener pressCalibrate=new OnClickListener()
    {

		@Override
		public void onClick(View v) {
			NativeLib.startCalibrate();
		}
    	
    };
    void checkStorage()
    {
    	String path=new String(Environment.getExternalStorageDirectory()+"/marker_data");
    	File storage=new File(path);
    	boolean success=false;
    	if(!storage.exists())
    	{
    		success=storage.mkdir();
        	if(!success)//folder exist, try to load cam param
        	{
        		String s=new String("folder creat fail, ");
        		s=s+path;
        		showMessage(s);
        		showCameraParam(storagePath);
        	}
        	else// folder doesn't exist, create the folder
        	{
        		String s=new String("folder created, ");
        		s=s+path;
        		showMessage(s);
        	}
    	}
    	else
    	{
    		String s=new String("folder exist, ");
    		s=s+path;
    		showMessage(s);
    	}

    }
    
    void showCameraParam(String filePath)
    {
    	try {
    		int width=0,height=0;
    		float values[]=new float[13];
			FileReader fin=new FileReader(filePath);
			Scanner scanner=new Scanner(fin);
			if(scanner.hasNext())
			{
				if(scanner.hasNextInt())
					width=scanner.nextInt();
				else
					Log.e("read camera param", "width wrong");
				if(scanner.hasNextInt())
					height=scanner.nextInt();
				else
					Log.e("read camera param", "height wrong");
			}
			for(int i=0;i<13;i++)
			{
				if(scanner.hasNextFloat())
					values[i]=scanner.nextFloat();
				else
					Log.e("read camera param", "cam param wrong at: "+i);
			}
			try {
				fin.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
			
			cameraParamOutput=new String("");
			cameraParamOutput=cameraParamOutput+"width: "+width+", height: "+height+"\n";
			cameraParamOutput=cameraParamOutput+"camera matrix:\n";
			cameraParamOutput=cameraParamOutput+values[0]+" "+values[1]+" "+values[2]+"\n";
			cameraParamOutput=cameraParamOutput+values[3]+" "+values[4]+" "+values[5]+"\n";
			cameraParamOutput=cameraParamOutput+values[6]+" "+values[7]+" "+values[8]+"\n";
			cameraParamOutput=cameraParamOutput+"distortion params:\n";
			cameraParamOutput=cameraParamOutput+values[9]+" "+values[10]+" "+values[11]+" "+values[12];
			CameraCalibrationActivity.this.runOnUiThread(new Runnable(){

				@Override
				public void run() {
					Message.setText(cameraParamOutput);		
				}
			});
			
			
		} catch (FileNotFoundException e) {
			errorMessage=new String("cam param file not found");
			CameraCalibrationActivity.this.runOnUiThread(new Runnable(){

				@Override
				public void run() {
					Message.setText(errorMessage);		
				}
			});
			e.printStackTrace();
		}
    }
 
    void showMessage(final String msg)
    {
    	CameraCalibrationActivity.this.runOnUiThread(new Runnable(){

			@Override
			public void run() {
				Message.setText(msg);		
			}
		});
    }
    
    
    
	public void setupCamera()
	{
		
        mCamera=Camera.open();
	    synchronized (CameraCalibrationActivity.this) {
	          if (mCamera != null) {  
	        	  Log.d("setupCamera", "camera opened");
	             Camera.Parameters params = mCamera.getParameters();
	             int mFrameWidth = width;
	             int mFrameHeight = height;
	              mFrameWidth = width;
	              mFrameHeight = height;
	             params.setPreviewSize(mFrameWidth, mFrameHeight);      
	             mCamera.setParameters(params);
	             NativeLib.InitializedOpenCVGlobalVar(mFrameWidth, mFrameHeight);
		         previewCallback=new MyPreviewCallback();
		   	     mCamera.setPreviewCallback(previewCallback);
		   	     mHolder=CamSurfaceView.getHolder();
		   	     previewCallback.setSurfaceHolder(mHolder);
		   	     previewCallback.setupPreviewBitmap(mFrameHeight, mFrameWidth);
		   	     previewCallback.setupMat();
		   	     Log.d("setupCamera", "previewCallback Ok");
			     try {
			 		mCamera.setPreviewDisplay(mHolder);
			 	 }catch (IOException e) {
			 			e.printStackTrace();
			 	 }
	             params = mCamera.getParameters();
	             int size = params.getPreviewSize().width * params.getPreviewSize().height;
	             size  = size * ImageFormat.getBitsPerPixel(params.getPreviewFormat()) / 8;
	             mBuffer = new byte[size];

	             mFrame = new byte [size];

	              mCamera.startPreview();
	              Log.d("setupCamera", "start preview");
	            }	        
	          }
	}
	
    
    

}
