package com.example.markerpositionestimationchangerescleared;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Collection;
import java.util.Iterator;
import java.util.Scanner;
import java.util.Timer;
import java.util.TimerTask;

import nativeFunctions.MyInt;
import nativeFunctions.NativeLib;


import android.os.Bundle;
import android.os.Environment;
import android.app.Activity;
import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

public class MarkerPositionEstimationChangeResClearedActivity extends Activity {

	Button SaveFrame;
	  Button ComputePosition;
	  Button SaveResult;
	  TextView status;
	  Button ClearInvalid;
	  Button reCalMarker;
	  Camera mCamera;
		SurfaceView CamSurfaceView;
		MyPreviewCallback previewCallback;
		byte [] mBuffer;
		byte [] mFrame;
		SurfaceHolder mHolder;
		boolean setupCamLock;
		Timer timer;
		int mWidth;
		int mHeight;
		String storageFolder;
		
		@Override
	    public void onCreate(Bundle savedInstanceState) {
	        super.onCreate(savedInstanceState);
	        setContentView(R.layout.activity_marker_position_estimation_change_res_cleared);
	        CamSurfaceView=(SurfaceView)findViewById(R.id.CamSurfaceView);
	        SaveFrame=(Button)findViewById(R.id.SaveFrame);
	        ComputePosition=(Button)findViewById(R.id.ComputeMarkerPos);
	        SaveResult=(Button)findViewById(R.id.SaveResult);
	        status=(TextView)findViewById(R.id.Status);
	        SaveFrame.setOnClickListener(PressSaveFrame);
	        ComputePosition.setOnClickListener(PressComputeMarkerPos);
	        SaveResult.setOnClickListener(PressSaveResult);
	        ClearInvalid=(Button)findViewById(R.id.ClearInvalid);
	        ClearInvalid.setOnClickListener(PressClearInvalid);
	        reCalMarker=(Button)findViewById(R.id.Recalc);
	        reCalMarker.setOnClickListener(PressReCalc);
	        Log.d("On Create", "OpenCV loaded");
			storageFolder=new String(Environment.getExternalStorageDirectory()+"/marker_data");
			loadSelecteResolution();
	        setupCamera();
	        new Thread(previewCallback).start();
	        timer=new Timer();
	        timer.schedule(new refreshMarkerInfo(), 2000, 500);

	    }
	    
		private OnClickListener PressSaveFrame =new OnClickListener(){

			@Override
			public void onClick(View v) {
				previewCallback.toSaveMarkerFrame();
			}
			
		};
	    
		private OnClickListener PressComputeMarkerPos =new OnClickListener(){

			@Override
			public void onClick(View v) {
				NativeLib.ComputeMarkerPosition(previewCallback.FMA);
				Collection collection=previewCallback.FMA.BuiltMarkerSet;
				Iterator it=collection.iterator();
				String temp=new String("");
				temp+="marker reconstruction result\n";
				while(it.hasNext())
				{	
					MyInt MI=(MyInt)it.next();
						temp=temp+"ID: "+MI.value+" built\n";	
				}
				previewCallback.SaveMarkerFrameFinished=false;
				status.setText(temp);
			}
			
		};
		
	    
		private OnClickListener PressSaveResult =new OnClickListener(){

			@Override
			public void onClick(View v) {
				NativeLib.saveAllMarkers();
			}
			
		};
		
		private OnClickListener PressClearInvalid=new OnClickListener(){

			@Override
			public void onClick(View v) {
				NativeLib.clearNonValidMarkerImage();
				status.setText("");
			}
			
		};
		
		private OnClickListener PressReCalc=new OnClickListener(){

			@Override
			public void onClick(View v) {
				NativeLib.reComputeMarkerPosition();
				status.setText("Marker recalculated");
			}
			
		};
	    
		
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
	    
		public void setupCamera()
		{
			
	        mCamera=Camera.open();
		    synchronized ( MarkerPositionEstimationChangeResClearedActivity.this) {
		          if (mCamera != null) {  
		        	  Log.d("setupCamera", "camera opened");
		             Camera.Parameters params = mCamera.getParameters();
		             int mFrameWidth;
		             int mFrameHeight;
	                 mFrameWidth=mWidth;
	                 mFrameHeight=mHeight;
		             params.setPreviewSize(mFrameWidth, mFrameHeight);
		             NativeLib.InitialozeOpenCVGlobalVarByLoad(storageFolder);     
		                
		             mCamera.setParameters(params);
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
		              /* Now allocate the buffer */
		             params = mCamera.getParameters();
		             int size = params.getPreviewSize().width * params.getPreviewSize().height;
		             size  = size * ImageFormat.getBitsPerPixel(params.getPreviewFormat()) / 8;
		             mBuffer = new byte[size];
		             /* The buffer where the current frame will be copied */
		             mFrame = new byte [size];
		             previewCallback.setFrameSize(size);
		              mCamera.startPreview();
		              Log.d("setupCamera", "start preview");
		            }	        
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
		
		private class refreshMarkerInfo extends TimerTask
		{

			@Override
			public void run() {
				MarkerPositionEstimationChangeResClearedActivity.this.runOnUiThread(new Runnable(){

					@Override
					public void run() {
						String temp=new String("");
						if(previewCallback.SaveMarkerFrameFinished)
						{
							
							for(int i=0;i<previewCallback.FMA.NumMarkerFound;i++)
							{
								temp=temp+"added, ID: "+previewCallback.FMA.MarkerID[i]+" ,numImgs: "+previewCallback.FMA.numImgsGot[i]+"\n";
							}
							status.setText(temp);
						}
					}
		        	
		        });
			}
			
		}
}
