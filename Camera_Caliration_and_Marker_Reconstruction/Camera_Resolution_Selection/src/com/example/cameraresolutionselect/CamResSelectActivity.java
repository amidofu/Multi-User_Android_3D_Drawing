package com.example.cameraresolutionselect;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;




import android.hardware.Camera;
import android.os.Bundle;
import android.os.Environment;
import android.app.Activity;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

public class CamResSelectActivity extends Activity {

	List<Camera.Size> sizes;
	Camera.Size selectedSize;
	Camera mCamera;
	TextView Message;
	Button Save;
	String storagePath;
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_cam_res_select);
        getCameraSizeAndSetupMenu();
        storagePath=new String("");
        Message=(TextView)findViewById(R.id.Message);
        Save=(Button)findViewById(R.id.Save);
        Save.setOnClickListener(pressSave);
        checkStorage();
        showMessage(storagePath);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.activity_cam_res_select, menu);
        return true;
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
    public boolean onPrepareOptionsMenu(Menu menu){
    	menu.clear();
    	for(int i=0;i<sizes.size();i++)
    	{
    		Camera.Size size=sizes.get(i);
    		String string=new String("");
    		string=string+size.width;
    		string=string+"x";
    		string=string+size.height;
    		menu.add(Menu.NONE,Menu.FIRST+i,i,string);
    	}
    	return true;
    }
	@Override
	public boolean onOptionsItemSelected(MenuItem item)
	{
		int sizeID=item.getItemId();
		int page=(sizeID-1)/5;
		int offset=(sizeID-1)%5;
		selectedSize=sizes.get(page*5+offset);
		String s=new String("");
		s=s+"current select size: "+selectedSize.width+"x"+selectedSize.height;
		showMessage(s);
		return true;
	}
    void getCameraSizeAndSetupMenu()
    {
    	 mCamera=Camera.open();
 	    synchronized (CamResSelectActivity.this) {
 	          if (mCamera != null) {  
 	        	  Log.d("setupCamera", "camera opened");
 	             Camera.Parameters params = mCamera.getParameters();
 	             sizes = params.getSupportedPreviewSizes();
 	          }
 	    }
    }
    
    void checkStorage()
    {
    	String path=new String(Environment.getExternalStorageDirectory()+"/marker_data");
    	File storage=new File(path);
    	boolean success=false;
    	if(!storage.exists())
    	{
    		success=storage.mkdir();
    	}
    	if(!success)//folder exist, try to load cam param
    	{
    		String s=new String("folder exists");
    		showMessage(s);
    		storagePath=path+"/SelectRes.txt";
    	}
    	else// folder doesn't exist, create the folder
    	{
    		String s=new String("folder doesn't exist");
    		showMessage(s);
    		storagePath=path+"/SelectRes.txt";
    	}
    }
    
    
    void showMessage(final String msg)
    {
    	CamResSelectActivity.this.runOnUiThread(new Runnable(){

			@Override
			public void run() {
				Message.setText(msg);		
			}
		});
    }
    
    private OnClickListener pressSave=new OnClickListener()
    {

		@Override
		public void onClick(View v) {
			if(selectedSize==null)
			{
				String s=new String("please select resolution from menu first");
				showMessage(s);
				return;
			}
			FileWriter fw;
			try {
				fw = new FileWriter(storagePath);

			String s=new String("");
			s=s+selectedSize.width+" "+selectedSize.height;
			fw.write(s);
			fw.close();
			} catch (IOException e) {
				String s=new String("save wrong");
				showMessage(s);
				e.printStackTrace();
			}
		}
    	
    };
}
