package com.example.singleusermultimarker;

import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.app.Activity;
import android.content.Intent;
import android.util.Log;
import android.view.Menu;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;

public class SingleUserMultiMarkerActivity extends Activity {

	Button GoOSG;
	Button End;
	@Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_single_user_multi_marker);
        GoOSG=(Button)findViewById(R.id.GoOSG);
        GoOSG.setOnClickListener(NextActivity);
        End=(Button)findViewById(R.id.End);
        End.setOnClickListener(EndActivity);
    }
    
    private OnClickListener NextActivity = new OnClickListener(){

		@Override
		public void onClick(View v) {
			//start OSGActivity (go to OSG view)
			Log.i("start", "OSG");
			Intent intent = new Intent(SingleUserMultiMarkerActivity.this,OSGActivity.class);
			startActivity(intent);
			
		}
    	
    };
    
    private OnClickListener EndActivity=new OnClickListener(){

		@Override
		public void onClick(View v) {
			Log.i("end", "end");
			finish();
		}
    	
    };
    
    //get local IP address
    public String getLocalIpAddress() {
    	WifiManager wifiM=(WifiManager)getSystemService(WIFI_SERVICE);
    	WifiInfo wifiInfo=wifiM.getConnectionInfo();
    	int IPAddr=wifiInfo.getIpAddress();
    	return ((IPAddr)&0xFF)+"."+((IPAddr>>8)&0xFF)+"."+((IPAddr>>16)&0xFF)+"."+((IPAddr>>24)&0xFF);
    }
    
    
    @Override
    public void onBackPressed(){
		finish();
    }
}
