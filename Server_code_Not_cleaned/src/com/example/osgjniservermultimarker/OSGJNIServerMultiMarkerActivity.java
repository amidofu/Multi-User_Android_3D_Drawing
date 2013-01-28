package com.example.osgjniservermultimarker;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Timer;
import java.util.TimerTask;

import nativeFunctions.NativeLib;

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
import android.widget.TextView;

public class OSGJNIServerMultiMarkerActivity extends Activity {

	TextView viewIP;
	Button StartServer;
	Button GoOSG;
	//OSGActivity mOSGAct;
	Button End;
	Timer timer;
	@Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_osgjniserver_multi_marker);
        timer=new Timer();
        viewIP=(TextView)findViewById(R.id.ViewIP);
        /*
        String someIP=new String(" some IP");
        String host=new String("www.google.com.tw");
        try {
			someIP=someIP+InetAddress.getByName(host).getHostAddress();
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		*/
        viewIP.setText(getLocalIpAddress());
        StartServer=(Button)findViewById(R.id.StartServer);
        StartServer.setOnClickListener(pressStartServer);
        GoOSG=(Button)findViewById(R.id.GoOSG);
        GoOSG.setOnClickListener(NextActivity);
        End=(Button)findViewById(R.id.End);
        End.setOnClickListener(EndActivity);
    }
    
    
	private class exchangeAcceInfo extends TimerTask{

		@Override
		public void run() {
			// TODO Auto-generated method stub
			//Log.d("server", "timer task");
			NativeLib.sendAndRecvAcceInfoToOther();
		}
	};
    
    private OnClickListener pressStartServer=new OnClickListener(){

		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			NativeLib.initServer();
			timer.scheduleAtFixedRate(new exchangeAcceInfo(), 5000, 100);
			
		}
    	
    };
    private OnClickListener NextActivity = new OnClickListener(){

		@Override
		public void onClick(View v) {
			//start OSGActivity (go to OSG view)
			Log.i("start", "OSG");
			Intent intent = new Intent(OSGJNIServerMultiMarkerActivity.this,OSGActivity.class);
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
    	/*
        try {
            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces(); en.hasMoreElements();) {
                NetworkInterface intf = en.nextElement();
                for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses(); enumIpAddr.hasMoreElements();) {
                    InetAddress inetAddress = enumIpAddr.nextElement();
                    if (!inetAddress.isLoopbackAddress()) {
                        return inetAddress.getHostAddress().toString();
                    }
                }
            }
        } catch (SocketException ex) {
            Log.e("socket exception", ex.toString());
        }
        return null;
        */
    	
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
