package com.example.osgjniclientmultimarkercleared;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;
import java.util.Timer;
import java.util.TimerTask;
import nativeFunctions.NativeLib;
import android.os.Bundle;
import android.os.Environment;
import android.app.Activity;
import android.content.Intent;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ToggleButton;

public class OSGJNIClientMultiMarkerClearedActivity extends Activity {

	Timer timer;
    Button StartClient;
    Button ConnectServer;
    Button GoOSG;
    Button End;
    EditText ServerIP;
    String ServerIPAddr;
    Button SaveServer;
    ToggleButton UseIP;
	@Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_osgjniclient_multi_marker_cleared);
        timer=new Timer();
        StartClient=(Button)findViewById(R.id.StartClient);
        StartClient.setOnClickListener(pressStart);
        ConnectServer=(Button)findViewById(R.id.Connect);
        ConnectServer.setOnClickListener(pressConnectServer);
        GoOSG=(Button)findViewById(R.id.GoOSG);
        GoOSG.setOnClickListener(OSGActivity);
        End=(Button)findViewById(R.id.End);
        End.setOnClickListener(EndActivity);
        ServerIP=(EditText)findViewById(R.id.ServerIP);
        //load server's IP address or domain name
        String server=loadServer();
        if(server!=null)
        	ServerIP.setText(server);
        
        SaveServer=(Button)findViewById(R.id.SaveServer);
        SaveServer.setOnClickListener(pressSaveServer);
        
        UseIP=(ToggleButton)findViewById(R.id.UseIPAddr);
        
    }
    
    private class exchangeAcceInfo extends TimerTask{

		@Override
		public void run() {
			NativeLib.sendAndRecvAcceInfoToOther();
		}
	};
	
	private OnClickListener pressStart=new OnClickListener(){

		@Override
		public void onClick(View v) {
			ServerIPAddr=ServerIP.getText().toString();
			String IP=ServerIPAddr;
			if(UseIP.isChecked())
				NativeLib.getServerIPandStartClient(IP);
			else
				NativeLib.getServerIPByDomainNameAndStartClient(IP);
		}
		
	};
	
	//save server's IP address or domain name
	private OnClickListener pressSaveServer=new OnClickListener()
	{

		@Override
		public void onClick(View arg0) {
			saveServer();
		}
		
		
	};
	
    private OnClickListener pressConnectServer=new OnClickListener(){

		@Override
		public void onClick(View v) {
			NativeLib.connectToServer();
			timer.scheduleAtFixedRate(new exchangeAcceInfo(), 5000, 100);
		}
    	
    };
    
    //start OSG
    private OnClickListener OSGActivity = new OnClickListener(){

		@Override
		public void onClick(View v) {
			Intent intent = new Intent(OSGJNIClientMultiMarkerClearedActivity.this,OSGActivity.class);
			startActivity(intent);
		}
    	
    };
    
    private OnClickListener EndActivity=new OnClickListener(){
		@Override
		public void onClick(View v) {
			finish();
		}
    };
    
    String loadServer()
    {
    	String path=new String(Environment.getExternalStorageDirectory()+"/marker_data/Server.txt");
    	try {
			FileReader fr=new FileReader(path);
			Scanner scanner=new Scanner(fr);
			
			if(scanner.hasNext())
			{
					return scanner.next();
			}
			else
				return null;
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return null;
		}
    }
    
    void saveServer()
    {
    	String path=new String(Environment.getExternalStorageDirectory()+"/marker_data/Server.txt");
    	try {
			FileWriter fw=new FileWriter(path);
			BufferedWriter out=new BufferedWriter(fw);
			out.write(ServerIP.getText().toString());
			out.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
}
