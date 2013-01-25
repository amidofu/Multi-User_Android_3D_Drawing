package com.example.osgjniclientmultimarkercleared;

import nativeFunctions.NativeLib;
import android.app.Dialog;
import android.content.Context;
//import android.content.DialogInterface;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.Button;
//import android.widget.FrameLayout;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
//import android.view.View.OnClickListener;
public class ChangeColorUI extends Dialog implements OnTouchListener{

	Button OK;
	SeekBar Red;
	SeekBar Green;
	SeekBar Blue;
	DrawView view;
	public int red,green,blue;
	public ChangeColorUI(Context context) {
		super(context);
		setContentView(R.layout.colorchange);
		setTitle("Change Color");
		OK=(Button)findViewById(R.id.ConfirmColor);
		OK.setOnClickListener( pressConfirmColor);
		Red=(SeekBar)findViewById(R.id.Red);
		Red.setOnSeekBarChangeListener(new  mSeekBarListener());
		
		Green=(SeekBar)findViewById(R.id.Green);
		Green.setOnSeekBarChangeListener(new  mSeekBarListener());
		
		Blue=(SeekBar)findViewById(R.id.Blue);
		Blue.setOnSeekBarChangeListener(new  mSeekBarListener());
		view=(DrawView)findViewById(R.id.ColorShowView);
		red=255;
		green=255;
		blue=255;
		
		
		
	}
	
	class mSeekBarListener implements OnSeekBarChangeListener
	{
		int  mProgress;

		public mSeekBarListener()
		{
			mProgress=0;
		}
		@Override
		public void onProgressChanged(SeekBar seekBar, int progress,
				boolean fromUser) {
			red=Red.getProgress();
			green=Green.getProgress();
			blue=Blue.getProgress();
			view.mColor=computeColor();
			view.invalidate();//redraw color preview square
			Log.i("mColor", ""+view.mColor+" r:"+red+", green:"+green+", blue:"+blue);
		}

		@Override
		public void onStartTrackingTouch(SeekBar seekBar) {
			
		}

		@Override
		public void onStopTrackingTouch(SeekBar seekBar) {
			
		}
		
	}

	@Override
	public boolean onTouch(View v, MotionEvent event) {

		red=Red.getProgress();
		green=Green.getProgress();
		blue=Blue.getProgress();
		view.mColor=computeColor();
		view.invalidate();
		Log.i("mColor", ""+view.mColor);

		return false;
	};
	
	//map RGB value to hexadecimal
	int computeColor()
	{
		int pre=0xff000000;
		pre=pre+(red<<(4*4));
		pre=pre+(green<<(4*2));
		pre=pre+blue;
		return pre;
	}
	
	//pass current selected color to native part for OSG
	private android.view.View.OnClickListener pressConfirmColor=new android.view.View.OnClickListener()
	{

		@Override
		public void onClick(View v) {
			
			float r=((float)red)/255.0f;
			float g=((float)green)/255.0f;
			float b=((float)blue)/255.0f;
			NativeLib.setColor(r, g, b);
			dismiss();
		}
		
	};
	

}
