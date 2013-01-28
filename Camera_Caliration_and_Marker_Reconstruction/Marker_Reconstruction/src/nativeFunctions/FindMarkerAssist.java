package nativeFunctions;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import com.example.markerpositionestimationchangerescleared.MyPreviewCallback;



public class FindMarkerAssist {
	public int [] MarkerID;
	public int [] numImgsGot;
	boolean saveFrameOK;
	public int NumMarkerFound;
	public HashSet<MyInt> BuiltMarkerSet;
	boolean finished;
	public FindMarkerAssist()
	{
		finished=true;
		MarkerID=new int[MyPreviewCallback.detectMarkerOneTime];
		numImgsGot=new int[MyPreviewCallback.detectMarkerOneTime];
		saveFrameOK=false;
		BuiltMarkerSet=new HashSet<MyInt>();
	}
	public void ToFindMarker()
	{
		finished=false;
	}
}
