package nativeFunctions;
import java.util.HashSet;
import com.example.markerpositionestimationchangerescleared.MyPreviewCallback;
//class that is used to store info about the marker reconstruction from native code


public class FindMarkerAssist {
	//store the ID of detected markers in camera preview
	public int [] MarkerID;
	//store the number of images that is stored for each detected marker
	public int [] numImgsGot;
	//indicate if images info for non-reconstructed markers is saved
	boolean saveFrameOK;
	//shows the number of detected marker
	public int NumMarkerFound;
	//store the info of markers
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
