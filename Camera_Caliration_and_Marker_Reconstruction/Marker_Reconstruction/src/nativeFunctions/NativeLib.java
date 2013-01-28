package nativeFunctions;

public class NativeLib {

	static {
		try{
		System.loadLibrary("NativeLib");
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}

	native public static void FindRectangle(long matAddrRgba,int height,int width,int RectLimit);
	native public static void InitializedOpenCVGlobalVar(int width, int height);
	native public static void InitialozeOpenCVGlobalVarByLoad(String filePath);
	native public static void SaveNewMarkerFrame(long matAddrRgba,int height,int width,int RectLimit,FindMarkerAssist FMA);
	native public static void ComputeMarkerPosition(FindMarkerAssist FMA);
	native public static void saveAllMarkers();
	native public static void clearNonValidMarkerImage();
	native public static void reComputeMarkerPosition();
}
