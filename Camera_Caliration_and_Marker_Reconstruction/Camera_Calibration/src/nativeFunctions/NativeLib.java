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
	native public static void saveCameraParam(String filePath);
	native public static void startCalibrate();
	native public static void InitializedOpenCVGlobalVar(int width, int height);
}
