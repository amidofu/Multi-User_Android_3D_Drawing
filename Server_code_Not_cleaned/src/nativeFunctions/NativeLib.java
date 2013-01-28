package nativeFunctions;


public class NativeLib {
	static {
		System.loadLibrary("NativeLib");
	}
	public static native void initServer();
	public static native void sendMsg();
	public static native void recvMsg();
	public static native void connectToServer(String IP);
	public static native void sendAndRecvAcceInfoToOther();
	
	//OSG Functions
	public static native void OSGRun();
	public static native void initOSG(int width,int height);
    public static native void mouseButtonPressEvent(float x,float y, int button);
    public static native void mouseButtonReleaseEvent(float x,float y, int button);
    public static native void mouseMoveEvent(float x,float y);
    public static native void sendRotation(float axisX, float axisY, float axisZ, float angle);
    public static native void sendKeyDown(int key);
    public static native void sendGeometry();
    public static native void recvGeometry();
    public static native void sendMainScene();
    public static native void toggleDrawLine();
    public static native void getNewPt();
    public static native boolean toggleEdit();
    public static native void EditR(int axis,float amount);
    public static native void Edit(int type,int axis,float amount);
    public static native void deleteGeometry();
    native public static void addBox();
    native public static void setColor(float r,float g, float b);
    native public static void changeColor();
    
    //OpenCV Funcitons
    native public static void FindFeatures(long matAddrGr, long matAddrRgba);
    native public static void MarkerDetection(long matAddrRgba,int height,int width,int RectLimit);
    
    native public static void InitializedOpenCVGlobalVar(int width, int height);
    native public static void InitialozeOpenCVGlobalVarByLoad(String filePath);
    
    //others
    native public static void sendSMarker();
    native public static void saveSMarker(String filePath);
	native public static void saveScene(String OSGPath);
	native public static void loadScene(String OSGPath);
}
