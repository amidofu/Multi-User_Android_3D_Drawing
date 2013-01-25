package nativeFunctions;



public class NativeLib {
	static {
		System.loadLibrary("NativeLib");
	}
	//use domain name as server's IP address
	public static native void getServerIPByDomainNameAndStartClient(String IP);
	//use real server's IP address
	public static native void getServerIPandStartClient(String IP);
	//public static native void sendMsg();
	//public static native void recvMsg();
	//start connection
	public static native void connectToServer();
	//public static native void sendAcceleration(float x, float y, float z);
	public static native void sendAndRecvAcceInfoToOther();

	
	//OSG Functions
	

    //public static native void resetRotation();	
	public static native void OSGRun();
	//initialize OSG objects
	public static native void initOSG(int width,int height);
    //handle mouse
	public static native void mouseButtonPressEvent(float x,float y, int button);
    public static native void mouseButtonReleaseEvent(float x,float y, int button);
    public static native void mouseMoveEvent(float x,float y);
    //used by gyro
    public static native void sendRotation(float axisX, float axisY, float axisZ, float angle);
    public static native void sendKeyDown(int key);
    //public static native void sendGeometry();
    //public static native void recvGeometry();
    //send all geometries to the other user
    public static native void sendMainScene();
    //toggle drawing line mode
    public static native void toggleDrawLine();
    //add new point for line
    public static native void getNewPt();
    //toggle geometry modification mode
    public static native boolean toggleEdit();
    //public static native void EditR(int axis,float amount);
    //send geometry modification command
    /**
     * 
     * @param type modification type (rotation, translation, scale)
     * @param axis transformation around axis
     * @param amount amount of transformation
     */
    public static native void Edit(int type,int axis,float amount);
    //delete selected geometry
    public static native void deleteGeometry();
    //add box in space
    native public static void addBox();
    //send current selected color to OSG
    native public static void setColor(float r,float g, float b);
    //confirm to change color for selected geometry
    native public static void changeColor();
    
    //OpenCV Funcitons
    //native public static void FindFeatures(long matAddrGr, long matAddrRgba);
    native public static void MarkerDetection(long matAddrRgba,int height,int width,int RectLimit);
    
    //native public static void InitializedOpenCVGlobalVar(int width, int height);
    native public static void InitialozeOpenCVGlobalVarByLoad(String filePath);
    
    //others
    //send saved marker info to other user
    native public static void sendSMarker();
    //save marker info that is sent from other user
    native public static void saveSMarker(String filePath);
	//save created geometries
    native public static void saveScene(String OSGPath);
	//load created geometries
    native public static void loadScene(String OSGPath);
}
