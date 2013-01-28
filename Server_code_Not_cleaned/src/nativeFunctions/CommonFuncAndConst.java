package nativeFunctions;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OptionalDataException;
import java.io.StreamCorruptedException;
import java.net.DatagramPacket;

public class CommonFuncAndConst {

	//set the size of byte buffer which is for send/read data by net working
	
	public static final int BufferSize=1024;
	/*
	public static ObjectOutputStream oos=null;
	public static ObjectInputStream ois=null;
	public static ByteArrayOutputStream bos=null;
	public static ByteArrayInputStream bis=null;
	*/
	//convert a serializable class to byte array
	public static <T extends Object> byte[] classObjectToByteArray(T in){
		ByteArrayOutputStream bos =new ByteArrayOutputStream();
		ObjectOutputStream oos=null;
		try {
			oos=new ObjectOutputStream(bos);
		} catch (IOException e) {
			e.printStackTrace();
		}
		try {
			oos.writeObject(in);
			oos.flush();
			oos.close();
			
		} catch (IOException e) {
			e.printStackTrace();
		}
		byte [] data=bos.toByteArray();

		try {
			bos.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return data;
	}
	
	//convert a byte array to Object class, the output can be cast to other subclass of Object
	public static Object byteArraytoObject(byte[] data){
		if(data.length==0)
			return null;
		ByteArrayInputStream bis=new ByteArrayInputStream(data);
		if(bis==null)
			return null;
        ObjectInputStream ois;
        try {
			ois=new ObjectInputStream(bis);
		} catch (StreamCorruptedException e) {
			
			e.printStackTrace();
			return null;
		} catch (IOException e) {
			
			e.printStackTrace();
			return null;
		}
        Object ans=null;
        try {
        	if(ois!=null)
        		ans=ois.readObject();
		} catch (OptionalDataException e) {
			e.printStackTrace();
			return null;
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
			return null;
		} catch (IOException e) {
			e.printStackTrace();
			return null;
		}
        return ans;
	}
    public static final float NS2S = 1.0f / 1000000000.0f;
    public static final float AccelerationScalar=15.0f;
    public static final  float EPSILON=0.001f;
    public static final  float AccelerationThreshold=0.13f;
}
