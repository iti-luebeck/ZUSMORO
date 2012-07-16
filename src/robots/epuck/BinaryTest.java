package robots.epuck;

import java.util.Observable;
import java.util.Observer;

public class BinaryTest implements Observer {

	public static void main(String[] args) throws Exception {

//		BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
//		String read = reader.readLine();
//		System.out.println("Gelesenes Zeichen: " + read + " Code: "+((int)read.charAt(0)));
//		System.out.println("Gelesenes Zeichen: " + read + " Code: "+((int)read.charAt(1)));
		byte[] array = {(byte) 183, 0};
		//byte[] array2 = {(byte) 178,(byte) 191, 0};
		String bla = new String(array);
		//String bla2 = new String(array2);
		System.out.println(bla);
		Communicator comm = new Communicator("/dev/rfcomm1");
		comm.addObserver(new BinaryTest());
		comm.writeToStream("V\n");
		comm.writeToStream("V\n");
		//comm.writeToStream(bla2);
		Thread.sleep(1000);
		comm.writeToStream(bla);
		Thread.sleep(1000);
//		comm.writeToStream(bla2);
//		Thread.sleep(1000);
//		comm.writeToStream(bla);
//		Thread.sleep(1000);
//		comm.writeToStream(bla2);
//		Thread.sleep(1000);
//		comm.writeToStream(bla);
//		Thread.sleep(1000);
//		comm.writeToStream("V\n");
//		Thread.sleep(1000);
		comm.disconnect();
//		char c = 'I';
//		byte c2 = (byte) -c;
//
//		System.out.println(c);
//		System.out.println((char)-c);
	}

	public BinaryTest() {
		super();
	}

	public void update(Observable arg0, Object arg1) {
		System.out.println("Empfangen: "+arg1);
	}

}
