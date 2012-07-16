package tests;

import java.io.File;
import java.util.Observable;
import java.util.Observer;

import model.Automat;
import model.onboard.BTLayer;
import model.onboard.TransmitAutomat;
import robots.epuck.Communicator;

public class OnBoardTest implements Observer {

	/**
	 * @param args
	 */

	public static void main(String[] args) {

		try {
			OnBoardTest test = new OnBoardTest();
			File f = new File("/mnt/mp3/BA/beeperZUSMORO.xml");
			TransmitAutomat ta = new TransmitAutomat(Automat.loadAutomat(f));
			// ta.printTest();
			Communicator c = new Communicator("/dev/rfcomm1");
			BTLayer btl = new BTLayer(c, ta);
			if (!btl.start()) {
				System.out.println("Antwort daneben.");

			} else {
				while (!ta.isComplete()) {
					Thread.sleep(100);
				}
				c.addObserver(test);
				c.writeToStream("Sx\r");
				Thread.sleep(1000);
				c.writeToStream("d7\r");
				Thread.sleep(1000);
				c.writeToStream("d1\r");
				Thread.sleep(1000 * 10);
				c.writeToStream("sx\r");
				Thread.sleep(1000*3);
				c.writeToStream("Sx\r");

				Thread.sleep(1000 * 20);
				c.writeToStream("sx\r");

			}
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	@Override
	public void update(Observable o, Object arg) {
		System.out.println("Debug:" + (String) arg);

	}

}
