package tests;

import java.io.File;
import java.util.Observable;
import java.util.Observer;

import model.Automat;
import model.onboard.BTLayer;
import model.onboard.TransmitAutomat;
import robots.epuck.Communicator;

public class OnBoardTestControlledMotors extends TransmitAutomat implements Observer {

		public OnBoardTestControlledMotors() throws Exception {
			super(Automat.loadAutomat(new File("G:\\ZUSMORO\\zusmoro20101004\\run.xml")));
		printTest();
			/*
			sequence.clear();

			sequence.add("Z2");
			sequence.add("z0,1");
			sequence.add("L1");
			sequence.add("l0,-300,300,1000,0,800");
			sequence.add("r0,-300,300,1000,0,800");
			sequence.add("G0");
			sequence.add("y0");

			sequence.add("z1,1");
			sequence.add("L16");
			sequence.add("l-500,-300,300,0,0,500");
			sequence.add("r-500,-300,300,0,0,500");
			sequence.add("G1");
			sequence.add("y1");

			sequence.add("Y2");

			sequence.add("T2");

			sequence.add("t0,2,1");
			sequence.add("g0");
			sequence.add("g1");
			sequence.add("u0");

			sequence.add("t1,1,0");
			sequence.add("g2");
			sequence.add("u1");

			sequence.add("U2");
			sequence.add("B3");
			sequence.add("b0,0,0,805");
			sequence.add("b1,5,0,795");
			sequence.add("b2,0,-1112,-800");

			sequence.add("C3");

			sequence.add("A0");
			*/

		}





	/**
	 * @param args
	 */

	public static void main(String[] args) {

		try {
			OnBoardTestControlledMotors test = new OnBoardTestControlledMotors();
			// ta.printTest();


			Communicator c = new Communicator("COM45");
			BTLayer btl = new BTLayer(c, test);
			if (!btl.start()) {
				System.out.println("Antwort daneben.");

			} else {
				while (!test.isComplete()) {
					Thread.sleep(100);
				}
				c.addObserver(test);
				c.writeToStream("d3\r");
				c.writeToStream("Sx\r");
				Thread.sleep(1000);
				Thread.sleep(1000);
				Thread.sleep(1000 * 8);
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
