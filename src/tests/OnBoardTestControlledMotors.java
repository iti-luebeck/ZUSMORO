/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
