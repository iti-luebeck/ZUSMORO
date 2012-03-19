package tests;

import java.util.Observable;
import java.util.Observer;

import model.onboard.BTLayer;
import model.onboard.TransmissionJob;
import robots.epuck.Communicator;

public class OnBoardTestMemoryKiller implements Observer {

	/**
	 * @param args
	 */
	
	private enum teststate{
		Z,z,G,t,g,b, B;
	}
	
	private class MemoryKiller implements TransmissionJob {
		@Override
		public boolean check(String lastMsgRec, String lastMsgSent) {
			System.out.println("!!!!!!!MALLOC!!!!!!!!!!");
			ok=false;
			return true;
		}
		teststate s;
		private int i;
		private int j;
		boolean ok;
		

		public MemoryKiller() {
			s=teststate.Z;
			i=45;
			j=0;
			ok=true;
		}

		@Override
		public double completionStatus() {
			// TODO Auto-generated method stub
			return 0;
		}

		@Override
		public String getFirst() {
			String st="Error";
			switch (s) {
			case Z:
				st=Z(i);
				break;
			case z:
				st=z(j);
				break;
			case G:
				st=G(j);
				break;
			case t:
				st=t(j);
				break;
			case g:
				st=g(j);
				break;
			case B:
					st=B();
					break;
				case b:
					st=b();
					break;
		}
			return st;
		}

		private String B() {
			s=teststate.b;
			return "B1";
		}

		private String b() {
			i++;
			s=teststate.Z;
			return "b0,0,0";
		}

		private String g(int j2) {
			j++;
			s=teststate.t;
			return "g0";
		}

		private String t(int j2) {
			if(j<i){
			s=teststate.g;
			return "t"+j+",1,"+(j+1);
			}
			else{
				s=teststate.B;
				return "t"+j+",1,0";
			}
		}

		private String G(int j2) {
			s=teststate.z;
			j++;
			return "G"+j;
		}

		private String z(int j2) {
			if(j<i){
			s=teststate.G;
			return "z"+j+",1";}
			else{
				j=0;
				s=teststate.t;
				return "T"+i;
			}
		}

		private String Z(int i) {
			j=0;
			s=teststate.z;
			return "Z"+i;
		}

		@Override
		public boolean isComplete() {
			// TODO Auto-generated method stub
			return !ok;
		}

		@Override
		public void removeFirst() {
			// TODO Auto-generated method stub
			
		};
	}

	static MemoryKiller mk;

	public OnBoardTestMemoryKiller() {
		mk = new MemoryKiller();
	}

	public MemoryKiller getMk() {
		return mk;
	}

	public static void main(String[] args) {

		try {
			OnBoardTestMemoryKiller test = new OnBoardTestMemoryKiller();
			// ta.printTest();
			MemoryKiller mk = test.getMk();

			Communicator c = new Communicator("/dev/rfcomm1");
			BTLayer btl = new BTLayer(c, mk);
			if (!btl.start()) {
				System.out.println("Antwort daneben.");

			} else {
				while (!mk.isComplete()) {
					Thread.sleep(100);
				}

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
