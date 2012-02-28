package model.onboard;

import java.io.IOException;
import java.util.Observable;
import java.util.Observer;

import robots.epuck.Communicator;

public class BTLayer implements Observer {

	private String lastMsgRec;

	private String lastMsgSent;

	private final String VERSION = "Vzusmoro1.1.0";

	private Communicator comm;
	private TransmissionJob job;

	boolean started;
	boolean ok;

	public BTLayer(String portName, TransmissionJob job) {
		comm = new Communicator(portName);
		comm.addObserver(this);
		lastMsgSent = null;
		lastMsgRec = "";
		this.job = job;
		started = false;
	}

	public BTLayer(Communicator comm, TransmissionJob job) {
		this.comm = comm;
		comm.addObserver(this);
		lastMsgSent = null;
		lastMsgRec = "";
		this.job = job;

		started = false;
		ok = false;
	}

	@Override
	public void update(Observable o, Object arg) {

		if (arg.equals(VERSION)) {
			ok = true;
		}
		started = true;

		lastMsgRec = (String) arg + "\r";
		System.out.println("Ret:" + lastMsgRec);
		if (lastMsgSent.contentEquals(lastMsgRec)) {
			job.removeFirst();
			send();
		} else {
			if (!job.isComplete() && !job.check(lastMsgRec, lastMsgSent)) {
				send();
			}

		}

	}

	private void send() {
		try {

			if (!job.isComplete()) {
				lastMsgSent = job.getFirst() + "\r";
				System.out.println("Sending:" + lastMsgSent);
				comm.writeToStream(lastMsgSent);
			} else {
				comm.deleteObserver(this);
			}

		} catch (IOException e) {
			System.out.println("Error Sending:" + lastMsgSent);
			e.printStackTrace();
		}

	}

	public boolean start() {
		for (int i = 0; i < 5 && !ok; i++) {
			try {
				comm.writeToStream("vx\r");
				Thread.sleep(200);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				return false;
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				return false;
			}

		}
		if (ok) {
			send();
		}
		return ok;
	}

}
