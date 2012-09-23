package model.onboard;

import java.io.IOException;
import java.util.Observable;
import java.util.Observer;

import javax.swing.SwingUtilities;

import model.Automat;
import robots.epuck.Communicator;
import robots.epuck.EPuckSensorI;
import view.MainFrame;

public class OnBoardEpuck implements Observer, EPuckSensorI {

	private Communicator comm;
	private boolean connected;

	private BTLayer btl;
	private TransmitAutomat ta;

	private int debugLevel;

	private int[] sensordata;
	private int[] motorData;
	private int[] ledData;

	private int activeState;

	/*
	 * private boolean debugSensorData; private boolean debugOnState; private
	 * boolean debugPeriodicState;
	 */

	private boolean memoryError;
	private OnBoardEPuckDebugView debugView;
	private boolean transmitted;

	public OnBoardEpuck() {
		debugLevel = 0;
		/*
		 * debugOnState = false; debugPeriodicState = false; debugSensorData =
		 * false;
		 */
		btl = null;
		ta = null;
		connected = false;
		comm = null;
		memoryError = false;
		sensordata = new int[13];
		ledData = new int[8];
		motorData = new int[2];
		activeState = -1;
	}

	public int getDebugLevel() {
		return debugLevel;
	}

	public void setDebugLevel(boolean debugOnState, boolean debugPeriodicState,
			boolean debugSensorData) {
		debugLevel = 0;
		if (debugSensorData) {
			debugLevel += 4;
		}
		if (debugPeriodicState) {
			debugLevel += 2;
		}
		if (debugOnState) {
			debugLevel += 1;
		}
		if (transmissionIsComplete()) {
			try {
				comm.writeToStream("d" + debugLevel + "\r");
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	public boolean transmit(Automat a) {
		if (!connected) {
			return false;
		} else {
			transmitted = false;
			ta = new TransmitAutomat(a);
			clearBTLayer(btl);
			btl = new BTLayer(comm, ta);
			boolean ok = btl.start();
			memoryError = false;
			return ok;
		}
	}

	private void clearBTLayer(BTLayer btl) {
		if (btl != null) {
			btl.deattachObserver();
		}
	}

	public boolean start() {
		if (connected && transmissionIsComplete()) {
			try {
				comm.writeToStream("Sx\r");
				MainFrame.toolPanel.enableStop(true);
				MainFrame.toolPanel.enableStart(false);
				MainFrame.toolPanel.enableEditing(false);
				MainFrame.toolPanel.enableTransmit(false);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return true;
		} else {
			return false;
		}

	}

	public boolean hasMemoryError() {
		return memoryError;
	}

	public void stop() {
		if (connected) {
			try {
				comm.writeToStream("d0\r");
				comm.writeToStream("sx\r");
				MainFrame.toolPanel.enableStop(false);
				MainFrame.toolPanel.enableStart(true);
				MainFrame.toolPanel.enableEditing(true);
				MainFrame.toolPanel.enableTransmit(true);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}

	/**
	 * @return connected
	 * 
	 *         Getter for connected-variable to check for a standing connection
	 */
	public boolean getConnected() {
		return this.connected;
	}

	/**
	 * @return transmitted
	 * 
	 *         Getter for the compeltion of the transmission job to check for a
	 *         complete transmission
	 */
	public boolean getTransmissionComplete() {
		return ta == null;
	}

	public void connect(String comPort) {
		comm = new Communicator(comPort);
		connected = comm.isConnected();
		if (connected) {
			comm.addObserver(this);
		}
	}

	public void disconnect() {
		comm.disconnect();
		connected = comm.isConnected();
		if (!connected) {
			comm.deleteObserver(this);
		}
	}

	public boolean isConnected() {
		return connected;
	}

	public double completionStatus() {
		return (ta == null ? 0 : ta.completionStatus());
	}

	public boolean transmissionIsComplete() {
		return ta != null && ta.isComplete();
	}

	@Override
	public void update(Observable o, Object arg) {
		String s = (String) arg;
		boolean updateView = false;
		if (s.equals("Malloc Error!\r")) {
			memoryError = true;
			System.out.println("Memory error");
		}

		if (transmitted) {
			System.out.println(System.currentTimeMillis() + ":Debug:" + s);
			if (s.startsWith("D")) {
				s = s.substring(1);
				String[] split = s.split(",");
				for (int i = 0; i < 13; i++) {
					sensordata[i] = Integer.parseInt(split[i]);
				}
				updateView = true;
			}
			if (s.startsWith("z")) {

				updateView = true;

				s = (String) arg;
				s = s.substring(1);
				s = s.replaceAll(",.*", "");
				setState(Integer.parseInt(s));

			}
			if (s.startsWith("L")) {
				s = s.replaceFirst(".*L", "");
				// s = s.replaceAll("G.*", "");
				System.out.println("Motor" + motorData[0] + "," + motorData[1]
						+ "LEDS:" + s);
				ledData = LEDSet.getLEDArray(Integer.parseInt(s));
				updateView = true;

			}
			if (s.startsWith("A")) {
				s = s.substring(1);
				setState(Integer.parseInt(s));
			}
			if (s.startsWith("M")) {
				s = s.substring(1);
				String[] split = s.split(",");
				updateView = true;
				motorData[0] = Integer.parseInt(split[0]);
				motorData[1] = Integer.parseInt(split[1]);

			}

			if (updateView && Automat.runningAutomat != null) {
				if (debugView != null) {
					debugView.updateView();
				} else {
					debugView = new OnBoardEPuckDebugView(this);
					SwingUtilities.invokeLater(new Runnable() {
						public void run() {
							debugView.setVisible(true);
						}
					});
					debugView.updateView();
				}
			}
		} else {
			MainFrame.statusBar
					.setInfoText(hasMemoryError() ? "Fehler bei Übertragung: Der Automat ist zu groß. Nicht genügend Speicher auf dem EPuck."
							: "Übertragung bei " + completionStatus() * 100
									+ "%");
			transmitted = transmissionIsComplete();
		}
	}

	private void setState(int state) {
		if (state != activeState && state >= 0) {
			Automat.runningAutomat.setActiveState(Automat.runningAutomat
					.getStates().get(state));
		}
	}

	@Override
	public int[] getFloorIr() {
		int[] res = { sensordata[8], sensordata[9], sensordata[10] };
		return res;
	}

	@Override
	public int[] getIrDistances() {
		int[] res = { sensordata[0], sensordata[1], sensordata[2],
				sensordata[3], sensordata[4], sensordata[5], sensordata[6],
				sensordata[7] };
		return res;
	}

	public int getTimerState() {
		return 30000 * sensordata[12] + sensordata[11];
	}

	@Override
	public int[] getLedState() {
		return ledData;
	}

	@Override
	public int[] getMotorState() {
		return motorData;
	}

}
