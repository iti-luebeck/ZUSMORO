package robots.epuck;

import java.awt.Component;
import java.awt.GridLayout;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;

import model.AbstractRobot;
import model.AbstractSettingPanel;
import model.Action;
import model.Automat;
import model.State;
import model.Transition;
import smachGenerator.ISmachableAction;
import view.AbstractStatePanel;
import view.AbstractTransitionPanel;
import view.MainFrame;

public class EPuckRobot extends AbstractRobot implements Observer, EPuckSensorI {

	private Communicator commPort;
	private int[] irDistances;
	private int[] irLumity;
	private int[] microphones;
	private int[] floorIr;
	private int[] accelerations;
	private int[] ledState;
	private int[] motorState;
	private int bodyLED = 0;
	private long lastSoundPlayed = 0;
	private int unacknowledgedCmds = 0;
	private Object monitor = new Object();
	private DebugView debugView = null;
	private String lastInput = "/dev/rfcomm1";

	public EPuckRobot() {
		irDistances = new int[8];
		irLumity = new int[8];
		microphones = new int[3];
		floorIr = new int[3];
		accelerations = new int[3];
		ledState = new int[8];
		motorState = new int[2];
	}

	/**
	 * Uebersetzt die gegebenen Actions in Befehle für den e-puck und sendet
	 * diese an ihn.
	 * 
	 * @param actions
	 *            die auszuführenden Actions
	 * @throws IOException
	 */
	@Override
	public void executeActions(ArrayList<Action> actions) throws IOException {
		StringBuilder buffer = new StringBuilder(100);
		if (commPort == null) {
			throw new IOException("Programm ist zu keinem Roboter verbunden");
		}
		if (MainFrame.DEBUG) {
			System.out.println("Unbestätigte Befehle: "
					+ getDesiredAdditionalTimeout());
		}
		if (getDesiredAdditionalTimeout() > 60) {
			throw new IOException(
					"<html>Die Verbindung zum Roboter ist gestört."
							+ "<br>Mehr als 60 Befehle wurden nicht vom Roboter bestätigt!");
		}
		// Dummy Kommando senden, da erster Befehl irgendwo verschluckt wird.
		// commPort.writeToStream("v");
		// incUnAckCmds();
		int motor0Value = 0;
		int motor1Value = 0;
		int[] ledValues = new int[8];
		for (ISmachableAction action : actions) {
			String key = action.getKey();
			int value = action.getValue();
			if (key.startsWith("LED")) { // LEDS setzen
				int led = Character.getNumericValue(key.charAt(3));
				ledValues[led] = value;
				// if (ledState[led] != Integer.parseInt(value)) {
				// commPort.writeToStream("l," + led + ",2");
				// incUnAckCmds();
				// ledState[led] ^= 1;
				// }
			} else if (key.startsWith("MOTOR")) { // MOTOR Einstellungen setzen
				if (key.charAt(5) == '1') {
					motor0Value = value;
				} else {
					motor1Value = value;
				}
			} else if (key.equals("BEEP") && value == 1
					&& System.currentTimeMillis() - lastSoundPlayed > 300) { // Sound
																				// ausgeben
				lastSoundPlayed = System.currentTimeMillis();
				buffer.append("T,4\n");
				// commPort.writeToStream("t,5");
				incUnAckCmds();
			}
		}
		for (int i = 0; i < ledValues.length; i++) {
			if (ledValues[i] != ledState[i]) {
				buffer.append("L," + i + "," + ledValues[i] + "\n");
				// commPort.writeToStream("l," + i + "," + ledValues[i]);
				incUnAckCmds();
				ledState[i] = ledValues[i];
			}
		}
		if (motorState[0] != motor0Value || motorState[1] != motor1Value) {
			// Motor Einstellungen schreiben
			buffer.append("D," + motor0Value + "," + motor1Value + "\n");
			// commPort.writeToStream("d," + motor0Value + "," + motor1Value);
			incUnAckCmds();
			motorState[0] = motor0Value;
			motorState[1] = motor1Value;
		}
		if (bodyLED == 0) {
			buffer.append("B,1\n");
			// commPort.writeToStream("B,1");
			incUnAckCmds();
			bodyLED = 1;
		}
		if (buffer.length() > 0) {
			commPort.writeToStream(buffer.toString());
		}
	}

	@Override
	public int getVariableValue(String variable) {
		int result = 0;
		if (variable.startsWith("IR")) {
			result = irDistances[Character.getNumericValue(variable.charAt(2))];
		} else if (variable.startsWith("UIR")) {
			result = floorIr[Character.getNumericValue(variable.charAt(3))];
		} else if (variable.startsWith("MIC")) {
			result = microphones[Character.getNumericValue(variable.charAt(3))];
		} else if (variable.startsWith("ACC")) {
			switch (variable.charAt(3)) {
			case 'X':
				result = accelerations[0];
				break;
			case 'Y':
				result = accelerations[1];
				break;
			case 'Z':
				result = accelerations[2];
			}
		} else if (variable.startsWith("LUM")) {
			result = irLumity[Character.getNumericValue(variable.charAt(3))];
		}
		return result;
	}

	@Override
	public void updateSensors() throws IOException {
		StringBuilder buffer = new StringBuilder();
		if (commPort == null) {
			throw new IOException("Programm ist zu keinem Roboter verbunden");
		}
		if (getDesiredAdditionalTimeout() > 40) {
			throw new IOException(
					"Er reagiert nicht mehr auf die gesendeten Befehle!");
		}
		// Get acceleration
		// commPort.writeToStream("A");
		// incUnAckCmds();
		// get FloorSensors
		// commPort.writeToStream("M");
		buffer.append("M\n");
		incUnAckCmds();
		// get proximity Sensors
		// commPort.writeToStream("N");
		buffer.append("N\n");
		incUnAckCmds();
		// get Lightsensors
		// commPort.writeToStream("O");
		// incUnAckCmds();
		// get microphones
		// commPort.writeToStream("U");
		// incUnAckCmds();
		commPort.writeToStream(buffer.toString());
	}

	public void update(Observable arg0, Object arg1) {
		decUnAckCmds();
		// System.out.println(arg1);
		boolean updateView = false;
		String[] splitMsg;
		if (arg1 instanceof String && ((String) arg1).length() > 0) {
			splitMsg = ((String) arg1).split(",");
			if (splitMsg.length > 2 && splitMsg[0].length() > 0) {
				switch (splitMsg[0].charAt(0)) {
				case 'a':
					accelerations[0] = Integer.parseInt(splitMsg[1]);
					accelerations[1] = Integer.parseInt(splitMsg[2]);
					accelerations[2] = Integer.parseInt(splitMsg[3]);
					break;
				case 'm':
					floorIr[0] = Integer.parseInt(splitMsg[1]);
					floorIr[1] = Integer.parseInt(splitMsg[2]);
					floorIr[2] = Integer.parseInt(splitMsg[3]);
					updateView = true;
					break;
				case 'n':
					irDistances[0] = Integer.parseInt(splitMsg[1]);
					irDistances[1] = Integer.parseInt(splitMsg[2]);
					irDistances[2] = Integer.parseInt(splitMsg[3]);
					irDistances[3] = Integer.parseInt(splitMsg[4]);
					irDistances[4] = Integer.parseInt(splitMsg[5]);
					irDistances[5] = Integer.parseInt(splitMsg[6]);
					irDistances[6] = Integer.parseInt(splitMsg[7]);
					irDistances[7] = Integer.parseInt(splitMsg[8]);
					updateView = true;
					break;
				case 'o':
					irLumity[0] = Integer.parseInt(splitMsg[1]);
					irLumity[1] = Integer.parseInt(splitMsg[2]);
					irLumity[2] = Integer.parseInt(splitMsg[3]);
					irLumity[3] = Integer.parseInt(splitMsg[4]);
					irLumity[4] = Integer.parseInt(splitMsg[5]);
					irLumity[5] = Integer.parseInt(splitMsg[6]);
					irLumity[6] = Integer.parseInt(splitMsg[7]);
					irLumity[7] = Integer.parseInt(splitMsg[8]);
					break;
				case 'u':
					microphones[0] = Integer.parseInt(splitMsg[1]);
					microphones[1] = Integer.parseInt(splitMsg[2]);
					microphones[2] = Integer.parseInt(splitMsg[3]);
					break;
				default:
				}
			}
		}
		if (updateView && Automat.runningAutomat != null) {
			if (debugView != null) {
				debugView.updateView();
			} else {
				debugView = new DebugView(this);
				SwingUtilities.invokeLater(new Runnable() {
					public void run() {
						debugView.setVisible(true);
					}
				});
				debugView.updateView();
			}
		}
	}

	@Override
	public void stop() {
		if (Automat.runningAutomat != null) {
			MainFrame.onBoard.stop();
			Automat.runningAutomat = null;
		}
		if (MainFrame.DEBUG) {
			System.out.println("Robot stopping...");
		}
		try {
			commPort.writeToStream("S\nT,0\n");
			ledState = new int[8];
			motorState = new int[2];
			bodyLED = 0;
			synchronized (monitor) {
				unacknowledgedCmds = 0;
			}
		} catch (Exception e) {
			// komme was wolle
		}
		if (debugView != null) {
			debugView.dispose();
			debugView = null;
		}
		if (MainFrame.DEBUG) {
			System.out.println("Robot stopped!");
		}
	}

	private int incUnAckCmds() {
		synchronized (monitor) {
			unacknowledgedCmds++;
		}
		return unacknowledgedCmds;
	}

	private int decUnAckCmds() {
		synchronized (monitor) {
			if (unacknowledgedCmds > 0) {
				unacknowledgedCmds--;
			}
		}
		return unacknowledgedCmds;
	}

	@Override
	public int getDesiredAdditionalTimeout() {
		synchronized (monitor) {
			return unacknowledgedCmds;
		}
	}

	@Override
	public boolean connect(String connectTo) {
		connectTo = JOptionPane
				.showInputDialog(MainFrame.mainFrame,
						"Bitte einen COM-Port wählen (/dev/rfcommx||COMx):",
						lastInput);
		if (connectTo == null) {
			connectTo = lastInput;
		}
		if (this.commPort != null) {
			this.commPort.disconnect();
		}
		this.commPort = new Communicator(connectTo);
		this.commPort.addObserver(this);
		/*
		 * commPort.addObserver(new Observer() {
		 * 
		 * @Override public void update(Observable o, Object arg) {
		 * System.out.println("Rückgabe:"+(String)arg);
		 * 
		 * } });
		 */
		unacknowledgedCmds = 0;
		// commPort will be used in Automat.run()
		// so do not block
		// basically just test for connection
		this.commPort.disconnect();

		// if (comPort != null && comPort.startsWith("COM")) {
		if (connectTo != null) {

			lastInput = connectTo;
			MainFrame.onBoard.connect(connectTo);
		}

		return commPort.isConnected();
	}

	@Override
	public void disconnect() {
		if (MainFrame.DEBUG) {
			System.out.println("Robot disconnecting...");
		}
		if (commPort != null) {
			commPort.disconnect();
		}
		commPort = null;
		if (MainFrame.DEBUG) {
			System.out.println("Robot disconnected!");
		}

		if (Automat.runningAutomat != null) {
			MainFrame.showErrInfo("<html>Die Verbindung wird getrennt."
					+ "<br>Danach werden keine Debug-Ausgaben empfangen."
					+ "<br>Der EPuck wird ggf. weiter aktiv sein.",
					"Verbindung wird getrennt!");
		}
		MainFrame.onBoard.disconnect();
	}

	@Override
	public AbstractStatePanel getStatePanel(State state) {
		return new EPuckStatePanel(state);
	}

	@Override
	public AbstractTransitionPanel getTransitionPanel(Transition trans) {
		return new EPuckTransitionPanel(trans);
	}

	/*
	 * wird nicht benutzt public int[] getIrLumity() { return irLumity; }
	 * 
	 * public int[] getMicrophones() { return microphones; }
	 * 
	 * public int[] getAccelerations() { return accelerations; }
	 */

	/*
	 * (non-Javadoc)
	 * 
	 * @see robots.epuck.EPuckSensorI#getFloorIr()
	 */
	public int[] getFloorIr() {
		return floorIr;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see robots.epuck.EPuckSensorI#getIrDistances()
	 */
	public int[] getIrDistances() {
		return irDistances;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see robots.epuck.EPuckSensorI#getLedState()
	 */
	public int[] getLedState() {
		return ledState;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see robots.epuck.EPuckSensorI#getMotorState()
	 */
	public int[] getMotorState() {
		return motorState;
	}

	@Override
	public int getUnAcknowledgedCmds() {
		return unacknowledgedCmds;
	}

	@Override
	public String getRobotName() {
		return "ePuck Robot";
	}

	@Override
	public boolean transmit() {
		boolean ok = MainFrame.onBoard.transmit(MainFrame.automat);
		if (!ok) {
			MainFrame.showErrInfo("<html>Die Übertragung ist fehlgeschlagen."
					+ "<br>Dies kann bedeuten, dass auf dem EPuck die"
					+ "<br>falsche Software installiert oder"
					+ "<br>der EPuck nicht verbunden ist.",
					"Übertragung fehlgeschlagen!");
		}
		return ok;
	}

	@Override
	public void play() {
		double status = MainFrame.onBoard.completionStatus();
		if (!MainFrame.onBoard.transmissionIsComplete()) {
			int i = (int) (status * 100);
			MainFrame.showErrInfo(
					"<html>Die Übertragung ist noch nicht abgechlossen."
							+ "<br>Übertragung bei " + i + "%",
					"Der EPuck kann noch nicht gestartet werden!");
		} else if (MainFrame.onBoard.hasMemoryError()) {
			MainFrame.showErrInfo("<html>Die Übertragung ist fehlgeschlagen."
					+ "<br>Der Automat ist zu groß um in den"
					+ "<br>Speicher aufgenommen zu werden",
					"Der EPuck sollte nicht gestartet werden!");
		} else {
			MainFrame.automat.start();
			if (MainFrame.onBoard.start()) {
				Automat.runningAutomat = MainFrame.automat;
			} else {
				MainFrame
						.showErrInfo(
								"<html>Die Übertragung kann nicht gestartet werden"
										+ "<br>Mögliche Ursachen:"
										+ "<br>Die Verbindung ist noch nicht aufgebaut"
										+ "<br>Die Übertragung ist noch nicht abgeschlossen",
								"Der EPuck kann nicht gestartet werden!");
			}
		}

	}

	@Override
	public void debug() {
		JCheckBox debug1 = new JCheckBox(
				"Konfiguration eines neuen Zustands senden");
		JCheckBox debug2 = new JCheckBox(
				"Kontinuierlich die MotorSteuerung senden");
		JCheckBox debug3 = new JCheckBox(
				"Kontinuierlich die SensorDaten senden");
		Object[] params = { debug1, debug2, debug3 };
		int n = JOptionPane.showConfirmDialog((Component) MainFrame.mainFrame,
				params, "Debug-Optionen", JOptionPane.OK_CANCEL_OPTION);
		if (n == 0) {
			MainFrame.onBoard.setDebugLevel(debug1.isSelected(),
					debug2.isSelected(), debug3.isSelected());
		}
	}

	@Override
	public String getLastConnectedTo() {
		return lastInput;
	}

	@Override
	public AbstractSettingPanel getSettingsPanel() {
		return new EPuckSettingsPanel(this);
	}
}
