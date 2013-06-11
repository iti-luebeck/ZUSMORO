package view.listeners;

import java.awt.Component;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.rmi.AlreadyBoundException;
import java.util.TreeMap;

import javax.naming.directory.NoSuchAttributeException;
import javax.swing.JCheckBox;
import javax.swing.JOptionPane;
import robots.beep.BeepDevice;
import robots.beep.BeepRobot;
import smachGenerator.SmachableActuators;
import smachGenerator.SmachableSensors;
import smachGenerator.SmachAutomat;

import model.Automat;
import view.MainFrame;

public class RunMenuListener implements ActionListener {

	private interface Method {
		void doEvent(ActionEvent e);
	}

	private Method connect = new Method() {
		public void doEvent(ActionEvent e) {
			String comPort = JOptionPane.showInputDialog(
					(Component) e.getSource(),
					"Bitte einen COM-Port wählen (/dev/rfcommx||COMx):",
					lastInput);

			// if (comPort != null && comPort.startsWith("COM")) {
			if (comPort != null) {

				lastInput = comPort;
				MainFrame.onBoard.connect(comPort);
				MainFrame.toolPanel.setConnected(MainFrame.onBoard
						.isConnected());
			}
		}
	};

	private Method debug = new Method() {
		@Override
		public void doEvent(ActionEvent e) {
			JCheckBox debug1 = new JCheckBox(
					"Konfiguration eines neuen Zustands senden");
			JCheckBox debug2 = new JCheckBox(
					"Kontinuierlich die MotorSteuerung senden");
			JCheckBox debug3 = new JCheckBox(
					"Kontinuierlich die SensorDaten senden");
			Object[] params = { debug1, debug2, debug3 };
			int n = JOptionPane.showConfirmDialog((Component) e.getSource(),
					params, "Debug-Optionen", JOptionPane.OK_CANCEL_OPTION);
			if (n == 0) {
				MainFrame.onBoard.setDebugLevel(debug1.isSelected(),
						debug2.isSelected(), debug3.isSelected());
			}
		}
	};

	private Method disconnect = new Method() {
		public void doEvent(ActionEvent e) {
			if (Automat.runningAutomat != null) {
				JOptionPane
						.showMessageDialog(
								(Component) e.getSource(),
								"<html>Die Verbindung wird getrennt."
										+ "<br>Danach werden keine Debug-Ausgaben empfangen."
										+ "<br>Der EPuck wird ggf. weiter aktiv sein.",
								"Verbindung wird getrennt!",
								JOptionPane.WARNING_MESSAGE);
			}
			MainFrame.onBoard.disconnect();
			MainFrame.toolPanel.setConnected(false);
		};
	};

	private String lastInput = "/dev/rfcomm1";

	TreeMap<String, Method> methods;

	private Method run = new Method() {
		public void doEvent(ActionEvent e) {
			double status = MainFrame.onBoard.completionStatus();
			if (!MainFrame.onBoard.transmissionIsComplete()) {
				int i = (int) (status * 100);
				JOptionPane.showMessageDialog((Component) e.getSource(),
						"<html>Die Übertragung ist noch nicht abgechlossen."
								+ "<br>Übertragung bei " + i + "%",
						"Der EPuck kann noch nicht gestartet werden!",
						JOptionPane.WARNING_MESSAGE);
			} else if (MainFrame.onBoard.hasMemoryError()) {
				JOptionPane.showMessageDialog((Component) e.getSource(),
						"<html>Die Übertragung ist fehlgeschlagen."
								+ "<br>Der Automat ist zu groß um in den"
								+ "<br>Speicher aufgenommen zu werden",
						"Der EPuck sollte nicht gestartet werden!",
						JOptionPane.WARNING_MESSAGE);
			} else {
				if (MainFrame.onBoard.start()) {
					Automat.runningAutomat = MainFrame.automat;
				} else {
					JOptionPane
							.showMessageDialog(
									(Component) e.getSource(),
									"<html>Die Übertragung kann nicht gestartet werden"
											+ "<br>Mögliche Ursachen:"
											+ "<br>Die Verbindung ist noch nicht aufgebaut"
											+ "<br>Die Übertragung ist noch nicht abgeschlossen",
									"Der EPuck kann nicht gestartet werden!",
									JOptionPane.WARNING_MESSAGE);
				}
			}
		}
	};

	private Method stop = new Method() {
		public void doEvent(ActionEvent e) {
			if (Automat.runningAutomat != null) {
				MainFrame.onBoard.stop();
				Automat.runningAutomat = null;
			}
		};
	};

	private Method transmit = new Method() {
		public void doEvent(ActionEvent e) {
			boolean ok = MainFrame.onBoard.transmit(MainFrame.automat);
			if (!ok) {
				JOptionPane
						.showMessageDialog(
								(Component) e.getSource(),
								"<html>Die Übertragung ist fehlgeschlagen."
										+ "<br>Dies kann bedeuten, dass auf dem EPuck die"
										+ "<br>falsche Software installiert oder"
										+ "<br>der EPuck nicht verbunden ist.",
								"Übertragung fehlgeschlagen!",
								JOptionPane.WARNING_MESSAGE);
			}
		}
	};

	public RunMenuListener() {
		methods = new TreeMap<String, Method>();
		methods.put("run", run);
		methods.put("stop", stop);
		methods.put("connect", connect);
		methods.put("disconnect", disconnect);
		methods.put("transmit", transmit);
		methods.put("debug", debug);
	}

	public void actionPerformed(ActionEvent e) {
		if (e.getActionCommand().equals("connect")) {
			if (MainFrame.automat.checkNames()) {
				SmachableSensors sensors = new SmachableSensors();
				sensors.add(new BeepDevice("IR0", "topic/IR0", "Int8",
						"std_msgs.msg", "data"));
				sensors.add(new BeepDevice("IR1", "topic/IR1", "Int8",
						"std_msgs.msg", "data"));
				sensors.add(new BeepDevice("IR2", "topic/IR2", "Int8",
						"std_msgs.msg", "data"));
				sensors.add(new BeepDevice("IR3", "topic/IR3", "Int8",
						"std_msgs.msg", "data"));
				sensors.add(new BeepDevice("imu_x", "topic/imu", "Imu",
						"sensor_msgs.msg", "linear_acceleration.x"));
				sensors.add(new BeepDevice("imu_y", "topic/imu", "Imu",
						"sensor_msgs.msg", "linear_acceleration.y"));
				sensors.add(new BeepDevice("imu_z", "topic/imu", "Imu",
						"sensor_msgs.msg", "linear_acceleration.z"));
				SmachableActuators actuators = new SmachableActuators();
				actuators.add(new BeepDevice("MOTOR1", "topic/motors",
						"Motors", "beep.msg", "links"));
				actuators.add(new BeepDevice("MOTOR2", "topic/motors",
						"Motors", "beep.msg", "rechts"));
				actuators.add(new BeepDevice("LED1", "topic/LED1", "Int8",
						"std_msgs.msg", "data"));
				actuators.add(new BeepDevice("LED2", "topic/LED2", "Int8",
						"std_msgs.msg", "data"));
				actuators.add(new BeepDevice("LED3", "topic/LED3", "Int8",
						"std_msgs.msg", "data"));
				actuators.add(new BeepDevice("LED4", "topic/LED4", "Int8",
						"std_msgs.msg", "data"));
				actuators.add(new BeepDevice("LED5", "topic/LED5", "Int8",
						"std_msgs.msg", "data"));
				actuators.add(new BeepDevice("LED6", "topic/LED6", "Int8",
						"std_msgs.msg", "data"));
				actuators.add(new BeepDevice("LED7", "topic/LED7", "Int8",
						"std_msgs.msg", "data"));
				actuators.add(new BeepDevice("LED0", "topic/LED0", "Int8",
						"std_msgs.msg", "data"));
				BeepDevice beep = new BeepDevice("BEEP", "topic/beep", "Int8",
						"std_msgs.msg", "data");
				actuators.add(beep);

				// XML datei erstellen
				BeepRobot rob = new BeepRobot();
				File file = new File("Robot.xml");
				//BeepRobot.saveBeepRobot(rob, file);
				BeepRobot xmlRobot = BeepRobot.loadBeepRobot("Robot.xml");
				
				SmachAutomat sa;			
				
				try {
					sa = new SmachAutomat(MainFrame.automat.getStates(),
							sensors, actuators);
					sa.saveToFile("test");
				} catch (NoSuchAttributeException | AlreadyBoundException e1) {
					JOptionPane.showMessageDialog(MainFrame.mainFrame, e1,
							"Automat kann nicht ausgeführt werden!",
							JOptionPane.WARNING_MESSAGE);
				}
			}
		}
		Method m = methods.get(e.getActionCommand());// TODO wieder
														// einkommentieren
		// try {
		// m.doEvent(e);
		// } catch (NullPointerException ex) {
		// System.out.println("ActionCommand nicht gefunden:"
		// + e.getActionCommand());
		// ex.printStackTrace();
		//
		// }

	}

	

}
