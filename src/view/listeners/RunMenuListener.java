package view.listeners;

import java.awt.Component;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.TreeMap;

import javax.swing.JCheckBox;
import javax.swing.JOptionPane;
import view.MainFrame;

public class RunMenuListener implements ActionListener {

	private interface Method {
		void doEvent(ActionEvent e);
	}

	private Method connect = new Method() {
		public void doEvent(ActionEvent e) {
			boolean connected = MainFrame.robot.connect(null);
			MainFrame.toolPanel.setConnected(connected);
		}
	};

	private Method debug = new Method() {// TODO auslagern, Für beep smach
											// viewer starten
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
			MainFrame.robot.disconnect();
		};
	};

	TreeMap<String, Method> methods;

	private Method run = new Method() {
		public void doEvent(ActionEvent e) {
			MainFrame.robot.play();
		}
	};

	private Method stop = new Method() {
		public void doEvent(ActionEvent e) {
			MainFrame.robot.stop();
		};
	};

	private Method transmit = new Method() {
		public void doEvent(ActionEvent e) {
			boolean transmitted = MainFrame.robot.transmit();
			MainFrame.toolPanel.enableStart(transmitted);
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
		Method m = methods.get(e.getActionCommand());// TODO wieder
														// einkommentieren
		try {
			m.doEvent(e);
		} catch (NullPointerException ex) {
			System.out.println("ActionCommand nicht gefunden:"
					+ e.getActionCommand());
			ex.printStackTrace();

		}

	}

}
