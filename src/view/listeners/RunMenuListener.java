package view.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.TreeMap;
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
			MainFrame.robot.debug();			
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
			MainFrame.toolPanel.enableStop(true);
			MainFrame.toolPanel.enableDebug(true);
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
		Method m = methods.get(e.getActionCommand());
		try {
			m.doEvent(e);
		} catch (NullPointerException ex) {
			System.out.println("ActionCommand nicht gefunden:"
					+ e.getActionCommand());
			ex.printStackTrace();

		}

	}

}
