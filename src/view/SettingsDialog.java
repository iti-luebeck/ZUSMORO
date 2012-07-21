package view;

import java.awt.Container;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JTextField;

import model.AbstractRobot;
import model.Automat;

/**
 * @author ida
 * 
 *         Settings dialog for general robot configuration allowing the user to
 *         set EVA-frequency, robot type and allow loops, editable transition
 *         order and debug messages
 */
public class SettingsDialog extends JDialog implements ActionListener {

	private static final long serialVersionUID = 4333492082030324369L;
	private JTextField evaFreq;
	private JComboBox<String> robot;
	private JCheckBox allowLoops;
	private JCheckBox allowTransSeq;
	private JCheckBox debugging;
	private JButton okButton;
	private JButton cancelButton;

	/**
	 * Constructor for the settings dialog adding all necessary checkboxes,
	 * menus and buttons
	 */
	public SettingsDialog() {
		super(MainFrame.mainFrame, "Automateneinstellungen", true);
		setDefaultCloseOperation(JDialog.DISPOSE_ON_CLOSE);
		Container cPane = this.getContentPane();
		cPane.setLayout(new GridLayout(6, 2));
		addEVAFrequencyBox(cPane);
		addRobotTypeChooser(cPane);
		allowLoops = addCheckbox(cPane, "Schlaufen:", "erlauben",
				Automat.loopsAllowed);
		allowTransSeq = addCheckbox(cPane, "Transitionsreihenfolge:",
				"einstellbar", Automat.changeableTransSeq);
		debugging = addCheckbox(cPane, "Debugausgaben:",
				"ausgeben (auf Konsole)", MainFrame.DEBUG);
		okButton = makeButton(cPane, "OK");
		cancelButton = makeButton(cPane, "Abbrechen");
		pack();
		setLocationRelativeTo(MainFrame.mainFrame);
	}

	private JButton makeButton(Container cPane, String label) {
		JButton button = new JButton(label);
		button.addActionListener(this);
		cPane.add(button);
		return button;
	}

	private JCheckBox addCheckbox(Container cPane, String label, String action,
			boolean target) {
		cPane.add(new JLabel(label));
		JCheckBox checkBox = new JCheckBox(action, target);
		cPane.add(checkBox);
		return checkBox;
	}

	private void addRobotTypeChooser(Container cPane) {
		cPane.add(new JLabel(" Roboter: "));
		robot = new JComboBox<String>(getRobots());
		cPane.add(robot);
	}

	private void addEVAFrequencyBox(Container cPane) {
		cPane.add(new JLabel(" EVA Frequenz (Hz): "));
		evaFreq = new JTextField("" + 1000 / Automat.progDelay, 5);
		evaFreq.setHorizontalAlignment(JTextField.RIGHT);
		cPane.add(evaFreq);
	}

	private String[] getRobots() {
		File robotDir = new File(SettingsDialog.class.getResource("../robots")
				.getFile().replace("%20", " "));
		File[] robotDirs = robotDir.listFiles();
		String[] robotClassNames = new String[robotDirs.length];
		int foundRobots = 0;
		File[] robotFiles;
		for (File dir : robotDirs) {
			robotFiles = dir.listFiles();
			for (File file : robotFiles) {
				String name = file.getName();
				if (name.endsWith("Robot.class")) {
					robotClassNames[foundRobots] = dir.getName() + "."
							+ file.getName().substring(0, name.length() - 6);
					foundRobots++;
				}
			}
		}
		return robotClassNames;
	}

	public void actionPerformed(ActionEvent e) {
		Object source = e.getSource();
		if (source == cancelButton) {
			this.dispose();
		} else if (source == okButton) {
			if (setSettings()) {
				this.dispose();
			}
		}
	}

	@SuppressWarnings("unchecked")
	// Typsicherheit manuell sichergestellt.
	private boolean setSettings() {
		try {
			// Frequenz
			double freq = Double.parseDouble(evaFreq.getText());
			int delay = (int) (1000.0 / freq);
			if (delay <= 0 || delay >= 60000) {
				throw new Exception("Frequenz nicht im zulässigen Bereich.");
			}
			// Roboter
			String robotClassName = "robots."
					+ robot.getSelectedItem().toString();
			Class<?> robotClass = Class.forName(robotClassName);
			if (!(robotClass.newInstance() instanceof AbstractRobot)) {
				throw new ClassCastException(
						"Gewählter Roboter ist inkompatibel.");
			}
			// Schlaufen
			boolean allow = allowLoops.isSelected();
			// Transitionsreihenfolge
			boolean transSeq = allowTransSeq.isSelected();
			// Debugausgaben auf Kosnsole
			boolean debug = debugging.isSelected();
			// Set values:
			Automat.progDelay = delay;
			MainFrame.robotClass = (Class<AbstractRobot>) robotClass;
			Automat.loopsAllowed = allow;
			Automat.changeableTransSeq = transSeq;
			MainFrame.DEBUG = debug;
		} catch (Exception e) {
			JOptionPane
					.showMessageDialog(this,
							"<html>Es trat folgender Fehler auf:<br>" + e
									+ "</html>",
							"Fehler beim Setzen der Werte",
							JOptionPane.WARNING_MESSAGE);
			return false;
		}
		return true;
	}
}
