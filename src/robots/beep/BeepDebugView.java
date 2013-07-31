package robots.beep;

import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Iterator;
import java.util.LinkedList;

import javax.swing.JDialog;
import javax.swing.JSlider;
import javax.swing.JSpinner;
import javax.swing.SwingConstants;
import robots.epuck.MotorModel;
import robots.epuck.MotorSpinnerModel;
import smachGenerator.ISmachableActuator;
import view.MainFrame;

public class BeepDebugView extends JDialog implements ActionListener {

	private static final long serialVersionUID = 5138791031355097563L;

	private BeepRobot robot;
	private LinkedList<CirclePanel> leds;

	private JSlider motor1slider;
	private JSlider motor2slider;

	private JSpinner motor1spinner;
	private JSpinner motor2spinner;

	protected BeepDebugBackgroundPanel panel;

	public BeepDebugView(BeepRobot robot) {
		super(MainFrame.mainFrame, "Debugansicht", false);
		setDefaultCloseOperation(HIDE_ON_CLOSE);
		this.robot = robot;
		panel = new BeepDebugBackgroundPanel(
				BeepDebugBackgroundPanel.debug_background);
		panel.setLayout(null);
		initComponents();
		setComponentBounds();

		panel.add(motor1slider);
		panel.add(motor2slider);
		panel.add(motor1spinner);
		panel.add(motor2spinner);

		add(panel);
		pack();
	}

	private void setComponentBounds() {
		int ledRadius = 15;

		try {
			// Set position of the LEDs on the Robot
			Iterator<CirclePanel> led = leds.iterator();
			led.next().setBounds(192, 34, ledRadius, ledRadius);
			led.next().setBounds(328, 111, ledRadius, ledRadius);
			led.next().setBounds(351, 193, ledRadius, ledRadius);
			led.next().setBounds(304, 305, ledRadius, ledRadius);
			led.next().setBounds(164, 351, ledRadius, ledRadius);
			led.next().setBounds(220, 351, ledRadius, ledRadius);
			led.next().setBounds(78, 306, ledRadius, ledRadius);
			led.next().setBounds(31, 193, ledRadius, ledRadius);

			if (led.hasNext())
				System.err
						.println("Do not have enough led-positions for led-amount. Check setComponentBounds in BeepDebugView!");
		} catch (Exception e) {
			System.err
					.println("Tried to set to many led-positions. Check setComponentBounds in BeepDebugView!");
		}

		motor1slider.setBounds(65, 138, 20, 127);
		motor2slider.setBounds(315, 138, 20, 127);

		Dimension prefSize = motor1spinner.getPreferredSize();

		motor1spinner.setBounds(95, 165, 70, prefSize.height);
		motor2spinner.setBounds(224, 165, 70, prefSize.height);
	}

	private void initComponents() {
		leds = new LinkedList<>();
		for (ISmachableActuator actuator : robot.getActuators()) {
			if (actuator.getName().startsWith("LED")) {
				CirclePanel led = new CirclePanel();
				// led.addMouseListener(ledListener);
				leds.add(led);
				add(led);
			}

		}

		MotorModel mm1 = new MotorModel(-1000, 1000, 0);
		MotorSpinnerModel msm1 = new MotorSpinnerModel(mm1);
		MotorModel mm2 = new MotorModel(-1000, 1000, 0);
		MotorSpinnerModel msm2 = new MotorSpinnerModel(mm2);

		motor1slider = initMotorSlider(mm1);
		motor2slider = initMotorSlider(mm2);
		//
		motor1spinner = new JSpinner(msm1);
		motor2spinner = new JSpinner(msm2);
		motor1spinner.setEnabled(false);
		motor2spinner.setEnabled(false);
	}

	private JSlider initMotorSlider(MotorModel mm) {
		JSlider motorslider = new JSlider(mm);
		motorslider.setOrientation(SwingConstants.VERTICAL);
		motorslider.setOpaque(false);
		motorslider.setEnabled(false);
		return motorslider;
	}

	public void updateView() {
		for (ISmachableActuator actuator : robot.getActuators()) {
			if (actuator.getName().startsWith("LED")) {
				// TODO welche Farbe soll gesetzt werden
				// leds.get(Integer.parseInt(actuator.getName().substring(3))).setBackground(Color.BLACK);
			}//TODO was ist mit motoren und anderen Aktuatoren
		}
		// motor1slider.setValue(robot.getMotorState()[0]);
		// motor2slider.setValue(robot.getMotorState()[1]);
		panel.update(robot);
		repaint();
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		if (isVisible()){
			updateView();
		}else{
			robot.rosComm.shutdown();
		}
	}
}
