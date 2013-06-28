package robots.epuck;

import java.awt.Dimension;

import javax.swing.JDialog;
import javax.swing.JRadioButton;
import javax.swing.JSlider;
import javax.swing.JSpinner;
import javax.swing.SwingConstants;

import view.MainFrame;

public class DebugView extends JDialog {

	private static final long serialVersionUID = 5138791031355097563L;
	private EPuckSensorI robot;

	// TODO: Hieraus ein array?
	private JRadioButton led0;
	private JRadioButton led1;
	private JRadioButton led2;
	private JRadioButton led3;
	private JRadioButton led4;
	private JRadioButton led4a;
	private JRadioButton led5;
	private JRadioButton led6;
	private JRadioButton led7;

	private JSlider motor1slider;
	private JSlider motor2slider;

	private JSpinner motor1spinner;
	private JSpinner motor2spinner;

	protected BackgroundPanel panel;

	public DebugView(EPuckSensorI robot) {
		super(MainFrame.mainFrame, "Debugansicht", false);
		setDefaultCloseOperation(JDialog.DO_NOTHING_ON_CLOSE);
		this.robot = robot;
		panel = new BackgroundPanel(BackgroundPanel.debug_background);
		panel.setLayout(null);
		initComponents();
		setComponentBounds();
		panel.add(led0);
		panel.add(led1);
		panel.add(led2);
		panel.add(led3);
		panel.add(led4);
		panel.add(led4a);
		panel.add(led5);
		panel.add(led6);
		panel.add(led7);
		panel.add(motor1slider);
		panel.add(motor2slider);
		panel.add(motor1spinner);
		panel.add(motor2spinner);

		add(panel);
		pack();


	}

	private void setComponentBounds() {
		Dimension prefSize = this.led0.getPreferredSize();

		led0.setBounds(192, 34, prefSize.width, prefSize.height);
		led1.setBounds(328, 111, prefSize.width, prefSize.height);
		led2.setBounds(351, 193, prefSize.width, prefSize.height);
		led3.setBounds(304, 305, prefSize.width, prefSize.height);
		led4.setBounds(164, 351, prefSize.width, prefSize.height);
		led4a.setBounds(220, 351, prefSize.width, prefSize.height);
		led5.setBounds(78, 306, prefSize.width, prefSize.height);
		led6.setBounds(31, 193, prefSize.width, prefSize.height);
		led7.setBounds(53, 113, prefSize.width, prefSize.height);

		motor1slider.setBounds(65, 138, 20, 127);
		motor2slider.setBounds(315, 138, 20, 127);

		prefSize = motor1spinner.getPreferredSize();

		motor1spinner.setBounds(95, 165, 70, prefSize.height);
		motor2spinner.setBounds(224, 165, 70, prefSize.height);
	}

	private void initComponents() {
		led0 = initLED();
		led1 = initLED();
		led2 = initLED();
		led3 = initLED();
		led4 = initLED();
		led4a = initLED();
		led5 = initLED();
		led6 = initLED();
		led7 = initLED();

		MotorModel mm1 = new MotorModel(-1000, 1000, 0);
		MotorSpinnerModel msm1 = new MotorSpinnerModel(mm1);
		MotorModel mm2 = new MotorModel(-1000, 1000, 0);
		MotorSpinnerModel msm2 = new MotorSpinnerModel(mm2);

		motor1slider = initMotorSlider(mm1);
		motor2slider = initMotorSlider(mm2);

		motor1spinner = new JSpinner(msm1);
		motor2spinner = new JSpinner(msm2);
		motor1spinner.setEnabled(false);
		motor2spinner.setEnabled(false);
	}

	private JRadioButton initLED() {
		JRadioButton led = new JRadioButton();
		led.setEnabled(false);
		led.setOpaque(false);
		return led;
	}

	private JSlider initMotorSlider(MotorModel mm) {
		JSlider motorslider = new JSlider(mm);
		motorslider.setOrientation(SwingConstants.VERTICAL);
		motorslider.setOpaque(false);
		motorslider.setEnabled(false);
		return motorslider;
	}

	public void updateView() {
		int[] leds = robot.getLedState();
		led0.setSelected(leds[0] == 1);
		led1.setSelected(leds[1] == 1);
		led2.setSelected(leds[2] == 1);
		led3.setSelected(leds[3] == 1);
		led4.setSelected(leds[4] == 1);
		led4a.setSelected(leds[4] == 1);
		led5.setSelected(leds[5] == 1);
		led6.setSelected(leds[6] == 1);
		led7.setSelected(leds[7] == 1);
		motor1slider.setValue(robot.getMotorState()[0]);
		motor2slider.setValue(robot.getMotorState()[1]);
		panel.update(robot);
		repaint();
	}

}
