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

		led0 = new JRadioButton();
		led0.setEnabled(false);
		led1 = new JRadioButton();
		led1.setEnabled(false);
		led2 = new JRadioButton();
		led2.setEnabled(false);
		led3 = new JRadioButton();
		led3.setEnabled(false);
		led4 = new JRadioButton();
		led4.setEnabled(false);
		led4a = new JRadioButton();
		led4a.setEnabled(false);
		led5 = new JRadioButton();
		led5.setEnabled(false);
		led6 = new JRadioButton();
		led6.setEnabled(false);
		led7 = new JRadioButton();
		led7.setEnabled(false);

		led0.setOpaque(false);
		led1.setOpaque(false);
		led2.setOpaque(false);
		led3.setOpaque(false);
		led4.setOpaque(false);
		led4a.setOpaque(false);
		led5.setOpaque(false);
		led6.setOpaque(false);
		led7.setOpaque(false);

		MotorModel mm1 = new MotorModel(-1000, 1000, 0);
		MotorSpinnerModel msm1 = new MotorSpinnerModel(mm1);
		MotorModel mm2 = new MotorModel(-1000, 1000, 0);
		MotorSpinnerModel msm2 = new MotorSpinnerModel(mm2);

		motor1slider = new JSlider(mm1);
		motor2slider = new JSlider(mm2);
		motor1slider.setOrientation(SwingConstants.VERTICAL);
		motor2slider.setOrientation(SwingConstants.VERTICAL);
		motor1slider.setOpaque(false);
		motor2slider.setOpaque(false);
		motor1slider.setEnabled(false);
		motor2slider.setEnabled(false);

		motor1spinner = new JSpinner(msm1);
		motor2spinner = new JSpinner(msm2);
		motor1spinner.setEnabled(false);
		motor2spinner.setEnabled(false);
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
