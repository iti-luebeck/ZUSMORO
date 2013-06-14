package robots.beep;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;

import javax.swing.ImageIcon;
import javax.swing.JCheckBox;
import javax.swing.JColorChooser;
import javax.swing.JSlider;
import javax.swing.JSpinner;
import javax.swing.SpinnerNumberModel;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import model.Action;
import model.State;
import robots.beep.CirclePanel;
import view.AbstractStatePanel;

public class BeepStatePanel extends AbstractStatePanel implements
		ChangeListener {

	private static final long serialVersionUID = -278302991810159232L;
	private final Image state_background = new ImageIcon(
			BeepStatePanel.class.getResource("beep-background.png")).getImage();

	private LinkedList<CirclePanel> leds = new LinkedList<>();
	CirclePanel c = new CirclePanel();

	private JSlider motor1slider;
	private JSlider motor2slider;

	private JSpinner motor1spinner;
	private JSpinner motor2spinner;

	private JCheckBox beeper;// TODO mp3-Auswahl erstellen

	private State state;

	public BeepStatePanel(State state) {
		this.state = state;
		setLayout(null);
		initComponents();
		setValues();
		setComponentBounds();

		add(this.motor1slider);
		add(this.motor2slider);
		add(this.motor1spinner);
		add(this.motor2spinner);
		add(this.beeper);
		setPreferredSize(new Dimension(400, 400));
	}

	private void initComponents() {
		MouseListener ledListener = new MouseListener() {

			@Override
			public void mouseReleased(MouseEvent e) {
			}

			@Override
			public void mousePressed(MouseEvent e) {
			}

			@Override
			public void mouseExited(MouseEvent e) {
			}

			@Override
			public void mouseEntered(MouseEvent e) {
			}

			@Override
			public void mouseClicked(MouseEvent e) {
				Color color = e.getComponent().getBackground();
				color = JColorChooser.showDialog(BeepStatePanel.this,
						"Wähle eine Farbe", color);
				e.getComponent().setBackground(color);
			}
		};

		// add all LEDs
		for (int i = 0; i < 9; i++) {
			CirclePanel led = new CirclePanel();
			led.addMouseListener(ledListener);
			leds.add(led);
			add(led);
		}

		motor1slider = new JSlider(SwingConstants.VERTICAL, -1000, 1000, 0);
		motor2slider = new JSlider(SwingConstants.VERTICAL, -1000, 1000, 0);
		motor1slider.addChangeListener(this);
		motor2slider.addChangeListener(this);
		motor1slider.setOpaque(false);
		motor2slider.setOpaque(false);

		motor1spinner = new JSpinner(new SpinnerNumberModel(0, -1000, 1000, 1));
		motor2spinner = new JSpinner(new SpinnerNumberModel(0, -1000, 1000, 1));
		motor1spinner.addChangeListener(this);
		motor2spinner.addChangeListener(this);

		this.beeper = new JCheckBox();
	}

	private void setValues() {
		ArrayList<Action> actions = state.getActions();
		for (Action action : actions) {
			if (action.getKey().startsWith("LED")) {
				leds.get(Integer.parseInt(action.getKey().substring(3)))
						.setBackground(new Color(action.getValue()));
			} else if (action.getKey().equals("MOTOR1CONTROLLER")) {
				motor1slider.setValue(0);
				motor1spinner.setValue(0);
				motor1slider.setEnabled(false);
				motor1spinner.setEnabled(false);
			} else if (action.getKey().equals("MOTOR2CONTROLLER")) {
				motor2slider.setValue(0);
				motor2spinner.setValue(0);
				motor2slider.setEnabled(false);
				motor2spinner.setEnabled(false);
			} else if (action.getKey().equals("MOTOR1")) {
				motor1slider.setValue(action.getValue());
			} else if (action.getKey().equals("MOTOR2")) {
				motor2slider.setValue(action.getValue());
			} else if (action.getKey().equals("BEEP")) {
				beeper.setSelected((action.getValue() == 1));
			}
		}
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
			led.next().setBounds(53, 113, ledRadius, ledRadius);

			if (led.hasNext())
				System.err
						.println("Do not have enough led-positions for led-amount. Check setComponentBounds in BeepStatePanel!");
		} catch (Exception e) {
			System.err
					.println("Tried to set to many led-positions. Check setComponentBounds in BeepStatePanel!");
		}

		this.motor1slider.setBounds(65, 138, 20, 127);
		this.motor2slider.setBounds(315, 138, 20, 127);

		Dimension prefSize = this.motor1spinner.getPreferredSize();
		this.motor1spinner.setBounds(95, 165, 70, prefSize.height);
		this.motor2spinner.setBounds(224, 165, 70, prefSize.height);

		prefSize = this.beeper.getPreferredSize();
		this.beeper.setBounds(191, 265, prefSize.width, prefSize.height);
	}

	@Override
	public ArrayList<Action> getActions() {
		ArrayList<Action> actions = new ArrayList<Action>(11);
		for (int i = 0; i < 9; i++) {
			actions.add(new Action("LED" + i, leds.get(i).getBackground()
					.getRGB()));
		}

		Action left = null;
		Action right = null;
		for (Action a : state.getActions()) {
			if (a.getKey().equals("MOTOR1CONTROLLER")) {
				left = a;
			}
			if (a.getKey().equals("MOTOR2CONTROLLER")) {
				right = a;
			}
		}
		if (left == null) {
			actions.add(new Action("MOTOR1", (Integer) motor1spinner.getValue()));
		} else {
			actions.add(left);
		}
		if (right == null) {
			actions.add(new Action("MOTOR2", (Integer) motor2spinner.getValue()));
		} else {
			actions.add(right);
		}
		actions.add(new Action("BEEP", convertBool(beeper.isSelected())));
		return actions;
	}

	private int convertBool(boolean bool) {
		return bool ? 1 : 0;
	}

	public void stateChanged(ChangeEvent ce) {
		if (ce.getSource() == motor1spinner) {
			motor1slider.setValue((Integer) motor1spinner.getValue());
		} else if (ce.getSource() == motor2spinner) {
			motor2slider.setValue((Integer) motor2spinner.getValue());
		} else if (ce.getSource() == motor1slider) {
			motor1spinner.setValue(motor1slider.getValue());
		} else if (ce.getSource() == motor2slider) {
			motor2spinner.setValue(motor2slider.getValue());
		}
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		g.drawImage(state_background, 0, 0, null);
	}
}
