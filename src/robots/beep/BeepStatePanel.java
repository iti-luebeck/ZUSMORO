/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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

	//private JCheckBox beeper;// TODO mp3-Auswahl erstellen

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
		//add(this.beeper);
		setPreferredSize(new Dimension(390, 400));
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
		for (int i = 0; i < 8; i++) {
			CirclePanel led = new CirclePanel();
			led.addMouseListener(ledListener);
			leds.add(led);
			add(led);
		}

		motor1slider = new JSlider(SwingConstants.VERTICAL, -128, 127, 0);
		motor2slider = new JSlider(SwingConstants.VERTICAL, -128, 127, 0);
		motor1slider.addChangeListener(this);
		motor2slider.addChangeListener(this);
		motor1slider.setOpaque(false);
		motor2slider.setOpaque(false);

		motor1spinner = new JSpinner(new SpinnerNumberModel(0, -128, 127, 1));
		motor2spinner = new JSpinner(new SpinnerNumberModel(0, -128, 127, 1));
		motor1spinner.addChangeListener(this);
		motor2spinner.addChangeListener(this);

		//this.beeper = new JCheckBox();
	}

	private void setValues() {
		ArrayList<Action> actions = state.getActions();
		for (Action action : actions) {
			if (action.getActuatorName().startsWith("LED")) {
				leds.get(Integer.parseInt(action.getActuatorName().substring(3)))
						.setBackground(new Color(action.getValue()));
			} else if (action.getActuatorName().equals("MOTOR1CONTROLLER")) {
				motor1slider.setValue(0);
				motor1spinner.setValue(0);
				motor1slider.setEnabled(false);
				motor1spinner.setEnabled(false);
			} else if (action.getActuatorName().equals("MOTOR2CONTROLLER")) {
				motor2slider.setValue(0);
				motor2spinner.setValue(0);
				motor2slider.setEnabled(false);
				motor2spinner.setEnabled(false);
			} else if (action.getActuatorName().equals("MOTOR1")) {
				motor1slider.setValue(action.getValue());
			} else if (action.getActuatorName().equals("MOTOR2")) {
				motor2slider.setValue(action.getValue());
//			} else if (action.getActuatorName().equals("BEEP")) {
//				beeper.setSelected((action.getValue() == 1));
			}
		}
	}

	private void setComponentBounds() {
		int ledRadius = 15;

		try {
			// Set position of the LEDs on the Robot
			Iterator<CirclePanel> led = leds.iterator();
			
			led.next().setBounds(377, 238, ledRadius, ledRadius);
			led.next().setBounds(246, 374, ledRadius, ledRadius);
			led.next().setBounds(120, 370, ledRadius, ledRadius);
			led.next().setBounds(8, 238, ledRadius, ledRadius);
			led.next().setBounds(8, 145, ledRadius, ledRadius);
			led.next().setBounds(135, 10, ledRadius, ledRadius);
			led.next().setBounds(258, 17, ledRadius, ledRadius);
			led.next().setBounds(375, 145, ledRadius, ledRadius);
			if (led.hasNext())
				System.err
						.println("Do not have enough led-positions for led-amount. Check setComponentBounds in BeepStatePanel!");
		} catch (Exception e) {
			System.err
					.println("Tried to set to many led-positions. Check setComponentBounds in BeepStatePanel!");
		}

		this.motor1slider.setBounds(48, 112, 20, 180);
		this.motor2slider.setBounds(328, 112, 20, 180);

		Dimension prefSize = this.motor1spinner.getPreferredSize();
		this.motor1spinner.setBounds(75, 187, 70, prefSize.height);
		this.motor2spinner.setBounds(255, 187, 70, prefSize.height);

//		prefSize = this.beeper.getPreferredSize();
//		this.beeper.setBounds(191, 265, prefSize.width, prefSize.height);
	}

	@Override
	public ArrayList<Action> getActions() {
		ArrayList<Action> actions = new ArrayList<Action>(11);
		for (int i = 0; i < leds.size(); i++) {
			if (leds.get(i).getBackground() != null) {
				actions.add(new Action("LED" + i, leds.get(i).getBackground()
						.getRGB()));
			}
		}

		Action left = null;
		Action right = null;
		for (Action a : state.getActions()) {
			if (a.getActuatorName().equals("MOTOR1CONTROLLER")) {
				left = a;
			}
			if (a.getActuatorName().equals("MOTOR2CONTROLLER")) {
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
//		actions.add(new Action("BEEP", convertBool(beeper.isSelected())));
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
