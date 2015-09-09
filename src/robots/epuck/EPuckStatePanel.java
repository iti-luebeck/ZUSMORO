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
package robots.epuck;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.ArrayList;

import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JRadioButton;
import javax.swing.JSlider;
import javax.swing.JSpinner;
import javax.swing.SpinnerNumberModel;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import robots.beep.CirclePanel;

import model.Action;
import model.ActionController;
import model.State;
import view.AbstractStatePanel;

public class EPuckStatePanel extends AbstractStatePanel implements ChangeListener,
		MouseListener {

	private static final long serialVersionUID = -278302991810159232L;
	private final Image state_background = new ImageIcon(
			EPuckStatePanel.class.getResource("e-puck-background.png")).getImage();

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

	private JCheckBox beeper;

	private State state;

	private JButton controllerLeft;
	private JButton controllerRight;

	public EPuckStatePanel(State state) {
		this.state = state;
		setLayout(null);
		initComponents();
		setValues();
		setComponentBounds();

		add(this.led0);
		add(this.led1);
		add(this.led2);
		add(this.led3);
		add(this.led4);
		add(this.led4a);
		add(this.led5);
		add(this.led6);
		add(this.led7);
		
		add(this.motor1slider);
		add(this.motor2slider);
		add(this.motor1spinner);
		add(this.motor2spinner);
		add(this.beeper);
		add(controllerLeft);
		add(controllerRight);
		setPreferredSize(new Dimension(400, 400));
	}

	private void initComponents() {
		this.led0 = new JRadioButton();
		this.led1 = new JRadioButton();
		this.led2 = new JRadioButton();
		this.led3 = new JRadioButton();
		this.led4 = new JRadioButton();
		this.led4.addChangeListener(this);
		this.led4a = new JRadioButton();
		this.led4a.addChangeListener(this);
		this.led5 = new JRadioButton();
		this.led6 = new JRadioButton();
		this.led7 = new JRadioButton();

		this.led0.setOpaque(false);
		this.led1.setOpaque(false);
		this.led2.setOpaque(false);
		this.led3.setOpaque(false);
		this.led4.setOpaque(false);
		this.led4a.setOpaque(false);
		this.led5.setOpaque(false);
		this.led6.setOpaque(false);
		this.led7.setOpaque(false);


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
		controllerLeft = new JButton("Motor Links Regler");
		controllerRight = new JButton("Motor Rechts Regler");
		controllerLeft.addMouseListener(this);
		controllerRight.addMouseListener(this);
	}

	private void setComponentBounds() {
		Dimension prefSize = this.led0.getPreferredSize();

		this.led0.setBounds(192, 34, prefSize.width, prefSize.height);
		this.led1.setBounds(328, 111, prefSize.width, prefSize.height);
		this.led2.setBounds(351, 193, prefSize.width, prefSize.height);
		this.led3.setBounds(304, 305, prefSize.width, prefSize.height);
		this.led4.setBounds(164, 351, prefSize.width, prefSize.height);
		this.led4a.setBounds(220, 351, prefSize.width, prefSize.height);
		this.led5.setBounds(78, 306, prefSize.width, prefSize.height);
		this.led6.setBounds(31, 193, prefSize.width, prefSize.height);
		this.led7.setBounds(53, 113, prefSize.width, prefSize.height);

		this.motor1slider.setBounds(65, 138, 20, 127);
		this.motor2slider.setBounds(315, 138, 20, 127);

		prefSize = this.motor1spinner.getPreferredSize();

		this.motor1spinner.setBounds(95, 165, 70, prefSize.height);
		this.motor2spinner.setBounds(224, 165, 70, prefSize.height);

		prefSize = this.beeper.getPreferredSize();

		this.beeper.setBounds(191, 265, prefSize.width, prefSize.height);

		controllerLeft.setBounds(0, 370, 140, 20);
		controllerRight.setBounds(260, 370, 140, 20);
	}

	private void setValues() {
		ArrayList<Action> actions = state.getActions();
		for (Action action : actions) {
			if (action.getKey().equals("LED0")) {
				led0.setSelected(action.getValue() == 1);
			} else if (action.getKey().equals("LED1")) {
				led1.setSelected(action.getValue() == 1);
			} else if (action.getKey().equals("LED2")) {
				led2.setSelected((action.getValue() == 1));
			} else if (action.getKey().equals("LED3")) {
				led3.setSelected((action.getValue() == 1));
			} else if (action.getKey().equals("LED4")) {
				led4.setSelected((action.getValue() == 1));
			} else if (action.getKey().equals("LED4a")) {
				led4a.setSelected((action.getValue() == 1));
			} else if (action.getKey().equals("LED5")) {
				led5.setSelected((action.getValue() == 1));
			} else if (action.getKey().equals("LED6")) {
				led6.setSelected((action.getValue() == 1));
			} else if (action.getKey().equals("LED7")) {
				led7.setSelected((action.getValue() == 1));
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

	@Override
	public ArrayList<Action> getActions() {
		ArrayList<Action> actions = new ArrayList<Action>(11);
		actions.add(new Action("LED0", convertBool(led0.isSelected())));

		actions.add(new Action("LED1", convertBool(led1.isSelected())));
		actions.add(new Action("LED2", convertBool(led2.isSelected())));
		actions.add(new Action("LED3", convertBool(led3.isSelected())));
		actions.add(new Action("LED4", convertBool(led4.isSelected())));
		// actions.add(new Action("LED4a", convertBool(led4a.isSelected())));
		actions.add(new Action("LED5", convertBool(led5.isSelected())));
		actions.add(new Action("LED6", convertBool(led6.isSelected())));
		actions.add(new Action("LED7", convertBool(led7.isSelected())));

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
		if (ce.getSource() == led4) {
			led4a.setSelected(led4.isSelected());
		} else if (ce.getSource() == led4a) {
			led4.setSelected(led4a.isSelected());
		} else if (ce.getSource() == motor1spinner) {
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

	@Override
	public void mouseClicked(MouseEvent e) {
		ActionController ac = null;
		ArrayList<Action> actions = state.getActions();
		if (e.getSource().equals(controllerLeft)) {
			ac = new ActionController("MOTOR1CONTROLLER");
		} else if (e.getSource().equals(controllerRight)) {
			ac = new ActionController("MOTOR2CONTROLLER");
		}

		int index = -1;
		if (ac != null) {
			for (int i = 0; i < actions.size(); i++) {
				Action a = actions.get(i);
				if (a.getKey().equals(ac.getKey())) {
					ac = (ActionController) a;
					index = i;
				}
			}
			MotorControllerPanel cp = new MotorControllerPanel();
			ac = cp.showPanel(e, ac);
			if (index >= 0) {
				actions.remove(index);
			}
			if (ac != null) {
				actions.add(ac);
			}
			state.setActions(actions);
		}

	}

	@Override
	public void mouseEntered(MouseEvent e) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseExited(MouseEvent e) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mousePressed(MouseEvent e) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseReleased(MouseEvent e) {
		// TODO Auto-generated method stub

	}
}
