/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of L�beck
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

import java.awt.Component;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.ArrayList;
import java.util.List;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

import smachGenerator.ISmachableTransition;

import model.Transition;
import model.bool.BooleanExpression;
import model.bool.HugeAnd;
import model.bool.Variable;

public class DifferencePanel extends JPanel implements MouseListener {

	private List<Variable> guards;
	private List<JButton> buttons;
	private JButton newButton;

	private String[] sensors = { "IR0", "IR1", "IR2", "IR3", "IR4", "IR5",
			"IR6", "IR7", "UIR0", "UIR1", "UIR2" };

	public DifferencePanel(Transition trans) {
		super();

		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		newButton = new JButton("Neue Sensordifferenz als Variable");
		newButton.addMouseListener(this);
		add(newButton);
		guards = new ArrayList<Variable>();
		buttons = new ArrayList<JButton>();
		if (trans.getGuard() instanceof HugeAnd) {
			for (BooleanExpression b : ((HugeAnd) trans.getGuard())
					.getOperands()) {
				if (((Variable) b).getVariableName().startsWith("DIFFERENCE")) {
					Variable v = (Variable) b;
					guards.add(v);
					String buttonText = toButtonLabel(v);
					JButton button = new JButton(buttonText);
					button.addMouseListener(this);
					buttons.add(button);
					add(button);
				}
			}
		}

	}

	private String toButtonLabel(Variable v) {
		String[] split = v.getVariableName().split("_");
		String buttonText = split[1] + "-" + split[2] + v.getOperator()
				+ v.getCompValue();
		return buttonText;
	}

	@Override
	public void mouseClicked(MouseEvent e) {
		if (e.getSource().equals(newButton)) {
			Variable v = createNewVar();
			if (v != null)
				addVar(v);
		}
		int n = buttons.indexOf(e.getSource());
		if (n >= 0) {
			Variable v = guards.get(n);
			guards.remove(n);
			remove(buttons.get(n));
			buttons.remove(n);

			String[] split = v.getVariableName().split("_");

			JComboBox sensor1 = new JComboBox(sensors);
			sensor1.setSelectedItem(split[1]);
			JComboBox sensor2 = new JComboBox(sensors);
			sensor2.setSelectedItem(split[2]);
			SensorPanel sp = new SensorPanel();
			sp.setCompValue(v.getCompValue());
			sp.setOperator(sp.getOperator());
			Object[] params = { sensor1, sensor2, sp };
			int m = JOptionPane.showConfirmDialog((Component) e.getSource(),
					params, "Differenz", JOptionPane.OK_CANCEL_OPTION);
			if (m == 0) {
				try {
					v = new Variable("DIFFERENCE_" + sensor1.getSelectedItem()
							+ "_" + sensor2.getSelectedItem(),
							sp.getOperator(), sp.getCompValue());
					addVar(v);
				} catch (Exception ex) {
					// do nothing sp.getCompValue not successful
				}

			}
		}
		revalidate();
		repaint();
	}

	public Variable createNewVar() {
		{
			JComboBox sensor1 = new JComboBox(sensors);
			JComboBox sensor2 = new JComboBox(sensors);
			SensorPanel sp = new SensorPanel();
			Object[] params = { sensor1, sensor2, sp };
			int n = JOptionPane.showConfirmDialog(this, params, "Differenz",
					JOptionPane.OK_CANCEL_OPTION);
			if (n == 0) 
			try{ 
				Variable v = new Variable("DIFFERENCE_"
						+ sensor1.getSelectedItem() + "_"
						+ sensor2.getSelectedItem(), sp.getOperator(), sp
						.getCompValue());
				return v;
			}catch(Exception e){
				//sp.getCompValue() not successful
			}
			return null;
		}
	}

	private void addVar(Variable v) {
		guards.add(v);
		JButton b = new JButton(toButtonLabel(v));
		buttons.add(b);
		b.addMouseListener(this);
		add(b);
	}

	public List<Variable> getVars() {
		return guards;
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
