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
import java.awt.Container;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.JComboBox;
import javax.swing.JPanel;
import javax.swing.JTextField;

import model.bool.Variable;
import model.bool.Variable.Operator;

public class SensorPanel extends JPanel implements ActionListener {

	private static final long serialVersionUID = 1465749684105013561L;

	private JComboBox<Operator> opSelect;
	private JTextField compValue;

	public SensorPanel() {
		super();
		setBorder(BorderFactory.createLineBorder(Color.BLACK));
		opSelect = new JComboBox<Operator>(Variable.Operator.values());
		opSelect.setPreferredSize(new Dimension(50, 27));
		compValue = new JTextField(3);
		compValue.addActionListener(this);
		add(opSelect);
		add(compValue);
		compValue.requestFocus();
	}

	public int getCompValue() {
		return Integer.parseInt(compValue.getText());
	}

	public void setCompValue(int compValue) throws NumberFormatException {
		this.compValue.setText(Integer.toString(compValue));
	}

	public Operator getOperator() {
		return (Operator) opSelect.getSelectedItem();
	}

	public void setOperator(Operator op) {
		opSelect.setSelectedItem(op);
	}

	public String getLabel() {
		return opSelect.getSelectedItem().toString() + " " + compValue.getText();
	}

	public void actionPerformed(ActionEvent e) {
		Container parent = this.getParent();
		parent.remove(this);
		parent.validate();
		parent.repaint();
	}
}
