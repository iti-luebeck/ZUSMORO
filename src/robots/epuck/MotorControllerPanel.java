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

import java.awt.Component;
import java.awt.event.MouseEvent;

import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JSpinner;

import model.ActionController;

/**
 * @author ida
 * 
 *         Panel to set the parameters for the motor's p-controller
 */
public class MotorControllerPanel {

	private JSpinner offsetSpinner;
	private JSpinner minSpinner;
	private JSpinner maxSpinner;
	private JSpinner kprSpinner;
	private JLabel offset;
	private JLabel min;
	private JLabel max;
	private JLabel kpr;
	private String[] sensors = { "IR0", "IR1", "IR2", "IR3", "IR4", "IR5",
			"IR6", "IR7", "UIR0", "UIR1", "UIR2" };
	private JLabel sensor1;
	private JComboBox<String> sensor1Box;
	private JCheckBox enableSensor2;
	private JLabel sensor2;
	private JComboBox<String> sensor2Box;
	private JLabel cV;
	private JSpinner cVSpinner;
	private JLabel formel1;
	private JLabel formel2;
	private JLabel formel3;
	private JCheckBox enabled;
	private ActionController ac;

	/**
	 * Constructor for the MotorControllerPanel, generating all the data fields
	 * and descriptions
	 */
	public MotorControllerPanel() {
		offset = new JLabel("Offset");
		offsetSpinner = new JSpinner();

		min = new JLabel("Minimum");
		minSpinner = new JSpinner();

		max = new JLabel("Maxmimum");
		maxSpinner = new JSpinner();

		kpr = new JLabel("Kpr");
		kprSpinner = new JSpinner();

		sensor1 = new JLabel("Sensor1");
		sensor1Box = new JComboBox<String>(sensors);

		enableSensor2 = new JCheckBox("Sensor2 verwenden");

		sensor2 = new JLabel("Sensor2");
		sensor2Box = new JComboBox<String>(sensors);

		cV = new JLabel("Sollwert");
		cVSpinner = new JSpinner();

		formel1 = new JLabel(
				"X = (Kpr/1000) * ( Sollwert - (Sensor1() - Sensor2())");
		formel2 = new JLabel("Minimum <= X <= Maximum");
		formel3 = new JLabel("Stellgröße = Offset + X");

		enabled = new JCheckBox("Regler aktivieren");
		ac = null;
	}

	public ActionController showPanel(MouseEvent e, ActionController ac2) {
		setValues(ac2);
		Object[] params = { offset, offsetSpinner, min, minSpinner, max,
				maxSpinner, kpr, kprSpinner, sensor1, sensor1Box,
				enableSensor2, sensor2, sensor2Box, cV, cVSpinner, formel1,
				formel2, formel3, enabled };
		int n = JOptionPane.showConfirmDialog((Component) e.getSource(),
				params, "P-Regler", JOptionPane.OK_CANCEL_OPTION);

		if (!enabled.isSelected()) {
			return null;
		} else if (n == 0) {
			ac.setOffset((Integer) offsetSpinner.getValue());
			ac.setMin((Integer) minSpinner.getValue());
			ac.setMax((Integer) maxSpinner.getValue());
			ac.setKpr((Integer) kprSpinner.getValue());
			ac.setVar(enableSensor2.isSelected() ? "DIFFERRENCE_"
					+ (String) sensor1Box.getSelectedItem() + "_"
					+ (String) sensor2Box.getSelectedItem()
					: (String) sensor1Box.getSelectedItem());
			ac.setCompvalue((Integer) cVSpinner.getValue());
		}
		return ac;

	}

	private void setValues(ActionController ac) {
		this.ac = ac;
		offsetSpinner.setValue(ac.getOffset());
		maxSpinner.setValue(ac.getMax());
		minSpinner.setValue(ac.getMin());
		kprSpinner.setValue(ac.getKpr());
		cVSpinner.setValue(ac.getCompvalue());
		enabled.setSelected(true);

		if (ac.getVar() != null) {
			if (ac.getVar().startsWith("DIFFERENCE")) {
				enableSensor2.setSelected(true);
				String[] split = ac.getVar().split("_");
				sensor1Box.setSelectedItem(split[1]);
				sensor1Box.setSelectedItem(split[2]);
			} else {
				sensor1Box.setSelectedItem(ac.getVar());
			}
		}

	}

}
