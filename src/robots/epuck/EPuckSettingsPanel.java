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

import java.awt.GridLayout;

import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;

import view.MainFrame;
import model.AbstractSettingPanel;
import model.Automat;

public class EPuckSettingsPanel extends AbstractSettingPanel {

	private JTextField evaFreq;
	private JCheckBox allowLoops;
	private JCheckBox allowTransSeq;
	private JCheckBox debugging;
	
	public EPuckSettingsPanel(EPuckRobot robot) {
		this.setLayout(new GridLayout(4, 2));

		
		this.add(new JLabel(" EVA Frequenz (Hz): "));
		evaFreq = new JTextField("" + 1000 / Automat.progDelay, 5);
		evaFreq.setHorizontalAlignment(JTextField.RIGHT);
		this.add(evaFreq);
		
		this.add(new JLabel(" Schlaufen: "));
		allowLoops = new JCheckBox("erlauben", Automat.loopsAllowed);
		this.add(allowLoops);

		this.add(new JLabel(" Transitionenreihenfolge: "));
		allowTransSeq = new JCheckBox("einstellbar", Automat.changeableTransSeq);
		this.add(allowTransSeq);

		this.add(new JLabel(" Debugausgaben: "));
		debugging = new JCheckBox("ausgeben (auf Konsole)", MainFrame.DEBUG);
		this.add(debugging);
	}

	@Override
	public boolean setSettings() {
		try {
			// Frequenz
			double freq = Double.parseDouble(evaFreq.getText());
			int delay = (int) (1000.0 / freq);
			if (delay <= 0 || delay >= 60000) {
				throw new Exception("Frequenz nicht im zulässigen Bereich.");
			}
			// Schlaufen
			boolean allow = allowLoops.isSelected();
			// Transitionsreihenfolge
			boolean transSeq = allowTransSeq.isSelected();
			// Debugausgaben auf Kosnsole
			boolean debug = debugging.isSelected();

			// Set values:
			Automat.progDelay = delay;
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
