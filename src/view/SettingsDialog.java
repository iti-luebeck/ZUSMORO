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
package view;

import java.awt.Container;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.util.LinkedList;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JTextField;

import sun.org.mozilla.javascript.EcmaError;

import model.AbstractRobot;
import model.Automat;

public class SettingsDialog extends JDialog implements ActionListener {

	private static final long serialVersionUID = 4333492082030324369L;
	private JTextField evaFreq;
	private JComboBox<String> robot;
	private JCheckBox allowLoops;
	private JCheckBox allowTransSeq;
	private JCheckBox debugging;
	private JButton okButton;
	private JButton cancelButton;

	public SettingsDialog() {
		super(MainFrame.mainFrame, "Automateneinstellungen", true);
		setDefaultCloseOperation(JDialog.DISPOSE_ON_CLOSE);
		Container cPane = this.getContentPane();
		cPane.setLayout(new GridLayout(6, 2));
		cPane.add(new JLabel(" EVA Frequenz (Hz): "));
		evaFreq = new JTextField("" + 1000 / Automat.progDelay, 5);
		evaFreq.setHorizontalAlignment(JTextField.RIGHT);
		cPane.add(evaFreq);
		cPane.add(new JLabel(" Roboter: "));
		robot = new JComboBox<String>();
		for (Class<AbstractRobot> rob : getRobots()) {
			try {
				robot.addItem(rob.newInstance().getRobotName());
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		cPane.add(robot);
		if (MainFrame.robot != null && MainFrame.robotClass != null) {
			try {
				robot.setSelectedItem(MainFrame.robot.getRobotName());
			} catch (Exception e) {
				System.err.println(MainFrame.robotClass.getName()
						+ " konnte nicht instanziiert werden!");
				e.printStackTrace();
			}
		}
		cPane.add(new JLabel(" Schlaufen: "));
		allowLoops = new JCheckBox("erlauben", Automat.loopsAllowed);
		cPane.add(allowLoops);
		cPane.add(new JLabel(" Transitionenreihenfolge: "));
		allowTransSeq = new JCheckBox("einstellbar", Automat.changeableTransSeq);
		cPane.add(allowTransSeq);
		cPane.add(new JLabel(" Debugausgaben: "));
		debugging = new JCheckBox("ausgeben (auf Konsole)", MainFrame.DEBUG);
		cPane.add(debugging);
		okButton = new JButton("OK");
		okButton.addActionListener(this);
		cancelButton = new JButton("Abbrechen");
		cancelButton.addActionListener(this);
		cPane.add(okButton);
		cPane.add(cancelButton);
		pack();
		setLocationRelativeTo(MainFrame.mainFrame);
	}

	private LinkedList<Class<AbstractRobot>> getRobots() {
		File robotDir = new File(SettingsDialog.class.getResource("../robots")
				.getFile().replace("%20", " "));
		File[] robotDirs = robotDir.listFiles();
		LinkedList<Class<AbstractRobot>> robots = new LinkedList<>();
		for (File dir : robotDirs) {
			for (File file : dir.listFiles()) {
				try {
					Class<?> rob = Class.forName("robots." + dir.getName()
							+ "." + file.getName().replace(".class", ""));
					if (rob.newInstance() instanceof AbstractRobot) {

						robots.add((Class<AbstractRobot>) rob);
					}
				} catch (Exception e) {
				}
			}
		}
		return robots;
	}

	public void actionPerformed(ActionEvent e) {
		Object source = e.getSource();
		if (source == cancelButton) {
			this.dispose();
		} else if (source == okButton) {
			if (setSettings()) {
				this.dispose();
			}
		}
	}

	@SuppressWarnings("unchecked")
	// Typsicherheit manuell sichergestellt.
	private boolean setSettings() {
		try {
			// Frequenz
			double freq = Double.parseDouble(evaFreq.getText());
			int delay = (int) (1000.0 / freq);
			if (delay <= 0 || delay >= 60000) {
				throw new Exception("Frequenz nicht im zulässigen Bereich.");
			}
			// Roboter
			Class<AbstractRobot> robotClass = null;
			for (Class<AbstractRobot> r : getRobots()) {
				if (r.newInstance().getRobotName()
						.equals(robot.getSelectedItem())) {
					robotClass = (Class<AbstractRobot>) r;
				}
			}
			// Schlaufen
			boolean allow = allowLoops.isSelected();
			// Transitionsreihenfolge
			boolean transSeq = allowTransSeq.isSelected();
			// Debugausgaben auf Kosnsole
			boolean debug = debugging.isSelected();
			// Set values:
			Automat.progDelay = delay;
			MainFrame.robotClass = robotClass;
			MainFrame.robot = robotClass.newInstance();
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
