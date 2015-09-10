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

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.text.ChangedCharSetException;

import model.AbstractRobot;
import model.AbstractSettingPanel;

public class SettingsDialog extends JDialog implements ActionListener {

	private static final long serialVersionUID = 4333492082030324369L;
	private AbstractRobot selectedRobot;
	private JComboBox<String> robot;
	private JButton okButton;
	private JButton cancelButton;
	private AbstractSettingPanel robotSettings; //robot specific Panel containing all config options

	public SettingsDialog() {
		super(MainFrame.mainFrame, "Automateneinstellungen", true);
		setDefaultCloseOperation(JDialog.DISPOSE_ON_CLOSE);
		Container cPane = this.getContentPane();
		cPane.setLayout(new BorderLayout());

		JPanel robotPanel = new JPanel();
		robotPanel.add(new JLabel("Roboter: "));
		robot = new JComboBox<String>();
		for (AbstractRobot rob : MainFrame.robots) {
			try {
				robot.addItem(rob.getRobotName());
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		if (MainFrame.robot != null && MainFrame.robotClass != null) {
			selectedRobot = MainFrame.robot;
			robot.setSelectedItem(selectedRobot.getRobotName());
		}
		
		robot.addItemListener(new ItemListener() {
			@Override
			public void itemStateChanged(ItemEvent arg0) {
				if (!selectedRobot.getRobotName().equals(arg0.getItem())) {
					for (AbstractRobot rob : MainFrame.robots) {
						if (rob.getRobotName().equals(arg0.getItem())) {
							selectedRobot = rob;
							chagneSettingsPanel(rob.getSettingsPanel());
							break;
						}
					}
				}
			}
		});

		robotPanel.add(robot);
		cPane.add(robotPanel, BorderLayout.NORTH);

		robotSettings = MainFrame.robot.getSettingsPanel();
		cPane.add(robotSettings, BorderLayout.CENTER);

		JPanel controlPanel = new JPanel();
		okButton = new JButton("OK");
		okButton.addActionListener(this);
		cancelButton = new JButton("Abbrechen");
		cancelButton.addActionListener(this);
		controlPanel.add(okButton);
		controlPanel.add(cancelButton);

		cPane.add(controlPanel, BorderLayout.SOUTH);

		pack();
		setLocationRelativeTo(MainFrame.mainFrame);
	}
	
	private void chagneSettingsPanel(AbstractSettingPanel newPanel){
		this.remove(robotSettings);
		this.add(newPanel, BorderLayout.CENTER);
		robotSettings = newPanel;	
		pack();
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

	// Typsicherheit manuell sichergestellt.
	private boolean setSettings() {
		setRobotType((String) robot.getSelectedItem());
		return robotSettings.setSettings();
	}

	/**
	 * Checks the ../Robots directory for the first fitting abstract Robot
	 * Class.<br>
	 * Fitting means the result of the <code>getRobotName</code> function of the
	 * class equals the <code>robotName</code> parameter.
	 * 
	 * @param robotName
	 *            Name of the Robot (result of the <code>getRobotName</code>
	 *            function)
	 */
	public static void setRobotType(String robotName) {
		Class<AbstractRobot> robotClass = null;
		for (AbstractRobot r : MainFrame.robots) {
			if (r.getRobotName().equals(robotName)) {
				robotClass = (Class<AbstractRobot>) r.getClass();
				MainFrame.robot = r;
				MainFrame.robotClass = robotClass;
				break;
			}
		}

		MainFrame.toolPanel.setRobotName(robotName);
	}
}
