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
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSpinner;
import javax.swing.JTextField;
import javax.swing.SpinnerNumberModel;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

import model.Automat;
import model.Transition;

;

/**
 * The dialog to edit a specific transition, including the transition's name and
 * its rank
 * 
 * @author ida
 * 
 */
public class TransitionView extends JDialog implements ActionListener,
		DocumentListener {

	private static final long serialVersionUID = 1652072051432599068L;

	private JTextField label;
	private JSpinner priority;
	private JButton acceptButton;
	private JButton discardButton;

	private Transition transition;
	private AbstractTransitionPanel transPanel;

	/**
	 * Constructor adding all components
	 * 
	 * @param trans
	 *            the transition's name
	 */
	public TransitionView(Transition trans) {
		super(MainFrame.mainFrame, "Transition " + trans.getLabel()
				+ " bearbeiten...", true);
		this.transition = trans;
		setLayout(new BorderLayout());
		// panel = new BackgroundPanel(BackgroundPanel.trans_background);
		// panel.setLayout(null);
		initComponents();
		// setValues();
		// panel.update(this.sensorPanels);
		JPanel northPanel = new JPanel();
		northPanel.add(new JLabel("Bezeichnung: "));
		northPanel.add(label);
		if (Automat.changeableTransSeq) {
			northPanel.add(new JLabel("Auswertungsrang: "));
			northPanel.add(priority);
		}
		add(northPanel, BorderLayout.NORTH);
		add(transPanel, BorderLayout.CENTER);
		JPanel southPanel = new JPanel();
		southPanel.setLayout(new FlowLayout(FlowLayout.RIGHT));
		southPanel.add(discardButton);
		southPanel.add(acceptButton);
		add(southPanel, BorderLayout.SOUTH);
		pack();
		setResizable(false);
		// addMouseListener(this);
		this.setLocationRelativeTo(trans);
	}

	private void initComponents() {
		label = new JTextField(transition.getLabel(), 15);
		label.getDocument().addDocumentListener(this);
		ArrayList<Transition> transList = transition.getRootState()
				.getTransitions();
		int transPos = transList.indexOf(transition);
		int listSize = transList.size();
		priority = new JSpinner(new SpinnerNumberModel(transPos, 0,
				listSize - 1, 1));
		try {
			transPanel = MainFrame.robotClass.newInstance().getTransitionPanel(
					transition);
		} catch (Exception e) {
			e.printStackTrace();
		}
		acceptButton = new JButton("Übernehmen und Schließen");
		acceptButton.addActionListener(this);
		discardButton = new JButton("Abbrechen");
		discardButton.addActionListener(this);
		// sensorPanels = new SensorPanel[12];
		// for (int i = 0; i < sensorPanels.length; i++) {
		// sensorPanels[i] = new SensorPanel();
		// }
	}

	public void actionPerformed(ActionEvent e) {
		if (e.getSource() == discardButton) {
			dispose();
		} else if (e.getSource() == acceptButton) {
			transition.setLabel(label.getText());
			transition.setGuard(transPanel.getGuard());
			// System.out.println("trans: " + transPanel.getGuard());
			// transition.setToolTipText("<html><pre>" +
			// transition.getGuard().toString().replace("\r\n", "<br>")
			// + "</pre></html>");
			ArrayList<Transition> transList = transition.getRootState()
					.getTransitions();
			if (transList.size() > 1) {
				Transition[] transis = new Transition[transList.size() - 1];
				transList.remove(transition);
				transis = transList.toArray(transis);
				transList.clear();
				int newPos = (Integer) priority.getValue();
				for (int i = 0; i <= transis.length; i++) {
					if (i == newPos) {
						transList.add(transition);
					}
					if (i < transis.length) {
						transList.add(transis[i]);
					}
				}
			}
			dispose();
		}
	}

	public void changedUpdate(DocumentEvent e) {
		setTitle("Transition " + label.getText() + " bearbeiten...");
	}

	public void insertUpdate(DocumentEvent e) {
		setTitle("Transition " + label.getText() + " bearbeiten...");
	}

	public void removeUpdate(DocumentEvent e) {
		setTitle("Transition " + label.getText() + " bearbeiten...");
	}
}
