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

import java.awt.FlowLayout;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JSeparator;
import javax.swing.SwingConstants;
import javax.swing.border.BevelBorder;

/**
 * @author ida
 * 
 *         Status bar at the bottom of the window
 */
public class StatusBar extends JPanel {

	private static final long serialVersionUID = -7960319300551657388L;
	private JLabel infoLabel;
	private JLabel eModeLabel;
	private JProgressBar progressBar;

	/**
	 * Constructor for status bar setting default text before any selection has
	 * been made
	 */
	public StatusBar() {
		super();
		setLayout(new FlowLayout(FlowLayout.LEFT));
		setBorder(BorderFactory.createBevelBorder(BevelBorder.LOWERED));
		infoLabel = new JLabel("Willkommen");
		eModeLabel = new JLabel();
		add(infoLabel);
		progressBar = new JProgressBar(0, 100);
		add(progressBar);
		progressBar.setVisible(false);
		add(new JSeparator(SwingConstants.VERTICAL));
		add(new JLabel("EditorMode: "));
		add(eModeLabel);
	}

	/**
	 * @param text
	 *            text to set
	 * 
	 *            Set the text into the bar
	 */
	public void setInfoText(String text) {
		infoLabel.setText(text);
	}

	/**
	 * @param mode
	 *            mode to set
	 * 
	 *            Set the mode into the bar
	 */
	public void setEditorMode(EditorPanel.EditorMode mode) {
		eModeLabel.setText(mode.toString());
	}

	/**
	 * Hide the progress bar because there is no transmission
	 */
	public void hideProgressBar() {
		progressBar.setVisible(false);
	}

	/**
	 * Show Progress bar and add progress value to be displayed
	 * 
	 * @param i
	 *            the progress value to be set
	 */
	public void setProgressBar(int i) {
		progressBar.setVisible(true);
		progressBar.setValue(i);
	}
}