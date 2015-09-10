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
package view.listeners;

import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.JToggleButton;

import view.MainFrame;

public class InfoTextMouseListener implements MouseListener {

	public void mouseClicked(MouseEvent arg0) {
		// Auto-generated method stub
	}

	public void mouseEntered(MouseEvent arg0) {
		if(arg0.getSource() instanceof JToggleButton) {
			JToggleButton button = (JToggleButton)arg0.getSource();
			if(button.getActionCommand().equals("moveTool")) {
				MainFrame.statusBar.setInfoText("Werkzeug zum Verschieben");
			} else if(button.getActionCommand().equals("stateTool")) {
				MainFrame.statusBar.setInfoText("Werkzeug zum Erstellen von Zuständen");
			} else if(button.getActionCommand().equals("transitionTool")) {
				MainFrame.statusBar.setInfoText("Werkzeug zum Erstellen von Transitionen");
			} else if (button.getActionCommand().equals("deleteTool")) {
				MainFrame.statusBar.setInfoText("Werkzeug zum Löschen von Zuständen oder Transitionen");
			} else {
				MainFrame.statusBar.setInfoText("Unbekannt");
			}
		}
	}

	public void mouseExited(MouseEvent arg0) {
		MainFrame.statusBar.setInfoText("Bewegen sie die Maus über ein Bedienelement um seine Beschreibung anzuzeigen");
	}

	public void mousePressed(MouseEvent arg0) {
		// Auto-generated method stub
	}

	public void mouseReleased(MouseEvent arg0) {
		// Auto-generated method stub
	}
}