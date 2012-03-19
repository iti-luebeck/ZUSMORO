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