package view.listeners;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;

import model.Automat;
import view.MainFrame;

public class UndoListener extends AbstractAction {

	private static final long serialVersionUID = -5326415837264816909L;

	@Override
	public void actionPerformed(ActionEvent e) {
		if (Automat.runningAutomat == null) {
			if (e.getActionCommand().equals("undo")) {
				MainFrame.editorPanel.undo();
			} else { //redo
				MainFrame.editorPanel.redo();
			}
		}
	}
}
