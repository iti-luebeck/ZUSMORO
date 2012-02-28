package view.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import model.Automat;
import view.SettingsDialog;
;

public class SettingsMenuListener implements ActionListener {

	public void actionPerformed(ActionEvent e) {
		if (e.getActionCommand().equals("progSet") && Automat.runningAutomat == null) {
			new SettingsDialog().setVisible(true);
		}
	}
}