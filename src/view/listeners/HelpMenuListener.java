package view.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import view.AboutDialog;

public class HelpMenuListener implements ActionListener {

	public void actionPerformed(ActionEvent e) {
		if(e.getActionCommand().equals("about")) {
			new AboutDialog().setVisible(true);
		}
	}
}