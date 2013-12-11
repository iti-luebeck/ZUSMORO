package robots.beep;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JLabel;
import model.AbstractSettingPanel;

public class BeepSettingPanel extends AbstractSettingPanel {

	private static final long serialVersionUID = 1464618307104618595L;
	private final JButton btnloadConfig;
	private final BeepRobot robot;
	
	public BeepSettingPanel(BeepRobot robot){		
		this.robot = robot;
		this.add(new JLabel("Config laden:"));
		btnloadConfig = new JButton("Auswählen");
		btnloadConfig.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				BeepSettingPanel.this.robot.changeConfig();
				setSettings();
			}
		});
		
		this.add(btnloadConfig);
	}
	
	@Override
	public boolean setSettings() {
		return true;
		//TODO nicht sofort ändern, sondern erst bei setSettings() Aufruf
	}
	
	

}
