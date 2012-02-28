package view;

import javax.swing.BorderFactory;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;

public class AboutDialog extends JDialog {

	private static final long serialVersionUID = 2566945927276159128L;
	private final String aboutText = "<html><center><b>ZUSMORO Version "+MainFrame.VERSION +"</b><br><br>" +
			" Dieses Programm wurde von<br>Tobias Meyer<br>" +
			"im Rahmen der Bachelorarbeit<br>" +
			"\"Entwicklung einer zustandsbasierten<br>" +
			"Steuerungssoftware für mobile Roboter\"<br>" +
			"am Institut für Technische Informatik<br>" +
			"an der Universität zu Lübeck<br>" +
			"geschrieben</center></html>";

	public AboutDialog() {
		super(MainFrame.mainFrame, "About...", true);
		JPanel panel = new JPanel();
		panel.setBorder(BorderFactory.createTitledBorder("About"));
		//panel.setLayout(new GridLayout(2, 1));
		panel.add(new JLabel(aboutText));
		//panel.add(new JLabel("20.08.2009"));
		this.add(panel);
		this.pack();
		this.setLocationRelativeTo(MainFrame.mainFrame);
	}
}
