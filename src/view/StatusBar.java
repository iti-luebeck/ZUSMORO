package view;

import java.awt.FlowLayout;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.SwingConstants;
import javax.swing.border.BevelBorder;

public class StatusBar extends JPanel {

	private static final long serialVersionUID = -7960319300551657388L;
	private JLabel infoLabel;
	private JLabel eModeLabel;
	
	public StatusBar() {
		super();
		setLayout(new FlowLayout(FlowLayout.LEFT));
		setBorder(BorderFactory.createBevelBorder(BevelBorder.LOWERED));
		infoLabel = new JLabel("Willkommen");
		eModeLabel = new JLabel();
		add(infoLabel);
		add(new JSeparator(SwingConstants.VERTICAL));
		add(new JLabel("EditorMode: "));
		add(eModeLabel);
	}
	
	public void setInfoText(String text) {
		infoLabel.setText(text);
	}
	
	public void setEditorMode(EditorPanel.EditorMode mode) {
		eModeLabel.setText(mode.toString());
	}
}