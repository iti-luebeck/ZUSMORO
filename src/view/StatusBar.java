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