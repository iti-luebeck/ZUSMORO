package view;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

import model.State;

public class StateView extends JDialog implements ActionListener, DocumentListener {

	private static final long serialVersionUID = -8023383434693559831L;

	private State state;
	private boolean wasInitialState;

	private JTextField label;
	private JCheckBox initialState;
	private JButton discardButton;
	private JButton acceptButton;
	private AbstractStatePanel statePanel;

	public StateView(State state) {
		super(MainFrame.mainFrame, "Zustand " + state.getText() + " bearbeiten...", true);
		this.state = state;
		this.wasInitialState = state.isInitialState();
		setLayout(new BorderLayout());
		//BackgroundPanel panel = new BackgroundPanel(BackgroundPanel.state_background);
		//panel.setLayout(null);
		initComponents();
		//setValues();
		//setComponentBounds();
		JPanel labelPanel = new JPanel();
		labelPanel.add(new JLabel("Bezeichnung: "));
		labelPanel.add(label);
		labelPanel.add(initialState);
		JPanel buttonsPanel = new JPanel(new FlowLayout(FlowLayout.RIGHT));
		buttonsPanel.add(discardButton);
		buttonsPanel.add(acceptButton);
		Container contentPane = this.getContentPane();
		contentPane.add(labelPanel, BorderLayout.NORTH);
		contentPane.add(statePanel, BorderLayout.CENTER);
		contentPane.add(buttonsPanel, BorderLayout.SOUTH);
		this.pack();
		this.setResizable(false);
		this.setLocationRelativeTo(state);
	}

	private void initComponents() {
		label = new JTextField(state.getText(), 17);
		label.getDocument().addDocumentListener(this);
		label.setSelectionStart(0);
		label.setSelectionEnd(label.getText().length());
		wasInitialState = state.isInitialState();
		initialState = new JCheckBox("Startzustand", wasInitialState);

		try {
			statePanel = MainFrame.robotClass.newInstance().getStatePanel(state);
		} catch (Exception e) {
			e.printStackTrace();
		}

		discardButton = new JButton("Abbrechen");
		discardButton.addActionListener(this);
		acceptButton = new JButton("Übernehmen und Schließen");
		acceptButton.addActionListener(this);
	}

	public void actionPerformed(ActionEvent e) {
		if (e.getSource() == discardButton) {
			dispose();
		} else if (e.getSource() == acceptButton) {
			if (initialState.isSelected()) {
				MainFrame.automat.setInitialState(state);
			} else {
				if (wasInitialState) {
					state.setInitialState(false);
					MainFrame.automat.setInitialState(null);
				}
			}
			if (label.getText().trim().length() >= 1) {
				state.setText(label.getText());
				state.setActions(statePanel.getActions());
				dispose();
			} else {
				JOptionPane.showMessageDialog(this, "Bitte eine sinnvolle Bezeichnung eingeben",
						"Zur kurze Bezeichnung", JOptionPane.WARNING_MESSAGE);
				label.requestFocus();
			}
		}
	}

	public void changedUpdate(DocumentEvent e) {
		setTitle("Zustand " + label.getText() + " bearbeiten...");
	}

	public void insertUpdate(DocumentEvent e) {
		setTitle("Zustand " + label.getText() + " bearbeiten...");
	}

	public void removeUpdate(DocumentEvent e) {
		setTitle("Zustand " + label.getText() + " bearbeiten...");
	}
}
