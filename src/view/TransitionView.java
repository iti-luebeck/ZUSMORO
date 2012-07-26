package view;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSpinner;
import javax.swing.JTextField;
import javax.swing.SpinnerNumberModel;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

import model.Automat;
import model.Transition;
;

public class TransitionView extends JDialog implements ActionListener, DocumentListener {

	private static final long serialVersionUID = 1652072051432599068L;

	private JTextField label;
	private JSpinner priority;
	private JButton acceptButton;
	private JButton discardButton;

	private Transition transition;
	private AbstractTransitionPanel transPanel;

	public TransitionView(Transition trans) {
		super(MainFrame.mainFrame, "Transition " + trans.getLabel() + " bearbeiten...", true);
		this.transition = trans;
		setLayout(new BorderLayout());
		// panel = new BackgroundPanel(BackgroundPanel.trans_background);
		// panel.setLayout(null);
		initComponents();
		// setValues();
		// panel.update(this.sensorPanels);
		JPanel northPanel = new JPanel();
		northPanel.add(new JLabel("Bezeichnung: "));
		northPanel.add(label);
		if (Automat.changeableTransSeq) {
			northPanel.add(new JLabel("Auswertungsrang: "));
			northPanel.add(priority);
		}
		add(northPanel, BorderLayout.NORTH);
		add(transPanel, BorderLayout.CENTER);
		JPanel southPanel = new JPanel();
		southPanel.setLayout(new FlowLayout(FlowLayout.RIGHT));
		southPanel.add(discardButton);
		southPanel.add(acceptButton);
		add(southPanel, BorderLayout.SOUTH);
		pack();
		setResizable(false);
		// addMouseListener(this);
		this.setLocationRelativeTo(trans);
	}

	private void initComponents() {
		label = new JTextField(transition.getLabel(), 15);
		label.getDocument().addDocumentListener(this);
		ArrayList<Transition> transList = transition.getRootState().getTransitions();
		int transPos = transList.indexOf(transition);
		int listSize = transList.size();
		priority = new JSpinner(new SpinnerNumberModel(transPos, 0, listSize - 1, 1));
		try {
			transPanel = MainFrame.robotClass.newInstance().getTransitionPanel(transition);
		} catch (Exception e) {
			e.printStackTrace();
		}
		acceptButton = new JButton("Übernehmen und Schließen");
		acceptButton.addActionListener(this);
		discardButton = new JButton("Abbrechen");
		discardButton.addActionListener(this);
		// sensorPanels = new SensorPanel[12];
		// for (int i = 0; i < sensorPanels.length; i++) {
		// sensorPanels[i] = new SensorPanel();
		// }
	}

	public void actionPerformed(ActionEvent e) {
		if (e.getSource() == discardButton) {
			dispose();
		} else if (e.getSource() == acceptButton) {
			transition.setLabel(label.getText());
			transition.setGuard(transPanel.getGuard());
			//System.out.println("trans: " + transPanel.getGuard());
			//transition.setToolTipText("<html><pre>" + transition.getGuard().toString().replace("\r\n", "<br>")
			//		+ "</pre></html>");
			ArrayList<Transition> transList = transition.getRootState().getTransitions();
			if (transList.size() > 1) {
				Transition[] transis = new Transition[transList.size() - 1];
				transList.remove(transition);
				transis = transList.toArray(transis);
				transList.clear();
				int newPos = (Integer) priority.getValue();
				for (int i = 0; i <= transis.length; i++) {
					if (i == newPos) {
						transList.add(transition);
					}
					if (i < transis.length) {
						transList.add(transis[i]);
					}
				}
			}
			dispose();
		}
	}

	public void changedUpdate(DocumentEvent e) {
		setTitle("Transition " + label.getText() + " bearbeiten...");
	}

	public void insertUpdate(DocumentEvent e) {
		setTitle("Transition " + label.getText() + " bearbeiten...");
	}

	public void removeUpdate(DocumentEvent e) {
		setTitle("Transition " + label.getText() + " bearbeiten...");
	}
}
