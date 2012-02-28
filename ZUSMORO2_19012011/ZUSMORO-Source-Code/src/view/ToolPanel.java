package view;

import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseListener;

import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JToggleButton;

import view.EditorPanel.EditorMode;
import view.img.Images;
import view.listeners.InfoTextMouseListener;
import view.listeners.RunMenuListener;

public class ToolPanel extends JPanel implements ActionListener {

	private JToggleButton moveButton;
	private JToggleButton stateButton;
	private JToggleButton transitionButton;
	private JToggleButton deleteButton;
	private JButton connectButton;
	private JButton startButton;
	private JButton stopButton;
	private ButtonGroup buttonGroup;
	private MouseListener mouseListener;
	private JButton transmitButton;
	private JButton debugButton;

	private static final long serialVersionUID = -1528913814588121352L;

	public ToolPanel() {
		super();
		mouseListener = new InfoTextMouseListener();
		setLayout(new BorderLayout());
		JPanel toolPanel = new JPanel(new GridLayout(4, 1));
		toolPanel.setBorder(BorderFactory.createTitledBorder("Werkzeuge:"));
		buttonGroup = new ButtonGroup();
		moveButton = new JToggleButton(Images.MOVETOOL);
		moveButton.setSelected(true);
		moveButton.setActionCommand("moveTool");
		moveButton.addMouseListener(mouseListener);
		moveButton.addActionListener(this);
		moveButton.setToolTipText("Mit gedrückter Maustaste Transitionen oder Zustände verschieben");
		toolPanel.add(moveButton);
		buttonGroup.add(moveButton);

		stateButton = new JToggleButton(Images.STATETOOL);
		stateButton.setActionCommand("stateTool");
		stateButton.addMouseListener(mouseListener);
		stateButton.addActionListener(this);
		stateButton.setToolTipText("Zustand durch klicken auf die Arbeitsfläche hinzufügen");
		toolPanel.add(stateButton);
		buttonGroup.add(stateButton);

		transitionButton = new JToggleButton(Images.TRANSITIONTOOL);
		transitionButton.setActionCommand("transitionTool");
		transitionButton.addMouseListener(mouseListener);
		transitionButton.addActionListener(this);
		transitionButton.setToolTipText("<html>Auf einem Zustand linke Maustaste drücken,<br>halten und über Endzustand loslassen</html>");
		toolPanel.add(transitionButton);
		buttonGroup.add(transitionButton);

		deleteButton = new JToggleButton(Images.DELETETOOL);
		deleteButton.setActionCommand("deleteTool");
		deleteButton.addMouseListener(mouseListener);
		deleteButton.addActionListener(this);
		deleteButton.setToolTipText("Transition oder Zustand zum Löschen anklicken");
		toolPanel.add(deleteButton);
		buttonGroup.add(deleteButton);
		buttonGroup.setSelected(moveButton.getModel(), true);

		JPanel controlPanel = new JPanel(new GridLayout(3,1));
		ActionListener runListener = new RunMenuListener();
		controlPanel.setBorder(BorderFactory.createTitledBorder("Programm:"));
		connectButton = new JButton(Images.CONNECT);
		connectButton.setActionCommand("connect");
		connectButton.addMouseListener(mouseListener);
		connectButton.addActionListener(runListener);
		controlPanel.add(connectButton);

		transmitButton = new JButton(Images.TRANSMIT);
		transmitButton.setActionCommand("transmit");
		transmitButton.addMouseListener(mouseListener);
		transmitButton.addActionListener(runListener);
		controlPanel.add(transmitButton);

		startButton = new JButton(Images.START);
		startButton.setActionCommand("run");
		startButton.addMouseListener(mouseListener);
		startButton.addActionListener(runListener);
		controlPanel.add(startButton);

		stopButton = new JButton(Images.STOP);
		stopButton.setActionCommand("stop");
		stopButton.addMouseListener(mouseListener);
		stopButton.addActionListener(runListener);
		controlPanel.add(stopButton);

		debugButton = new JButton(Images.DEBUG);
		debugButton.setActionCommand("debug");
		debugButton.addMouseListener(mouseListener);
		debugButton.addActionListener(runListener);
		controlPanel.add(debugButton);


		add(toolPanel, BorderLayout.NORTH);
		add(controlPanel, BorderLayout.SOUTH);
	}

	public void setConnected(boolean connected) {
		connectButton.setIcon(connected ? Images.DISCONNECT : Images.CONNECT);
		connectButton.setActionCommand(connected ? "disconnect" : "connect");
	}

	public void actionPerformed(ActionEvent event) {
		Object source = event.getSource();
		if (source == moveButton) {
			MainFrame.editorPanel.setMode(EditorMode.EDIT);
		} else if (source == stateButton) {
			MainFrame.editorPanel.setMode(EditorMode.CREATE_STATE);
		} else if (source == transitionButton) {
			MainFrame.editorPanel.setMode(EditorMode.CREATE_TRANSITION);
		} else if (source == deleteButton) {
			MainFrame.editorPanel.setMode(EditorMode.DELETE);
		}
	}
}