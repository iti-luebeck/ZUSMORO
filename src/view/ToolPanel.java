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

/**
 * @author ida
 * 
 *         Panel on the left side containing all buttons
 */
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

	/**
	 * Constructor for the ToolPannel
	 * 
	 * @param connected
	 *            Is the robot connected to the computer?
	 * @param transmitted 
	 * 			  Is the automat transmitted completely?
	 */
	public ToolPanel() {
		super();
		mouseListener = new InfoTextMouseListener();
		setLayout(new BorderLayout());
		JPanel toolPanel = new JPanel(new GridLayout(4, 1));
		toolPanel.setBorder(BorderFactory.createTitledBorder("Werkzeuge:"));
		buttonGroup = new ButtonGroup();

		createMoveButton(toolPanel);
		createStateButton(toolPanel);
		createTransitionButton(toolPanel);
		createDeleteButton(toolPanel);

		buttonGroup.setSelected(moveButton.getModel(), true);

		JPanel controlPanel = new JPanel(new GridLayout(3, 1));
		ActionListener runListener = new RunMenuListener();
		controlPanel.setBorder(BorderFactory.createTitledBorder("Programm:"));

		createConnectButton(controlPanel, runListener);
		createTransmitButton(controlPanel, runListener);
		createStartButton(controlPanel, runListener);
		createStopButton(controlPanel, runListener);
		createDebugButton(controlPanel, runListener);
		add(toolPanel, BorderLayout.NORTH);
		add(controlPanel, BorderLayout.SOUTH);
	}

	private void createDebugButton(JPanel controlPanel,
			ActionListener runListener) {
		debugButton = new JButton(Images.DEBUG);
		debugButton.setActionCommand("debug");
		debugButton.addMouseListener(mouseListener);
		debugButton.addActionListener(runListener);
		controlPanel.add(debugButton);
		debugButton.setEnabled(false);
	}

	private void createStopButton(JPanel controlPanel,
			ActionListener runListener) {
		stopButton = new JButton(Images.STOP);
		stopButton.setActionCommand("stop");
		stopButton.addMouseListener(mouseListener);
		stopButton.addActionListener(runListener);
		controlPanel.add(stopButton);
		stopButton.setEnabled(false);
	}

	private void createStartButton(JPanel controlPanel,
			ActionListener runListener) {
		startButton = new JButton(Images.START);
		startButton.setActionCommand("run");
		startButton.addMouseListener(mouseListener);
		startButton.addActionListener(runListener);
		controlPanel.add(startButton);
		startButton.setEnabled(false);
	}

	private void createTransmitButton(JPanel controlPanel,
			ActionListener runListener) {
		transmitButton = new JButton(Images.TRANSMIT);
		transmitButton.setActionCommand("transmit");
		transmitButton.addMouseListener(mouseListener);
		transmitButton.addActionListener(runListener);
		controlPanel.add(transmitButton);
		transmitButton.setEnabled(false);
	}

	private void createConnectButton(JPanel controlPanel,
			ActionListener runListener) {
		connectButton = new JButton(Images.CONNECT);
		connectButton.setActionCommand("connect");
		connectButton.addMouseListener(mouseListener);
		connectButton.addActionListener(runListener);
		controlPanel.add(connectButton);
	}

	private void createDeleteButton(JPanel toolPanel) {
		deleteButton = new JToggleButton(Images.DELETETOOL);
		deleteButton.setActionCommand("deleteTool");
		deleteButton.addMouseListener(mouseListener);
		deleteButton.addActionListener(this);
		deleteButton
				.setToolTipText("Transition oder Zustand zum Löschen anklicken");
		toolPanel.add(deleteButton);
		buttonGroup.add(deleteButton);
	}

	private void createTransitionButton(JPanel toolPanel) {
		transitionButton = new JToggleButton(Images.TRANSITIONTOOL);
		transitionButton.setActionCommand("transitionTool");
		transitionButton.addMouseListener(mouseListener);
		transitionButton.addActionListener(this);
		transitionButton
				.setToolTipText("<html>Auf einem Zustand linke Maustaste drücken,<br>halten und über Endzustand loslassen</html>");
		toolPanel.add(transitionButton);
		buttonGroup.add(transitionButton);
	}

	private void createStateButton(JPanel toolPanel) {
		stateButton = new JToggleButton(Images.STATETOOL);
		stateButton.setActionCommand("stateTool");
		stateButton.addMouseListener(mouseListener);
		stateButton.addActionListener(this);
		stateButton
				.setToolTipText("Zustand durch klicken auf die Arbeitsfläche hinzufügen");
		toolPanel.add(stateButton);
		buttonGroup.add(stateButton);
	}

	private void createMoveButton(JPanel toolPanel) {
		moveButton = new JToggleButton(Images.MOVETOOL);
		moveButton.setSelected(true);
		moveButton.setActionCommand("moveTool");
		moveButton.addMouseListener(mouseListener);
		moveButton.addActionListener(this);
		moveButton
				.setToolTipText("Mit gedrückter Maustaste Transitionen oder Zustände verschieben");
		toolPanel.add(moveButton);
		buttonGroup.add(moveButton);
	}

	/**
	 * @param connected
	 *            is there an active connection?
	 * 
	 *            Set connection status
	 */
	public void setConnected(boolean connected) {
		connectButton.setIcon(connected ? Images.DISCONNECT : Images.CONNECT);
		connectButton.setActionCommand(connected ? "disconnect" : "connect");
		enableTransmit(connected);
		enableStart(connected);
		enableStop(connected);
		enableDebug(connected);
	}
	
	public void enableTransmit(boolean enable) {
		transmitButton.setEnabled(enable);
	}
	
	public void enableStart(boolean enable) {
		startButton.setEnabled(enable);
	}
	
	public void enableDebug(boolean enable) {
		debugButton.setEnabled(enable);
	}
	
	public void enableStop(boolean enable) {
		stopButton.setEnabled(enable);
	}
	
	public void enableEditing(boolean enable) {
		transitionButton.setEnabled(enable);
		moveButton.setEnabled(enable);
		stateButton.setEnabled(enable);
		deleteButton.setEnabled(enable);
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