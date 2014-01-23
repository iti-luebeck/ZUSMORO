package model;

import java.awt.Point;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Observable;

import javax.swing.JOptionPane;
import javax.swing.SwingUtilities;

import model.ChangeEvent.ChangeEventType;
import model.bool.BooleanExpression;
import smachGenerator.ISmachableAction;
import view.MainFrame;
import view.SettingsDialog;

public class Automat extends Observable implements Runnable {

	public static Automat runningAutomat = null;
	public static int progDelay = 50;
	public static boolean loopsAllowed = false;
	public static boolean changeableTransSeq = false;

	private AbstractRobot robot;
	private final ArrayList<State> states;
	private State activeState = null;
	private State initialState = null;
	private long lastStateChange = 0;
	private String saveToFile = null;
	private volatile boolean cancelRequested = false;

	public Automat() {
		states = new ArrayList<State>();
	}

	public Automat(String saveFile) {
		// states = new ArrayList<State>();
		this();
		saveToFile = saveFile;
	}

	public void addState(State newState) {
		if (Automat.runningAutomat == null) {
			if (newState.isInitialState()) {
				setInitialState(newState);
			}
			for (State s : states) {
				if (s.getText().equals(newState.getText())) {
					newState.setText(getUniqueStateName(newState.getText()));
					break;
				}
			}
			states.add(newState);
			this.setChanged();
			this.notifyObservers(new ChangeEvent(ChangeEventType.STATE_CREATE,
					newState, false));
		}
	}

	public void removeState(State state) {
		states.remove(state);
		if (state == initialState) {
			setInitialState(null);
			state.setInitialState(true); // fuer redo...
		}
		ArrayList<Transition> transis;
		Transition transi;
		transis = state.getTransitions();
		for (int i = 0; i < transis.size(); i++) {
			MainFrame.automat.setChanged(new ChangeEvent(
					ChangeEventType.TRANSITION_DELETE, transis.get(i), true));
			state.getTransitions().get(i).setLabel("");
		}
		transis.clear();
		for (State stat : states) {
			transis = stat.getTransitions();
			for (int i = 0; i < transis.size(); i++) {
				transi = transis.get(i);
				if (state == transi.getFollowerState()) {
					stat.removeTransition(transi);
					MainFrame.automat.setChanged(new ChangeEvent(
							ChangeEventType.TRANSITION_DELETE, transi, true));
					// this.setChanged();
					// this.notifyObservers(new
					// ChangeEvent(ChangeEventType.TRANSITION_DELETE, transi));
					i--;
				}
			}
		}
		this.setChanged();
		this.notifyObservers(new ChangeEvent(ChangeEventType.STATE_DELETE,
				state, false));
	}

	public void clearSelection() {
		for (State state : this.states) {
			state.setSelected(false);
		}
	}

	public ArrayList<State> getStates() {
		return this.states;
	}

	public int getStateCount() {
		return this.states.size();
	}

	public ArrayList<State> getSelectedStates() {
		ArrayList<State> selectedStates = new ArrayList<State>();
		for (State state : this.states) {
			if (state.isSelected()) {
				selectedStates.add(state);
			}
		}
		return selectedStates;
	}

	public State getActiveState() {
		return activeState;
	}

	public void setActiveState(State activeState) {
		this.activeState = activeState;
		setChanged();
		notifyObservers(new ChangeEvent(ChangeEventType.ACTIVE_STATE_CHANGE,
				activeState, false));
		if (activeState == null) {
			requestCancel(
					"Bei der Ausführung des Programms kam es zu einer unauthorisierten Übertragung nach Redmond!",
					true);
		}
	}

	public AbstractRobot getRobot() {
		return robot;
	}

	public void setRobot(AbstractRobot newRobot) {
		this.robot = newRobot;
	}

	public boolean checkProgramm() {
		boolean checked = false;
		if (this.robot == null) {
			JOptionPane
					.showMessageDialog(
							MainFrame.mainFrame,
							"<html>Programm ist mit keinem Roboter verbunden!<br>"
									+ "Stellen Sie zunächst eine Verbindung mit einem Roboter her.</html>",
							"Automat kann nicht ausgeführt werden!",
							JOptionPane.WARNING_MESSAGE);
		} else if (initialState == null) {
			JOptionPane
					.showMessageDialog(
							MainFrame.mainFrame,
							"<html>Es ist kein Startzustand vorhanden!<br>"
									+ "Öffnen Sie einen Zustand und markieren Sie ihn als Startzustand.</html>",
							"Automat kann nicht ausgeführt werden!",
							JOptionPane.WARNING_MESSAGE);
		} else if (!checkNames()) {
			JOptionPane
					.showMessageDialog(
							MainFrame.mainFrame,
							"<html>Es gibt gleichnamige Zust�nde!<br>"
									+ "Öffnen Sie einen dieser Zust�nde und geben Sie ihm einen eindeutigen Namen.</html>",
							"Automat kann nicht ausgeführt werden!",
							JOptionPane.WARNING_MESSAGE);
		} else {
			checked = true;
		}
		return checked;
	}

	public boolean checkNames() {
		LinkedList<String> transNames = new LinkedList<>();
		for (State s1 : states) {
			for (Transition t : s1.getTransitions()) {
				if (transNames.contains(t.getLabel())) { // Check for double
															// transition names
					return false;
				} else {
					transNames.add(t.getLabel());
				}
			}
			for (State s2 : states) {
				if (!s1.equals(s2) && s1.getText().equals(s2.getText())) { // Check
																			// for
																			// double
																			// State
																			// Name
					return false;
				}
			}
		}
		return true;
	}

	public void setChanged(ChangeEvent event) {
		this.setChanged();
		this.notifyObservers(event);
	}

	public int getVariableValue(String variable) {
		if (variable.equalsIgnoreCase("timer")) {
			return (int) (System.currentTimeMillis() - lastStateChange);
		} else {
			return robot.getVariableValue(variable);
		}
	}

	public void start() {
		if (Automat.runningAutomat == null && checkProgramm()) {
			Automat.runningAutomat = this;
			cancelRequested = false;
			new Thread(this).start();
		}
	}

	public void run() {
		activeState = initialState;
		lastStateChange = System.currentTimeMillis();
		long computationStart = System.currentTimeMillis();
		setChanged();
		notifyObservers(new ChangeEvent(ChangeEventType.ACTIVE_STATE_CHANGE,
				activeState, false));
		while (!cancelRequested) {
			try {
				robot.executeActions(activeState.getActions());
				robot.updateSensors();
			} catch (Exception e) {
				requestCancel(
						"<html>Die Verbindung zum Roboter ist gestört:<br>"
								+ e.getMessage() + "</html>", true);
			}
			try {
				Thread.sleep(Math.max(
						Automat.progDelay
								+ robot.getDesiredAdditionalTimeout()
								- (System.currentTimeMillis() - computationStart),
						0));
			} catch (InterruptedException e) {
				// never mind
			}
			computationStart = System.currentTimeMillis();
			for (Transition trans : activeState.getTransitions()) {
				if (trans.eval()) {
					activeState = trans.getFollowerState();
					if (MainFrame.DEBUG) {
						System.out
								.println("Time between State Change: "
										+ (System.currentTimeMillis() - lastStateChange));
					}
					lastStateChange = System.currentTimeMillis();
					setChanged();
					notifyObservers(new ChangeEvent(
							ChangeEventType.ACTIVE_STATE_CHANGE, activeState,
							false));
					if (activeState == null) {
						requestCancel(
								"Bei der Ausführung des Programms kam es zu einem Pinguinangriff!",
								true);
					}
				}
			}
			System.out.println(robot.getUnAcknowledgedCmds());
			if (robot.getUnAcknowledgedCmds() > 20) {
				requestCancel(
						"<html>Die Verbindung zum Roboter ist gestört:<br>Zu viele Befehle wurden nicht bestätigt",
						true);
			}
		}
		activeState = null;
		Automat.runningAutomat = null;
		robot.stop();
	}

	public State getInitialState() {
		return this.initialState;
	}

	public void setInitialState(State initialState) {
		if (this.initialState != null) {
			this.initialState.setInitialState(false);
		}
		this.initialState = initialState;
		if (this.initialState != null) {
			this.initialState.setInitialState(true);
		}
		MainFrame.editorPanel.repaint();
	}

	public void requestCancel(final String reason, boolean abnormal) {
		cancelRequested = true;
		if (MainFrame.DEBUG) {
			System.out.println("Cancel requested, Grund: " + reason
					+ " abnormal: " + abnormal);
		}
		if (abnormal) {
			SwingUtilities.invokeLater(new Runnable() {
				public void run() {
					JOptionPane.showMessageDialog(MainFrame.mainFrame, reason,
							"Programmausführung abgebrochen",
							JOptionPane.WARNING_MESSAGE);
				}
			});
			// disconnect();
		}
		// Programm.runningProg = null;
	}

	public void connect(String connectTo) {
		disconnect();
		try {
			robot = MainFrame.robotClass.newInstance();
		} catch (Exception e) {
			JOptionPane.showMessageDialog(MainFrame.mainFrame,
					"<html>Bei der Initialisierung der Roboters trat ein Fehler auf!<br>"
							+ e.getMessage(),
					"Verbindung konnte nicht hergestellt werden!",
					JOptionPane.WARNING_MESSAGE);
			return;
		}
		if (MainFrame.DEBUG) {
			System.out.println("Automat connecting to: " + connectTo + " ...");
		}
		if (robot.connect(connectTo)) {
			MainFrame.toolPanel.setConnected(true);
			if (MainFrame.DEBUG) {
				System.out.println("Automat connected!");
			}
		} else {
			if (MainFrame.DEBUG) {
				System.out.println("Automat connection failed");
			}
		}
	}

	public void disconnect() {
		if (robot != null) {
			if (MainFrame.DEBUG) {
				System.out.println("Automaten disconnecting...");
			}
			robot.disconnect();
			robot = null;
			MainFrame.toolPanel.setConnected(false);
			if (MainFrame.DEBUG) {
				System.out.println("Automat disconnected!");
			}
		}
	}

	public String getSaveToFile() {
		return saveToFile;
	}

	public boolean saveAutomat(File file) throws IOException {
		RandomAccessFile saveFile;
		String fileName = file.getAbsolutePath();
		fileName = fileName.endsWith(".xml") ? fileName : (fileName + ".xml");
		if (MainFrame.DEBUG) {
			System.out.println("Saving Automat under: " + fileName);
		}
		try {
			saveFile = new RandomAccessFile(fileName, "rw");
		} catch (FileNotFoundException e) {
			return false;
		}
		saveFile.setLength(1);
		saveFile.writeBytes("<Automat robotName='"
				+ MainFrame.robot.getRobotName() + "' >\r\n");
		ArrayList<Transition> transitions = new ArrayList<Transition>(
				states.size() * 3);
		for (State state : states) {
			transitions.addAll(state.getTransitions());
			saveState(state, saveFile);
		}
		for (Transition transition : transitions) {
			saveTransition(transition, saveFile);
		}
		saveFile.writeBytes("</Automat>");
		saveFile.close();
		if (MainFrame.DEBUG) {
			System.out.println("Saving complete!");
		}
		saveToFile = fileName;
		return true;
	}

	public String getUniqueStateName(String oldName) {
		boolean unique;
		int i = 2;
		do {
			unique = true;
			for (State s : states) {
				if (s.getText().equals(oldName + "(" + i + ")")) {
					unique = false;
					i++;
				}
			}
		} while (!unique);
		return oldName + "(" + i + ")";
	}

	private void saveState(State state, RandomAccessFile saveFile)
			throws IOException {
		saveFile.writeBytes("<State>\r\n");
		saveFile.writeBytes("<initial> " + state.isInitialState()
				+ " </initial>\r\n");
		saveFile.writeBytes("<label> " + state.getText() + " </label>\r\n");
		saveFile.writeBytes("<location> " + state.getX() + " " + state.getY()
				+ " </location>\r\n");
		for (ISmachableAction action : state.getActions()) {
			saveFile.writeBytes("<action> " + action.toString()
					+ " </action>\r\n");
		}
		saveFile.writeBytes("</State>\r\n");
	}

	private void saveTransition(Transition transition, RandomAccessFile saveFile)
			throws IOException {
		saveFile.writeBytes("<Transition>\r\n");
		saveFile.writeBytes("<label> " + transition.getLabel()
				+ " </label>\r\n");
		saveFile.writeBytes("<xOffset> " + transition.getXOffset()
				+ " </xOffset>\r\n");
		saveFile.writeBytes("<rootState> "
				+ states.indexOf(transition.getRootState())
				+ " </rootState>\r\n");
		saveFile.writeBytes("<followerState> "
				+ states.indexOf(transition.getFollowerState())
				+ " </followerState>\r\n");
		saveFile.writeBytes("<guard>\r\n" + transition.getGuard().toString()
				+ "\r\n</guard>\r\n");
		saveFile.writeBytes("</Transition>\r\n");
	}

	public static Automat loadAutomat(File file) throws Exception {
		String fileName = file.getAbsolutePath();
		if (MainFrame.DEBUG) {
			System.out.println("Trying to load Automat from: " + fileName);
		}
		Automat automat = new Automat(fileName);
		BufferedReader reader = new BufferedReader(new FileReader(file));
		String line;
		line = reader.readLine();
		if (line != null) {
			if (line.startsWith("<Automat robotName='")) {
				line = line.substring(20, line.lastIndexOf("'"));
				SettingsDialog.setRobotType(line);
			}
		}
		while ((line = reader.readLine()) != null) {
			if (line.equals("<State>")) {
				boolean initial = false;
				String label = "";
				Point location = new Point(0, 0);
				ArrayList<Action> actions = new ArrayList<Action>(15);
				while (line != null
						&& !(line = reader.readLine()).equals("</State>")) {
					String[] splitLine = line.split(" ");
					if (splitLine[0].equals("<action>")) {
						if (!splitLine[1].endsWith("CONTROLLER")) {
							actions.add(new Action(splitLine[1], Integer
									.parseInt(splitLine[2])));
						} else {
							ActionController ac = new ActionController(
									splitLine[1]);
							ac.setOffset(Integer.parseInt(splitLine[2]));
							ac.setMin(Integer.parseInt(splitLine[3]));
							ac.setMax(Integer.parseInt(splitLine[4]));
							ac.setKpr(Integer.parseInt(splitLine[5]));
							ac.setVar(splitLine[6]);
							ac.setCompvalue(Integer.parseInt(splitLine[7]));
							actions.add(ac);
						}
					} else if (splitLine[0].equals("<initial>")) {
						initial = Boolean.parseBoolean(splitLine[1]);
					} else if (splitLine[0].equals("<label>")) {
						for (int i = 1; i < splitLine.length - 1; i++) {
							label += splitLine[i] + " ";
						}
						label = label.trim();
					} else if (splitLine[0].equals("<location>")) {
						location = new Point(Integer.parseInt(splitLine[1]),
								Integer.parseInt(splitLine[2]));
					}
				}
				automat.addState(new State(actions, label, location, initial));
			} else if (line.equals("<Transition>")) {
				State rootState = null;
				State followerState = null;
				String label = "";
				int xOffset = 0;
				BooleanExpression guard = null;
				while (line != null
						&& !(line = reader.readLine()).equals("</Transition>")) {
					String[] splitLine = line.split(" ");
					if (splitLine[0].equals("<rootState>")) {
						rootState = automat.getStates().get(
								Integer.parseInt(splitLine[1]));
					} else if (splitLine[0].equals("<followerState>")) {
						followerState = automat.getStates().get(
								Integer.parseInt(splitLine[1]));
					} else if (splitLine[0].equals("<label>")) {
						label = splitLine[1];
					} else if (splitLine[0].equals("<xOffset>")) {
						xOffset = Integer.parseInt(splitLine[1]);
					} else if (splitLine[0].equals("<guard>")) {
						guard = BooleanExpression.parseExpr(reader);
					}
				}
				rootState.addTransition(new Transition(rootState,
						followerState, label, xOffset, guard));
			}
		}
		reader.close();	
		//automat.setRobot(MainFrame.automat.getRobot());
		if (MainFrame.DEBUG) {
			System.out.println("Loading succeded!");
		}
		automat.robot = MainFrame.robot;
		return automat;
	}
}
