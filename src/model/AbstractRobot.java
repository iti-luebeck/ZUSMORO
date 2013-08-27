package model;

import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JPanel;
import javax.xml.bind.annotation.XmlSeeAlso;
import robots.beep.BeepRobot;

import view.AbstractStatePanel;
import view.AbstractTransitionPanel;

@XmlSeeAlso({BeepRobot.class})
public abstract class AbstractRobot {
	public AbstractRobot() {
		//nothing
	}
	public abstract String getLastConnectedTo();
	public abstract boolean connect(String connectTo);
	public abstract void disconnect();
	public abstract int getVariableValue(String variable);
	public abstract void executeActions(ArrayList<Action> actions) throws IOException;
	public abstract void updateSensors() throws IOException;
	public abstract void stop();
	public abstract boolean transmit();
	public abstract void play();
	public abstract void debug();
	public abstract int getDesiredAdditionalTimeout();
	public abstract AbstractStatePanel getStatePanel(State state);
	public abstract AbstractTransitionPanel getTransitionPanel(Transition trans);
	public abstract int getUnAcknowledgedCmds();
	public abstract AbstractSettingPanel getSettingsPanel();
	
	public abstract String getRobotName();
}
