package smachGenerator;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.rmi.AlreadyBoundException;
import java.util.ArrayList;
import java.util.LinkedList;

import javax.naming.directory.NoSuchAttributeException;

import model.bool.True;

/**
 * Creates the python Source-Code for a Smach state machine for ROS (Robot
 * Operation System) It can be stored to a file
 * 
 * @author Marvin Lindenberg
 * 
 */
public class SmachAutomat {

	private ArrayList<? extends ISmachableState> states = new ArrayList<>();
	private LinkedList<String> smachStates = new LinkedList<>();
	private int initialStateIndex;
	private SmachableSensors sensors;

	/**
	 * Constructs a Smach state machine form the given states
	 * 
	 * @param states
	 *            used to create the state machine
	 * @throws NoSuchAttributeException
	 *             if sensors are used that are not specified in the
	 *             {@link ISmachableSensors} object.
	 */
	public SmachAutomat(ArrayList<? extends ISmachableState> states,
			SmachableSensors sensors) throws NoSuchAttributeException {
		this.states = states;
		this.sensors = sensors;
		for (ISmachableState s : states) {
			if (s.isInitialState()) {
				initialStateIndex = states.indexOf(s);
			}
			smachStates.add(getSmachState(s));
		}
		try {
			for (String cb : sensors.getCallbacks())
				System.out.println(cb);
		} catch (AlreadyBoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private String getImports() {
		String imports = "#!/usr/bin/env python\n\n";
		imports += "import roslib; roslib.load_manifest('xxxxxx')\n";// TODO
																		// Manifest
																		// festlegen
		imports += "import rospy\n";
		imports += "import smach\n";
		imports += "import smach_ros\n\n";
		// messege dependencies
		for (String dep : sensors.getMsgDeps()) {
			imports += dep + "\n";
		}
		return imports;
	}

	/**
	 * Rewrites the ISmachableState s to a Smach state
	 * 
	 * @param s
	 *            ISmachableState that will be converted to a smach state
	 * @throws NoSuchAttributeException
	 */
	private String getSmachState(ISmachableState s)
			throws NoSuchAttributeException {
		String state = "class " + s.getText().replace(" ", "")
				+ "(smach.State):\n";
		// create __init__ function for state
		state += "\tdef __init__(self):\n";
		state += "\t\tsmach.State.__init__(self, outcomes=[";
		// get transitions for __init__ function
		// a state
		if (s.getTransitions().size() > 0) {
			for (ISmachableTransition t : s.getTransitions()) {
				state += "'" + t.getLabel() + "',";
			}
			state = state.substring(0, state.length() - 1) + "])\n\n";
		} else {
			state += "])\n\n";
		}

		// create execute function for the state
		state += "\tdef execute(self, userdata):\n";
		state += "\t\trospy.loginfo('Executing state " + s.getText() + "')";
		//TODO declare all (needed) variables global
		// TODO publish all actions to correct topics
		for (ISmachableAction a : s.getActions()) {
			state += "\n\t\t" + a.getKey() + " " + a.getValue();
		}

		// check for transition
		state += "\n\t\twhile True:\n";
		for (ISmachableTransition t : s.getTransitions()) {
			if (t.getSmachableGuard() != null) {
				ISmachableGuard guard = t.getSmachableGuard();
				if (t.getSmachableGuard() instanceof True) {
					state += "\t\t\treturn '" + t.getLabel() + "'\n";
					break;
				}
				if (guard.getSensorNames().size() > 0) {
					state += "\t\t\tif(";
					for (int i = 0; i < guard.getSensorNames().size(); i++) {
						if (sensors.getSensorTopic(guard.getSensorNames()
								.get(i)) != "") {
							state += guard.getSensorNames().get(i)
									+ guard.getOperators().get(i)
									+ guard.getCompValues().get(i) + " and ";
						}
					}
					state = state.substring(0, state.length() - 5) + "):\n";
				}
				state += "\t\t\t\treturn '" + t.getLabel() + "'\n";
			}
		}
		state += "\t\t\ttime.sleep(0.01)\n";
		return state;
	}

	private String getSmachStateMachine(String sm) {
		// prepare all states to add them to the Smach state machine
		ArrayList<String> stateToAdd = new ArrayList<>();
		for (int i = 0; i < states.size(); i++) {
			String state = "smach.StateMachine.add('"
					+ states.get(i).getText().replace(" ", "") + "', "
					+ states.get(i).getText().replace(" ", "")
					+ "(), transitions={";
			for (ISmachableTransition t : states.get(i).getTransitions()) {
				state += "'" + t.getLabel() + "':'"
						+ t.getFollowerState().getText().replace(" ", "")
						+ "',";
			}
			if (states.get(i).getTransitions().size() > 0) {
				state = state.substring(0, state.length() - 1) + "})";
			} else {
				state += "})";
			}
			stateToAdd.add(state);
		}
		// create state machine and add states in correct order
		String stateMachine = "\t" + sm
				+ " = smach.StateMachine(outcomes=[])\n";
		stateMachine += "\twith " + sm + ":\n";
		stateMachine += "\t\t" + stateToAdd.get(initialStateIndex) + "\n";
		for (int i = 0; i < stateToAdd.size(); i++) {
			if (i != initialStateIndex) {
				stateMachine += "\t\t" + stateToAdd.get(i) + "\n";
			}
		}
		return stateMachine;
	}

	private String getMainMethode() {
		String main = "if __name__ == '__main__':\n";
		main += "\trospy.init_node('zusmoro_state_machine')\n";
		for (String subSetup : sensors.getSubscriberSetups()) {
			main += "\t" + subSetup;
		}
		main += getSmachStateMachine("sm");
		main += "\tsm.execute()";
		return main;
	}

	public boolean saveToFile(String fileName) throws NoSuchAttributeException,
			AlreadyBoundException {
		String pythonNode = getImports() + "\n\n";
		for(ISmachableSensor sensor : sensors){
			pythonNode += sensor.getName()+"\n";
		}
		pythonNode+="\n\n";
		
		for (int i = 0; i < smachStates.size(); i++) {
			pythonNode += smachStates.get(i) + "\n";
		}
		for (String cb : sensors.getCallbacks()) {
			pythonNode += cb + "\n";
		}
		pythonNode += getMainMethode();
		try {
			PrintWriter out = new PrintWriter(fileName + ".py");
			out.print(pythonNode);
			out.close();
			File py = new File(fileName + ".py");
			py.setExecutable(true);
			return true;
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return false;
		}
	}

}
