package smachGenerator;

import java.rmi.AlreadyBoundException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;

import javax.naming.directory.NoSuchAttributeException;

public class SmachableSensors extends LinkedList<ISmachableSensor> {

	private static final long serialVersionUID = -7460092781520543805L;

	/**
	 * Searches for the sensor and returns the first {@link ISmachableSensor}
	 * instance with this name.
	 * 
	 * @param sensorName
	 * @return first {@link ISmachableSensor} with the specified name or
	 *         <code>null</code> if there is no sensor with this name
	 */
	public ISmachableSensor getSensor(String sensorName) {
		for (ISmachableSensor sensor : this) {
			if (sensor.getName().equals(sensorName))
				return sensor;
		}
		return null;
	}

	/**
	 * Creates a list of Strings. Each representing a callback for a topic. The
	 * returned LinkedList contains all callbacks, that are needed to receive
	 * the data of all {@link ISmachableSensor}s stored in this instance. Every
	 * leaf entry will be stored in the global variable with the same name as
	 * the the name of the {@link ISmachableSensor}.
	 * <p>
	 * Example: the the topic <code>Exampletopic</code> contains two elements
	 * <code>e1, e2</code>. <code>e2</code> also contains two elements
	 * <code>e3, e4</code>. <code>e1, e3, e4</code> will be stored.
	 * <p>
	 * You can access the stored data by writing
	 * <code>global <i>sensorname</i></code> in the function that will use
	 * <code>sensorname</code> and afterwards simply use <code>sensorname</code>.
	 * 
	 * @return {@link LinkedList} with the callbacks for all
	 *         {@link ISmachableSensor} stored in this instance
	 * @throws AlreadyBoundException
	 *             if two {@link ISmachableSensor} have the same name or two
	 *             different sensors are stored at the same leaf in the same
	 *             topic.
	 */
	public LinkedList<String> getCallbacks() throws AlreadyBoundException {
		HashMap<String, String> callbacks = new HashMap<>();
		for (ISmachableSensor sensor : this) {
			String cb;
			if (callbacks.containsKey(sensor.getTopic())) {
				cb = "\tglobal " + sensor.getName() + "\n\t" + sensor.getName()
						+ " = msg." + sensor.getObejctInMessage() + "\n";
				if (callbacks.get(sensor.getTopic()).contains(
						"\t" + sensor.getName() + " = msg.")
						|| callbacks.get(sensor.getTopic()).contains(
								" = msg." + sensor.getObejctInMessage() + "\n")) {
					throw new AlreadyBoundException(
							sensor.getName()
									+ " is not unique. It has the same name as an other sensor or cannot be differentiated by topic and objectInMessage from an other Sensor");
				}
				cb = callbacks.get(sensor.getTopic()) + cb;
			} else {
				cb = "def callback_" + sensor.getTopic().replace("/", "_")
						+ "(msg):\n" + "\tglobal " + sensor.getName() + "\n\t"
						+ sensor.getName() + " = msg."
						+ sensor.getObejctInMessage() + "\n";

			}
			callbacks.put(sensor.getTopic(), cb);
		}
		LinkedList<String> results = new LinkedList<String>();
		for (String cb : callbacks.values()) {
			results.add(cb);
		}
		return results;
	}

	/**
	 * Creates Strings representing the definition of all subscribers that are
	 * needed to receive the data of all {@link ISmachableSensor} stored in this
	 * instance. The name of the callback for a certain topic is "callback_
	 * <code>topic</code> ", where all "/" in <code>tobic</code> will be
	 * replaced by "_".
	 * <p>
	 * Example: the name of the callback function for the topic
	 * <code>abc/def</code> will be <code>callback_abc_def</code>.
	 * 
	 * @return {@link HashSet} of subscriber definitions
	 */
	public HashSet<String> getSubscriberSetups() {
		HashSet<String> subs = new HashSet<>();
		for (ISmachableSensor sensor : this) {
			subs.add("rospy.Subscriber('" + sensor.getTopic() + "', "
					+ sensor.getTopicType() + ", callback_"
					+ sensor.getTopic().replace("/", "_") + ")\n");
		}
		return subs;
	}

	/**
	 * Creates Strings representing all message imports that are needed to
	 * communicate with all {@link ISmachableSensor} in this instance
	 * 
	 * @return {@link HashSet} of all message imports
	 */
	public HashSet<String> getMsgDeps() {
		HashSet<String> deps = new HashSet<>();
		for (ISmachableSensor sensor : this) {
			deps.add("from " + sensor.getTopicPackage() + " import "
					+ sensor.getTopicType());
		}
		return deps;
	}

}
