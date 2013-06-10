package smachGenerator;

import java.rmi.AlreadyBoundException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;

import javax.naming.directory.NoSuchAttributeException;

public class SmachableSensors extends LinkedList<ISmachableSensor> {

	private static final long serialVersionUID = -7460092781520543805L;

	public String getSensorTopic(String sensorName)
			throws NoSuchAttributeException {
		for (ISmachableSensor sensor : this) {
			if (sensor.getName().equals(sensorName))
				return sensor.getTopic();
		}
		throw new NoSuchAttributeException("Zugriff auf unbekannten Sensor "
				+ sensorName + ".");
	}

	public String getCallback(String topic) {
		String cb = "def callback_" + topic.replace("/", "_") + "(msg)\n";
		for (ISmachableSensor sensor : this) {
			if (sensor.getTopic().equals(topic)) {
				cb += "\t" + sensor.getName() + " = msg."
						+ sensor.getObejctInMessage() + "\n";
			}
		}
		return cb;
	}

	public LinkedList<String> getCallbacks() throws AlreadyBoundException {
		HashMap<String, String> callbacks = new HashMap<>();
		for (ISmachableSensor sensor : this) {
			String cb;
			if (callbacks.containsKey(sensor.getTopic())) {
				cb = "\t" + sensor.getName() + " = msg."
						+ sensor.getObejctInMessage() + "\n";
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
						+ "(msg):\n\t" + sensor.getName() + " = msg."
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

	public HashSet<String> getSubscriberSetups() {
		HashSet<String> subs = new HashSet<>();
		for (ISmachableSensor sensor : this) {
			subs.add("rospy.Subscriber('" + sensor.getTopic() + "', "
					+ sensor.getTopicType() + ", callback_"
					+ sensor.getTopic().replace("/", "_") + ")\n");
		}
		return subs;
	}

	public HashSet<String> getMsgDeps() {
		HashSet<String> deps = new HashSet<>();
		for (ISmachableSensor sensor : this) {
			deps.add("from " + sensor.getTopicImport() + " import "
					+ sensor.getTopicType());
		}
		return deps;
	}


}
