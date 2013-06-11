package smachGenerator;


import java.util.HashSet;
import java.util.LinkedList;

import javax.naming.directory.NoSuchAttributeException;

public class SmachableActuators extends LinkedList<ISmachableDevice>{

	
	private static final long serialVersionUID = 4879265984143961788L;

	public ISmachableDevice getActuator(String actuatorName)
			throws NoSuchAttributeException {
		for (ISmachableDevice actuator : this) {
			if (actuator.getName().equals(actuatorName))
				return actuator;
		}
		throw new NoSuchAttributeException("Zugriff auf unbekannten Aktor "
				+ actuatorName + ".");
	}

	public HashSet<String> getPublisherSetups() {
		HashSet<String> pubs = new HashSet<>();
		for (ISmachableDevice actuator : this) {
			pubs.add("pub_"+actuator.getTopic().replace("/", "_")+ " = rospy.Publisher('" + actuator.getTopic() + "', "
					+ actuator.getTopicType() + ")");
		}
		return pubs;
	}

	public HashSet<String> getMsgDeps() {
		HashSet<String> deps = new HashSet<>();
		for (ISmachableDevice actuator : this) {
			deps.add("from " + actuator.getTopicPackage() + " import "
					+ actuator.getTopicType());
		}
		return deps;
	}
	
}
