package robots.beep;

import java.util.HashSet;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import smachGenerator.ISmachableAction;
import smachGenerator.ISmachableActuator;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepMotor implements ISmachableActuator {

	private String name;
	private String topic;
	private final String topicType = "Int8";

	public BeepMotor(String name, String topic) {
		this.name = name;
		this.topic = topic;
	}

	public BeepMotor() {
		name = null;
		topic = null;
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public String getPublisherSetup() {
		String pub = getPublisherName() + " = rospy.Publisher('" + topic
				+ "', " + topicType + ")";
		return pub;
	}

	@Override
	public String[] getPublishMessage(ISmachableAction a) {
		String[] result = new String[3];
		result[0] = name + " = " + topicType + "()";
		result[1] = name + ".data = " + a.getValue();
		result[2] = getPublisherName() + ".publish(" + name + ")";
		return result;
	}

	@Override
	public HashSet<String> getImports() {
		HashSet<String> result = new HashSet<String>();
		result.add("from std_msgs.msg import " + topicType);
		return result;
	}

	@Override
	public String getPublisherName() {
		return "pub_" + name;
	}

}
