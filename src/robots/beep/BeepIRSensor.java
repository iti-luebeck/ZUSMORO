package robots.beep;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import RosCommunication.ISubscriberInfo;

import model.bool.Variable.Operator;

import smachGenerator.ISmachableSensor;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepIRSensor implements ISmachableSensor, ISubscriberInfo {

	private String name;
	private String topic;
	private final int irIndex;
	private final String topicType = std_msgs.Int32._TYPE;

	public BeepIRSensor(String name, String topic, int irIndex) {
		this.name = name;
		this.topic = topic;
		this.irIndex = irIndex;
	}

	public BeepIRSensor() {
		name = null;
		topic = null;
		irIndex = 0;
	}

	@Override
	public String getTopic() {
		return topic;
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public String getTransitionCondition(Operator op, int compVal) {
		return getValueIdentifier() + op + compVal;
	}

	@Override
	public String getImports() {
		String temp[] = topicType.split("/");
		String res = "";
		res = "from " + temp[0] + ".msg import " + temp[1];
		return res;
	}

	@Override
	public String getCallback() {
		String res = "";
		res += "def ir_cb(msg):\n";
		res += "\tglobal ir\n";
		res += "\tir = msg.ir\n";
		return res;
	}

	@Override
	public String getSubscriberSetup() {
		String res = "";
		res = "rospy.Subscriber('" + topic + "', " + topicType.split("/")[1]
				+ ", ir_cb)\n";
		return res;
	}

	@Override
	public String getValueIdentifier() {
		return "ir[" + irIndex + "]";
	}

	@Override
	public String getTopicType() {
		return topicType;
	}

	@Override
	public String getIdentifierInit() {
		return "ir = array([0,0,0,0,0,0,0,0])";
	}

}
