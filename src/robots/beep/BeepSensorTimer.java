package robots.beep;

import java.util.HashSet;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import model.bool.Variable.Operator;
import smachGenerator.ISmachableSensor;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepSensorTimer implements ISmachableSensor {

	private final String name;

	public BeepSensorTimer(String name) {
		this.name = name;
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public HashSet<String> getImports() {
		return new HashSet<>();
	}

	@Override
	public String getCallback() {
		return "";
	}

	@Override
	public String getSubscriberSetup() {
		return "";
	}

	@Override
	public String getValueIdentifier() {
		return "t_" + name;
	}

	@Override
	/**
	 * Hacked: 
	 * Two statements (second: 2 tabs)
	 * Second sets timer to current time!
	 */
	public String getGlobalIdentifier() {
		return getValueIdentifier() + "\n\t\t" + getIdentifierInit();
	}

	@Override
	public String getIdentifierInit() {
		return getValueIdentifier() + " = rospy.get_time()";
	}

	@Override
	public String getTopicType() {
		return std_msgs.Float32._TYPE;
	}

	@Override
	public String getTransitionCondition(Operator op, int compVal) {
		return "rospy.get_time()-" + getValueIdentifier() + op + compVal / 1000;
	}

}
