package robots.beep;

import java.awt.Color;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import RosCommunication.ISubscriberInfo;

import model.bool.Variable.Operator;
import smachGenerator.ISmachableSensor;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepColorSensor implements ISmachableSensor, ISubscriberInfo {

	private String name;
	private String topic;
	private final String topicType = "beep_msgs.msg/Color_sensors";
	private final int sensorIndex;

	public BeepColorSensor(String name, String topic, int sensorIndex) {
		this.name = name;
		this.topic = topic;
		this.sensorIndex = sensorIndex;
	}

	public BeepColorSensor() {
		name = null;
		topic = null;
		sensorIndex = 0;
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
	public String getTopicType() {
		return topicType;
	}

	@Override
	public String getTransitionCondition(Operator op, int compVal) {
		Color col = new Color(compVal);
		float[] hsbCol = Color.RGBtoHSB(col.getRed(), col.getGreen(),
				col.getBlue(), null);

		return getValueIdentifier() + ">" + (hsbCol[0] - 0.1 + 1) % 1 + " and " + getValueIdentifier() + "<"
				+ (hsbCol[0] + 0.1) % 1;
	}

	@Override
	public String getImports() {
		String res = "";
		res += "from " + topicType.split("/")[0] + ".msg import "
				+ topicType.split("/")[1]+"\n";
		res += "import colorsys\n";
		return res;
	}

	@Override
	public String getCallback() {
		String res = "";
		res += "def color_cb(msg):\n";
		res += "\tglobal colorSensor\n";
		res += "\tfor (i, sensor) in enumerate(msg.sensors)\n";
		res += "\t\tgroundColor[i] = colorsys.rgb_to_hsv(sensor.r, sensor.g, sensor.b)[0]\n";				
		return res;
	}

	@Override
	public String getSubscriberSetup() {
		String res = "";
		res += "rospy.Subscriber('" + topic + "', " + topicType.split("/")[1]
				+ ", color_cb)\n";
		return res;
	}

	@Override
	public String getValueIdentifier() {
		return "colorSensor[" + sensorIndex + "]";
	}
	
	public String getIdentifierInit(){
		return "colorSensor = array([0,0,0])";
	}

}
