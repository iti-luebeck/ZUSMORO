package robots.beep;

import java.awt.Color;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import smachGenerator.ISmachableAction;
import smachGenerator.ISmachableActuator;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepRgbLed implements ISmachableActuator {

	private String name;
	private String topic;
	private int ledIndex;

	public BeepRgbLed(String name, String topic, int ledIndex) {
		this.name = name;
		this.topic = topic;
		this.ledIndex = ledIndex;
	}

	public BeepRgbLed() {
		name = null;
		topic = null;
		ledIndex = 0;
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public String getPublisherSetup() {
		return "pub_led = rospy.Publisher('" + topic + "', Led)";
	}

	@Override
	public String getPublisherName() {
		return "pub_led";
	}

	@Override
	public String[] getPublishMessage(ISmachableAction a) {
		String[] res = new String[10];
		Color c = new Color(a.getValue());
		String col = "c" + ledIndex;
		String led = "led" + ledIndex;
		
		res[0] = col + " = Color()";
		res[1] = col + ".r = " + c.getRed();
		res[2] = col + ".g = " + c.getGreen();
		res[3] = col + ".b = " + c.getBlue();
		res[4] = led + " = Led()";
		res[5] = led + ".header.frame_id = 'led'";
		res[6] = led + ".header.stamp = rospy.get_rostime()";
		res[7] = led + ".col = " + col;
		res[8] = led + ".led = " + ledIndex;
		res[9] = getPublisherName() + ".publish(" + led + ")";
		
		return res;
	}

	@Override
	public String[] getImports() {
		String[] res = new String[2];
		res[0] = "from beep_msgs.msg import Led";
		res[1] = "from beep_msgs.msg import Color";
		return res;
	}

}
