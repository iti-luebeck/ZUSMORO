package robots.beep;

import java.awt.Color;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import model.bool.Variable.Operator;
import smachGenerator.ISmachableSensor;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepColorSensor implements ISmachableSensor {

	private String name;
	private String topic;
	private String objectInMessage;
	private String topicType;
	private String topicPackage;

	public BeepColorSensor(String name, String topic, String topicType,
			String topicPackage, String objectInMessage) {
		this.name = name;
		this.topic = topic;
		this.objectInMessage = objectInMessage;
		this.topicType = topicType;
		this.topicPackage = topicPackage;
	}

	public BeepColorSensor() {
		name = null;
		topic = null;
		objectInMessage = null;
		topicType = null;
		topicPackage = null;
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
	public String getObejctInMessage() {
		return objectInMessage;
	}

	public String getTopicType() {
		return topicType;
	}

	public String getTopicPackage() {
		return topicPackage;
	}

	public boolean equals(Object o) {
		if (!(o instanceof BeepIRSensor)) {
			return false;
		} else {
			BeepIRSensor s = (BeepIRSensor) o;
			return (name.equals(s.getName()) || (topic.equals(s.getTopic()) && objectInMessage
					.equals(s.getObejctInMessage())));
		}
	}

	@Override
	public String getTransitionCondition(Operator op, int compVal) {
		Color col = new Color(compVal);
		float[] hsbCol =  Color.RGBtoHSB(col.getRed(), col.getGreen(), col.getBlue(), null);
		
		return name + ">" + (hsbCol[0]-0.1+1)%1 + " and " + name + "<" + (hsbCol[0]+0.1)%1;
	}

}
