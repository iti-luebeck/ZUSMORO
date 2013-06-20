package robots.beep;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import smachGenerator.ISmachableActuator;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepActuator implements ISmachableActuator {

	private String name;
	private String topic;
	private String objectInMessage;
	private String topicType;
	private String topicPackage;

	public BeepActuator(String name, String topic, String topicType,
			String topicPackage, String objectInMessage) {
		this.name = name;
		this.topic = topic;
		this.objectInMessage = objectInMessage;
		this.topicType = topicType;
		this.topicPackage = topicPackage;
	}

	public BeepActuator() {
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
		if (!(o instanceof BeepActuator)) {
			return false;
		} else {
			BeepActuator s = (BeepActuator) o;
			return (name.equals(s.getName()) || (topic.equals(s.getTopic()) && objectInMessage
					.equals(s.getObejctInMessage())));
		}
	}
	
}