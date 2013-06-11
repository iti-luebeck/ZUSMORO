package robots.beep;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import smachGenerator.ISmachableDevice;

@XmlAccessorType(XmlAccessType.FIELD)

public class BeepDevice implements ISmachableDevice {

	private String topic;
	private String name;
	private String objectInMessage;
	private String topicType;
	private String topicPackage;

	public BeepDevice(String name, String topic, String topicType,
			String topicPackage, String objectInMessage) {
		this.name = name;
		this.topic = topic;
		this.topicType = topicType;
		this.topicPackage = topicPackage;
		this.objectInMessage = objectInMessage;
	}
	
	public BeepDevice(){
		//TODO
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
		if (!(o instanceof BeepDevice)) {
			return false;
		} else {
			BeepDevice s = (BeepDevice) o;
			return (name.equals(s.getName()) || (topic.equals(s.getTopic()) && objectInMessage
					.equals(s.getObejctInMessage())));
		}
	}
}
