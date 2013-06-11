package robots.beep;

import smachGenerator.ISmachableDevice;

public class BeepDevice implements ISmachableDevice {

	private String topic;
	private String name;
	private String objectInMessage;
	private String topicType;
	private String topicImport;

	public BeepDevice(String name, String topic, String topicType,
			String topicImport, String objectInMessage) {
		this.name = name;
		this.topic = topic;
		this.topicType = topicType;
		this.topicImport = topicImport;
		this.objectInMessage = objectInMessage;
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
		return topicImport;
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
