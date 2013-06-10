package smachGenerator;

public interface ISmachableSensor {
	
	public abstract String getTopic();
	public abstract String getName();
	public abstract String getObejctInMessage();
	public String getTopicType();
	public String getTopicImport();
	
}
