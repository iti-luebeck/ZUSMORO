package smachGenerator;


public interface ISmachableActuator {
	
	
//	public abstract String getTopic();
	public abstract String getName();
	public abstract String getPublisherSetup();
	public abstract String getPublisherName();
	public abstract String[] getPublishMessage(ISmachableAction a);
	public abstract String[] getImports();
	//	public abstract String getObejctInMessage();
//	public abstract String getTopicType();
	
}
