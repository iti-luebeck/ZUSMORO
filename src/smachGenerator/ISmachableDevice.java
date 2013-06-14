package smachGenerator;

import model.bool.Variable.Operator;

public interface ISmachableDevice {
	
	public abstract String getTopic();
	public abstract String getName();
	public abstract String getObejctInMessage();
	public abstract String getTopicType();
	public abstract String getTopicPackage();
	
}
