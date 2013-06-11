package robots.beep;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import view.AbstractStatePanel;
import view.AbstractTransitionPanel;
import model.AbstractRobot;
import model.Action;
import model.State;
import model.Transition;

@XmlRootElement
@XmlAccessorType(XmlAccessType.NONE)
public class BeepRobot extends AbstractRobot {

	@XmlElement(name="sensor")
	List<BeepDevice> sensors = new ArrayList<BeepDevice>();
	
	@XmlElement(name="actuator")
	List<BeepDevice> actuators = new ArrayList<BeepDevice>();
	
	public BeepRobot() {
		sensors.add(new BeepDevice("IR8","top","topTyp","myPack","obj10"));
		sensors.add(new BeepDevice("IR2","top","topTyp","myPack","obj2"));
		
		actuators.add(new BeepDevice("Motor1","top","topTyp","myPack","obj1"));
		actuators.add(new BeepDevice("Motor2","top","topTyp","myPack","obj2"));
	}
	
	public List<BeepDevice> getSensors(){
		return sensors;
	}
	
	public List<BeepDevice> getActuators(){
		return actuators;
	}
	
	@Override
	public boolean connect(String connectTo) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void disconnect() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int getVariableValue(String variable) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void executeActions(ArrayList<Action> actions) throws IOException {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void updateSensors() throws IOException {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int getDesiredAdditionalTimeout() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public AbstractStatePanel getStatePanel(State state) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public AbstractTransitionPanel getTransitionPanel(Transition trans) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int getUnAcknowledgedCmds() {
		// TODO Auto-generated method stub
		return 0;
	}
	
	/**
	 * Stores a given {@link BeepRobot} as a xml file to the given path
	 * @param robot that will be stored
	 * @param file where the xml file will be stored
	 * @return <code>null</code> if no error occurred else an error information. 
	 */
	public static String saveBeepRobot(BeepRobot robot, File file) {
		try {
			JAXBContext context = JAXBContext.newInstance(BeepRobot.class);
			Marshaller m = context.createMarshaller();
			m.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
			file.setWritable(true);
			Path toPath = file.toPath();
			try {
				BufferedWriter newBufferedWriter = Files.newBufferedWriter(
						toPath, Charset.defaultCharset());
				m.marshal(robot, newBufferedWriter);
				newBufferedWriter.flush();
			} catch (IOException ex) {
				return "Can't write File: " + file.getAbsolutePath()
						+ " . No Write Access";
			}
		} catch (JAXBException ex) {
			ex.printStackTrace();
		}
		return null;
	}

	/**
	 * Creates a {@link BeepRobot} from a given xml file.
	 * The expected pattern is:
	 * 	beepRobot(Device*)
	 *  Device(topic, name, objectInMessage, topicType, topicPackage)
	 *  		
	 * @param fileName the xml is stored
	 * @return a {@link BeepRobot}
	 */
	public static BeepRobot loadBeepRobot(String fileName) {
		try {
			if(!fileName.endsWith(".xml")){
				fileName += ".xml";
			}
			File file = new File(fileName);
			if (file.exists()) {
				JAXBContext context = JAXBContext.newInstance(BeepRobot.class);
				Unmarshaller u = context.createUnmarshaller();
				BeepRobot robot = (BeepRobot) u.unmarshal(file);
				return robot;
			} else {
				return null;
			}
		} catch (JAXBException ex) {
			ex.printStackTrace();
		}
		return null;
	}
	

}
