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

import smachGenerator.SmachableActuators;
import smachGenerator.SmachableSensors;
import view.AbstractStatePanel;
import view.AbstractTransitionPanel;
import model.AbstractRobot;
import model.Action;
import model.State;
import model.Transition;

@XmlRootElement
@XmlAccessorType(XmlAccessType.NONE)
public class BeepRobot extends AbstractRobot {

	@XmlElement(name = "sensor")
	List<BeepDevice> sensors = new ArrayList<BeepDevice>();// Schöner als
															// SmachableSensor
															// aber wird nicht
															// als XML
															// gespeichert

	@XmlElement(name = "actuator")
	List<BeepDevice> actuators = new ArrayList<BeepDevice>();// Schöner als
																// SmachableActuators
																// aber wird
																// nicht als XML
																// gespeichert

	public BeepRobot() {
		// Define default Beep sensors
		sensors.add(new BeepDevice("IR0", "topic/IR0", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("IR1", "topic/IR1", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("IR2", "topic/IR2", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("IR3", "topic/IR3", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("IR4", "topic/IR4", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("IR5", "topic/IR5", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("IR6", "topic/IR6", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("IR7", "topic/IR7", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("UIR0", "topic/UIR0", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("UIR1", "topic/UIR2", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("UIR2", "topic/UIR3", "Int8", "std_msgs.msg",
				"data"));
		sensors.add(new BeepDevice("imu_x", "topic/imu", "Imu",
				"sensor_msgs.msg", "linear_acceleration.x"));
		sensors.add(new BeepDevice("imu_y", "topic/imu", "Imu",
				"sensor_msgs.msg", "linear_acceleration.y"));
		sensors.add(new BeepDevice("imu_z", "topic/imu", "Imu",
				"sensor_msgs.msg", "linear_acceleration.z"));
		sensors.add(new BeepDevice("timer", "topic/Timer", "Int8", "std_msgs.msg",
				"data"));
		
		// Define default Beep actuators
		actuators.add(new BeepDevice("MOTOR1", "topic/motors", "Motors",
				"beep.msg", "links"));
		actuators.add(new BeepDevice("MOTOR2", "topic/motors", "Motors",
				"beep.msg", "rechts"));
		actuators.add(new BeepDevice("LED1", "topic/LED1", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepDevice("LED2", "topic/LED2", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepDevice("LED3", "topic/LED3", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepDevice("LED4", "topic/LED4", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepDevice("LED5", "topic/LED5", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepDevice("LED6", "topic/LED6", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepDevice("LED7", "topic/LED7", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepDevice("LED0", "topic/LED0", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepDevice("LED8", "topic/LED8", "Int8",
				"std_msgs.msg", "data"));
		BeepDevice beep = new BeepDevice("BEEP", "topic/beep", "Int8",
				"std_msgs.msg", "data");
		actuators.add(beep);
	}

	public SmachableSensors getSensors() {
		SmachableSensors sen = new SmachableSensors();
		sen.addAll(sensors);
		return sen;
	}

	public SmachableActuators getActuators() {
		SmachableActuators act = new SmachableActuators();
		act.addAll(actuators);
		return act;
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
		return new BeepStatePanel(state);
	}

	@Override
	public AbstractTransitionPanel getTransitionPanel(Transition trans) {
		// TODO Return/create beep transition
		return new BeepTransitionPanel(trans);
	}

	@Override
	public int getUnAcknowledgedCmds() {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * Stores a given {@link BeepRobot} as a xml file to the given path
	 * 
	 * @param robot
	 *            that will be stored
	 * @param file
	 *            where the xml file will be stored
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
	 * Creates a {@link BeepRobot} from a given xml file. The expected pattern
	 * is: beepRobot(Device*) Device(topic, name, objectInMessage, topicType,
	 * topicPackage)
	 * 
	 * @param fileName
	 *            the xml is stored
	 * @return a {@link BeepRobot}
	 */
	public static BeepRobot loadBeepRobot(String fileName) {
		try {
			if (!fileName.endsWith(".xml")) {
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

	@Override
	public String getRobotName() {
		return "Beep Robot";
	}
	

}
