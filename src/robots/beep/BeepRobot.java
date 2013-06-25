package robots.beep;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
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

import ch.ethz.ssh2.Connection;
import ch.ethz.ssh2.SCPClient;
import ch.ethz.ssh2.Session;
import ch.ethz.ssh2.StreamGobbler;

import smachGenerator.SmachableActuators;
import smachGenerator.SmachableSensors;
import view.AbstractStatePanel;
import view.AbstractTransitionPanel;
import view.MainFrame;
import model.AbstractRobot;
import model.Action;
import model.State;
import model.Transition;

@XmlRootElement
@XmlAccessorType(XmlAccessType.NONE)
public class BeepRobot extends AbstractRobot {

	@XmlElement(name = "IR_Sensor")
	List<BeepIRSensor> sensorsIR = new ArrayList<BeepIRSensor>();

	@XmlElement(name = "Color_Sensor")
	List<BeepColorSensor> sensorsCol = new ArrayList<BeepColorSensor>();

	@XmlElement(name = "actuator")
	List<BeepActuator> actuators = new ArrayList<BeepActuator>();

	/**
	 * true, if connected to Beep <=> piIn/piOut/sess will be != null
	 */
	boolean connected;
	/**
	 * IP or url of the robot that was connected to last time or that is
	 * currently used
	 */
	String beepIP;

	/**
	 * ssh2 Connection to the robot
	 */
	Connection conn;
	/**
	 * ssh2 Session that can be interacted with by giving commands to piIn and
	 * reading output from piOut
	 */
	Session sess;
	/**
	 * input for commandline statements to beep of the currend session (sess)
	 */
	PrintWriter piIn;
	/**
	 * output of the commandline of the current session (sess)
	 */
	BufferedReader piOut;

	/**
	 * Directory on beep for all smach automates
	 */
	String piDirAutomat;
	/**
	 * Name (including file ending '.py') of the automate. it is used to store
	 * and start the automat
	 */
	String automatFileName;

	/**
	 * Creates a new {@link BeepRobot} instance with default sensors, actuators
	 * and configurations.
	 */
	public BeepRobot() {
		connected = false;

		// Define default Beep sensors
		sensorsIR.add(new BeepIRSensor("IR0", "topic/IR0", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR1", "topic/IR1", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR2", "topic/IR2", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR3", "topic/IR3", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR4", "topic/IR4", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR5", "topic/IR5", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR6", "topic/IR6", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR7", "topic/IR7", "Int8",
				"std_msgs.msg", "data"));
		sensorsCol.add(new BeepColorSensor("UIR0", "topic/UIR0", "Int8",
				"std_msgs.msg", "data"));
		sensorsCol.add(new BeepColorSensor("UIR1", "topic/UIR2", "Int8",
				"std_msgs.msg", "data"));
		sensorsCol.add(new BeepColorSensor("UIR2", "topic/UIR3", "Int8",
				"std_msgs.msg", "data"));

		// Define default Beep actuators
		// actuators.add(new BeepActuator("MOTOR1", "topic/motors", "Motors",
		// "beep.msg", "links"));
		// actuators.add(new BeepActuator("MOTOR2", "topic/motors", "Motors",
		// "beep.msg", "rechts"));
		actuators.add(new BeepActuator("LED1", "topic/LED1", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED2", "topic/LED2", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED3", "topic/LED3", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED4", "topic/LED4", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED5", "topic/LED5", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED6", "topic/LED6", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED7", "topic/LED7", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED0", "topic/LED0", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED8", "topic/LED8", "Int8",
				"std_msgs.msg", "data"));
		BeepActuator beep = new BeepActuator("BEEP", "topic/beep", "Int8",
				"std_msgs.msg", "data");
		actuators.add(beep);

		beepIP = "141.83.158.207";
		piDirAutomat = "/home/pi/Beep/Software/catkin_ws/src/beep_imu";
		automatFileName = "test.py";
	}

	/**
	 * returns a {@link SmachableSensors} instance containing all sensors of
	 * this instance.
	 * 
	 * @return {@link SmachableActuators} of this instance
	 */
	public SmachableSensors getSensors() {
		SmachableSensors sen = new SmachableSensors();
		sen.addAll(sensorsIR);
		sen.addAll(sensorsCol);
		return sen;
	}

	/**
	 * returns a {@link SmachableActuators} instance containing all actuators of
	 * this instance.
	 * 
	 * @return {@link SmachableActuators} of this instance
	 */
	public SmachableActuators getActuators() {
		SmachableActuators act = new SmachableActuators();
		act.addAll(actuators);
		return act;
	}

	@Override
	public boolean connect(String connectTo) {
		if (connectTo == null) {
			connectTo = beepIP;
		}
		beepIP = connectTo;
		conn = new Connection(connectTo);
		try {
			// connect and authorize
			conn.connect();
			conn.authenticateWithPassword("pi", "beep");

			// start a compatible shell
			sess = conn.openSession();
			sess.requestDumbPTY();
			sess.startShell();

			piIn = new PrintWriter(sess.getStdin(), true);

			// echo to recognize when shell is ready to use
			piIn.println("echo 'ready to start'");

			// print output and block until shell is "ready to start"
			InputStream stdout = new StreamGobbler(sess.getStdout());
			piOut = new BufferedReader(new InputStreamReader(stdout));
			String line = piOut.readLine();
			while (line != null) {
				System.out.println(line);
				if (line.equals("ready to start")) {
					break;
				}
				line = piOut.readLine();
			}

			connected = true;
			return true;
		} catch (Exception e) {
			e.printStackTrace();
			MainFrame
					.showErrInfo(
							"<html>Es konnte keine Verbindung mit "
									+ connectTo
									+ "hergestellt werden.<br>Bitte überprüfe die IP des Roboters und ob dieser über eine aktive W-Lan Verbindung verfügt!",
							"Verbindung fehlgeschlagen");
		}
		return false;
	}

	@Override
	public void disconnect() {
		connected = false;
		piIn.close();
		piIn = null;
		try {
			piOut.close();
		} catch (IOException e) {
		}
		piOut = null;
		sess.close();
		sess = null;
		conn.close();
		conn = null;
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
		if (connected) {
			try {
				piIn.println("pkill -f 'python " + piDirAutomat + "/"
						+ automatFileName + "'");
				piOut.readLine(); // command
			} catch (IOException e) {
				e.printStackTrace();
			}

		}

	}

	@Override
	public boolean transmit() {
		SCPClient client = new SCPClient(conn);
		try {
			client.put(automatFileName, piDirAutomat);
			return true;
		} catch (IOException e) {
			e.printStackTrace();
			MainFrame.showErrInfo(
					"Die generierte Datei konnte nicht übertragen werden.",
					"Übertragungsfehler");
			return false;
		}
	}

	@Override
	public void play() {
		if (connected) {
			piIn.println("mkdir -p ~/log");
			try {
				piOut.readLine(); // command
			} catch (IOException e) {
				e.printStackTrace();
			}
			if (!isRoscoreRunning()) {
				startNewRoscore();
			}
			startAutomatOnPi();
		}
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
		return new BeepTransitionPanel(trans);
	}

	@Override
	public int getUnAcknowledgedCmds() {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * checks, if there is a running roscore process on the Beep
	 * 
	 * @return true, if roscore is running
	 */
	private boolean isRoscoreRunning() {
		if (connected) {
			piIn.println("ps -ef | grep 'roscore' | grep -v 'grep' | wc -l");
			try {
				piOut.readLine(); // command
				if (piOut.readLine().equals("1")) { // number of roscore
													// processes running
					return true;
				}
			} catch (IOException e) {
				e.printStackTrace();
			}

		}
		return false;
	}

	/**
	 * Terminates every running roscore process and starts a new roscore.
	 * Returns, when roscore is running and ready to use.
	 */
	private void startNewRoscore() {
		if (connected) {
			piIn.println("pkill roscore");
			piIn.println("nohup roscore 2> ~/log/roscore-err.log 1> ~/log/roscore-out.log &");
			String line;
			try {
				line = piOut.readLine();
				while (line != null) {
					if (line.equals("started core service [/rosout]")) {// TODO
																		// how
																		// to
																		// check
																		// if
																		// roscore
																		// is
																		// ready?
						return;
					}
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Stops all already running Smach automates by calling {@link stop()}. Then
	 * starts the Smach automate <code>automateFileName</code> on Beep. Will
	 * store output- and error stream to log files in <code>~/log</code> on Beep
	 * 
	 * Be sure that there is a roscore running before calling this method.
	 */
	private void startAutomatOnPi() {
		stop(); // Stop old automate
		piIn.println("nohup rosrun beep_imu ir_distance.py 2> ~/log/ir_distance-err.log 1> ~/log/ir_distance-out.log &");
		try {
			piOut.readLine(); // command
			piOut.readLine(); // Process ID
		} catch (IOException e) {
			e.printStackTrace();
		}
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

	@Override
	public void debug() {
		// TODO Auto-generated method stub
		
	}

}
