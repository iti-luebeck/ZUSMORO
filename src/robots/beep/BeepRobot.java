package robots.beep;

import java.awt.Color;
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
import java.util.LinkedList;
import java.util.List;

import javax.naming.directory.NoSuchAttributeException;
import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import RosCommunication.ISubscriberInfo;
import RosCommunication.RosCommunicator;

import ch.ethz.ssh2.Connection;
import ch.ethz.ssh2.SCPClient;
import ch.ethz.ssh2.Session;
import ch.ethz.ssh2.StreamGobbler;
import smachGenerator.ISmachableSensor;
import smachGenerator.SmachAutomat;
import smachGenerator.SmachableActuators;
import smachGenerator.SmachableSensors;
import view.AbstractStatePanel;
import view.AbstractTransitionPanel;
import view.MainFrame;
import model.AbstractRobot;
import model.AbstractSettingPanel;
import model.Action;
import model.State;
import model.Transition;

@XmlRootElement
@XmlAccessorType(XmlAccessType.NONE)
public class BeepRobot extends AbstractRobot {

	@XmlElement(name = "IR_Sensor")
	List<BeepSensorIR> sensorsIR = new ArrayList<BeepSensorIR>();

	@XmlElement(name = "Color_Sensor")
	List<BeepSensorColor> sensorsCol = new ArrayList<BeepSensorColor>();

	SmachableSensors smachableSensors;

	@XmlElement(name = "Motors")
	List<BeepMotor> motors = new ArrayList<BeepMotor>();

	@XmlElement(name = "rgbLEDs")
	List<BeepRgbLed> rgbLEDs = new ArrayList<BeepRgbLed>();

	@XmlElement(name = "TIMER")
	BeepSensorTimer beepTimer;
	/**
	 * true, if connected to Beep <=> piIn/piOut/sess will be != null
	 */
	boolean connected;

	/**
	 * IP or url of the robot that was connected to last time or that is
	 * currently used
	 */
	@XmlElement(name = "beepIP")
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
	@XmlElement(name = "piDirAutomat")
	String piDirAutomat;
	/**
	 * Name (including file ending '.py') of the automate. it is used to store
	 * and start the automat
	 */
	@XmlElement(name = "automatFileName")
	String automatFileName;

	/**
	 * RosJava component to receive sensor data for debug_view
	 */
	RosCommunicator rosComm;

	/**
	 * Panel to view Debuginformation
	 */
	BeepDebugView debugView;

	/**
	 * Creates a new {@link BeepRobot} instance with default sensors, actuators
	 * and configurations.
	 */
	public BeepRobot() {
		// TODO Read Robot XML

		connected = false;

		setDefaultConfig();

		saveBeepRobot(this, new File("BeepDefault.xml"));

		debugView = new BeepDebugView(this);
	}

	/**
	 * Define default Beep sensors and actuators. Also defines Default IP,
	 * directory for Automates on beep and automatFileName
	 */
	private void setDefaultConfig() {
		// DO NOT CHANGE THE MANES OF THE SENSORS AND ACTUATORS!
		sensorsIR.add(new BeepSensorIR("IR0", "/IR_filtered", 0));
		sensorsIR.add(new BeepSensorIR("IR1", "/IR_filtered", 1));
		sensorsIR.add(new BeepSensorIR("IR2", "/IR_filtered", 2));
		sensorsIR.add(new BeepSensorIR("IR3", "/IR_filtered", 3));
		sensorsIR.add(new BeepSensorIR("IR4", "/IR_filtered", 4));
		sensorsIR.add(new BeepSensorIR("IR5", "/IR_filtered", 5));
		sensorsIR.add(new BeepSensorIR("IR6", "/IR_filtered", 6));
		sensorsIR.add(new BeepSensorIR("IR7", "/IR_filtered", 7));
		sensorsCol.add(new BeepSensorColor("UIR0", "/ground_colors", 0));
		sensorsCol.add(new BeepSensorColor("UIR1", "/ground_colors", 1));
		sensorsCol.add(new BeepSensorColor("UIR2", "/ground_colors", 2));
		beepTimer = new BeepSensorTimer("timer");

		smachableSensors = new SmachableSensors();
		smachableSensors.addAll(sensorsIR);
		smachableSensors.addAll(sensorsCol);
		smachableSensors.add(beepTimer);

		// Define default Beep actuators
		motors.add(new BeepMotor("MOTOR1", "/motor_l"));
		motors.add(new BeepMotor("MOTOR2", "/motor_r"));

		rgbLEDs.add(new BeepRgbLed("LED0", "/leds", 0));
		rgbLEDs.add(new BeepRgbLed("LED1", "/leds", 1));
		rgbLEDs.add(new BeepRgbLed("LED2", "/leds", 2));
		rgbLEDs.add(new BeepRgbLed("LED3", "/leds", 3));
		rgbLEDs.add(new BeepRgbLed("LED4", "/leds", 4));
		rgbLEDs.add(new BeepRgbLed("LED5", "/leds", 5));
		rgbLEDs.add(new BeepRgbLed("LED6", "/leds", 6));
		rgbLEDs.add(new BeepRgbLed("LED7", "/leds", 7));

		motors.add(new BeepMotor("BEEP", "/beep"));// TODO!!

		beepIP = "141.83.158.160"; // "141.83.158.207";
		piDirAutomat = "$HOME/catkin_ws/src/zusmoro_state_machine";
		automatFileName = "Automat.py";
	}

	public void changeConfig() {
		JFileChooser fChooser = new JFileChooser(".");
		fChooser.showDialog(MainFrame.mainFrame, "Config laden");
		File config = fChooser.getSelectedFile();
		MainFrame.robot = BeepRobot.loadBeepRobot(config.getAbsolutePath());
		if (connected) {
			MainFrame.robot.connect(null);
		}
	}

	/**
	 * returns a {@link SmachableSensors} instance containing all sensors of
	 * this instance.
	 * 
	 * @return {@link SmachableActuators} of this instance
	 */
	public SmachableSensors getSensors() {
		return smachableSensors;
	}

	/**
	 * returns a {@link SmachableActuators} instance containing all actuators of
	 * this instance.
	 * 
	 * @return {@link SmachableActuators} of this instance
	 */
	public SmachableActuators getActuators() {
		SmachableActuators act = new SmachableActuators();
		act.addAll(motors);
		act.addAll(rgbLEDs);
		return act;
	}

	@Override
	public boolean connect(String connectTo) {
		if (connectTo == null) {
			connectTo = beepIP;
		}
		connectTo = JOptionPane.showInputDialog(MainFrame.mainFrame,
				"Bitte die IP-Adresse des Beeps angeben.", connectTo);
		if (connectTo == null) {
			SmachAutomat sA;
			try {
				sA = new SmachAutomat(MainFrame.automat.getStates(),
						smachableSensors, getActuators(),
						"zusmoro_state_machine");
				File file = new File(automatFileName);
				sA.saveToFile(file);
			} catch (NoSuchAttributeException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return false;
		}
		beepIP = connectTo;
		conn = new Connection(connectTo);
		try {
			// connect and authorize
			conn.connect();

			// try to authenticate with private key
			// TODO allow password protected private keys, user selection of key file
			conn.authenticateWithPublicKey("pi", new File(System.getProperty("user.home")+"/.ssh/id_rsa"), null);
			
			//conn.authenticateWithPassword("pi", "beep");

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
			
			// ensure that directory for path automate exists
			piIn.println("mkdir -p " + piDirAutomat);

			// ensure log directory exists
			piIn.println("mkdir -p ~/log");
			
			// Start a new roscore
			try {
				piOut.readLine(); // command
			} catch (IOException e) {
				e.printStackTrace();
			}
			if (!isRoscoreRunning()) {
				startNewRoscore();
				try {
					Thread.sleep(4000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			// start BEEP nodes (old nodes will shutdown automatically)
			startBeepNode();

			try {
				Thread.sleep(8000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			return true;
		} catch (Exception e) {
			e.printStackTrace();
			MainFrame
					.showErrInfo(
							"Es konnte keine Verbindung mit "
									+ connectTo
									+ " hergestellt werden.<br>Bitte überprüfe die IP des Roboters und ob dieser über eine aktive W-Lan Verbindung verfügt!",
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
			piOut = null;
			sess.close();
			sess = null;
			conn.close();
			conn = null;
		} catch (IOException e) {
		}

	}

	@Override
	public int getVariableValue(String variable) {
		Object msg = rosComm.getSensorMsg(variable);
		ISmachableSensor sen = smachableSensors.getSensor(variable);
		ISubscriberInfo senInfo = null;
		if (sen instanceof ISubscriberInfo) {
			senInfo = (ISubscriberInfo) sen;
		} else {
			System.err.println(sen.getName()
					+ " does not implement ISubscriberInfo!");
			return 0;
		}
		if (msg != null && senInfo != null) {
			if (sen.getTopicType().equals(std_msgs.Int32._TYPE)) {
				return ((std_msgs.Int32) msg).getData();
			} else if (sen.getTopicType().equals(std_msgs.Int16._TYPE)) {
				return ((std_msgs.Int16) msg).getData();
			} else if (sen.getTopicType().equals(std_msgs.Float32._TYPE)) {
				return Math.round(((std_msgs.Float32) msg).getData());
			} else if (sen.getTopicType().equals(beep_msgs.IR._TYPE)) {
				// Sensorname: IRx ; x: index in ir-array of the message
				int index = Integer.parseInt(variable.substring(2));
				return ((beep_msgs.IR) msg).getIr()[index];
			} else if (sen.getTopicType().equals(beep_msgs.Color_sensors._TYPE)) {
				int index = Integer.parseInt(variable.substring(3));
				beep_msgs.Color col = ((beep_msgs.Color_sensors) msg)
						.getSensors().get(index);
				Color c = new Color(col.getR(), col.getG(), col.getB(),
						col.getW());
				return c.getRGB();
			}// add new message-types here
		}
		return 0;
	}

	@Override
	public void executeActions(ArrayList<Action> actions) throws IOException {
		// TODO what to do with this?
	}

	@Override
	public void updateSensors() throws IOException {
		// not necessary, automatically updated by RosCommunicator
	}

	@Override
	public void stop() {
		if (connected) {
			try {
				piIn.println("pkill -2 -f " + automatFileName);
				piOut.readLine(); // command
			} catch (IOException e) {
				e.printStackTrace();
			}

		}

	}

	@Override
	public boolean transmit() {
		try {
			if (MainFrame.automat.getStates().isEmpty()) {
				MainFrame.showErrInfo(
						"Ein leerer Automat kann nicht übertragen werden.<br>"
								+ " Bitte lege mindestens einen State an!",
						"Fehler beim Erstellen des Automats");
			}
			// safe smach automate to file
			SmachAutomat sA = new SmachAutomat(MainFrame.automat.getStates(),
					smachableSensors, getActuators(), "zusmoro_state_machine");
			File file = new File(automatFileName);
			sA.saveToFile(file);
		} catch (NoSuchAttributeException e1) {
			// unknown Sensors used
			e1.printStackTrace();
			return false;
		}

		try {
			SCPClient client = new SCPClient(conn);
			client.put(automatFileName, piDirAutomat);
			piIn.println("chmod +x " + piDirAutomat + "/" + automatFileName);
			return true;
		} catch (IOException e) {
			e.printStackTrace();
			return false;
		}
	}

	@Override
	public void play() {
		if (connected) {
			if (isRoscoreRunning()) {
				startAutomatOnPi();
			} else {
				startNewRoscore();
				startBeepNode();
				startAutomatOnPi();
			}
		}
	}

	@Override
	public int getDesiredAdditionalTimeout() {
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
		return 0;
	}

	/**
	 * checks, if there is a running roscore process on the Beep
	 * 
	 * @return true, if roscore is running
	 */
	private boolean isRoscoreRunning() {
		System.out.print("ROS Master running: ");
		if (connected) {
			piIn.println("pgrep -x rosmaster | wc -l");
			try {
				String line = piOut.readLine();
				while (line.length() != 1) {
					line = piOut.readLine();
				}
				int n;
				
				try {
					n = Integer.parseInt(line);
				} catch (NumberFormatException e) {
					System.out.println("false");
					return false;
				}

				if (n == 1) {
					System.out.println("true");
					return true;
				} else if (n>1) {
					System.out.println("true (multiple?!)");
					return true;
				} else{
					System.out.println("false");
					return false;
				}
			} catch (Exception e) {
				e.printStackTrace();
			}

		}

		System.out.println("not connected");
		return false;
	}

	/**
	 * Terminates every running roscore process (rosmaster, roslaunch) and starts a new roscore. Mind,
	 * that the roscore needs some time to start up after the call of this
	 * function.
	 */
	private void startNewRoscore() {
		if (connected) {
			piIn.println("killall roslaunch");
			piIn.println("killall roscore");
			piIn.println("killall rosmaster");
			System.out.println("Starting new Roscore");
			piIn.println("nohup roscore 2> ~/log/roscore-err.log 1> ~/log/roscore-out.log &");
		}
	}

	/**
	 * Starts a new _Beep node. Mind, that the node needs some time to start up
	 * after the call of this function.
	 */
	private void startBeepNode() {
		if (connected) {
			System.out.println("Starting Beep-node");
			piIn.println("nohup roslaunch beep_nodes zusmoro.launch  2> ~/log/beep_nodes-err.log 1> ~/log/beep_nodes-out.log &");
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
		// TODO use ros package
		piIn.println("nohup " + piDirAutomat + "/" + automatFileName
				+ " 2> ~/log/" + automatFileName + "-err.log 1> ~/log/"
				+ automatFileName + "-out.log &");
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
		if (connected) {
			LinkedList<ISubscriberInfo> info = new LinkedList<>();
			info.addAll(sensorsCol);
			info.addAll(sensorsIR);
			// info.add(sensorsIR.get(5));
			if (rosComm == null) {
				rosComm = new RosCommunicator("http://" + beepIP + ":11311/",
						info);
				rosComm.addActionListener(debugView);
			}

			rosComm.startCommunication();

			debugView.setVisible(true);
		}
	}

	@Override
	public String getLastConnectedTo() {
		return beepIP;
	}

	@Override
	public AbstractSettingPanel getSettingsPanel() {
		return new BeepSettingPanel(this);
	}

}
