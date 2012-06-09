package view;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Graphics2D;
import java.awt.RenderingHints;

import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.swing.ToolTipManager;
import javax.swing.UIManager;

import model.AbstractRobot;
import model.Automat;
import model.onboard.OnBoardEpuck;

/**
 * @author ida
 *
 * Main class starting Zusmoro, generating main window with standard layout
 */
//removed logger, maybe put back in?
//TODO: was kann privat werden?


public class MainFrame extends JFrame {

	/* view */
	/**
	 * Main window holding all other elements
	 */
	public static final MainFrame mainFrame = new MainFrame();
	/**
	 * Text field at bottom of mainFrame for user information, e.g. transmission
	 * status, mode, actions on mouse over etc.
	 */
	public static StatusBar statusBar;
	/**
	 * Panel holding buttons for user actions, e.g. adding components to state
	 * chart, start debugging etc.
	 */
	public static ToolPanel toolPanel;
	/**
	 * Menu bar at top of mainFrame for saving, loading etc.
	 */
	public static MenuBar menuBar;
	/**
	 * The editor panel holding the state chart in progress
	 */
	public volatile static EditorPanel editorPanel;

	/* model */
	/**
	 * The automat in the editor window
	 */
	public volatile static Automat automat;
	/**
	 * define which kind of robot is used
	 */
	public volatile static Class<AbstractRobot> robotClass;

	// TODO: should be control?
	/**
	 * Interaction with EPuck: start & stop program, debug
	 */
	public volatile static OnBoardEpuck onBoard;

	/**
	 * Version of ZUSMORO
	 */
	public static final float VERSION = 1.1f;
	/**
	 * debug mode on?
	 */
	public static boolean DEBUG = false;

	private static final long serialVersionUID = -5093064061027429275L;
	Container cPane;
	private JScrollPane scrollPane;

	/**
	 * Constructor generating mainFrame
	 */
	@SuppressWarnings("unchecked")
	public MainFrame() {
		super();
		try {
			UIManager
					.setLookAndFeel("com.sun.java.swing.plaf.nimbus.NimbusLookAndFeel");
		} catch (Exception e) {
			e.printStackTrace();
			try {
				UIManager.setLookAndFeel(UIManager
						.getSystemLookAndFeelClassName());
			} catch (Exception e2) {
				e2.printStackTrace();
			}
		}
		try {
			MainFrame.robotClass = (Class<AbstractRobot>) Class
					.forName("robots.epuck.EPuckRobot");
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			// robotClass = EPuckRobot.class;
		}
		setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		setTitle("ZUSMORO - ZUstandbasierte Steuerung für MObile ROboter v"
				+ VERSION);
		setSize(1000, 700);
		MainFrame.menuBar = new MenuBar();
		setJMenuBar(MainFrame.menuBar);
		MainFrame.statusBar = new StatusBar();
		MainFrame.toolPanel = new ToolPanel();
		MainFrame.editorPanel = new EditorPanel();
		cPane = this.getContentPane();
		cPane.setLayout(new BorderLayout());
		cPane.add(MainFrame.toolPanel, BorderLayout.WEST);
		scrollPane = new JScrollPane(MainFrame.editorPanel);
		cPane.add(scrollPane, BorderLayout.CENTER);
		cPane.add(MainFrame.statusBar, BorderLayout.SOUTH);
		this.setLocationRelativeTo(null);

		onBoard = new OnBoardEpuck();

	}

	/**
	 * @param args
	 *            default
	 * @throws Exception
	 * 
	 *             main method starting ZUSMORO
	 */
	public static void main(String[] args) throws Exception {
		CommPortFinder.listPorts();
		mainFrame.setVisible(true);
		ToolTipManager.sharedInstance().setInitialDelay(400);
		((Graphics2D) mainFrame.getGraphics()).setRenderingHint(
				RenderingHints.KEY_TEXT_ANTIALIASING,
				RenderingHints.VALUE_TEXT_ANTIALIAS_LCD_HRGB);
	}

	@Override
	public void dispose() {
		if (MainFrame.automat.getStateCount() > 0) {
			int confirm = JOptionPane
					.showConfirmDialog(
							MainFrame.mainFrame,
							"<html>Soll das bestehende Programm<br>wirklich verworfen werden?</html>",
							"Programm schließen", JOptionPane.YES_NO_OPTION);
			if (confirm == JOptionPane.YES_OPTION) {
				System.exit(0);
			}
		} else {
			System.exit(0);
		}
	}
}
