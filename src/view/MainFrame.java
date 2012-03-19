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

public class MainFrame extends JFrame {

	// public static Logger logger = Logger.getLogger("view.MainFrameLogger");
	public static final MainFrame mainFrame = new MainFrame();
	public static final float VERSION = 1.1f;
	public static boolean DEBUG = false;
	public static StatusBar statusBar;
	public static ToolPanel toolPanel;
	public static MenuBar menuBar;
	public volatile static EditorPanel editorPanel;
	public volatile static Automat automat;
	public volatile static Class<AbstractRobot> robotClass;

	public volatile static OnBoardEpuck onBoard;
	// protected boolean fileIsInSync = false;

	private static final long serialVersionUID = -5093064061027429275L;
	Container cPane;
	private JScrollPane scrollPane;

	@SuppressWarnings("unchecked")
	public MainFrame() {
		super();
		try {
			UIManager.setLookAndFeel("com.sun.java.swing.plaf.nimbus.NimbusLookAndFeel");
		} catch (Exception e) {
			e.printStackTrace();
			// MainFrame.logger.log(Level.CONFIG,
			// "Loading Nimbus Look and Feel failed.", e);
			try {
				UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
			} catch (Exception e2) {
				e2.printStackTrace();
				// MainFrame.logger.log(Level.CONFIG,
				// "Loading System Look and Feel failed.", e);
			}
		}
		try {
			MainFrame.robotClass = (Class<AbstractRobot>) Class.forName("robots.epuck.EPuckRobot");
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			// robotClass = EPuckRobot.class;
		}
		setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		setTitle("ZUSMORO - ZUstandbasierte Steuerung für MObile ROboter v" + VERSION);
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

	public static void main(String[] args) throws Exception {
		CommPortFinder.listPorts();
		// logger.setLevel(Level.ALL);
		// logger.addHandler(new FileHandler("logfile.txt"));
		// logger.entering("MainFrame", "main", args);
		// mainFrame = new MainFrame();
		mainFrame.setVisible(true);
		ToolTipManager.sharedInstance().setInitialDelay(400);
		// SwingUtilities.invokeLater(new Runnable() {
		// public void run() {
		// new TransitionView().setVisible(true);
		// }
		// });
		((Graphics2D) mainFrame.getGraphics()).setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
				RenderingHints.VALUE_TEXT_ANTIALIAS_LCD_HRGB);
		// logger.exiting("MainFrame", "main");
	}

	public void setEditorPanel(EditorPanel ePanel) {
		cPane.remove(scrollPane);
		MainFrame.editorPanel = ePanel;
		scrollPane = new JScrollPane(MainFrame.editorPanel);
		cPane.add(scrollPane, BorderLayout.CENTER);
		validate();
		repaint();
	}

	@Override
	public void dispose() {
		if (MainFrame.automat.getStateCount() > 0) {
			int confirm = JOptionPane.showConfirmDialog(MainFrame.mainFrame,
					"<html>Soll das bestehende Programm<br>wirklich verworfen werden?</html>", "Programm schließen",
					JOptionPane.YES_NO_OPTION);
			if (confirm == JOptionPane.YES_OPTION) {
				System.exit(0);
			}
		} else {
			System.exit(0);
		}
	}
}
