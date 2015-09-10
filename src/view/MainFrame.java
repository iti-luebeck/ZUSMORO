/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package view;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.io.File;
import java.util.LinkedList;

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
 *         Main class starting Zusmoro, generating main window with standard
 *         layout
 */
// removed logger, maybe put back in?
// TODO: was kann privat werden?

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

	public volatile static AbstractRobot robot;
	
	public volatile static LinkedList<AbstractRobot> robots;

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
		
		robots = getRobots();
		
		robot = robots.get(0);
		robotClass = (Class<AbstractRobot>) robot.getClass();
		
//		try {
//			robotClass = (Class<AbstractRobot>) Class
//					.forName("robots.beep.BeepRobot");
//			robot = robotClass.newInstance();
//		} catch (Exception e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			// robotClass = EPuckRobot.class;
//		}
		onBoard = new OnBoardEpuck();
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

	}

	/**
	 * Resetting the editor panel after loading file
	 * 
	 * @param ePanel
	 *            active Editor Panel
	 */
	public void setEditorPanel(EditorPanel ePanel) {
		cPane.remove(scrollPane);
		MainFrame.editorPanel = ePanel;
		scrollPane = new JScrollPane(MainFrame.editorPanel);
		cPane.add(scrollPane, BorderLayout.CENTER);
		validate();
		repaint();
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
				File f = new File("/var/lock/lockdev/LCK..rfcomm0");
				if (f.exists() && f.isFile()) {
					f.delete();
				}
				System.exit(0);
			}
		} else {
			File f = new File("/var/lock/lockdev/LCK..rfcomm0");
			if (f.exists() && f.isFile()) {
				f.delete();
			}
			System.exit(0);
		}
	}

	public static void showErrInfo(String errLong, String errShort) {
		JOptionPane.showMessageDialog(mainFrame, errLong, errShort,
				JOptionPane.WARNING_MESSAGE);
	}
	
	private static LinkedList<AbstractRobot> getRobots() {
		File robotDir = new File(SettingsDialog.class.getResource("../robots")
				.getFile().replace("%20", " "));
		File[] robotDirs = robotDir.listFiles();
		LinkedList<AbstractRobot> robots = new LinkedList<>();
		for (File dir : robotDirs) {
			for (File file : dir.listFiles()) {
				try {
					Class<?> rob = Class.forName("robots." + dir.getName()
							+ "." + file.getName().replace(".class", ""));
					if (rob.newInstance() instanceof AbstractRobot) {
						robots.add((AbstractRobot)rob.newInstance());
					}
				} catch (Exception e) {
				}
			}
		}
		return robots;
	}
}