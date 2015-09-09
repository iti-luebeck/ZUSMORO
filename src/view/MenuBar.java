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

import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;

import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JSeparator;
import javax.swing.KeyStroke;

import view.listeners.FileMenuListener;
import view.listeners.HelpMenuListener;
import view.listeners.RunMenuListener;
import view.listeners.SettingsMenuListener;
import view.listeners.UndoListener;

public class MenuBar extends JMenuBar {

	private static final long serialVersionUID = -111659266189879540L;

	private final JMenu fileMenu;
	private final JMenu editMenu;
	private final JMenu settingsMenu;
	private final JMenu runMenu;
	private final JMenu helpMenu;

	public final JMenuItem undo;
	public final JMenuItem redo;

	private final ActionListener fileMenuListener;
	private final ActionListener settingsMenuListener;
	private final ActionListener runMenuListener;
	private final ActionListener helpMenuListener;
	private final UndoListener undoListener;

	public MenuBar() {
		super();
		helpMenuListener = new HelpMenuListener();
		fileMenuListener = new FileMenuListener();
		runMenuListener = new RunMenuListener();
		settingsMenuListener = new SettingsMenuListener();
		undoListener = new UndoListener();

		//---------------FILE MENU-----------------------------------
		fileMenu = new JMenu("Datei");
		fileMenu.setMnemonic(KeyEvent.VK_D);
		JMenuItem newFile = new JMenuItem("Neu", KeyEvent.VK_N);
		newFile.setActionCommand("newFile");
		newFile.setAccelerator(KeyStroke.getKeyStroke("ctrl N"));
		newFile.addActionListener(fileMenuListener);
		JMenuItem openFile = new JMenuItem("Öffnen...", KeyEvent.VK_F);
		openFile.setActionCommand("openFile");
		openFile.setAccelerator(KeyStroke.getKeyStroke("ctrl O"));
		openFile.addActionListener(fileMenuListener);
		JMenuItem saveFile = new JMenuItem("Speichern", KeyEvent.VK_S);
		//saveFile.setEnabled(false);
		saveFile.setActionCommand("saveFile");
		saveFile.setAccelerator(KeyStroke.getKeyStroke("ctrl S"));
		saveFile.addActionListener(fileMenuListener);
		JMenuItem saveUnder = new JMenuItem("Speichern unter...", KeyEvent.VK_U);
		saveUnder.setActionCommand("saveUnder");
		saveUnder.setAccelerator(KeyStroke.getKeyStroke("ctrl shift S"));
		saveUnder.addActionListener(fileMenuListener);
		JMenuItem print = new JMenuItem("Drucken...", KeyEvent.VK_D);
		print.setEnabled(false);
		print.setActionCommand("print");
		print.addActionListener(fileMenuListener);
		JMenuItem exit = new JMenuItem("Beenden", KeyEvent.VK_B);
		exit.setActionCommand("exit");
		exit.addActionListener(fileMenuListener);
		fileMenu.add(newFile);
		fileMenu.add(openFile);
		fileMenu.add(new JSeparator());
		fileMenu.add(saveFile);
		fileMenu.add(saveUnder);
		fileMenu.add(new JSeparator());
		fileMenu.add(print);
		fileMenu.add(new JSeparator());
		fileMenu.add(exit);
		this.add(fileMenu);

		//-------------EDIT MENU---------------------------------
		editMenu = new JMenu("Bearbeiten");
		editMenu.setMnemonic(KeyEvent.VK_B);
		undo = new JMenuItem("Undo", KeyEvent.VK_U);
		undo.setEnabled(false);
		undo.setActionCommand("undo");
//		undo.getInputMap(WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("ctrl Z"), "undo");
//		undo.getActionMap().put("undo", undoListener);
		undo.setAccelerator(KeyStroke.getKeyStroke("ctrl Z"));
		undo.addActionListener(undoListener);
		redo = new JMenuItem("Redo", KeyEvent.VK_R);
		redo.setEnabled(false);
		redo.setActionCommand("redo");
//		redo.getInputMap(WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("ctrl Y"), "redo");
//		redo.getActionMap().put("redo", undoListener);
		redo.setAccelerator(KeyStroke.getKeyStroke("ctrl Y"));
		redo.addActionListener(undoListener);
		editMenu.add(undo);
		editMenu.add(redo);
		this.add(editMenu);

		//-------------RUN MENU----------------------------------
		runMenu = new JMenu("Programm");
		runMenu.setMnemonic(KeyEvent.VK_P);
		JMenuItem connect = new JMenuItem("Verbinden...", KeyEvent.VK_V);
		connect.setActionCommand("connect");
		connect.addActionListener(runMenuListener);
		JMenuItem disconnect = new JMenuItem("Verbindung trennen", KeyEvent.VK_T);
		disconnect.setActionCommand("disconnect");
		disconnect.addActionListener(runMenuListener);
		JMenuItem runProgram = new JMenuItem("Ausführen", KeyEvent.VK_A);
		runProgram.setActionCommand("run");
		runProgram.setAccelerator(KeyStroke.getKeyStroke("F5"));
		runProgram.addActionListener(runMenuListener);
		JMenuItem cancelProgramm = new JMenuItem("Abbrechen", KeyEvent.VK_B);
		cancelProgramm.setActionCommand("stop");
		cancelProgramm.setAccelerator(KeyStroke.getKeyStroke("F6"));
		cancelProgramm.addActionListener(runMenuListener);
		runMenu.add(connect);
		runMenu.add(disconnect);
		runMenu.add(runProgram);
		runMenu.add(cancelProgramm);
		this.add(runMenu);

		//-------------SETTINGS MENU-----------------------------
		settingsMenu = new JMenu("Einstellungen");
		settingsMenu.setMnemonic(KeyEvent.VK_E);
		JMenuItem progSettings = new JMenuItem("Automateneinstellungen", KeyEvent.VK_A);
		progSettings.setActionCommand("progSet");
		progSettings.addActionListener(settingsMenuListener);
		settingsMenu.add(progSettings);
		this.add(settingsMenu);

		//--------------HELP MENU--------------------------------
		helpMenu = new JMenu("Hilfe");
		helpMenu.setMnemonic(KeyEvent.VK_H);
		JMenuItem about = new JMenuItem("About", KeyEvent.VK_A);
		about.setActionCommand("about");
		about.setAccelerator(KeyStroke.getKeyStroke("F1"));
		about.addActionListener(helpMenuListener);
		helpMenu.add(about);
		this.add(helpMenu);
	}
}
