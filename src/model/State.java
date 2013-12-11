package model;

import java.awt.BasicStroke;
import java.awt.Component;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;

import javax.swing.JPanel;
import javax.swing.JToggleButton;
import javax.swing.JToolTip;

import model.ChangeEvent.ChangeEventType;
import smachGenerator.ISmachableAction;
import smachGenerator.ISmachableState;
import view.EditorPanel;
import view.MainFrame;
import view.StateView;
import view.EditorPanel.EditorMode;

public class State extends JToggleButton implements MouseMotionListener, MouseListener, KeyListener, ISmachableState {

	public static final int STATE_WIDTH = 80;
	public static final int STATE_HEIGHT = 60;
	private static final long serialVersionUID = -9059752545868475959L;

	private ArrayList<Transition> transitions;

	private ArrayList<Action> actions;
	private boolean initialState;

	private char lastMouseEvent = '0';

	public State(String name) {
		super(name);
		transitions = new ArrayList<Transition>();
		actions = new ArrayList<Action>(15);
		initialState = false;
		this.addMouseMotionListener(this);
		this.addMouseListener(this);
		this.addKeyListener(this);
		this.setToolTipText(toString());
	}

	public State(String name, boolean isInitialState) {
		this(name);
		initialState = isInitialState;
	}

	/**
	 * Konstruktor der zum Laden eines Programms genutzt werden sollte.
	 *
	 * @param actions
	 *            die in diesem Zustand auszuführendes Aktionen
	 * @param label
	 *            die Bezeichnung dieses Zustands
	 * @param location
	 *            die Position des Zustands
	 * @param initial
	 *            besagt ob dieser Zustand der Startzustand ist
	 */
	State(ArrayList<Action> actions, String label, Point location, boolean initial) {
		super(label);
		transitions = new ArrayList<Transition>();
		this.actions = actions;
		setLocation(location);
		setSize(State.STATE_WIDTH, State.STATE_HEIGHT);
		initialState = initial;
		this.addMouseMotionListener(this);
		this.addMouseListener(this);
		this.addKeyListener(this);
		this.setToolTipText(toString());
	}

	/* (non-Javadoc)
	 * @see model.ISmachableState#getActions()
	 */
	@Override
	public ArrayList<Action> getActions() {
		return actions;
	}

	public void setActions(ArrayList<Action> actions) {
		this.actions = actions;
	}
	/* (non-Javadoc)
	 * @see model.ISmachableState#getTransitions()
	 */
	@Override
	public ArrayList<Transition> getTransitions() {
		return transitions;
	}

	public void addTransition(Transition transition) {
		if (Automat.runningAutomat == null) {
			this.transitions.add(transition);
			MainFrame.automat.setChanged(new ChangeEvent(ChangeEventType.TRANSITION_CREATE, transition, false));
		}
	}

	public void removeTransition(Transition transition) {
		transition.setLabel("");
		this.transitions.remove(transition);
//		MainFrame.automat.setChanged(new ChangeEvent(ChangeEventType.TRANSITION_DELETE, transition, false));
	}

	/* (non-Javadoc)
	 * @see model.ISmachableState#isInitialState()
	 */
	@Override
	public boolean isInitialState() {
		return initialState;
	}

	public void setInitialState(boolean initialState) {
		this.initialState = initialState;
	}

	@Override
	public JToolTip createToolTip() {
		setToolTipText(toString());
		return super.createToolTip();
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		if (initialState) {
			Graphics2D g2d = (Graphics2D) g;
			g2d.setStroke(new BasicStroke(1.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
			g.drawRoundRect(2, 2, State.STATE_WIDTH - 5, State.STATE_HEIGHT - 5, 8, 8);
			g.drawRoundRect(5, 5, State.STATE_WIDTH - 11, State.STATE_HEIGHT - 11, 8, 8);
		}
	}

	// ----------------------------------------MOUSE EVENTS------------------
	public void mouseDragged(MouseEvent e) {
		// System.out.println(getText()+" mouseDragged");
		EditorMode mode = MainFrame.editorPanel.getMode();
		if (mode == EditorMode.EDIT || mode == EditorMode.CREATE_STATE) {
			int x = e.getX() + this.getLocation().x;
			int y = e.getY() + this.getLocation().y;
			// Rectangle oldRec = this.getBounds();
			this.setBounds(Math.max(x - STATE_WIDTH / 2, 0), Math.max(y - STATE_HEIGHT / 2, 0), STATE_WIDTH, STATE_HEIGHT);
			// oldRec.add(this.getBounds());
			((JPanel) this.getParent()).repaint();
			ArrayList<State> selectedStates = MainFrame.automat.getSelectedStates();
			selectedStates.remove(this);
			for (State state : selectedStates) {
				state.mouseDrag(e);
			}
		}
		this.lastMouseEvent = 'D';
	}

	private void mouseDrag(MouseEvent e) {
		if (MainFrame.editorPanel.getMode() == EditorMode.EDIT) {
			int x = e.getX() + this.getLocation().x;
			int y = e.getY() + this.getLocation().y;
			Rectangle oldRec = this.getBounds();
			this.setBounds(Math.max(x - STATE_WIDTH / 2, 0), Math.max(y - STATE_HEIGHT / 2, 0), STATE_WIDTH, STATE_HEIGHT);
			oldRec.add(this.getBounds());
			((JPanel) this.getParent()).repaint();
		}
		this.lastMouseEvent = 'D';
	}

	public void mouseMoved(MouseEvent e) {
		// don't care
	}

	public void mouseClicked(MouseEvent e) {
		// System.out.println(getText()+" mouseClicked");
		this.requestFocusInWindow();
		int clickCount = e.getClickCount();
		if (MainFrame.editorPanel.getMode() == EditorMode.DELETE && Automat.runningAutomat == null) {
			MainFrame.automat.removeState(this);
			return;
		}
		if (clickCount == 2 && Automat.runningAutomat == null) {
//			if (MainFrame.programm.getRobot() != null) {
//				MainFrame.programm.getRobot().getStateView(this).setVisible(true);
//			} else {
//				try {
//					MainFrame.robotClass.newInstance().getStateView(this).setVisible(true);
//				} catch (InstantiationException e2) {
//					// Auto-generated catch block
//					e2.printStackTrace();
//				} catch (IllegalAccessException e3) {
//					// Auto-generated catch block
//					e3.printStackTrace();
//				}
				new StateView(this).setVisible(true);
//			}
		} else if (clickCount == 1) {
			int modifier = e.getModifiersEx();
			boolean oldStatus = isSelected();
			if (modifier != MouseEvent.CTRL_DOWN_MASK) {
				MainFrame.automat.clearSelection();
				this.setSelected(oldStatus);
			}
		}
		this.lastMouseEvent = 'C';
	}

	public void mouseEntered(MouseEvent e) {
		this.requestFocusInWindow();
	}

	public void mouseExited(MouseEvent e) {
		this.getParent().requestFocusInWindow();
	}

	public void mousePressed(MouseEvent e) {
		this.lastMouseEvent = 'P';
		// System.out.println(getText()+" mousePressed");
	}

	public void mouseReleased(MouseEvent e) {
		// System.out.println(getText()+" mouseReleased");
		EditorPanel ePanel = MainFrame.editorPanel;
		if (ePanel.getMode() == EditorMode.CREATE_TRANSITION) {
			int x = getX() + e.getX();
			int y = getY() + e.getY();
			Component comp = ePanel.getComponentAt(x, y);
			if (comp != null && comp instanceof State) {
				State targetState = (State) comp;
				if (targetState != this || Automat.loopsAllowed) {
					Transition trans = new Transition(this, targetState);
					this.addTransition(trans);
					// getParent().repaint();
				}
			}
		}
		if (this.lastMouseEvent == 'D') {
			this.setSelected(!this.isSelected());
		}
		this.lastMouseEvent = 'R';
	}

	// ----------------------------------KEYS----------------
	public void keyPressed(KeyEvent e) {
		// don't care
	}

	public void keyReleased(KeyEvent e) {
		if (e.getKeyCode() == KeyEvent.VK_DELETE && Automat.runningAutomat == null) {
			MainFrame.automat.removeState(this);
		}
	}

	public void keyTyped(KeyEvent e) {
		// don't care;
	}

	@Override
	public String toString() {
		StringBuilder buffer = new StringBuilder();
		buffer.append("<html><b>" + getText() + "</b>");
		for (ISmachableAction action : actions) {
			buffer.append("<br>" + action.toString());
		}
		buffer.append("</html>");
		return buffer.toString();
	}
}
