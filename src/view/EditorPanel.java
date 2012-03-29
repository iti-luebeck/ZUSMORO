package view;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JPanel;

import model.AbstractRobot;
import model.Automat;
import model.ChangeEvent;
import model.State;
import model.Transition;
import model.UndoRedoQueue;


/**
 * @author ida
 *
 * Creates the white editor window and adds all possible events 
 */
public class EditorPanel extends JPanel implements Observer, MouseListener, MouseMotionListener {

	private static final long serialVersionUID = 8588326049847550395L;
	private EditorMode mode;
	private int startX;
	private int startY;
	private Rectangle selectionRect;
	private UndoRedoQueue undoRedoQueue;

	/**
	 * @author ida
	 * 
	 * Editor modes including funny toString method
	 */
	public enum EditorMode {
		/**
		 * Edit existing state chart
		 */
		EDIT("Editieren"),
		/**
		 * Create new state in current chart
		 */
		CREATE_STATE("Zustand erstellen"),
		/**
		 * Create new transition in existing chart
		 */
		CREATE_TRANSITION("Transition erstellen"),
		/**
		 * Delete states or transitions in current chart
		 */
		DELETE("Zustände oder Transitionen löschen");

		private String s;
		private EditorMode(String s){
			this.s=s;
		}
		@Override
		public String toString() {
			return s;
		}
	}

	/**
	 * Constructor without parameters, is super really needed?
	 */
	public EditorPanel() {
		super();
		construct();
	}

	/**
	 * @param robot 
	 * 
	 * Constructor with robot, is super really needed?
	 */
	public EditorPanel(AbstractRobot robot) {
		super();
		construct();
		MainFrame.automat.setRobot(robot);
	}

	//actual constructor, why is it here?
	private void construct() {
		MainFrame.automat = new Automat();
		MainFrame.automat.addObserver(this);
		setMode(EditorMode.EDIT);
		undoRedoQueue = new UndoRedoQueue();
		this.selectionRect = new Rectangle();
		this.setAutoscrolls(true);
		this.setBackground(Color.WHITE);
		this.setLayout(null);
		this.addMouseListener(this);
		this.setPreferredSize(new Dimension(1500, 1500));
		this.addMouseMotionListener(this);
	}

	void setMode(EditorMode newMode) {
		this.mode = newMode;
		MainFrame.statusBar.setEditorMode(mode);
	}

	public void mouseClicked(MouseEvent e) {
		if (mode == EditorMode.CREATE_STATE) {
			State newState = new State("State " + (MainFrame.automat.getStateCount() + 1), MainFrame.automat
					.getStateCount() == 0);
			newState.setBounds(e.getX() - State.STATE_WIDTH / 2, e.getY() - State.STATE_HEIGHT / 2, State.STATE_WIDTH,
					State.STATE_HEIGHT);
			MainFrame.automat.addState(newState);
		} else if (mode == EditorMode.EDIT || mode == EditorMode.CREATE_TRANSITION) {
			MainFrame.automat.clearSelection();
		}
	}

	public void mouseEntered(MouseEvent e) {
		// don't care
	}

	public void mouseExited(MouseEvent e) {
		// don't care
	}

	public void mousePressed(MouseEvent e) {
		this.startX = e.getX();
		this.startY = e.getY();
	}

	public void mouseReleased(MouseEvent e) {
		if (mode == EditorMode.EDIT && startX * startY > 0) {
			this.selectStatesIn(this.selectionRect);
		}
		startX = 0;
		startY = 0;
		this.selectionRect = new Rectangle();
		repaint();
		// System.out.println("Panel release");
	}

	public void mouseDragged(MouseEvent e) {
		if (this.mode == EditorMode.EDIT) {
			// System.out.println("Panel drag");
			int leftEdgeX = Math.min(startX, e.getX());
			int leftEdgeY = Math.min(startY, e.getY());
			int rightEdgeX = Math.max(startX, e.getX());
			int rightEdgeY = Math.max(startY, e.getY());
			int width = rightEdgeX - leftEdgeX;
			int height = rightEdgeY - leftEdgeY;
			this.selectionRect = new Rectangle(leftEdgeX, leftEdgeY, width, height);
			this.selectStatesIn(this.selectionRect);
			repaint();
		}
	}

	public void mouseMoved(MouseEvent e) {
		// Component comp = getComponentAt(e.getX(), e.getY());
	}

	/** 
	 * @return current mode
	 * 
	 * getter for mode
	 */
	public EditorMode getMode() {
		return mode;
	}

	
	/**
	 * @param rect Selection in window (selected by dragging)
	 * 
	 * Marks several states at once
	 */
	public void selectStatesIn(Rectangle rect) {
		Component[] comps = this.getComponents();
		// ArrayList<State> selectedStates = new ArrayList<State>(comps.length);
		for (int i = 0; i < comps.length; i++) {
			Component comp = comps[i];
			if (comp instanceof State) {
				boolean contained = rect.contains(comp.getBounds());
				((State) comp).setSelected(contained);
				// if (contained) {
				// selectedStates.add((State) comp);
				// } else {
				// selectedStates.remove(comp);
				// }
			}
		}
		// return selectedStates.toArray(new State[selectedStates.size()]);
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		if (this.mode == EditorMode.EDIT) {
			g.setColor(Color.BLACK);
			g.drawRect(selectionRect.x, selectionRect.y, selectionRect.width, selectionRect.height);
			g.setColor(new Color(240, 240, 240));
			g.fillRect(selectionRect.x + 1, selectionRect.y + 1, selectionRect.width - 1, selectionRect.height - 1);
		}
		Graphics2D g2d = (Graphics2D) g;
		g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g2d.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_LCD_HRGB);
		g2d.setStroke(new BasicStroke(3.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
		int transWidth = State.STATE_WIDTH - 20;
		for (State state : MainFrame.automat.getStates()) {
			int transCount = state.getTransitions().size();
			int index = 0;
			for (Transition transition : state.getTransitions()) {
				g2d.setColor(transition.getMouseOn() ? Color.BLUE : Color.BLACK);
				if (transCount <= 1) {
					g2d.draw(transition.buildPath(0));
				} else {
					g2d.draw(transition.buildPath(transWidth / -2 + index * 60 / (transCount - 1)));
				}
				g2d.drawString(transition.getLabel(), transition.getStringX() + 3, transition.getStringY() + 4);
				index++;
			}
		}
		if (MainFrame.automat.getActiveState() != null) {
			Rectangle stateBounds = MainFrame.automat.getActiveState().getBounds();
			g2d.setColor(Color.MAGENTA);
			g2d
					.drawRoundRect(stateBounds.x + 1, stateBounds.y + 1, stateBounds.width - 3, stateBounds.height - 3,
							8, 8);
		}
	}

	public void update(Observable arg0, Object arg1) {
		if (arg1 != null && arg1 instanceof ChangeEvent) {
			ChangeEvent event = (ChangeEvent) arg1;
			Component comp = (Component) event.object;

			if (MainFrame.DEBUG) {
				System.out.println(event);
			}
			switch (event.type) {
			case STATE_CREATE:
			case TRANSITION_CREATE:
				add(comp);
				undoRedoQueue.enque(event);
				break;
			case STATE_DELETE:
			case TRANSITION_DELETE:
				remove(comp);
				undoRedoQueue.enque(event);
				break;
			case STATE_MOVE:
				break;
			case TRANSITION_MOVE:
				break;
			case ACTIVE_STATE_CHANGE:
				repaint();
				return;
			}
			validate();
			repaint();
			MainFrame.menuBar.undo.setEnabled(undoRedoQueue.hasUndoEvents());
			MainFrame.menuBar.redo.setEnabled(undoRedoQueue.hasRedoEvents());
		}
	}

	/**
	 * Repaints only valid components?!
	 */
	public void validateProgramm() {
		this.removeAll();
		ArrayList<Transition> transitions = new ArrayList<Transition>();
		for (State state : MainFrame.automat.getStates()) {
			this.add(state);
			transitions.addAll(state.getTransitions());
		}
		for (Transition transition : transitions) {
			this.add(transition);
		}
		this.validate();
		this.repaint();
		resetUndoRedoQueue();
	}

	/**
	 * Clear undoRedoQueue
	 */
	public void resetUndoRedoQueue() {
		undoRedoQueue.clear();
		MainFrame.menuBar.undo.setEnabled(false);
		MainFrame.menuBar.redo.setEnabled(false);
	}

	/**
	 * Undo the last action if possible
	 */
	public void undo() {
		if (undoRedoQueue.hasUndoEvents()) {
			ChangeEvent event = undoRedoQueue.dequeUndoEvent();
			switch (event.type) {
			case STATE_CREATE:
				MainFrame.automat.removeState((State) event.object);
				break;
			case TRANSITION_CREATE:
				((Transition) event.object).getRootState().removeTransition((Transition) event.object);
				repaint();
				break;
			case STATE_DELETE:
				MainFrame.automat.addState((State) event.object);
				while (undoRedoQueue.hasUndoEvents() && undoRedoQueue.nextUndoIsFollowEvent()) {
					event = undoRedoQueue.dequeUndoEvent();
					((Transition) event.object).getRootState().addTransition((Transition) event.object);
				}
				break;
			case TRANSITION_DELETE:
				((Transition) event.object).getRootState().addTransition((Transition) event.object);
				break;
			default:
				break;
			}
			undoRedoQueue.unlock();
		}
		MainFrame.menuBar.undo.setEnabled(undoRedoQueue.hasUndoEvents());
		MainFrame.menuBar.redo.setEnabled(undoRedoQueue.hasRedoEvents());
	}

	/**
	 * Redo last undone action if possible
	 */
	public void redo() {
		if (undoRedoQueue.hasRedoEvents()) {
			ChangeEvent event = undoRedoQueue.dequeRedoEvent();
			switch (event.type) {
			case STATE_CREATE:
				MainFrame.automat.addState((State) event.object);
				break;
			case STATE_DELETE:
				MainFrame.automat.removeState((State) event.object);
				break;
			case TRANSITION_CREATE:
				((Transition) event.object).getRootState().addTransition((Transition) event.object);
				break;
			case TRANSITION_DELETE:
				if (event.followEvent) {
					((Transition) event.object).getRootState().removeTransition((Transition) event.object);
					while ((event = undoRedoQueue.dequeRedoEvent()).followEvent) {
						((Transition) event.object).getRootState().removeTransition((Transition) event.object);
					}
					MainFrame.automat.removeState((State) event.object);
				} else {
					((Transition) event.object).getRootState().removeTransition((Transition) event.object);
				}
				break;
			default:
				break;
			}
			undoRedoQueue.unlock();
		}
		MainFrame.menuBar.undo.setEnabled(undoRedoQueue.hasUndoEvents());
		MainFrame.menuBar.redo.setEnabled(undoRedoQueue.hasRedoEvents());
	}
}