package frc.robot.utilities;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class EdgeDetector {
  private Debouncer m_debouncer;
  private BooleanSupplier m_input;
  private boolean m_state;

  public EdgeDetector(BooleanSupplier input, double debounceTime){
    m_input = input;
    m_state = m_input.getAsBoolean();
    m_debouncer = new Debouncer(debounceTime, DebounceType.kBoth);
    m_debouncer.calculate(m_input.getAsBoolean());
  }

  public boolean detectRising(){
    boolean nextState = m_debouncer.calculate(m_input.getAsBoolean());
    if (nextState && !m_state){
      m_state = nextState;
      return true;
    }
    m_state = nextState;
    return false;
  }

  public boolean detectFalling(){
    boolean nextState = m_debouncer.calculate(m_input.getAsBoolean());
    if (!nextState && m_state){
      m_state = nextState;
      return true;
    }
    m_state = nextState;
    return false;
  }

  public boolean get(){
    m_state = m_debouncer.calculate(m_input.getAsBoolean());
    return m_state;
  }
}
