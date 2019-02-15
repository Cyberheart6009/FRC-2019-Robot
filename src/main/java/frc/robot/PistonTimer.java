package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PistonTimer {
    boolean movePiston;
    Compressor c;
    double startTime;
    boolean startStartTime = true;
    DoubleSolenoid piston;

    public PistonTimer(DoubleSolenoid piston, Compressor c, boolean movePiston) {
        this.piston = piston;
        this.c = c;
        this.movePiston = movePiston;
    }

    public void startTimer() {
        this.startTime = System.currentTimeMillis();
    }

    public void movePistonFunction() {
        if (this.startStartTime) {
            startTimer();
            startStartTime = false;
        }
        if (System.currentTimeMillis() < this.startTime + 500) {
          this.c.setClosedLoopControl(false);
          this.piston.set(DoubleSolenoid.Value.kForward);
        } else {
          this.piston.set(DoubleSolenoid.Value.kReverse);
          this.c.setClosedLoopControl(true);
          this.startStartTime = true;
          this.movePiston = false;
        }
      }
}