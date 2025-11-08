// src/utils/ServoUtils.ts

/**
 * Utility functions for servo control operations
 * Provides conversion, validation, and sequence generation for servo commands
 */
export class ServoUtils {
  // PWM constants
  static readonly PWM_MIN = 1000;
  static readonly PWM_MAX = 2000;
  static readonly PWM_NEUTRAL = 1500;
  
  // Angle constants
  static readonly ANGLE_MIN = 0;
  static readonly ANGLE_MAX = 180;
  static readonly ANGLE_NEUTRAL = 90;
  
  // Servo channel limits
  static readonly SERVO_MIN = 1;
  static readonly SERVO_MAX = 16;

  /**
   * Convert angle (0-180°) to PWM (1000-2000μs)
   * Uses linear mapping: 0° = 1000μs, 90° = 1500μs, 180° = 2000μs
   * 
   * @param angle - Angle in degrees (0-180)
   * @returns PWM value in microseconds (1000-2000)
   * 
   * @example
   * ```ts
   * ServoUtils.angleToPwm(0)   // → 1000
   * ServoUtils.angleToPwm(90)  // → 1500
   * ServoUtils.angleToPwm(180) // → 2000
   * ```
   */
  static angleToPwm(angle: number): number {
    const clampedAngle = Math.max(
      this.ANGLE_MIN, 
      Math.min(this.ANGLE_MAX, angle)
    );
    return Math.round(
      this.PWM_MIN + (clampedAngle / this.ANGLE_MAX) * (this.PWM_MAX - this.PWM_MIN)
    );
  }

  /**
   * Convert PWM (1000-2000μs) to angle (0-180°)
   * Uses linear mapping: 1000μs = 0°, 1500μs = 90°, 2000μs = 180°
   * 
   * @param pwm - PWM value in microseconds (1000-2000)
   * @returns Angle in degrees (0-180)
   * 
   * @example
   * ```ts
   * ServoUtils.pwmToAngle(1000) // → 0
   * ServoUtils.pwmToAngle(1500) // → 90
   * ServoUtils.pwmToAngle(2000) // → 180
   * ```
   */
  static pwmToAngle(pwm: number): number {
    const clampedPwm = Math.max(
      this.PWM_MIN, 
      Math.min(this.PWM_MAX, pwm)
    );
    return Math.round(
      ((clampedPwm - this.PWM_MIN) / (this.PWM_MAX - this.PWM_MIN)) * this.ANGLE_MAX
    );
  }

  /**
   * Validate servo ID is within acceptable range (1-16)
   * 
   * @param id - Servo channel number
   * @returns true if valid, false otherwise
   */
  static isValidServoId(id: number): boolean {
    return Number.isInteger(id) && id >= this.SERVO_MIN && id <= this.SERVO_MAX;
  }

  /**
   * Validate angle is within acceptable range (0-180°)
   * 
   * @param angle - Angle in degrees
   * @returns true if valid, false otherwise
   */
  static isValidAngle(angle: number): boolean {
    return Number.isFinite(angle) && angle >= this.ANGLE_MIN && angle <= this.ANGLE_MAX;
  }

  /**
   * Validate PWM is within acceptable range (1000-2000μs)
   * 
   * @param pwm - PWM value in microseconds
   * @returns true if valid, false otherwise
   */
  static isValidPwm(pwm: number): boolean {
    return Number.isFinite(pwm) && pwm >= this.PWM_MIN && pwm <= this.PWM_MAX;
  }

  /**
   * Generate sequence of angles for smooth servo movement
   * Useful for creating sweeps, animations, or gradual transitions
   * 
   * @param startAngle - Starting angle in degrees
   * @param endAngle - Ending angle in degrees
   * @param steps - Number of intermediate steps
   * @returns Array of angles from start to end
   * 
   * @example
   * ```ts
   * ServoUtils.generateSequence(0, 180, 5)
   * // → [0, 36, 72, 108, 144, 180]
   * 
   * ServoUtils.generateSequence(90, 0, 3)
   * // → [90, 60, 30, 0]
   * ```
   */
  static generateSequence(startAngle: number, endAngle: number, steps: number): number[] {
    const sequence: number[] = [];
    const stepSize = (endAngle - startAngle) / steps;
    
    for (let i = 0; i <= steps; i++) {
      const angle = Math.round(startAngle + (stepSize * i));
      sequence.push(angle);
    }
    
    return sequence;
  }

  /**
   * Generate a sweep sequence (0° → max → 0°)
   * 
   * @param maxAngle - Maximum angle to sweep to (default: 180)
   * @param steps - Steps in each direction (default: 10)
   * @returns Array of angles for full sweep
   * 
   * @example
   * ```ts
   * ServoUtils.generateSweep(90, 3)
   * // → [0, 30, 60, 90, 60, 30, 0]
   * ```
   */
  static generateSweep(maxAngle: number = 180, steps: number = 10): number[] {
    const forward = this.generateSequence(0, maxAngle, steps);
    const backward = this.generateSequence(maxAngle, 0, steps).slice(1);
    return [...forward, ...backward];
  }

  /**
   * Clamp angle to valid range
   * 
   * @param angle - Input angle
   * @returns Clamped angle between 0-180
   */
  static clampAngle(angle: number): number {
    return Math.max(this.ANGLE_MIN, Math.min(this.ANGLE_MAX, angle));
  }

  /**
   * Clamp PWM to valid range
   * 
   * @param pwm - Input PWM value
   * @returns Clamped PWM between 1000-2000
   */
  static clampPwm(pwm: number): number {
    return Math.max(this.PWM_MIN, Math.min(this.PWM_MAX, pwm));
  }

  /**
   * Calculate angle difference (useful for detecting movement)
   * 
   * @param angle1 - First angle
   * @param angle2 - Second angle
   * @returns Absolute difference in degrees
   */
  static angleDifference(angle1: number, angle2: number): number {
    return Math.abs(angle1 - angle2);
  }

  /**
   * Interpolate between two angles
   * 
   * @param startAngle - Starting angle
   * @param endAngle - Ending angle
   * @param progress - Progress from 0 to 1
   * @returns Interpolated angle
   * 
   * @example
   * ```ts
   * ServoUtils.interpolateAngle(0, 180, 0.5)   // → 90
   * ServoUtils.interpolateAngle(0, 180, 0.25)  // → 45
   * ```
   */
  static interpolateAngle(startAngle: number, endAngle: number, progress: number): number {
    const clampedProgress = Math.max(0, Math.min(1, progress));
    return Math.round(startAngle + (endAngle - startAngle) * clampedProgress);
  }
}
