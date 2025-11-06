"""
WP_MARK State Machine Module
Provides robust state management with validation and transitions
"""

import time
import threading
from enum import Enum
from typing import Dict, Any, Optional, Callable, List
from dataclasses import dataclass


class MissionPhase(Enum):
    """Mission execution phases with enhanced state tracking"""
    IDLE = "idle"
    INITIALIZING = "initializing"
    NAVIGATING = "navigating"
    WAITING_ARRIVAL = "waiting_arrival"
    DELAY_BEFORE_START = "delay_before_start"
    SPRAYING = "spraying"
    DELAY_AFTER_STOP = "delay_after_stop"
    COMPLETED = "completed"
    ERROR = "error"
    RECOVERING = "recovering"


class StateTransitionError(Exception):
    """Raised when an invalid state transition is attempted"""
    pass


class StateValidationError(Exception):
    """Raised when state consistency validation fails"""
    pass


@dataclass
class StateContext:
    """Context information for state transitions"""
    phase: MissionPhase
    waypoint_index: int
    total_waypoints: int
    mission_active: bool
    last_action: str
    timestamp: float
    gps_valid: bool
    services_available: bool
    error_count: int
    retry_count: int


class StateValidator:
    """Validates state consistency and transitions"""

    @staticmethod
    def validate_transition(from_phase: MissionPhase, to_phase: MissionPhase,
                          context: StateContext) -> bool:
        """Validate if a state transition is allowed"""
        valid_transitions = {
            MissionPhase.IDLE: [MissionPhase.INITIALIZING],
            MissionPhase.INITIALIZING: [MissionPhase.NAVIGATING, MissionPhase.ERROR],
            MissionPhase.NAVIGATING: [MissionPhase.WAITING_ARRIVAL, MissionPhase.ERROR],
            MissionPhase.WAITING_ARRIVAL: [MissionPhase.DELAY_BEFORE_START, MissionPhase.NAVIGATING, MissionPhase.ERROR],
            MissionPhase.DELAY_BEFORE_START: [MissionPhase.SPRAYING, MissionPhase.ERROR],
            MissionPhase.SPRAYING: [MissionPhase.DELAY_AFTER_STOP, MissionPhase.ERROR],
            MissionPhase.DELAY_AFTER_STOP: [MissionPhase.NAVIGATING, MissionPhase.COMPLETED, MissionPhase.ERROR],
            MissionPhase.ERROR: [MissionPhase.RECOVERING, MissionPhase.IDLE],
            MissionPhase.RECOVERING: [MissionPhase.IDLE, MissionPhase.ERROR],
            MissionPhase.COMPLETED: [MissionPhase.IDLE]
        }

        if to_phase not in valid_transitions.get(from_phase, []):
            return False

        # Additional context validation
        if to_phase == MissionPhase.NAVIGATING:
            if not context.gps_valid:
                return False
            if context.waypoint_index >= context.total_waypoints:
                return False

        if to_phase == MissionPhase.SPRAYING:
            if not context.services_available:
                return False

        return True


class CircuitBreaker:
    """Circuit breaker pattern for handling transient failures"""

    def __init__(self, failure_threshold: int = 3, recovery_timeout: float = 60.0):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.failure_count = 0
        self.last_failure_time = None
        self.state = "CLOSED"
        self._lock = threading.Lock()

    def call(self, func: Callable, *args, **kwargs):
        with self._lock:
            if self.state == "OPEN":
                if time.time() - self.last_failure_time > self.recovery_timeout:
                    self.state = "HALF_OPEN"
                else:
                    raise Exception("Circuit breaker is OPEN")

            try:
                result = func(*args, **kwargs)
                self._on_success()
                return result
            except Exception as e:
                self._on_failure()
                raise e

    def _on_success(self):
        if self.state == "HALF_OPEN":
            self.state = "CLOSED"
            self.failure_count = 0

    def _on_failure(self):
        self.failure_count += 1
        self.last_failure_time = time.time()
        if self.failure_count >= self.failure_threshold:
            self.state = "OPEN"

    def get_state(self) -> str:
        with self._lock:
            return self.state


class StateMachine:
    """Robust state machine for WP_MARK mission management"""

    def __init__(self, logger=None):
        self.logger = logger
        self.current_phase = MissionPhase.IDLE
        self.context = StateContext(
            phase=MissionPhase.IDLE,
            waypoint_index=0,
            total_waypoints=0,
            mission_active=False,
            last_action="Initialized",
            timestamp=time.time(),
            gps_valid=False,
            services_available=False,
            error_count=0,
            retry_count=0
        )

        self._lock = threading.RLock()
        self._transition_listeners: List[Callable] = []
        self._servo_circuit_breaker = CircuitBreaker(failure_threshold=3, recovery_timeout=30.0)
        self._waypoint_circuit_breaker = CircuitBreaker(failure_threshold=2, recovery_timeout=15.0)

    def transition_to(self, new_phase: MissionPhase, action: str = "", force: bool = False) -> bool:
        """Transition to a new phase with validation"""
        with self._lock:
            old_phase = self.current_phase

            if not force and not StateValidator.validate_transition(old_phase, new_phase, self.context):
                error_msg = f"Invalid transition: {old_phase.value} -> {new_phase.value}"
                if self.logger:
                    self.logger.error(error_msg)
                raise StateTransitionError(error_msg)

            self.current_phase = new_phase
            self.context.phase = new_phase
            self.context.timestamp = time.time()

            if action:
                self.context.last_action = action

            if self.logger:
                self.logger.info(f"State transition: {old_phase.value} -> {new_phase.value}")

            # Notify listeners
            self._notify_transition_listeners(old_phase, new_phase, action)
            return True

    def update_context(self, **kwargs):
        """Update state context"""
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self.context, key):
                    setattr(self.context, key, value)

    def get_status(self) -> Dict[str, Any]:
        """Get current status"""
        with self._lock:
            return {
                'phase': self.current_phase.value,
                'context': {
                    'waypoint_index': self.context.waypoint_index,
                    'total_waypoints': self.context.total_waypoints,
                    'mission_active': self.context.mission_active,
                    'last_action': self.context.last_action,
                    'gps_valid': self.context.gps_valid,
                    'services_available': self.context.services_available,
                    'error_count': self.context.error_count,
                    'retry_count': self.context.retry_count
                },
                'health': {
                    'circuit_breakers': {
                        'servo': self._servo_circuit_breaker.get_state(),
                        'waypoint': self._waypoint_circuit_breaker.get_state()
                    }
                },
                'timestamp': self.context.timestamp
            }

    def add_transition_listener(self, listener: Callable):
        """Add transition listener"""
        with self._lock:
            self._transition_listeners.append(listener)

    def execute_with_circuit_breaker(self, operation: str, func: Callable, *args, **kwargs):
        """Execute function with circuit breaker"""
        breaker = {
            'servo': self._servo_circuit_breaker,
            'waypoint': self._waypoint_circuit_breaker
        }.get(operation, self._servo_circuit_breaker)

        return breaker.call(func, *args, **kwargs)

    def increment_error_count(self):
        """Increment error count"""
        with self._lock:
            self.context.error_count += 1

    def increment_retry_count(self):
        """Increment retry count"""
        with self._lock:
            self.context.retry_count += 1

    def reset_error_state(self):
        """Reset error counters"""
        with self._lock:
            self.context.error_count = 0
            self.context.retry_count = 0

    def _notify_transition_listeners(self, old_phase: MissionPhase, new_phase: MissionPhase, action: str):
        """Notify transition listeners"""
        for listener in self._transition_listeners:
            try:
                if listener:
                    listener(old_phase, new_phase, action)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Transition listener error: {str(e)}")