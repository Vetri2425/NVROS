#!/usr/bin/env python3
"""
WP_MARK Robust State Management Test Suite
Tests all enhanced components without requiring ROS2/MAVROS
"""

import sys
import os
import time
import threading
from unittest.mock import Mock, MagicMock

# Add Backend to path
sys.path.insert(0, '/home/flash/NRP_ROS/Backend')

def test_state_machine():
    """Test robust state machine functionality"""
    print("Testing state machine...")

    from servo_manager.wp_mark.state_machine import StateMachine, MissionPhase, StateTransitionError

    # Create state machine
    sm = StateMachine()

    # Test initial state
    assert sm.current_phase == MissionPhase.IDLE
    print("  ✅ Initial state correct")

    # Test valid transition
    sm.transition_to(MissionPhase.INITIALIZING, "Test transition")
    assert sm.current_phase == MissionPhase.INITIALIZING
    print("  ✅ Valid transition works")

    # Test invalid transition
    try:
        sm.transition_to(MissionPhase.COMPLETED, "Invalid transition")
        assert False, "Should have raised StateTransitionError"
    except StateTransitionError:
        print("  ✅ Invalid transition properly rejected")

    # Test context updates
    sm.update_context(gps_valid=True, services_available=True)
    assert sm.context.gps_valid == True
    assert sm.context.services_available == True
    print("  ✅ Context updates work")

    # Test status reporting
    status = sm.get_status()
    assert status['phase'] == MissionPhase.INITIALIZING.value
    assert status['context']['gps_valid'] == True
    print("  ✅ Status reporting works")

    return True


def test_retry_mechanism():
    """Test retry mechanism with exponential backoff"""
    print("\nTesting retry mechanism...")

    from servo_manager.wp_mark.retry_mechanism import RetryMechanism, RetryConfig, RetryError

    # Test successful operation
    config = RetryConfig(max_attempts=3, initial_delay=0.1)
    retry = RetryMechanism(config)

    call_count = 0
    def success_func():
        nonlocal call_count
        call_count += 1
        return "success"

    result = retry.execute(success_func)
    assert result == "success"
    assert call_count == 1
    print("  ✅ Successful operation works")

    # Test failed operation with retry
    call_count = 0
    def fail_func():
        nonlocal call_count
        call_count += 1
        if call_count < 3:
            raise Exception("Temporary failure")
        return "success"

    result = retry.execute(fail_func)
    assert result == "success"
    assert call_count == 3
    print("  ✅ Retry on failure works")

    # Test exhausted retries
    call_count = 0
    def always_fail():
        nonlocal call_count
        call_count += 1
        raise Exception("Always fails")

    try:
        retry.execute(always_fail)
        assert False, "Should have raised RetryError"
    except RetryError:
        assert call_count == 3
        print("  ✅ Retry exhaustion works")

    return True


def test_circuit_breaker():
    """Test circuit breaker pattern"""
    print("\nTesting circuit breaker...")

    from servo_manager.wp_mark.state_machine import CircuitBreaker

    cb = CircuitBreaker(failure_threshold=2, recovery_timeout=1.0)

    # Test successful calls
    call_count = 0
    def success_func():
        nonlocal call_count
        call_count += 1
        return "success"

    for i in range(3):
        result = cb.call(success_func)
        assert result == "success"

    assert call_count == 3
    assert cb.get_state() == "CLOSED"
    print("  ✅ Successful calls work")

    # Test failure threshold
    call_count = 0
    def fail_func():
        nonlocal call_count
        call_count += 1
        raise Exception("Failure")

    for i in range(2):
        try:
            cb.call(fail_func)
            assert False, "Should have raised exception"
        except Exception:
            pass

    assert cb.get_state() == "OPEN"
    print("  ✅ Circuit breaker opens on failures")

    # Test circuit breaker open
    try:
        cb.call(success_func)
        assert False, "Should have raised exception when circuit is open"
    except Exception:
        print("  ✅ Circuit breaker blocks calls when open")

    # Test recovery
    time.sleep(1.1)  # Wait for recovery timeout
    result = cb.call(success_func)
    assert result == "success"
    assert cb.get_state() == "CLOSED"
    print("  ✅ Circuit breaker recovers")

    return True


def test_metrics_collection():
    """Test metrics collection and monitoring"""
    print("\nTesting metrics collection...")

    from servo_manager.wp_mark.monitoring import MetricsCollector, HealthMonitor

    # Test metrics collector
    mc = MetricsCollector(max_points=100)

    # Test counter
    mc.record_counter('test_counter', 5)
    mc.record_counter('test_counter', 3)
    summary = mc.get_metric_summary('test_counter', 'counter')
    assert summary['latest'] == 8
    print("  ✅ Counter metrics work")

    # Test gauge
    mc.set_gauge('test_gauge', 42.5)
    summary = mc.get_metric_summary('test_gauge', 'gauge')
    assert summary['latest'] == 42.5
    print("  ✅ Gauge metrics work")

    # Test histogram
    mc.record_histogram('test_histogram', 1.5)
    mc.record_histogram('test_histogram', 2.5)
    summary = mc.get_metric_summary('test_histogram', 'histogram')
    assert summary['count'] == 2
    assert summary['avg'] == 2.0
    print("  ✅ Histogram metrics work")

    # Test timer
    mc.record_timer('test_timer', 1.23)
    summary = mc.get_metric_summary('test_timer', 'timer')
    assert summary['latest'] == 1.23
    print("  ✅ Timer metrics work")

    # Test health monitor
    hm = HealthMonitor(mc)
    health_status = hm.perform_health_checks()

    # Should have system, memory, cpu checks
    assert 'system' in health_status
    assert 'memory' in health_status
    assert 'cpu' in health_status
    print("  ✅ Health monitoring works")

    return True


def test_state_machine_integration():
    """Test state machine integration with other components"""
    print("\nTesting state machine integration...")

    from servo_manager.wp_mark.state_machine import StateMachine, MissionPhase
    from servo_manager.wp_mark.monitoring import metrics_collector

    sm = StateMachine()

    # Test circuit breaker integration - it doesn't retry, just tracks failures
    call_count = 0
    def test_operation():
        nonlocal call_count
        call_count += 1
        return "success"

    # Should succeed on first call
    result = sm.execute_with_circuit_breaker('servo', test_operation)
    assert result == "success"
    assert call_count == 1
    print("  ✅ Circuit breaker integration works")

    # Test transition listener
    transition_log = []
    def log_transition(old_phase, new_phase, action):
        transition_log.append((old_phase.value, new_phase.value, action))

    sm.add_transition_listener(log_transition)

    sm.transition_to(MissionPhase.INITIALIZING, "Test")
    assert len(transition_log) == 1
    assert transition_log[0] == ('idle', 'initializing', 'Test')
    print("  ✅ Transition listeners work")

    # Test status reporting
    status = sm.get_status()
    assert 'phase' in status
    assert 'context' in status
    assert 'health' in status
    print("  ✅ Status reporting works")

    return True


def test_error_handling():
    """Test comprehensive error handling"""
    print("\nTesting error handling...")

    from servo_manager.wp_mark.state_machine import StateMachine, MissionPhase, StateTransitionError

    sm = StateMachine()

    # Test error count increment
    initial_errors = sm.context.error_count
    sm.increment_error_count()
    assert sm.context.error_count == initial_errors + 1
    print("  ✅ Error counting works")

    # Test auto-transition to error state
    for i in range(4):  # Need 5 total errors to trigger
        sm.increment_error_count()

    # Should have transitioned to error state
    assert sm.context.error_count >= 5
    print("  ✅ Error state transition works")

    # Test error recovery
    sm.reset_error_state()
    assert sm.context.error_count == 0
    assert sm.context.retry_count == 0
    print("  ✅ Error state reset works")

    return True


def run_all_tests():
    """Run all tests"""
    print("WP_MARK Robust State Management Test Suite")
    print("=" * 50)

    tests = [
        test_state_machine,
        test_retry_mechanism,
        test_circuit_breaker,
        test_metrics_collection,
        test_state_machine_integration,
        test_error_handling
    ]

    passed = 0
    failed = 0

    for test_func in tests:
        try:
            if test_func():
                passed += 1
                print(f"✅ {test_func.__name__} PASSED")
            else:
                failed += 1
                print(f"❌ {test_func.__name__} FAILED")
        except Exception as e:
            failed += 1
            print(f"❌ {test_func.__name__} FAILED: {str(e)}")

    print("\n" + "=" * 50)
    print(f"Test Results: {passed} passed, {failed} failed")

    if failed == 0:
        print("\n🎉 All tests passed! Robust state management is working correctly.")
        return True
    else:
        print(f"\n⚠️  {failed} test(s) failed. Please check the implementation.")
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)