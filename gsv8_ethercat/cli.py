#!/usr/bin/env python3
"""
GSV-8 Command-Line Interface
Entry points for command-line tools
"""

import sys
from .sensor import GSV8Sensor
from .diagnostics import GSV8Diagnostics
from .calibration import GSV8Calibration
from .monitor import GSV8Monitor


def run_diagnostics_cli():
    """Command-line tool for running diagnostics"""
    if len(sys.argv) < 2:
        print("Usage: gsv8-diagnostics <adapter_name>")
        print("Example: gsv8-diagnostics '\\Device\\NPF_{YOUR_ID}'")
        sys.exit(1)
    
    adapter = sys.argv[1]
    
    print("\n" + "="*60)
    print("GSV-8 FULL DIAGNOSTICS")
    print("="*60)
    
    sensor = GSV8Sensor(adapter)
    
    if not sensor.connect():
        print("✗ Failed to connect to sensor")
        sys.exit(1)
    
    try:
        diag = GSV8Diagnostics(sensor)
        
        # Device info
        diag.print_device_info()
        
        # Calibration
        diag.print_calibration_info()
        
        # Read current data
        print("Reading current sensor state...")
        data = sensor.read_data()
        if data:
            diag.print_channel_status(data)
        
        # Communication test
        diag.check_communication(cycles=100)
        
        # Data analysis
        diag.analyze_channel_data(duration_sec=3.0, sample_rate=100.0)
        
        print("\n✓ Diagnostics complete")
        
    finally:
        sensor.disconnect()


def run_calibration_cli():
    """Command-line tool for interactive calibration"""
    if len(sys.argv) < 2:
        print("Usage: gsv8-calibrate <adapter_name>")
        print("Example: gsv8-calibrate '\\Device\\NPF_{YOUR_ID}'")
        sys.exit(1)
    
    adapter = sys.argv[1]
    
    sensor = GSV8Sensor(adapter)
    
    if not sensor.connect():
        print("✗ Failed to connect to sensor")
        sys.exit(1)
    
    try:
        cal = GSV8Calibration(sensor)
        diag = GSV8Diagnostics(sensor)
        
        while True:
            print("\n" + "="*60)
            print("CALIBRATION MENU")
            print("="*60)
            print("1. View current calibration")
            print("2. Zero all channels")
            print("3. Zero specific channel")
            print("4. Calibrate channel with known value")
            print("5. Reset calibration to defaults")
            print("6. Save calibration to file")
            print("7. Load calibration from file")
            print("8. Write calibration to device (PERMANENT)")
            print("9. View live data")
            print("0. Exit")
            print("="*60)
            
            choice = input("\nEnter choice: ").strip()
            
            if choice == '1':
                diag.print_calibration_info()
            
            elif choice == '2':
                cal.zero_all_channels()
            
            elif choice == '3':
                ch = int(input("Enter channel (0-7): "))
                cal.zero_channel(ch)
            
            elif choice == '4':
                ch = int(input("Enter channel (0-7): "))
                val = float(input("Enter known value: "))
                cal.calibrate_channel(ch, val)
            
            elif choice == '5':
                cal.reset_calibration()
            
            elif choice == '6':
                fname = input("Enter filename (default: gsv8_calibration.txt): ").strip()
                if not fname:
                    fname = "gsv8_calibration.txt"
                cal.save_calibration_to_file(fname)
            
            elif choice == '7':
                fname = input("Enter filename (default: gsv8_calibration.txt): ").strip()
                if not fname:
                    fname = "gsv8_calibration.txt"
                cal.load_calibration_from_file(fname)
            
            elif choice == '8':
                cal.write_calibration_to_device()
            
            elif choice == '9':
                monitor = GSV8Monitor(sensor)
                monitor.live_monitor(refresh_rate=10.0)
            
            elif choice == '0':
                break
            
            else:
                print("Invalid choice")
    
    finally:
        sensor.disconnect()


def run_monitor_cli():
    """Command-line tool for live monitoring"""
    if len(sys.argv) < 2:
        print("Usage: gsv8-monitor <adapter_name> [options]")
        print("Options:")
        print("  --log <filename>   Log to CSV file")
        print("  --duration <sec>   Logging duration (default: 10)")
        print("  --rate <hz>        Sample rate (default: 100)")
        print("\nExamples:")
        print("  gsv8-monitor '\\Device\\NPF_{YOUR_ID}'")
        print("  gsv8-monitor '\\Device\\NPF_{YOUR_ID}' --log data.csv --duration 60")
        sys.exit(1)
    
    adapter = sys.argv[1]
    
    # Parse options
    log_file = None
    duration = 10.0
    sample_rate = 100.0
    
    i = 2
    while i < len(sys.argv):
        if sys.argv[i] == '--log' and i + 1 < len(sys.argv):
            log_file = sys.argv[i + 1]
            i += 2
        elif sys.argv[i] == '--duration' and i + 1 < len(sys.argv):
            duration = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--rate' and i + 1 < len(sys.argv):
            sample_rate = float(sys.argv[i + 1])
            i += 2
        else:
            i += 1
    
    sensor = GSV8Sensor(adapter)
    
    if not sensor.connect():
        print("✗ Failed to connect to sensor")
        sys.exit(1)
    
    try:
        monitor = GSV8Monitor(sensor)
        
        if log_file:
            monitor.log_to_file(log_file, duration, sample_rate)
        else:
            monitor.live_monitor(refresh_rate=10.0)
    
    finally:
        sensor.disconnect()


if __name__ == '__main__':
    print("Use the installed command-line tools:")
    print("  gsv8-diagnostics")
    print("  gsv8-calibrate")
    print("  gsv8-monitor")
