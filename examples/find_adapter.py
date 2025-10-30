#!/usr/bin/env python3
"""
Find EtherCAT Network Adapter Script
This script scans for available network adapters and helps identify the correct
adapter to use with the GSV-8 sensor.
"""

import pysoem
import sys


def list_adapters():
    """
    List all available network adapters on the system

    Returns:
        List of adapter dictionaries with name and description
    """
    print("Scanning for network adapters...")
    print("=" * 70)

    try:
        # Get list of adapters from pysoem
        # Returns list of tuples: (name, description)
        adapters = pysoem.find_adapters()

        if not adapters:
            print("No network adapters found!")
            print("\nTroubleshooting:")
            print("1. Make sure you're running as Administrator (Windows) or root (Linux)")
            print("2. Check that npcap/winpcap is installed (Windows)")
            print("3. Check that your network adapter is enabled")
            return []

        print(f"Found {len(adapters)} network adapter(s):\n")

        for idx, adapter in enumerate(adapters):
            # adapter is a tuple: (name, description)
            adapter_name = adapter[0] if isinstance(adapter, tuple) else adapter
            adapter_desc = adapter[1] if isinstance(adapter, tuple) and len(adapter) > 1 else "No description"

            print(f"Adapter {idx + 1}:")
            print(f"  Name: {adapter_name}")
            print(f"  Description: {adapter_desc}")
            print()

        return adapters

    except Exception as e:
        print(f"Error scanning adapters: {e}")
        import traceback
        traceback.print_exc()
        return []


def scan_adapter_for_ethercat(adapter_name):
    """
    Scan if an adapter can find EtherCAT devices

    Args:
        adapter_name: Network adapter identifier

    Returns:
        Number of slaves found, or -1 on error
    """
    try:
        master = pysoem.Master()
        master.open(adapter_name)

        # Try to find EtherCAT slaves
        num_slaves = master.config_init()

        if num_slaves > 0:
            print(f"\n  Found {num_slaves} EtherCAT slave(s):")
            for i, slave in enumerate(master.slaves):
                print(f"    Slave {i}: {slave.name}")
                print(f"      Manufacturer: {slave.man}")
                print(f"      Product Code: {slave.id}")
                print(f"      Revision: {slave.rev}")

        master.close()
        return num_slaves

    except Exception as e:
        print(f"\n  Error testing adapter: {e}")
        return -1


def main():
    """Main function to find and test adapters"""
    print("GSV-8 EtherCAT Adapter Finder")
    print("=" * 70)
    print()

    # List all adapters
    adapters = list_adapters()

    if not adapters:
        print("\nNo adapters found. Exiting.")
        sys.exit(1)

    # Ask user which adapter to test
    print("\nSelect adapter to test for EtherCAT devices:")
    print("  Enter adapter number (1-{})".format(len(adapters)))
    print("  Enter 'a' to test all adapters")
    print("  Enter 'q' to quit")

    choice = input("\nYour choice: ").strip().lower()

    if choice == 'q':
        print("Exiting.")
        return

    if choice == 'a':
        # Test all adapters
        print("\nTesting all adapters for EtherCAT devices...")
        print("=" * 70)

        found_devices = False
        for idx, adapter in enumerate(adapters):
            adapter_name = adapter[0]
            adapter_desc = adapter[1] if len(adapter) > 1 else "No description"

            print(f"\nTesting Adapter {idx + 1}: {adapter_desc}")
            num_slaves = scan_adapter_for_ethercat(adapter_name)

            if num_slaves > 0:
                found_devices = True
                print(f"\n>>> FOUND ETHERCAT DEVICES ON THIS ADAPTER <<<")
                print(f">>> Use this adapter name in your code:")
                print(f">>> ADAPTER = r'{adapter_name}'")

        if not found_devices:
            print("\n" + "=" * 70)
            print("No EtherCAT devices found on any adapter.")
            print("\nTroubleshooting:")
            print("1. Make sure your GSV-8 device is powered on")
            print("2. Check the EtherCAT cable connection")
            print("3. Verify the network adapter is the one connected to GSV-8")
            print("4. Try running as Administrator/root")

    else:
        # Test specific adapter
        try:
            adapter_idx = int(choice) - 1
            if 0 <= adapter_idx < len(adapters):
                adapter = adapters[adapter_idx]
                adapter_name = adapter[0]
                adapter_desc = adapter[1] if len(adapter) > 1 else "No description"

                print(f"\nTesting Adapter {adapter_idx + 1}: {adapter_desc}")
                num_slaves = scan_adapter_for_ethercat(adapter_name)

                if num_slaves > 0:
                    print(f"\n>>> FOUND ETHERCAT DEVICES <<<")
                    print(f">>> Use this adapter name in your code:")
                    print(f">>> ADAPTER = r'{adapter_name}'")
                else:
                    print("\nNo EtherCAT devices found on this adapter.")
            else:
                print(f"Invalid adapter number. Please choose 1-{len(adapters)}")
        except ValueError:
            print("Invalid input. Please enter a number, 'a', or 'q'")

    print("\n" + "=" * 70)
    print("Done!")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting.")
        sys.exit(0)