.. zephyr:code-sample:: alif-se-services
   :name: Alif Ensemble Series SE services

   Demonstrates the use of SE services runtime APIs on Alif Ensemble Series boards.

Overview
********

The Secure Enclave (SE) on Alif Ensemble SoCs manages boot, power, and clocks, and provides
services to application cores over a mailbox. This sample calls the read-only SE service APIs
from ``main()`` and prints the results:

* SE synchronization and heartbeat
* SE firmware revision and TOC information
* Device part number, revision, lifecycle state, serial number, and EUI-48/EUI-64 extensions
* Random numbers from the SE
* Clock frequencies configured by the SE
* Run and off power profiles

The current run profile is written back unchanged with ``alif_se_service_set_run_cfg()`` to
demonstrate the local profile cache. After a 5 second delay, the sample asks the SE to reset the
CPU it runs on, so the output repeats.

Requirements
************

An Alif Ensemble board with the mailbox enabled

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/alif/se_services
   :board: ensemble_e8_dk/ae822fa0e5597ls0/rtss_hp
   :goals: build flash
   :compact:

Sample Output
*************

Values vary by device.

.. code-block:: console

   Alif SE services sample started
   SE is alive
   SE revision: SES A1 EVALUATION_BOARD SE_FW_1.104.000_DEV v1.104.0 Jun 20 2025 11:21:09
   TOC entries: 2
   TOC version: 0x00000001
   Part number: 0x0000e822
   Revision ID: 0x0000b400
   LCS: 1
   Serial number: 0123456789abcdef
   EUI-48 extension: 0a1b2c
   EUI-64 extension: 0a1b2c3d4e
   Random: 3f9e0c27d1a8b4e6f05c7a9d2b1e8c34
   HFOSC clock: 38400000 Hz
   EXTSYS0 clock: 400000000 Hz
   EXTSYS1 clock: 160000000 Hz
   AXI clock: 400000000 Hz
   AHB clock: 200000000 Hz
   APB clock: 100000000 Hz
   SYSREF clock: 100000000 Hz
   Run profile: power_domains 0x00000010, dcdc 825 mV, run_clk_src 2, cpu_clk_freq 1
   Cached run profile: power_domains 0x00000010
   Off profile: power_domains 0x00000000, wakeup_events 0x00000000
   Resetting CPU in 5 seconds
