TCN75A Serial Temperature Sensor
###########################################

Overview
********

This sample application demonstrates the alert feature of the TCN75A
temperature sensor. The sensor will be configured with an alert window
slightly above ambient temperature. The user can then warm the sensor up
to see the alert trigger.

Requirements
************

- TCN75A wired to your board's I2C bus, and ALERT pin wired to a board GPIO

References
**********

 - TCN75A: https://ww1.microchip.com/downloads/en/DeviceDoc/21935D.pdf

Building and Running
********************

This sample can be built with any board that supports I2C and GPIO. An
overlay for the :ref:`frdm_k22f` is provided, which assumes that the
temperature sensor is wired to I2C0 with an address of 0x48. The
ALERT signal should be connected to PTC2 (A3).

Sample Output
=============

The application will configure sensor upper and lower limits.
Once the user warms up the sensor and causes it to pass the upper limit,
an alert will fire. The sensor will then alert the user again as it cools off
and passes the lower limit.

.. code-block:: console

   Current temperature is 20.00
   Set temperature upper limit to: 21.50 C
   Set temperature lower limit to: 20.50 C
   Temperature above threshold: 21.75
   Temperature below threshold: 20.25
