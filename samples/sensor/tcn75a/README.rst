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

This sample can be built with any board that supports I2C and GPIO.

Sample Output
=============

The application will configure sensor upper and lower limits.
Once the user warms up the sensor and causes it to pass the upper limit,
an alert will fire. The sensor will then alert the user again as it cools off
and passes the lower limit.

.. code-block:: console

   Current temperature is 22.12
   Set temperature upper limit to: 23.62 C
   Set temperature lower limit to: 22.62 C
   Temperature above threshold: 23.75
   Temperature below threshold: 22.37
