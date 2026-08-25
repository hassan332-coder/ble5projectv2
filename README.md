**B2P Safety System – BLE 5 (LE Coded) Advertiser & Scanner on ESP32-C6**

A Bike-to-Pedestrian (B2P) proximity safety application built on the ESP32-C6, using Bluetooth 5 LE Coded PHY for long-range advertising. One device broadcasts GPS location as a BLE beacon; the other scans for it, calculates the distance between the two, and raises a proximity alert when they are within a critical range (e.g., under 10 meters).

**Overview**

This project uses two ESP32-C6 boards operating in complementary BLE roles:

Advertiser – Continuously broadcasts a BLE 5 advertisement (using LE Coded PHY for extended range) with the device's live GPS coordinates embedded in the payload.
Scanner – Listens for the advertising beacon, extracts the GPS payload, computes the real-world distance to the advertiser using its own GPS position, and triggers a proximity warning if the distance falls below a defined safety threshold (e.g., 10 meters).

The goal is a low-cost, real-time collision-avoidance aid — for example, alerting a pedestrian when a bike/rider equipped with the advertiser is nearby, even in low-visibility conditions.

**How It Works**
Advertiser side (bike/rider unit):
Reads current GPS coordinates from a connected GPS module.
Packages latitude/longitude (and optionally timestamp/speed) into the BLE advertising payload.
Broadcasts using BLE 5 LE Coded PHY, which extends range compared to standard LE 1M PHY — useful for early warning before the two parties are close.
Scanner side (pedestrian unit):
Scans for the advertiser's beacon on LE Coded PHY.
Parses the GPS payload from the received advertisement.
Reads its own current GPS location.
Calculates the distance between the two GPS points (e.g., using the Haversine formula).
If the calculated distance is less than the safety threshold (10 meters), triggers a proximity alert (e.g., buzzer, LED, or notification).
